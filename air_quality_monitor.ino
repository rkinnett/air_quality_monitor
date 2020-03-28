#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <DS3231.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1015.h>
#include <SoftwareSerial.h>
#include "index.h" //Our HTML webpage contents with javascripts

////////////////////////////////////////////////////////////////////////
// AIR MONITOR PROJECT, R. Kinnett, 2019
// https://hackaday.io/project/167435-a-more-complete-air-quality-monitor
//
// Sorry, this is a mess, not well commented or documented. 
// This code is designed to run on a Feather Huzzah32 board,
// with a specific hardware configuration. All necessary
// hardware are listed on the above project page.  If you 
// build your own, you will need to update many configuration
// parameters preceding the Setup() below.  Again, sorry
// it's not well organized.  Note that the strGasSensor struct
// contains many unit-specific calibration parameters which
// were calculated by hand by post-processing sample data
// and best-fitting to data from a local EPA station.  This
// project does not provide those offline processing methods.
//
// Your index.h file needs to be in the same directory as this
// Arduino code file.
//
///////////////////////////////////////////////////////////////////////////



// PREFERRED NETWORK CONFIG:
#ifndef STASSID
#define STASSID "YOUR_WIFI_SSID"
#define STAPSK  "YOUR_WIFI_SSID_PASSWORD"
#endif
const char* ssid = STASSID;
const char* password = STAPSK;


// FALLBACK network config:
const char* ssid_fallback_ap = "AirQual";
IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);


ESP8266WebServer server(80);   //instantiate server at port 80 (http port)
String page = "";

//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot() {
 String s = MAIN_page; //Read HTML contents
 server.sendHeader("Access-Control-Allow-Origin", "*");
 server.send(200, "text/html", s); //Send web page
}

void handleNotFound() {
    if (server.method() == HTTP_OPTIONS) {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Max-Age", "10000");
        server.sendHeader("Access-Control-Allow-Methods", "PUT,POST,GET,OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "*");
        server.send(204);
    } else {
        server.send(404, "text/plain", "");
    }
}


WiFiServer telnet(23);
WiFiClient telnetClient;
int disconnectedClient = 1;

#define TELNET_PRINT(x) { if(telnetClient) { telnetClient.print(x); } }


uint8_t addr_bme280 = 0x76;
uint8_t addr_adc[]  = {0x48, 0x4A};
uint8_t addr_rtc    = 0x68;


///////////// RTC stuff: //////////////
DS3231 Clock;
bool Century=false;
bool h12;
bool PM;
byte ADay, AHour, AMinute, ASecond, ABits;
bool ADy, A12h, Apm;
byte year, month, date, dayOfWeek, dayOfMonth, hour, minute, second;
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val) {  return( (val/10*16) + (val%10) ); }
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val) { return( (val/16*10) + (val%16) ); }


/////// BME280 stuff:  //////
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
unsigned long delayTime;
bool statusBme280;
float temp, temp_lpf;
uint16_t temp_lpf_scale = 120;


/////// ADC stuff:  //////
Adafruit_ADS1115 adc[2]{addr_adc[0], addr_adc[1]};
#define GAIN_IDX(x) ((x & ADS1015_REG_CONFIG_PGA_MASK)>>8)/2   // only used for gain readback during setup
float adc_gainvals[] = {0.666667, 1.0, 2.0, 4.0, 8.0, 16.0};  // only used for gain readback during setup
float adc_gain[2];
float adc_vrange[] = {4.096, 4.096};
float adcVolts(int dn, float vrange) { return (float)dn/32768.0*vrange; }




///// Particle sensor stuff: //////
SoftwareSerial pmsSerial(D7, D8);
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
  float pm10_lpf, pm25_lpf;
};
struct pms5003data pmData;
float lpfScalePm10 = 60;
float lpfScalePm25 = 60;

typedef struct strGasSensor {
  String gas;
  uint8_t adc;
  uint8_t adcChanVgas;
  uint8_t adcChanVbus;
  float gain;
  float sensitivity;
  float vbus;
  float vcal;
  float temp_coeffs_offset[3];
  float temp_correction_offset;
  float temp_coeffs_sensitivity[3];
  float temp_correction_sensitivity;
  float temp_sensitivity_ff;
  float vgas;
  float vgas_lpf;
  float ppm;
  float ppm_lpf;
};
struct strGasSensor gasSensor[] = {
  {"O3",  1, 1, 0, 499.0, -54.05, 0, -0.00050, {0.0015, 0.098,  0.01}, 0, {1.19, 0.0150, -100.0}, 1.0, 0, 0, 0, 0, 0},
  {"CO",  0, 2, 3, 100.0,   2.25, 0,  0.02010, {1.2000, 0.080, -6.00}, 0, {1.15, 0.0260,  -53.0}, 1.0, 0, 0, 0, 0, 0},
  {"NO2", 0, 0, 1, 499.0, -22.09, 0,  0.00540, {0.0024, 0.090,  0.01}, 0, {1.20, 0.0125, -120.0}, 1.0, 0, 0, 0, 0, 0},
  {"SO2", 1, 3, 2, 100.0,  30.16, 0,  0.13030, {0.3500, 0.090, -0.20}, 0, {1.25, 0.0263,  -38.0}, 1.0, 0, 0, 0, 0, 0},
};
enum gasSensor {O3, CO, NO2, SO2};

// Configure low-pass filters for each sensor
float lpfScale[] = {600, 600, 600, 600};
float lpfScaleVbus = 120;

float vSensorBus;
float vbus_lpf;


void setup() {
  Serial.begin(115200);
  Serial.flush();
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Failed to connect to default network.");
    WiFi.disconnect();
    
    Serial.print("Configuring access point...");
    Serial.print("Setting soft-AP configuration ... ");
    Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
    WiFi.softAP(ssid_fallback_ap);  // optional second arg password
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    break;
  }

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting telnet server");
  telnet.begin();
  telnet.setNoDelay(true);
  telnetClient.print("Telnet server started");

  Serial.println("Starting i2c");
  Wire.begin(12,14);

  Serial.println("Initializing RTC");
  // second, minute, hour, dayOfWeek, dayOfMonth, month, year
  // setDS3231time(00,18,9,3,29,5,19);

  Serial.println("Starting BME280");
  statusBme280 = bme.begin(addr_bme280, &Wire);  
  if (!statusBme280) {
      Serial.println("Could not find a valid BME280 sensor");
  }
  temp_lpf = bme.readTemperature();

  Serial.println("Starting ADCs");
  for (int i=0; i<=1; i++) {
    adc[i].begin();
    delay(50);
    adc[i].setGain(GAIN_ONE);    
    Serial.print("ADC ");
    Serial.print(i);
    Serial.print(" gain:  ");
    Serial.print(adc[i].getGain());
    Serial.print(" ");
    adc_gain[i] = adc_gainvals[GAIN_IDX(adc[i].getGain())];
    Serial.println(adc_gain[i]);
  }

  gasSensor[CO].vgas_lpf = gasSensor[CO].vcal;
  gasSensor[NO2].vgas_lpf = gasSensor[NO2].vcal;
  gasSensor[SO2].vgas_lpf = gasSensor[SO2].vcal;

  Serial.println("Starting PM2.5 sensor");
  pmsSerial.begin(9600);

  server.on("/", handleRoot);      //Which routine to handle at root location. This is display page
  server.on("/getVals", updateWebPageVals);
  server.on("/getVgas", sendVgasVals);
  server.on("/getLpf", sendLpfVals);
  server.on("/genericArgs", handleGenericArgs);
  server.on("/setVcal", setVcal);
  server.on("/getVcal", sendVcal);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Web server started!");

  // Pause here for sensors to stabilize (60 mins is recommended per spec sheets. Not waiting that long!)
  uint8_t warmupSec = 10;
  Serial.print("Waiting " + String(warmupSec) + " sec for sensors to warm up...");
  for(int i=0; i<warmupSec; i++){
    ArduinoOTA.handle();
    server.handleClient();
    babysitTelnet();
    delay(1000);
  }
  Serial.println("done.");
  Serial.println("Startup complete.");
  Serial.println("");
}





void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  babysitTelnet();

  displayTime();
  Serial.println();
  //Serial.print(" ");
  //printTPH();
  readAdc(0);
  updateGas(NO2);
  updateGas(CO);
  readAdc(1);
  updateGas(SO2);
  updateGas(O3);

  if(telnetClient && false) { 
    telnetClient.print(timeStr());
    telnetClient.print(String("\t"));
    telnetClient.print(String(temp_lpf, 3));
    telnetClient.print(String("\t"));
    telnetClient.print(String(vbus_lpf, 5));
    telnetClient.print(String("\t"));
    telnetClient.print(String(gasSensor[O3].vgas,4));
    telnetClient.print(String("\t"));
    telnetClient.print(String(gasSensor[O3].ppm,4));
    telnetClient.print(String("\t"));
    telnetClient.print(String(gasSensor[O3].ppm_lpf,4));
    telnetClient.println(String(""));
  }
  
  readPms();

  temp = bme.readTemperature();
  if(isnan(temp)){
    Serial.println("");
    Serial.println("Error:  BME temperature measurement failed");
    Serial.println("");
  } else if(temp<-20 && temp>100){
    Serial.println("Warning:  BME temperature measurement out of range?  " + String(temp,5));
  } else {
    temp_lpf = temp/temp_lpf_scale + temp_lpf*(1.0-1.0/temp_lpf_scale);
  }
  
  Serial.println(" ");
  delay(1000);
}







void updateWebPageVals(){
  String msg;
  msg = String(pmData.pm25_env) + ","\
      + String(pmData.pm10_env) + ","\
      + String(gasSensor[O3].ppm,4) + ","\
      + String(gasSensor[CO].ppm,4) + ","\
      + String(gasSensor[NO2].ppm,4) + ","\
      + String(gasSensor[SO2].ppm,4) + ","\
      \
      + String(pmData.pm25_lpf) + ","\
      + String(pmData.pm10_lpf) + ","\
      + String(gasSensor[O3].ppm_lpf,4) + ","\
      + String(gasSensor[CO].ppm_lpf,4) + ","\
      + String(gasSensor[NO2].ppm_lpf,4) + ","\
      + String(gasSensor[SO2].ppm_lpf,4) + ","\
      \
      + String(temp_lpf,2) + ","\
      + String(bme.readPressure() / 100.0F,1) + ","\
      + String(bme.readHumidity(),1) + ","\
      \
      + dateStr() + ","\
      + timeStr(); 
 server.sendHeader("Access-Control-Allow-Origin", "*");     
 server.send(200, "text/plane", msg); //Send ADC value only to client ajax request

}


void sendVgasVals(){
  String msg;
  msg = dateStr() + ","\
      + timeStr() + ","\ 
      + String(temp_lpf, 3) + ","\
      + String(vbus_lpf, 5) + ","\
      + String(gasSensor[O3].vgas_lpf, 5) + ","\
      + String(gasSensor[CO].vgas_lpf, 5) + ","\
      + String(gasSensor[NO2].vgas_lpf, 5) + ","\
      + String(gasSensor[SO2].vgas_lpf, 5);
  server.sendHeader("Access-Control-Allow-Origin", "*");     
  server.send(200, "text/plane", msg); //Send ADC value only to client ajax request
}

void sendLpfVals(){
  String msg;
  float temp_correction_offset, temp_correction_sensitivity, ppm;
  msg = dateStr() + "\n"\
      + timeStr() + "\n"\ 
      + String(temp_lpf, 3) + "\n"\
      + String(vbus_lpf, 5);
  for(int gas=O3; gas<=SO2; gas++){
    temp_correction_offset = gasSensor[gas].temp_coeffs_offset[0]*exp(gasSensor[gas].temp_coeffs_offset[1]*temp_lpf)+gasSensor[gas].temp_coeffs_offset[2];
    temp_correction_sensitivity = gasSensor[gas].temp_coeffs_sensitivity[0]*(1.0-exp((gasSensor[gas].temp_coeffs_sensitivity[2]-temp_lpf)*gasSensor[gas].temp_coeffs_sensitivity[1]));
    ppm = ((gasSensor[gas].vgas_lpf -vbus_lpf*0.495 -gasSensor[gas].vcal)/gasSensor[gas].gain*1.0e6 -temp_correction_offset) /gasSensor[gas].sensitivity *(2.0-temp_correction_sensitivity);
    msg += "\n" \
      + String(gasSensor[gas].vgas_lpf,5) + " " \
      + String(temp_correction_offset,5) + " " \
      + String(temp_correction_sensitivity,5) + " " \
      + String(gasSensor[gas].vcal,5) + " " \
      + String(ppm,5);
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");     
  server.send(200, "text/plane", msg); //Send ADC value only to client ajax request
}



String timeStr(){
  String str;
  str = String(hour, DEC) + ":" \
  + (minute<10? "0" : "") + String(minute, DEC) + ":" \
  + (second<10? "0" : "") + String(second, DEC);
  return str;
}

String dateStr(){
  String weekDays[] = {"","Mon","Tues","Weds","Thurs","Fri","Sat","Sun"};
  String str;
  str =  weekDays[dayOfWeek] + " " \
    + String(month, DEC) + "/" \
    + String(dayOfMonth, DEC) + "/" \
    + String(year, DEC);
  return str;
}



void handleGenericArgs() { //Handler
  String message = "Number of args received:";
  message += server.args();            //Get number of parameters
  message += "\n";                            //Add a new line
  for (int i = 0; i < server.args(); i++) {
    message += "Arg nbr" + String(i) + ":  ";   //Include the current iteration value
    message += server.argName(i) + ": ";     //Get the name of the parameter
    message += server.arg(i) + "\n";              //Get the value of the parameter
  } 
  server.send(200, "text/plain", message);       //Response to the HTTP request
}

void setVcal() {
  String msg = "";
  msg += "setVcal(): Received " + String(server.args()) + " args";
  String argName;
  float argVal;
  uint8_t gas;

  for (int i=0; i<server.args(); i++) {
    argName = server.argName(i);
    argVal = server.arg(i).toFloat();
    msg += "\narg " + String(i) + " \"" + argName + "\": " + String(argVal,5);
    if(argVal<-0.5 || argVal>3.0){
      msg += "\nError, unrecognized value";
      break;
    } 
    else if(argName=="o") gas = O3;
    else if(argName=="c") gas = CO;
    else if(argName=="n") gas = NO2;
    else if(argName=="s") gas = SO2;
    else {
      msg += "\nError, unrecognized arg name: " + argName;
      break;
    }
    msg += "\nSetting " + gasSensor[gas].gas + " Vcal to " + String(argVal,5) + "V";
    gasSensor[gas].vcal = argVal;
    gasSensor[gas].ppm_lpf = 0;
  }
  server.send(200, "text/plain", msg);       //Response to the HTTP request
}

void sendVcal() {
  String msg = "" \
    + String(gasSensor[O3].vcal,5) + "\n" \
    + String(gasSensor[CO].vcal,5) + "\n" \
    + String(gasSensor[NO2].vcal,5) + "\n" \
    + String(gasSensor[SO2].vcal,5) + "\n";
  server.send(200, "text/plain", msg);       //Response to the HTTP request
}


void updateGas(uint8_t idxGasSensor) {
  struct strGasSensor *sensor = &gasSensor[idxGasSensor];
  float avgVgas = 0;
  float avgVbus = 0;
  float Nsamples = 10;
  for(int i=0; i<Nsamples; i++){
    //avgVgas += adcVolts( adc[sensor->adc].readADC_SingleEnded(sensor->adcChanVgas), adc_vrange[sensor->adc])/Nsamples;
    avgVgas += adcVolts( adc[sensor->adc].readADC_SingleEnded(sensor->adcChanVgas), adc_vrange[sensor->adc])/Nsamples;
    delay(1);
    avgVbus += adcVolts( adc[sensor->adc].readADC_SingleEnded(sensor->adcChanVbus), adc_vrange[sensor->adc])/Nsamples;
    delay(1);
  }
  sensor->vgas = avgVgas;
  sensor->vbus = avgVbus;

  sensor->temp_correction_offset = sensor->temp_coeffs_offset[0]*exp(sensor->temp_coeffs_offset[1]*temp_lpf)+sensor->temp_coeffs_offset[2];
  sensor->temp_correction_sensitivity = sensor->temp_coeffs_sensitivity[0]*(1.0-exp((sensor->temp_coeffs_sensitivity[2]-temp_lpf)*sensor->temp_coeffs_sensitivity[1]));

  Serial.print(sensor->gas);
  Serial.print(": ");
  Serial.print(sensor->vgas, 4);
  Serial.print("V, ");
  Serial.print(sensor->vcal, 3);
  Serial.print("V ");
  Serial.print(sensor->sensitivity, 3);
  Serial.print("nA/ppm ");
  Serial.print(sensor->ppm, 3);
  Serial.print(" ppm ");
  sensor->ppm = ((sensor->vgas -avgVbus*0.495 -sensor->vcal)/sensor->gain*1.0e6 -sensor->temp_correction_offset) /sensor->sensitivity *(2.0-sensor->temp_correction_sensitivity*sensor->temp_sensitivity_ff);


  Serial.print(sensor->ppm, 3);
  Serial.print(" ppm  (");
  Serial.print("Vbus: "); Serial.print(sensor->vbus,3); Serial.print("V, ");
  Serial.print("T=");  Serial.print(temp_lpf,2);  Serial.print("C");
  Serial.print(", temp offset "); Serial.print(sensor->temp_correction_offset,5); Serial.print(" ppm");
  Serial.print(", temp scale ");  Serial.print(sensor->temp_correction_sensitivity,5);
  Serial.println("");

  if(sensor->ppm_lpf==0) {
    sensor->ppm_lpf = sensor->ppm;
    sensor->vgas_lpf = sensor->vgas;
    vbus_lpf = sensor->vbus;
  } else {
    sensor->ppm_lpf  = sensor->ppm/lpfScale[idxGasSensor] + sensor->ppm_lpf*(1.0-1.0/lpfScale[idxGasSensor]);
    sensor->vgas_lpf = sensor->vgas/lpfScale[idxGasSensor] + sensor->vgas_lpf*(1.0-1.0/lpfScale[idxGasSensor]);
    vbus_lpf = sensor->vbus/lpfScaleVbus + vbus_lpf*(1.0-1.0/lpfScaleVbus);
  }

  if(idxGasSensor==0 && telnetClient){
    telnetClient.print(sensor->gas);
    telnetClient.print(", vgas "); telnetClient.print(sensor->vgas, 4);
    telnetClient.print(", vbus "); telnetClient.print(sensor->vbus,3); 
    telnetClient.print(", ppm ");  telnetClient.print(sensor->ppm, 4);
    telnetClient.print(", ppm lpf "); telnetClient.print(sensor->ppm_lpf, 5);
    telnetClient.print(", "); telnetClient.print(temp_lpf,2);  telnetClient.print("C");
    telnetClient.print(", temp offset "); telnetClient.print(sensor->temp_correction_offset,5); telnetClient.print(" ppm");
    telnetClient.print(", temp scale ");  telnetClient.print(sensor->temp_correction_sensitivity,5);
    telnetClient.println("");
    delay(5);
  }
  
}



void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year) {
  // sets time and date data to DS3231
  Wire.beginTransmission(addr_rtc);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}



void readDS3231time() {
  Wire.beginTransmission(addr_rtc);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(addr_rtc, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  second = bcdToDec(Wire.read() & 0x7f);
  minute = bcdToDec(Wire.read());
  hour = bcdToDec(Wire.read() & 0x3f);
  dayOfWeek = bcdToDec(Wire.read());
  dayOfMonth = bcdToDec(Wire.read());
  month = bcdToDec(Wire.read());
  year = bcdToDec(Wire.read());
}

void displayTime() {
  // retrieve data from DS3231
  readDS3231time();
  // send it to the serial monitor
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  if (minute<10)
  {
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second<10)
  {
    Serial.print("0");
  }
  Serial.print(second, DEC);
  Serial.print(" ");
  /*
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.print(year, DEC);
  Serial.println();
  */
}

void babysitTelnet(){
  if (telnet.hasClient()){
    if (!telnetClient || !telnetClient.connected()){
      if(telnetClient) {
        Serial.println("telnet client disconnected");
        telnetClient.stop();
      }
      telnetClient = telnet.available();
      Serial.println("Telnet client connected");
      telnetClient.flush();
      //while(telnetClient.available()) Serial.write(telnetClient.read());
      //Serial.println("");
      telnetClient.println("welcome!");
      return;
    }
  } else if(telnetClient && !telnetClient.connected()){
    Serial.println("telnet client disconnected? ");
    telnetClient.stop();
  }
}


void printTPH(){
  if(statusBme280){
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    // Serial.print("Approx. Altitude = ");
    // Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    // Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
  }
}


void readAdc(uint8_t idx){
  int16_t a[4];
  Serial.print("adc ");
  Serial.print(idx);
  Serial.print(": ");
  for (int c=0; c<=3; c++){
    a[c] = adc[idx].readADC_SingleEnded(c);
    delay(10);
    Serial.print(adcVolts(a[c], adc_vrange[idx]), 4); Serial.print(" ");
  }
  Serial.println();
}


void readPms() {
  if (readPMSdata(&pmsSerial)) {
    // reading data was successful!
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(pmData.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(pmData.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(pmData.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(pmData.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(pmData.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(pmData.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(pmData.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(pmData.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(pmData.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(pmData.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(pmData.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(pmData.particles_100um);
    Serial.println("---------------------------------------");
  }
}



boolean readPMSdata(Stream *s) { 
  bool foundStart = false;

  // read up to 32 bytes until find start sequence:
  for (uint8_t count=0; count<32; count++){
    if (!s->available()) {
      return false;
    }
    if (s->read()==66 && s->read()==77){
      foundStart = true;
      break;
    }
  }
  if (!foundStart){
    Serial.println("");
    Serial.println("PMS5003: failed to find start frame");
    return false;
  }
  
  // Read next 30 bytes
  if (s->available() < 30) {
    Serial.println("");
    Serial.print("PMS5003: message length ");
    Serial.print(s->available());
    Serial.println(" (<30)");
    //return false;
  }
    
  uint8_t buffer[30];   
  s->readBytes(buffer, 30);
 
  // get checksum ready
  uint16_t sum = 66+77;  // add the message start byte values
  //uint16_t sum;
  for (uint8_t i=0; i<28; i++) {
    sum += buffer[i];
  }
 
  //* debugging
  Serial.println("");
  for (uint8_t i=0; i<30; i++) {
    //Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    Serial.print(buffer[i]); Serial.print(" ");
  }
  Serial.println();
  //*/
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[i*2 + 1];
    buffer_u16[i] += (buffer[i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&pmData, (void *)buffer_u16, 30);
 
  if (sum != pmData.checksum) {
    Serial.print("PMS5003: Checksum failure ");
    Serial.print(sum);
    Serial.print("!=");
    Serial.print(pmData.checksum);
    Serial.print("  ");
    Serial.print(buffer_u16[14]);
    Serial.println("");
    return false;
  }

  pmData.pm10_lpf =  float(pmData.pm10_env)/lpfScalePm10 + float(pmData.pm10_lpf) * (1.0 - 1.0/lpfScalePm10);
  pmData.pm25_lpf =  float(pmData.pm25_env)/lpfScalePm25 + float(pmData.pm25_lpf) * (1.0 - 1.0/lpfScalePm25);
  
  // success!
  return true;
}
