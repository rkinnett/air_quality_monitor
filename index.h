const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>

<head>
<style>
table {
  font-family: "Courier New", Courier, Monospace;
  border-collapse: collapse;
}
table.concentrations td, th {
  border: 1px solid #dddddd;
  text-align: left;
  padding: 8px;
}
table.tph td {
  border: 0px solid #ffffff;
  text-align: left;
  padding: 4px;
}
p { font-family: "Courier New", Courier, Monospace; }
</style>
</head>

<body onload="getData()">

<h1>Air Quality Monitor</h1>
<p>
  <span id="date_str">date</span>&nbsp;&nbsp; <span id="time_str">time</span> <br>
</p>

<table class=tph>
  <tr>  <td>Temperature:</td>  <td style="text-align:right"> <span id="temperature">0</span></td>   <td>C</td>
  <tr>  <td>Pressure:</td>     <td style="text-align:right"> <span id="pressure">0</span></td>      <td>hPa</td>
  <tr>  <td>Humidity:</td>     <td style="text-align:right"> <span id="humidity">0</span></td>      <td>%</td>
</table>
<br>

<div>
<table class=concentrations>
  <tr>  <th>Polutant</th>  <th>latest</th> <th>low pass</th>  <th>units</th> <th>AQI</th> <th>AQI Class</th> </tr>
  <tr>  <td>PM2.5</td>  <td style="text-align:right"> <span id="pm25">0</span> </td>     <td style="text-align:right"> <span id="pm25_lpf">0</span> </td>       <td>ug/m3</td>  <td><span id="pm25_aqi">0</span></td>   <td><span id="pm25_aqi_label">0</span></td>  </tr>
  <tr>  <td>PM10</td>   <td style="text-align:right"> <span id="pm10">0</span> </td>      <td style="text-align:right"> <span id="pm10_lpf">0</span> </td>      <td>ug/m3</td>  <td><span id="pm10_aqi">0</span></td>   <td><span id="pm10_aqi_label">0</span></td>  </tr>
  <tr>  <td>O3</td>     <td style="text-align:right"> <span id="o3_ppm">0</span> </td>    <td style="text-align:right"> <span id="o3_ppm_lpf">0</span> </td>    <td>ppm</td>    <td><span id="o3_aqi">0</span></td>     <td><span id="o3_aqi_label">0</span></td>  </tr>
  <tr>  <td>CO</td>     <td style="text-align:right"> <span id="co_ppm">0</span> </td>    <td style="text-align:right"> <span id="co_ppm_lpf">0</span> </td>    <td>ppm</td>    <td><span id="co_aqi">0</span></td>     <td><span id="co_aqi_label">0</span></td>  </tr>
  <tr>  <td>NO2</td>    <td style="text-align:right"> <span id="no2_ppm">0</span> </td>   <td style="text-align:right"> <span id="no2_ppm_lpf">0</span> </td>   <td>ppm</td>    <td><span id="no2_aqi">0</span></td>    <td><span id="no2_aqi_label">0</span></td>  </tr>
  <tr>  <td>SO2</td>    <td style="text-align:right"> <span id="so2_ppm">0</span> </td>   <td style="text-align:right"> <span id="so2_ppm_lpf">0</span> </td>   <td>ppm</td>    <td><span id="so2_aqi">0</span></td>    <td><span id="so2_aqi_label">0</span></td>  </tr>
</table>  

</div>

<script>

setInterval(function() {
  // Call a function repetatively with 2 Second interval
  getData();
}, 2000); //2000mSeconds update rate

sensors = {
  pm25:  {val: 0, rng: [0, 12.1, 35.5, 55.5, 150.5, 250.5, 500],      rngmax:350.5,  aqi_val:0,  aqi_lbl: "",  aqi_lbl_color: "#000"},
  pm10:  {val: 0, rng: [0, 55, 155, 255, 355, 425, 605],              rngmax:505,    aqi_val:0,  aqi_lbl: "",  aqi_lbl_color: "#000"},
  o3:    {val: 0, rng: [0, 0.055, 0.125, 0.165, 0.205, 0.405, 0.605], rngmax:0.605,  aqi_val:0,  aqi_lbl: "",  aqi_lbl_color: "#000"},
  co:    {val: 0, rng: [0, 4.5, 9.5, 12.5, 15.5, 30.5, 50.4],         rngmax:40.5,   aqi_val:0,  aqi_lbl: "",  aqi_lbl_color: "#000"},
  no2:   {val: 0, rng: [0, 0.054, 0.101, 0.361, 0.650, 1.250, 2.049], rngmax:1.65,   aqi_val:0,  aqi_lbl: "",  aqi_lbl_color: "#000"},
  so2:   {val: 0, rng: [0, 0.036, 0.076, 0.186, 0.305, 0.605, 1.004], rngmax:0.805,  aqi_val:0,  aqi_lbl: "",  aqi_lbl_color: "#000"}
};

function getData() {
  const url = "getVals";
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {

      console.log(this.responseText);
      var vals = this.responseText.split(",");
    
      if (vals.length==17){
        document.getElementById("pm25").innerHTML = vals[0];
        document.getElementById("pm10").innerHTML = vals[1];
        document.getElementById("o3_ppm").innerHTML = vals[2];
        document.getElementById("co_ppm").innerHTML = vals[3];
        document.getElementById("no2_ppm").innerHTML = vals[4];
        document.getElementById("so2_ppm").innerHTML = vals[5];
    
        document.getElementById("pm25_lpf").innerHTML = Math.round(vals[6]*10)/10;
        document.getElementById("pm10_lpf").innerHTML = Math.round(vals[7]*10)/10;
        document.getElementById("o3_ppm_lpf").innerHTML = vals[8];
        document.getElementById("co_ppm_lpf").innerHTML = Math.round(vals[9]*10)/10;
        document.getElementById("no2_ppm_lpf").innerHTML = Math.round(vals[10]*1000)/1000;
        document.getElementById("so2_ppm_lpf").innerHTML = Math.round(vals[11]*100)/100;
      
        document.getElementById("temperature").innerHTML = vals[12];
        document.getElementById("pressure").innerHTML = vals[13];
        document.getElementById("humidity").innerHTML = vals[14];
        document.getElementById("date_str").innerHTML = vals[15];
        document.getElementById("time_str").innerHTML = vals[16];
    
        sensors["pm25"].val = vals[6];
        sensors["pm10"].val = vals[7];
        sensors["o3"].val   = vals[8];
        sensors["co"].val   = vals[9];
        sensors["no2"].val  = vals[10];
        sensors["so2"].val  = vals[11];

        calculateAQI();

        for(sensorname in sensors){
          document.getElementById(sensorname+"_aqi").innerHTML = sensors[sensorname].aqi_val.toFixed(0);
          document.getElementById(sensorname+"_aqi_label").innerHTML = sensors[sensorname].aqi_lbl;
      document.getElementById(sensorname+"_aqi_label").style.color = sensors[sensorname].aqi_lbl_color;
        }   
      }
    }
  };
  xhttp.open("GET", url, true);
  xhttp.send();
}

function calculateAQI(){
  aqi_labels = ["Good", "Moderate", "Mildly unhealthy", "Unhealthy", "Very unhealthy", "Hazardous"];
  aqi_ranges = [0,50,100,150,200,300,400];
  aqi_colors = ["#0B0", "#EE1", "#FA0", "#F00", "#D0D", "#AOE"];
  aqi_nranges = 6;
  max_sensor_aqi = 0;
  max_aqi_label = "";

  for(sensorname in sensors){
    sensor = sensors[sensorname];
    if(sensor.val<=0){
    sensor.aqi_val = 0;
    sensor.aqi_lbl = "Good";
    sensor.aqi_lbl_color = aqi_colors[0];
  }
  else {
    sensor.aqi_val = NaN;
    sensor.aqi_lbl = "out of range?";
    for(i=0; i<aqi_nranges; i++){
      if(sensor.val>sensor.rng[i] && sensor.val<=sensor.rng[i+1]){
      sensor.aqi_val = Math.trunc(  aqi_ranges[i] + (aqi_ranges[i+1]-aqi_ranges[i])*(sensor.val-sensor.rng[i])/(sensor.rng[i+1]-sensor.rng[i])  );
      sensor.aqi_lbl = aqi_labels[i];
      sensor.aqi_lbl_color = aqi_colors[i];
      if(sensor.aqi_val > max_sensor_aqi){
        max_sensor_aqi = sensor.aqi_val;
        max_aqi_label = aqi_labels[i];
      } 
      break;
      }
    }
    }
  console.log(sensorname + ": " + sensor.val + " " + i + " " + sensor.aqi_val + " " + sensor.aqi_lbl); // show popup with a random number
  }
  console.log("aqi: " + max_sensor_aqi + " " + max_aqi_label);
}
  
  
</script>
</body>
</html>
)=====";
