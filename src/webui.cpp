#include "webui.h"
#include <WiFi.h>
#include <WebServer.h>

// Externs from main code
extern double input;
extern double output;
extern double Kp;
extern double Ki;
extern double Kd;
extern double setpoint;
double getLeftSpeedCps();
double getRightSpeedCps();
extern double Kp_speed_L;
extern double Ki_speed_L;
extern double Kd_speed_L;
extern double Kp_speed_R;
extern double Ki_speed_R;
extern double Kd_speed_R;
extern double lastMotorCmdL;
extern double lastMotorCmdR;
void saveDefaults();
extern double userTargetCps;
extern double userTurnCps;
extern const double MaxUserSpeed;
extern const double MaxUserTurn;

WebServer server(80);

const char INDEX_HTML[] = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <style>body{font-family:Arial,Helvetica,sans-serif;padding:10px}</style>
  <script>
    async function fetchStatus(){
      const r = await fetch('/status');
      const j = await r.json();
      document.getElementById('angle').innerText = j.angle.toFixed(2);
      document.getElementById('speedL').innerText = j.speedL.toFixed(1);
      document.getElementById('speedR').innerText = j.speedR.toFixed(1);
      document.getElementById('target').innerText = j.target.toFixed(1);
      document.getElementById('error').innerText = j.error.toFixed(2);
      document.getElementById('motorL').innerText = j.motorL.toFixed(1);
      document.getElementById('motorR').innerText = j.motorR.toFixed(1);
      // populate form inputs
      document.getElementById('Kp').value = j.Kp;
      document.getElementById('Ki').value = j.Ki;
      document.getElementById('Kd').value = j.Kd;
      document.getElementById('Kp_sL').value = j.Kp_sL;
      document.getElementById('Ki_sL').value = j.Ki_sL;
      document.getElementById('Kd_sL').value = j.Kd_sL;
      document.getElementById('Kp_sR').value = j.Kp_sR;
      document.getElementById('Ki_sR').value = j.Ki_sR;
      document.getElementById('Kd_sR').value = j.Kd_sR;
    }
    setInterval(fetchStatus,500);
    window.onload = fetchStatus;
  </script>
</head>
<body>
  <h2>Robot status</h2>
  <p>Angle: <span id='angle'>-</span></p>
  <p>Speed (L,R): <span id='speedL'>-</span>, <span id='speedR'>-</span></p>
  <p>Target Speed: <span id='target'>-</span></p>
  <p>PID Error: <span id='error'>-</span></p>
  <p>Motor Cmd (L,R): <span id='motorL'>-</span>, <span id='motorR'>-</span></p>

  <h3>Angle PID</h3>
  <form id='angleForm' onsubmit="fetch('/setpid',{method:'POST',body:new URLSearchParams(new FormData(this))}).then(()=>fetchStatus());return false;">
    Kp: <input id='Kp' name='Kp' value=''><br>
    Ki: <input id='Ki' name='Ki' value=''><br>
    Kd: <input id='Kd' name='Kd' value=''><br>
    <input type='submit' value='Set Angle PID'>
  </form>

  <h3>Speed PID (Left)</h3>
  <form id='speedLForm' onsubmit="fetch('/setspeed',{method:'POST',body:new URLSearchParams(new FormData(this))}).then(()=>fetchStatus());return false;">
    Kp: <input id='Kp_sL' name='Kp_sL' value=''><br>
    Ki: <input id='Ki_sL' name='Ki_sL' value=''><br>
    Kd: <input id='Kd_sL' name='Kd_sL' value=''><br>
    <input type='submit' value='Set Speed PID L'>
  </form>

  <h3>Speed PID (Right)</h3>
  <form id='speedRForm' onsubmit="fetch('/setspeed',{method:'POST',body:new URLSearchParams(new FormData(this))}).then(()=>fetchStatus());return false;">
    Kp: <input id='Kp_sR' name='Kp_sR' value=''><br>
    Ki: <input id='Ki_sR' name='Ki_sR' value=''><br>
    Kd: <input id='Kd_sR' name='Kd_sR' value=''><br>
    <input type='submit' value='Set Speed PID R'>
  </form>

  <button onclick="fetch('/savedefaults',{method:'POST'}).then(()=>alert('Saved'))">Save Defaults</button>

  <h3>Drive Controls</h3>
  <button onclick="fetch('/cmd',{method:'POST',body:'action=forward'})">Forward</button>
  <button onclick="fetch('/cmd',{method:'POST',body:'action=back'})">Back</button>
  <button onclick="fetch('/cmd',{method:'POST',body:'action=left'})">Left</button>
  <button onclick="fetch('/cmd',{method:'POST',body:'action=right'})">Right</button>
  <button onclick="fetch('/cmd',{method:'POST',body:'action=stop'})">Stop</button>

</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleSetPid() {
  if (server.hasArg("Kp")) Kp = server.arg("Kp").toFloat();
  if (server.hasArg("Ki")) Ki = server.arg("Ki").toFloat();
  if (server.hasArg("Kd")) Kd = server.arg("Kd").toFloat();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSetSpeed() {
  if (server.hasArg("Kp_sL")) Kp_speed_L = server.arg("Kp_sL").toFloat();
  if (server.hasArg("Ki_sL")) Ki_speed_L = server.arg("Ki_sL").toFloat();
  if (server.hasArg("Kd_sL")) Kd_speed_L = server.arg("Kd_sL").toFloat();
  if (server.hasArg("Kp_sR")) Kp_speed_R = server.arg("Kp_sR").toFloat();
  if (server.hasArg("Ki_sR")) Ki_speed_R = server.arg("Ki_sR").toFloat();
  if (server.hasArg("Kd_sR")) Kd_speed_R = server.arg("Kd_sR").toFloat();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStatus() {
  // Build JSON status
  String js = "{";
  js += "\"angle\":" + String(input) + ",";
  js += "\"speedL\":" + String(getLeftSpeedCps()) + ",";
  js += "\"speedR\":" + String(getRightSpeedCps()) + ",";
  double target = (output / 255.0) * 200.0;
  js += "\"target\":" + String(target) + ",";
  js += "\"error\":" + String(setpoint - input) + ",";
  js += "\"motorL\":" + String(lastMotorCmdL) + ",";
  js += "\"motorR\":" + String(lastMotorCmdR) + ",";
  // include current PID values so UI can populate fields
  js += "\"Kp\":" + String(Kp) + ",\"Ki\":" + String(Ki) + ",\"Kd\":" + String(Kd) + ",";
  js += "\"Kp_sL\":" + String(Kp_speed_L) + ",\"Ki_sL\":" + String(Ki_speed_L) + ",\"Kd_sL\":" + String(Kd_speed_L) + ",";
  js += "\"Kp_sR\":" + String(Kp_speed_R) + ",\"Ki_sR\":" + String(Ki_speed_R) + ",\"Kd_sR\":" + String(Kd_speed_R);
  js += "}";
  server.send(200, "application/json", js);
}

void webuiSetup() {
  WiFi.softAP("Robot_Balance", "12345678");
  server.on("/", handleRoot);
  server.on("/setpid", HTTP_POST, handleSetPid);
  server.on("/setspeed", HTTP_POST, handleSetSpeed);
  server.on("/cmd", HTTP_POST, [](){
    String action = server.arg("action");
    if (action == "forward") {
      userTargetCps = MaxUserSpeed;
      userTurnCps = 0;
    } else if (action == "back") {
      userTargetCps = -MaxUserSpeed;
      userTurnCps = 0;
    } else if (action == "left") {
      userTurnCps = -MaxUserTurn;
      userTargetCps = 0;
    } else if (action == "right") {
      userTurnCps = MaxUserTurn;
      userTargetCps = 0;
    } else { // stop
      userTargetCps = 0;
      userTurnCps = 0;
    }
    server.send(200, "text/plain", "OK");
  });
  server.on("/savedefaults", HTTP_POST, [](){ saveDefaults(); server.send(200, "text/plain", "OK"); });
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();
}

void webuiLoop() {
  server.handleClient();
}

