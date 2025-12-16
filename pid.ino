/*
  Особливості даної ЛР
  ESP8266 (Witty Cloud) + ADXL345 (GY-291) + ESC (Simonk) : PID стабілізація тангажу до 0°
  - WiFi AP з веб-сторінкою (графік setpoint vs pitch, тюнінг PID у реальному часі)
  - WebSocket телеметрія 20 Гц
  - PID оновлення ~100 Гц, ESC оновлення ~50 Гц (Servo PWM 1000-2000 мкс)
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

static const uint8_t PIN_SDA = 4;   // GPIO4
static const uint8_t PIN_SCL = 5;   // GPIO5
static const uint8_t PIN_ESC = 15;  // GPIO15 (PWM на ESC)

// -------------------- ESC PWM --------------------
static const int ESC_MIN_US   = 1000;   // мінімальний газ / армінг
static const int ESC_MAX_US   = 2000;   // максимальний газ
static const int ESC_SAFE_US  = 1000;   // безпечне значення (стоп)
static const int ESC_ARM_TIME_MS = 2500;

// -------------------- WiFi AP --------------------
static const char* AP_SSID = "PID_BALANCER";
static const char* AP_PASS = "12345678"; // мін 8 символів

ESP8266WebServer server(80);
WebSocketsServer ws(81);
Servo esc;

//  ADXL345 
static const uint8_t ADXL345_DEVID       = 0x00;
static const uint8_t ADXL345_POWER_CTL   = 0x2D;
static const uint8_t ADXL345_DATA_FORMAT = 0x31;
static const uint8_t ADXL345_BW_RATE     = 0x2C;
static const uint8_t ADXL345_DATAX0      = 0x32;

uint8_t adxlAddr = 0;
bool adxlOk = false;

//  PID 
struct PIDController {
  float kp = 2.0f;
  float ki = 0.0f;
  float kd = 0.05f;

  float integrator = 0.0f;
  float prevMeas   = 0.0f;

  float iMin = -200.0f;
  float iMax =  200.0f;

  float outMin = -400.0f; // ΔPWM (мкс)
  float outMax =  400.0f;

  void reset(float currentMeas) {
    integrator = 0.0f;
    prevMeas   = currentMeas;
  }

  float update(float setpoint, float measurement, float dt, bool &saturated) {
    float err = setpoint - measurement;

    // Derivative by measurement (стійкіше)
    float dMeas = (measurement - prevMeas) / (dt > 0 ? dt : 1e-3f);
    prevMeas = measurement;

    float p = kp * err;
    float d = -kd * dMeas;

    float iCandidate = integrator + (ki * err * dt);
    iCandidate = constrain(iCandidate, iMin, iMax);

    float outCandidate = p + iCandidate + d;

    float outSat = constrain(outCandidate, outMin, outMax);
    saturated = (outSat != outCandidate);

    bool pushingFurther =
      (outSat >= outMax && err > 0) ||
      (outSat <= outMin && err < 0);

    if (!saturated || !pushingFurther) {
      integrator = iCandidate;
    }

    float out = p + integrator + d;
    return constrain(out, outMin, outMax);
  }
};

PIDController pid;


volatile bool controlEnabled = false;
volatile bool invertOutput   = false;

float setpointDeg    = 0.0f;   // заданий тангаж (°)
int   baseThrottleUs = 1100;   // базовий газ (мкс) - підбирається

float pitchDeg         = 0.0f;
float pitchOffsetDeg   = 0.0f;
float pitchFilteredDeg = 0.0f;

float lpfTau = 0.15f;          // сек (фільтр)

uint32_t lastSensorUs    = 0;
uint32_t lastEscUs       = 0;
uint32_t lastTelemetryMs = 0;

static const uint32_t SENSOR_PERIOD_US      = 10000; // 100 Гц
static const uint32_t ESC_PERIOD_US         = 20000; // 50 Гц
static const uint32_t TELEMETRY_PERIOD_MS   = 50;    // 20 Гц

float maxSafeAngleDeg = 45.0f;

//  HTML 
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>PID Balancer</title>
  <style>
    body{font-family:Arial,Helvetica,sans-serif;margin:16px;background:#0f1115;color:#e9eef7}
    .grid{display:grid;grid-template-columns:1fr;gap:12px;max-width:1100px;margin:0 auto}
    .card{background:#161a22;border:1px solid #2a3140;border-radius:12px;padding:12px}
    .row{display:flex;gap:12px;flex-wrap:wrap;align-items:center}
    label{display:block;font-size:12px;color:#aeb8cc;margin-bottom:6px}
    input[type="number"],input[type="text"]{background:#0f1115;color:#e9eef7;border:1px solid #2a3140;border-radius:8px;padding:8px;width:120px}
    input[type="range"]{width:260px}
    button{background:#2b66ff;color:#fff;border:0;border-radius:10px;padding:10px 14px;cursor:pointer}
    button.secondary{background:#263042}
    .mono{font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace}
    canvas{background:#0b0d12;border:1px solid #2a3140;border-radius:12px;width:100%;height:320px}
    .small{font-size:12px;color:#aeb8cc}
    .status{padding:6px 10px;border-radius:999px;background:#263042;display:inline-block}
    .ok{background:#134e2a}
    .bad{background:#5a1b1b}
  </style>
</head>
<body>
<div class="grid">
  <div class="card">
    <div class="row" style="justify-content:space-between">
      <div>
        <div style="font-size:18px;font-weight:700">PID Balancer (ESP8266)</div>
        <div class="small">Графік: заданий кут vs фактичний кут (тангаж). PID тюнінг в реальному часі.</div>
      </div>
      <div class="status mono" id="conn">DISCONNECTED</div>
    </div>
  </div>

  <div class="card">
    <canvas id="chart"></canvas>
    <div class="row" style="margin-top:10px">
      <div class="mono">Pitch: <span id="pitch">0.0</span>°</div>
      <div class="mono">Setpoint: <span id="spv">0.0</span>°</div>
      <div class="mono">Throttle: <span id="thv">1000</span> us</div>
      <div class="mono">Enabled: <span id="env">false</span></div>
    </div>
  </div>

  <div class="card">
    <div class="row">
      <div>
        <label>Setpoint (°)</label>
        <input type="number" id="sp" step="0.1" value="0.0">
      </div>
      <div>
        <label>Base throttle (us)</label>
        <input type="number" id="base" min="1000" max="1800" step="1" value="1100">
      </div>
      <div>
        <label>Max safe angle (°)</label>
        <input type="number" id="msa" min="5" max="80" step="1" value="45">
      </div>
      <div>
        <label>LPF tau (s)</label>
        <input type="number" id="tau" min="0.02" max="1.0" step="0.01" value="0.15">
      </div>
      <div style="margin-top:18px">
        <button id="toggle" class="secondary">Enable</button>
      </div>
      <div style="margin-top:18px">
        <button id="zero" class="secondary">Zero offset</button>
      </div>
      <div style="margin-top:18px">
        <button id="invert" class="secondary">Invert: OFF</button>
      </div>
    </div>

    <hr style="border:0;border-top:1px solid #2a3140;margin:14px 0"/>

    <div class="row">
      <div>
        <label>Kp</label>
        <input type="range" id="kp" min="0" max="20" step="0.01" value="2.0">
        <div class="small mono">Kp = <span id="kpv">2.00</span></div>
      </div>
      <div>
        <label>Ki</label>
        <input type="range" id="ki" min="0" max="10" step="0.001" value="0.0">
        <div class="small mono">Ki = <span id="kiv">0.000</span></div>
      </div>
      <div>
        <label>Kd</label>
        <input type="range" id="kd" min="0" max="5" step="0.001" value="0.05">
        <div class="small mono">Kd = <span id="kdv">0.050</span></div>
      </div>
    </div>
  </div>

  <div class="card small">
    Поради:
    <ul>
      <li>Base throttle підбирається експериментально.</li>
      <li>Починайте з Kp, потім додавайте Kd, і лише потім мінімально Ki.</li>
      <li>Якщо реакція навпаки — натисніть Invert.</li>
      <li>Enable вмикайте лише коли механіка готова.</li>
    </ul>
  </div>
</div>

<script>
(() => {
  const $ = (id)=>document.getElementById(id);
  const wsUrl = `ws://${location.hostname}:81/`;
  let ws;

  const canvas = $("chart");
  const ctx = canvas.getContext("2d");
  const N = 300;
  const pitchBuf = Array(N).fill(0);
  const spBuf = Array(N).fill(0);

  function resizeCanvas(){
    const rect = canvas.getBoundingClientRect();
    canvas.width = Math.floor(rect.width * devicePixelRatio);
    canvas.height = Math.floor(rect.height * devicePixelRatio);
  }
  window.addEventListener("resize", resizeCanvas);
  resizeCanvas();

  function push(buf, v){
    buf.push(v);
    if (buf.length > N) buf.shift();
  }

  function draw(){
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0,0,w,h);

    ctx.globalAlpha = 1.0;
    ctx.lineWidth = 1;
    ctx.strokeStyle = "#263042";
    for(let i=0;i<=10;i++){
      const y = (h*i/10);
      ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke();
    }
    for(let i=0;i<=10;i++){
      const x = (w*i/10);
      ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,h); ctx.stroke();
    }

    const yMin = -30, yMax = 30;
    const yMap = (v)=> {
      const t = (v - yMin)/(yMax - yMin);
      return h - t*h;
    };

    ctx.lineWidth = 2;
    ctx.strokeStyle = "#2b66ff";
    ctx.beginPath();
    for(let i=0;i<spBuf.length;i++){
      const x = i*(w/(N-1));
      const y = yMap(spBuf[i]);
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();

    ctx.strokeStyle = "#00d1b2";
    ctx.beginPath();
    for(let i=0;i<pitchBuf.length;i++){
      const x = i*(w/(N-1));
      const y = yMap(pitchBuf[i]);
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();

    ctx.fillStyle = "#aeb8cc";
    ctx.font = `${12*devicePixelRatio}px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace`;
    ctx.fillText("SP (blue) vs Pitch (green), scale: -30..+30 deg", 10*devicePixelRatio, 18*devicePixelRatio);
  }

  function setConn(ok){
    const el = $("conn");
    if(ok){
      el.textContent = "CONNECTED";
      el.classList.remove("bad");
      el.classList.add("ok");
    } else {
      el.textContent = "DISCONNECTED";
      el.classList.remove("ok");
      el.classList.add("bad");
    }
  }

  function sendCfg(partial){
    if(!ws || ws.readyState !== 1) return;
    ws.send(JSON.stringify(partial));
  }

  function bindUI(){
    const kp = $("kp"), ki = $("ki"), kd = $("kd");
    const sp = $("sp"), base = $("base"), msa = $("msa"), tau = $("tau");

    kp.addEventListener("input", ()=>{
      $("kpv").textContent = Number(kp.value).toFixed(2);
      sendCfg({kp: Number(kp.value)});
    });
    ki.addEventListener("input", ()=>{
      $("kiv").textContent = Number(ki.value).toFixed(3);
      sendCfg({ki: Number(ki.value)});
    });
    kd.addEventListener("input", ()=>{
      $("kdv").textContent = Number(kd.value).toFixed(3);
      sendCfg({kd: Number(kd.value)});
    });

    sp.addEventListener("change", ()=> sendCfg({sp: Number(sp.value)}));
    base.addEventListener("change", ()=> sendCfg({base: Number(base.value)}));
    msa.addEventListener("change", ()=> sendCfg({msa: Number(msa.value)}));
    tau.addEventListener("change", ()=> sendCfg({tau: Number(tau.value)}));

    $("toggle").addEventListener("click", ()=> sendCfg({toggle: 1}));
    $("zero").addEventListener("click", ()=> sendCfg({zero: 1}));
    $("invert").addEventListener("click", ()=> sendCfg({invert: 1}));
  }

  function connect(){
    ws = new WebSocket(wsUrl);
    ws.onopen = ()=>{
      setConn(true);
      ws.send(JSON.stringify({get:1}));
    };
    ws.onclose = ()=>{
      setConn(false);
      setTimeout(connect, 800);
    };
    ws.onerror = ()=>{
      setConn(false);
      try{ ws.close(); }catch(e){}
    };
    ws.onmessage = (ev)=>{
      let msg;
      try{ msg = JSON.parse(ev.data); }catch(e){ return; }

      if(typeof msg.pitch === "number"){
        $("pitch").textContent = msg.pitch.toFixed(2);
        $("spv").textContent = msg.sp.toFixed(2);
        $("thv").textContent = String(msg.th);
        $("env").textContent = String(msg.en);

        push(pitchBuf, msg.pitch);
        push(spBuf, msg.sp);
        draw();
      }

      if(typeof msg.kp === "number"){
        $("kp").value = msg.kp; $("kpv").textContent = Number(msg.kp).toFixed(2);
        $("ki").value = msg.ki; $("kiv").textContent = Number(msg.ki).toFixed(3);
        $("kd").value = msg.kd; $("kdv").textContent = Number(msg.kd).toFixed(3);
        $("sp").value = msg.sp;
        $("base").value = msg.base;
        $("msa").value = msg.msa;
        $("tau").value = msg.tau;
        $("toggle").textContent = msg.en ? "Disable" : "Enable";
        $("invert").textContent = msg.inv ? "Invert: ON" : "Invert: OFF";
      }
    };
  }

  bindUI();
  connect();
})();
</script>
</body>
</html>
)rawliteral";

//  I2C helpers 
bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t readN = Wire.requestFrom((int)addr, (int)len);
  if (readN != len) return false;
  for (size_t i=0;i<len;i++) buf[i] = Wire.read();
  return true;
}

bool adxlDetectAndInit() {
  const uint8_t candidates[2] = {0x53, 0x1D};
  for (uint8_t i=0;i<2;i++) {
    uint8_t a = candidates[i];
    uint8_t id = 0;
    if (!i2cRead(a, ADXL345_DEVID, &id, 1)) continue;
    if (id != 0xE5) continue;

    adxlAddr = a;

    if (!i2cWrite8(adxlAddr, ADXL345_BW_RATE, 0x0A)) return false;        // 100 Hz
    if (!i2cWrite8(adxlAddr, ADXL345_DATA_FORMAT, 0x0B)) return false;    // FULL_RES, +-16g
    if (!i2cWrite8(adxlAddr, ADXL345_POWER_CTL, 0x08)) return false;      // Measure

    return true;
  }
  return false;
}

bool adxlReadRaw(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t b[6];
  if (!i2cRead(adxlAddr, ADXL345_DATAX0, b, 6)) return false;
  x = (int16_t)((b[1] << 8) | b[0]);
  y = (int16_t)((b[3] << 8) | b[2]);
  z = (int16_t)((b[5] << 8) | b[4]);
  return true;
}

float computePitchDegFromAccel(int16_t rx, int16_t ry, int16_t rz) {
  float ax = (float)rx;
  float ay = (float)ry;
  float az = (float)rz;

  float denom = sqrtf(ay*ay + az*az);
 float pitch = atan2f(+ax, denom) * 180.0f / PI;

  return pitch;
}

void calibrateZeroOffset() {
  const int N = 250;
  float sum = 0;
  int got = 0;

  for (int i=0;i<N;i++) {
    int16_t x,y,z;
    if (adxlReadRaw(x,y,z)) {
      sum += computePitchDegFromAccel(x,y,z);
      got++;
    }
    delay(4);
  }

  if (got > 0) {
    pitchOffsetDeg = sum / got;
    pitchFilteredDeg = 0;
    pid.reset(0);
  }
}

//  Web 
void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void sendConfigToClient(uint8_t num) {
  StaticJsonDocument<256> doc;
  doc["kp"] = pid.kp;
  doc["ki"] = pid.ki;
  doc["kd"] = pid.kd;
  doc["sp"] = setpointDeg;
  doc["base"] = baseThrottleUs;
  doc["msa"] = maxSafeAngleDeg;
  doc["tau"] = lpfTau;
  doc["en"] = controlEnabled;
  doc["inv"] = invertOutput;

  String out;
  serializeJson(doc, out);
  ws.sendTXT(num, out);
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    sendConfigToClient(num);
    return;
  }
  if (type != WStype_TEXT) return;

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, payload, length)) return;

  if (doc.containsKey("get")) sendConfigToClient(num);

  if (doc.containsKey("kp")) pid.kp = (float)doc["kp"];
  if (doc.containsKey("ki")) pid.ki = (float)doc["ki"];
  if (doc.containsKey("kd")) pid.kd = (float)doc["kd"];
  if (doc.containsKey("sp")) setpointDeg = (float)doc["sp"];

  if (doc.containsKey("base")) baseThrottleUs = constrain((int)doc["base"], 1000, 1800);
  if (doc.containsKey("msa"))  maxSafeAngleDeg = constrain((float)doc["msa"], 5.0f, 80.0f);
  if (doc.containsKey("tau"))  lpfTau = constrain((float)doc["tau"], 0.02f, 1.0f);

  if (doc.containsKey("toggle")) {
    controlEnabled = !controlEnabled;
    pid.reset(pitchFilteredDeg);
  }
  if (doc.containsKey("zero")) {
    calibrateZeroOffset();
  }
  if (doc.containsKey("invert")) {
    invertOutput = !invertOutput;
  }

  sendConfigToClient(num);
}

//  setup 
void setup() {
  Serial.begin(115200);
  delay(50);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  adxlOk = adxlDetectAndInit();

  esc.attach(PIN_ESC, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(ESC_SAFE_US);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();

  server.on("/", handleRoot);
  server.begin();

  ws.begin();
  ws.onEvent(onWsEvent);

  // Армінг ESC
  uint32_t t0 = millis();
  while (millis() - t0 < (uint32_t)ESC_ARM_TIME_MS) {
    esc.writeMicroseconds(ESC_SAFE_US);
    ws.loop();
    server.handleClient();
    delay(5);
  }

  if (adxlOk) calibrateZeroOffset();

  lastSensorUs = micros();
  lastEscUs = micros();
  lastTelemetryMs = millis();

  Serial.println();
  Serial.println("=== PID Balancer started ===");
  Serial.print("AP SSID: "); Serial.println(AP_SSID);
  Serial.print("AP IP  : "); Serial.println(ip);
  Serial.print("ADXL345: "); Serial.println(adxlOk ? "OK" : "NOT FOUND");
}

//  loop 
void loop() {
  server.handleClient();
  ws.loop();

  const uint32_t nowUs = micros();

  // 1) Сенсор + фільтр (100 Гц)
  if ((uint32_t)(nowUs - lastSensorUs) >= SENSOR_PERIOD_US) {
    float dt = (nowUs - lastSensorUs) / 1e6f;
    lastSensorUs = nowUs;

    if (adxlOk) {
      int16_t x,y,z;
      if (adxlReadRaw(x,y,z)) {
        float rawPitch = computePitchDegFromAccel(x,y,z) - pitchOffsetDeg;

        float tau = (lpfTau > 1e-3f) ? lpfTau : 1e-3f;
        float alpha = expf(-dt / tau);
        pitchFilteredDeg = alpha * pitchFilteredDeg + (1.0f - alpha) * rawPitch;

        pitchDeg = pitchFilteredDeg;
      }
    }
  }

  // 2) ESC (50 Гц)
  if ((uint32_t)(nowUs - lastEscUs) >= ESC_PERIOD_US) {
    float dt = (nowUs - lastEscUs) / 1e6f;
    lastEscUs = nowUs;

    int throttleUs = ESC_SAFE_US;

    if (!adxlOk) {
      throttleUs = ESC_SAFE_US;
      controlEnabled = false;
    } else {
      if (fabsf(pitchDeg) > maxSafeAngleDeg) {
        controlEnabled = false;
        throttleUs = ESC_SAFE_US;
        pid.reset(pitchDeg);
      } else if (!controlEnabled) {
        throttleUs = ESC_SAFE_US;
      } else {
        bool saturated = false;
        float u = pid.update(setpointDeg, pitchDeg, dt, saturated); // ΔPWM (мкс)
        if (invertOutput) u = -u;

        int cmd = baseThrottleUs + (int)lroundf(u);
        throttleUs = constrain(cmd, ESC_MIN_US, ESC_MAX_US);
      }
    }

    esc.writeMicroseconds(throttleUs);
  }

  // 3) Телеметрія (20 Гц)
  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastTelemetryMs) >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;

    StaticJsonDocument<256> doc;
    doc["pitch"] = pitchDeg;
    doc["sp"]    = setpointDeg;
    doc["th"]    = (int)esc.readMicroseconds();
    doc["en"]    = controlEnabled;

    doc["kp"] = pid.kp;
    doc["ki"] = pid.ki;
    doc["kd"] = pid.kd;
    doc["base"] = baseThrottleUs;
    doc["msa"]  = maxSafeAngleDeg;
    doc["tau"]  = lpfTau;
    doc["inv"]  = invertOutput;

    String out;
    serializeJson(doc, out);
    ws.broadcastTXT(out);
  }
}
