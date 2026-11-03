// ===== ESP A - PART 1 of 5: config, roads, state, logic =====
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ---- Wi-Fi: joins your phone hotspot only ----
const char* WIFI_SSID = "realme narzo N53";
const char* WIFI_PASS = "virajjjj";

// ---- Pins ----
#define ENC_CLK   32
#define ENC_DT    33
#define ENC_SW    25
#define RX2       16
#define TX2       17
#define OLED_ADDR 0x3C
#define BTN_PIN   27

// ---- Tuning ----
#define MAX_SPEED     150
#define ENC_STEP_KMH  5        // km/h per encoder click
#define ENC_COUNTS    4        // same value that worked in your test
#define TIME_SCALE    10.0f    // demo: 1 s of driving = 10 s of road travel
#define WARN_CM       60
#define BRAKE_CM      30
#define ALC_TEST_MS   4000
#define ACCEL_KMH_S   80.0f
#define DECEL_KMH_S   150.0f
#define REQUIRE_B     1        // 1 = stop if ESP B is silent. 0 only for UI testing

// ---- Roads (limit in km/h). Change limits here, e.g. COE after the local check ----
struct Road { const char* name; int limit; int n; double lat[3]; double lon[3]; };
Road roads[5] = {
  {"Phaltan-Baramati Highway", 40, 3, {18.022363, 18.046060, 18.071317}, {74.463257, 74.479466, 74.493764}},
  {"Baramati MIDC road",       30, 3, {18.166649, 18.177774, 18.195690}, {74.594080, 74.606568, 74.620769}},
  {"Phaltan Satara road",      60, 2, {17.984359, 17.981768, 0},         {74.403120, 74.390604, 0}},
  {"Pune-Pandharpur highway",  80, 2, {17.954849, 17.964370, 0},         {74.595027, 74.559701, 0}},
  {"COE Phaltan road",         30, 3, {17.957504, 17.957168, 17.978566}, {74.414828, 74.414723, 74.421812}}
};
float routeLenM[5];

Adafruit_SSD1306 display(128, 64, &Wire, -1);
WebServer server(80);

// ---- State ----
float curSpeed = 0, distM = 0;
int potKmh = 0;                 // requested speed from the encoder (name kept for the rest of the code)
int limitKmh = 0, route = -1, stCode = 0;
bool driving = false, arrived = false;
double curLat = 0, curLon = 0;

// from ESP B
int  distCm = 999, gasVal = 0, gasThr = 0;
bool bBrake = false, bAlc = false, bReady = false;
unsigned long lastB = 0;
String rxLine;

// alcohol test: 0 idle, 1 testing, 2 sober, 3 drunk
int alcState = 0, alcProg = 0, alcPeak = 0;
bool alcHit = false;
unsigned long alcStart = 0;

// start button
int btnCount = 0;
bool btnLast = HIGH;
unsigned long btnT = 0;

bool linkOk() { return millis() - lastB < 1000; }

// ---- Distance maths ----
double hav(double la1, double lo1, double la2, double lo2) {
  const double R = 6371000.0, d2r = M_PI / 180.0;
  double dLa = (la2 - la1) * d2r, dLo = (lo2 - lo1) * d2r;
  double a = sin(dLa / 2) * sin(dLa / 2) +
             cos(la1 * d2r) * cos(la2 * d2r) * sin(dLo / 2) * sin(dLo / 2);
  return 2 * R * asin(sqrt(a));
}

void computeRoutes() {
  for (int i = 0; i < 5; i++) {
    double t = 0;
    for (int j = 0; j < roads[i].n - 1; j++)
      t += hav(roads[i].lat[j], roads[i].lon[j], roads[i].lat[j + 1], roads[i].lon[j + 1]);
    routeLenM[i] = t;
  }
}

// simulated GPS: position after dm metres along road r
void positionAt(int r, double dm, double &lat, double &lon) {
  Road &R = roads[r];
  for (int i = 0; i < R.n - 1; i++) {
    double seg = hav(R.lat[i], R.lon[i], R.lat[i + 1], R.lon[i + 1]);
    if (dm <= seg || i == R.n - 2) {
      double f = seg > 0 ? dm / seg : 0;
      if (f < 0) f = 0;
      if (f > 1) f = 1;
      lat = R.lat[i] + (R.lat[i + 1] - R.lat[i]) * f;
      lon = R.lon[i] + (R.lon[i + 1] - R.lon[i]) * f;
      return;
    }
    dm -= seg;
  }
}

// ---- Rotary encoder: turn = requested speed, press = request 0 ----
volatile int encCount = 0;
volatile uint8_t encState = 0;
const int8_t ENC_TABLE[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
bool swLast = HIGH;
unsigned long swT = 0;

void IRAM_ATTR encISR() {
  encState = ((encState << 2) | (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT)) & 0x0F;
  encCount += ENC_TABLE[encState];
}

void encoderBegin() {
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  encState = (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), encISR, CHANGE);
}

void updateEncoder() {
  noInterrupts();
  int clicks = encCount / ENC_COUNTS;
  encCount -= clicks * ENC_COUNTS;
  interrupts();
  if (clicks) {
    potKmh += clicks * ENC_STEP_KMH;
    if (potKmh < 0) potKmh = 0;
    if (potKmh > MAX_SPEED) potKmh = MAX_SPEED;
  }
  bool r = digitalRead(ENC_SW);
  if (r != swLast && millis() - swT > 40) {
    swT = millis();
    swLast = r;
    if (r == LOW) potKmh = 0;
  }
}

// ---- Start button (GPIO27 to GND) ----
void updateButton() {
  bool r = digitalRead(BTN_PIN);
  if (r != btnLast && millis() - btnT > 40) {   // 40 ms debounce
    btnT = millis();
    btnLast = r;
    if (r == LOW) btnCount++;
  }
}

// ---- UART link to ESP B ----
void parseB(const String &l) {
  int d, g, t, b, a, r;
  if (sscanf(l.c_str(), "D,%d,%d,%d,%d,%d,%d", &d, &g, &t, &b, &a, &r) == 6) {
    distCm = d; gasVal = g; gasThr = t;
    bBrake = b; bAlc = a; bReady = r;
    lastB = millis();
  }
}

void readB() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') { parseB(rxLine); rxLine = ""; }
    else if (c != '\r' && rxLine.length() < 60) rxLine += c;
  }
}

void sendToB(int pwm) { Serial2.printf("S,%d\n", pwm); }

// ---- Alcohol test (progress runs only once the MQ2 is warm) ----
void alcBegin() { alcState = 1; alcStart = millis(); alcHit = false; alcPeak = 0; alcProg = 0; }

void updateAlcohol() {
  if (alcState != 1) return;
  if (!bReady || !linkOk()) { alcStart = millis(); alcProg = 0; return; }
  if (bAlc) alcHit = true;
  if (gasVal > alcPeak) alcPeak = gasVal;
  unsigned long e = millis() - alcStart;
  int p = (int)(e * 100UL / ALC_TEST_MS);
  alcProg = p > 100 ? 100 : p;
  if (e >= ALC_TEST_MS) alcState = alcHit ? 3 : 2;
}

// ---- Trip control (request resets to 0 so the car never jumps) ----
void selectRoute(int r) { if (r >= 0 && r < 5 && !driving) { route = r; distM = 0; arrived = false; } }
void tripStart() { if (route >= 0 && alcState == 2) { driving = true; arrived = false; distM = 0; potKmh = 0; } }
void tripEnd()   { driving = false; arrived = false; route = -1; distM = 0; curSpeed = 0; alcState = 0; potKmh = 0; }

// ---- THE LIMITER + safety, runs every 50 ms ----
void updateDrive(float dt) {
  limitKmh = (route >= 0) ? roads[route].limit : 0;
  bool can = driving && !arrived && alcState == 2 && route >= 0 && (linkOk() || !REQUIRE_B);

  float target = can ? (float)(potKmh < limitKmh ? potKmh : limitKmh) : 0;   // never above limit
  int s = 0;
  if (can) {
    if (distCm <= BRAKE_CM || bBrake) { target = 0; s = 3; }
    else if (distCm <= WARN_CM)       { target *= 0.5f; s = 2; }
  }

  if (!can || s == 3) curSpeed = 0;
  else if (curSpeed < target) { curSpeed += ACCEL_KMH_S * dt; if (curSpeed > target) curSpeed = target; }
  else                        { curSpeed -= DECEL_KMH_S * dt; if (curSpeed < target) curSpeed = target; }
  if (can && curSpeed > limitKmh) curSpeed = limitKmh;   // final hard clamp

  if (can) {
    distM += (curSpeed / 3.6f) * dt * TIME_SCALE;
    if (distM >= routeLenM[route]) { distM = routeLenM[route]; arrived = true; curSpeed = 0; }
  }
  if (route >= 0) positionAt(route, distM, curLat, curLon);

  // status: 0 driving, 1 limiter active, 2 collision warning, 3 emergency brake, 4 arrived
  if (arrived) stCode = 4;
  else if (s) stCode = s;
  else if (can && potKmh > limitKmh) stCode = 1;
  else stCode = 0;

  sendToB((int)(curSpeed * 255.0f / MAX_SPEED + 0.5f));
}

// ---- OLED ----
const char* statusText() {
  if (alcState == 1) return "ALCOHOL TEST";
  if (alcState == 3) return "ENGINE LOCKED";
  if (route < 0)     return "SELECT ROUTE";
  if (!driving)      return "READY";
  switch (stCode) {
    case 1: return "LIMITER ACTIVE";
    case 2: return "COLLISION WARN";
    case 3: return "EMERGENCY BRAKE";
    case 4: return "ARRIVED";
  }
  return "DRIVING";
}

void drawOled() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  if (route >= 0) display.printf("LIM %d   REQ %d", limitKmh, potKmh);
  else            display.printf("REQ %d", potKmh);
  display.setTextSize(4);
  display.setCursor(0, 14);
  display.printf("%d", (int)(curSpeed + 0.5f));
  display.setTextSize(1);
  display.setCursor(98, 38);
  display.print("km/h");
  display.setCursor(0, 56);
  display.print(statusText());
  display.display();
}

// ---- JSON for the web page ----
String statusJson() {
  char b[360];
  snprintf(b, sizeof(b),
    "{\"sp\":%d,\"rq\":%d,\"lim\":%d,\"st\":%d,\"d\":%d,\"g\":%d,\"th\":%d,\"gr\":%d,"
    "\"alc\":%d,\"ap\":%d,\"pk\":%d,\"pg\":%d,\"lat\":%.5f,\"lon\":%.5f,\"r\":%d,\"lk\":%d,\"bt\":%d}",
    (int)(curSpeed + 0.5f), potKmh, limitKmh, stCode, distCm > 999 ? 999 : distCm, gasVal, gasThr,
    bReady ? 1 : 0, alcState, alcProg, alcPeak,
    route >= 0 ? (int)(distM * 100.0f / routeLenM[route]) : 0,
    curLat, curLon, route, linkOk() ? 1 : 0, btnCount);
  return String(b);
}

String routesJson() {
  String s = "[";
  for (int i = 0; i < 5; i++) {
    if (i) s += ",";
    s += "{\"n\":\"" + String(roads[i].name) + "\",\"l\":" + roads[i].limit +
         ",\"km\":" + String(routeLenM[i] / 1000.0, 1) + "}";
  }
  return s + "]";
}
// ===== END OF PART 1 =====
// ===== END OF PART 1 =====

// ===== ESP A - PART 2 of 5: HTML head + CSS =====
const char PAGE_A[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>SmartDrive in-car screen</title>
<style>
:root{box-sizing:border-box;padding-top:env(safe-area-inset-top,0px);padding-bottom:env(safe-area-inset-bottom,0px);color-scheme:dark;
--bg:#070b12;--panel:#101826;--ink:#f1f5fa;--mute:#8fa1b8;--line:#223047;--ok:#35d07f;--bad:#ff5c5c;--warn:#ffb020;--acc:#4aa3ff}
html{height:100%;scroll-padding-top:env(safe-area-inset-top,0px)}
*{box-sizing:border-box}
body{margin:0;min-height:100%;background:var(--bg);color:var(--ink);font-family:system-ui,-apple-system,"Segoe UI",sans-serif;display:grid;place-items:center}
.stage{container-type:inline-size;position:relative;width:min(100%,177.78dvh);aspect-ratio:16/9;background:var(--bg);overflow:hidden}
.app{position:absolute;inset:0;font-size:2cqw;line-height:1.25}
.scr{position:absolute;inset:0;display:none;flex-direction:column;align-items:center;justify-content:center;text-align:center;padding:3%;gap:.7em}
.scr.on{display:flex;animation:in .5s ease}
@keyframes in{from{opacity:0;transform:scale(.97)}to{opacity:1;transform:none}}
h1,h2{margin:0;font-family:"Bricolage Grotesque",system-ui,sans-serif;letter-spacing:-.02em}
h1{font-size:2.8em}h2{font-size:2em}
.mute{color:var(--mute);font-size:.8em}
button{font:inherit;cursor:pointer;color:var(--ink)}
button:focus-visible{outline:.15em solid var(--warn);outline-offset:.15em}
.btn{padding:.7em 1.6em;border-radius:.7em;border:.1em solid var(--acc);background:var(--acc);color:#04101f;font-weight:700;font-size:1em}
.btn.g{background:transparent;color:var(--ink);border-color:var(--line)}
.btn:disabled{opacity:.35;cursor:default}
.brand{position:absolute;left:3%;top:3%;font:800 1em "Bricolage Grotesque",system-ui;color:var(--mute)}
.push{width:12em;height:12em;border-radius:50%;border:.25em solid var(--acc);background:radial-gradient(circle at 50% 30%,#1d2f4b,#0b1422);font:700 1.5em/1.2 "Bricolage Grotesque",system-ui;animation:ring 2s infinite}
@keyframes ring{0%{box-shadow:0 0 0 0 rgba(74,163,255,.55)}100%{box-shadow:0 0 0 2.6em rgba(74,163,255,0)}}
.zx{display:flex;font:800 7em/1 "Bricolage Grotesque",system-ui;animation:glow 1.2s 1.3s ease-in-out infinite alternate}
.zx span{display:inline-block;opacity:0;animation:slam .6s var(--d) cubic-bezier(.2,1.6,.4,1) forwards}
@keyframes slam{from{transform:translateY(-1.5em) scale(2.4) rotate(-14deg);opacity:0}to{transform:none;opacity:1}}
@keyframes glow{to{text-shadow:0 0 .35em var(--acc),0 0 .8em rgba(74,163,255,.5)}}
.arms{display:flex;align-items:center;gap:1.2em}
.arm{display:inline-block;font-size:3.6em;animation:flex .55s ease-in-out infinite alternate}
.arm.r{transform:scaleX(-1)}
@keyframes flex{from{transform:scale(1) rotate(-6deg)}to{transform:scale(1.3) rotate(6deg)}}
.sub{opacity:0;animation:in .6s 2.2s forwards;font-size:1.3em;color:var(--mute)}
.breath{width:7em;height:7em;border-radius:50%;border:.2em solid var(--acc);position:relative;animation:pulse 1.6s ease-in-out infinite}
.breath:before,.breath:after{content:"";position:absolute;inset:-.9em;border-radius:50%;border:.1em solid var(--acc);opacity:.4}
.breath:after{inset:-1.8em;opacity:.2}
@keyframes pulse{50%{transform:scale(1.12)}}
.pb{width:24em;height:.5em;background:var(--line);border-radius:1em;overflow:hidden}.pb i{display:block;height:100%;width:0;background:var(--acc)}
.ev{display:flex;gap:1em}
.card{background:var(--panel);border:1px solid var(--line);border-radius:.7em;padding:.6em .9em;width:15em;text-align:left;font-size:.8em}
.card b{display:block}.card .big{font:800 1.8em "Bricolage Grotesque",system-ui;color:var(--bad)}
.rts{display:flex;flex-wrap:wrap;justify-content:center;gap:1em;max-width:94%}
.rt{background:var(--panel);border:.1em solid var(--line);border-radius:.8em;padding:.8em 1em;width:13em;text-align:left;font-size:.85em}
.rt.on{border-color:var(--acc);box-shadow:0 0 0 .1em var(--acc)}
.rt b{display:block;font-size:1.15em;margin-bottom:.2em}.rt span{color:var(--mute)}
.drv{display:grid;grid-template-columns:1.4fr 1fr;gap:2em;align-items:center;width:100%;padding:0 4%}
.spd{font:800 10em/1 "Bricolage Grotesque",system-ui;font-variant-numeric:tabular-nums;letter-spacing:-.04em}
.sign{width:6.5em;height:6.5em;border-radius:50%;background:#fff;border:.45em solid #d62828;color:#111;font:800 2.8em/5.1 "Bricolage Grotesque",system-ui;text-align:center;margin:0 auto}
.chip{display:inline-block;padding:.3em 1em;border-radius:2em;font-weight:700;background:var(--ok);color:#04101f}
.bar{width:100%;height:.5em;background:var(--line);border-radius:1em;overflow:hidden;margin-top:.6em}.bar i{display:block;height:100%;width:0;background:var(--acc)}
.note{position:absolute;bottom:2.2%;left:0;right:0;text-align:center;color:var(--mute);font-size:.65em}
.ok{color:var(--ok)}.bad{color:var(--bad)}
@media(prefers-reduced-motion:reduce){*{animation-duration:.01s!important;animation-iteration-count:1!important}}
</style></head><body>
)rawliteral";
// ===== END OF PART 2 =====

// ===== ESP A - PART 3 of 5: HTML screens =====
const char PAGE_B[] PROGMEM = R"rawliteral(<div class="stage"><div class="app">

<section class="scr on" id="s0"><div class="brand">SmartDrive</div>
<button class="push" id="go">Push to start</button>
<p class="mute">Press the START button on the dashboard, or tap here. The alcohol check runs before the engine can start</p>
<p class="note" id="lkn">Prototype display for the physical car on our desk</p></section>

<section class="scr" id="s1">
<div class="arms"><span class="arm">💪</span><div class="zx"><span style="--d:.1s">T</span><span style="--d:.2s">E</span><span style="--d:.3s">A</span><span style="--d:.4s">M</span><span style="--d:.5s">&nbsp;</span><span style="--d:.7s">Z</span><span style="--d:.85s">X</span></div><span class="arm r">💪</span></div>
<p class="sub">presents SmartDrive</p></section>

<section class="scr" id="s2"><h1>Alcohol detection</h1>
<div class="breath" style="margin:1.4em 0"></div>
<p style="font-size:1.2em;margin:0">Checking that the driver is fresh and sober. Please blow into the sensor.</p>
<div class="pb"><i id="pg"></i></div>
<p class="mute" id="aw" style="margin:0"></p></section>

<section class="scr" id="s3"></section>

<section class="scr" id="s4"><h2>Select your route</h2>
<div class="rts" id="rts"></div>
<button class="btn" id="st" disabled>Start driving</button></section>

<section class="scr" id="s5">
<div class="brand" id="rn"></div>
<div class="drv"><div><div class="spd" id="sp">0</div><div class="mute" style="font-size:1.1em">km/h</div><div class="mute" id="rq" style="font-size:1em;margin-top:.3em">Pedal request: 0 km/h</div></div>
<div><div class="sign" id="lm">40</div><p style="margin:.6em 0 0"><span class="chip" id="chip">Driving</span></p><p class="mute" id="gap" style="font-size:1em;margin:.4em 0 0">Road clear</p></div></div>
<div style="width:86%"><div class="bar"><i id="rp"></i></div><p class="mute" id="pos" style="margin:.5em 0 0">-</p></div>
<button class="btn g" id="end">End trip</button></section>

</div></div>
)rawliteral";
// ===== END OF PART 3 =====

// ===== ESP A - PART 4 of 5: page script =====
const char PAGE_C[] PROGMEM = R"rawliteral(<script>
const $=s=>document.querySelector(s),A=s=>[...document.querySelectorAll(s)];
let R=[],route=null,tm=[],cur='s0',S={},lastBt=null,lastSt=0,armed=false;
const ping=p=>fetch(p,{cache:'no-store'}).catch(()=>{});
function say(t){try{speechSynthesis.cancel();speechSynthesis.speak(new SpeechSynthesisUtterance(t))}catch(e){}}
function loadRoutes(){fetch('/routes').then(r=>r.json()).then(j=>{R=j}).catch(()=>setTimeout(loadRoutes,1000))}

function go(id){tm.forEach(clearTimeout);tm=[];cur=id;A('.scr').forEach(e=>e.classList.toggle('on',e.id===id));
 if(id==='s1')tm.push(setTimeout(()=>go('s2'),4400));
 if(id==='s2'){armed=false;$('#pg').style.width='0';ping('/alc')}
 if(id==='s4')routes();
 if(id==='s5')$('#rn').textContent=R[route].n}

function begin(){ping('/end');lastSt=0;go('s1')}
$('#go').onclick=begin;

function alcTick(){
 if(S.alc===1)armed=true;
 $('#pg').style.width=(S.ap||0)+'%';
 $('#aw').textContent=!S.lk?'Waiting for the car controller...':(S.gr?'':'Sensor warming up, please wait...');
 if(armed&&(S.alc===2||S.alc===3)){armed=false;result()}}

function result(){
 const bad=S.alc===3,r=S.pk,t=new Date().toLocaleTimeString();
 $('#s3').innerHTML=bad?
 `<h1 class="bad">Alcohol detected: YES</h1><h2>Engine cannot start</h2>
 <div class="ev"><div class="card"><b>Photo 1: driver</b><svg viewBox="0 0 100 60" width="100%" style="background:#0b1422;border-radius:.4em"><circle cx="50" cy="24" r="12" fill="#8fa1b8"/><path d="M22 60c2-16 14-22 28-22s26 6 28 22z" fill="#8fa1b8"/></svg><span class="mute">Captured ${t}</span></div>
 <div class="card"><b>Photo 2: alcohol reading</b><span class="big">${r}</span><span class="mute">Trigger ${S.th} · ${t}</span></div></div>
 <p style="margin:0">Driving time logged at <b>${t}</b>. Emergency contact alerted.</p><button class="btn g" id="rt">Test again</button>`:
 `<h1 class="ok">Alcohol detected: NO</h1><h2>Driver is fresh. Engine can start.</h2><p class="mute" style="margin:0">Reading ${r} · trigger ${S.th} · ${t}</p><button class="btn" id="rt">Choose route</button>`;
 say(bad?'Alcohol detected. The engine cannot start.':'No alcohol detected. You can start the engine.');
 go('s3');$('#rt').onclick=()=>go(bad?'s2':'s4')}

function routes(){route=null;$('#st').disabled=true;
 $('#rts').innerHTML=R.map((r,i)=>`<button class="rt" data-i="${i}"><b>${r.n}</b>${r.km} km<br><span>Speed limit ${r.l} km/h</span></button>`).join('');
 A('.rt').forEach(b=>b.onclick=()=>{route=+b.dataset.i;A('.rt').forEach(x=>x.classList.toggle('on',x===b));$('#st').disabled=false;ping('/sel?r='+route)})}
$('#st').onclick=()=>{ping('/go');go('s5')};

function drive(){
 const nm=['Driving','Limiter active','Collision warning','Emergency brake','Arrived'],
       co=['var(--ok)','var(--warn)','var(--warn)','var(--bad)','var(--acc)'],
       vo=['','','Collision warning','Emergency brake','You have arrived'];
 $('#sp').textContent=S.sp;$('#lm').textContent=S.lim;$('#rp').style.width=S.pg+'%';
 $('#rq').textContent='Pedal request: '+S.rq+' km/h';
 const c=$('#chip');c.textContent=nm[S.st];c.style.background=co[S.st];
 $('#gap').textContent=S.st===4?'Trip complete':(S.st===3?'Obstacle stopped':(S.d<100?S.d+' cm ahead':'Road clear'));
 $('#pos').textContent=S.lat.toFixed(5)+', '+S.lon.toFixed(5);
 if(S.st!==lastSt){if(vo[S.st])say(vo[S.st]);lastSt=S.st}}
$('#end').onclick=()=>{ping('/end');go('s0')};

async function poll(){
 try{
  const r=await fetch('/status',{cache:'no-store'});S=await r.json();
  if(lastBt===null)lastBt=S.bt;
  if(S.bt!==lastBt){lastBt=S.bt;if(cur==='s0')begin()}
  $('#lkn').textContent=S.lk?'Prototype display for the physical car on our desk':'Car controller (ESP B) not connected';
  if(cur==='s2')alcTick();
  if(cur==='s5')drive();
 }catch(e){}
 setTimeout(poll,150)}
loadRoutes();poll();
</script></body></html>)rawliteral";
// ===== END OF PART 4 =====

// ===== ESP A - PART 5 of 5: Wi-Fi, web handlers, setup, loop =====
#include <ESPmDNS.h>

unsigned long bootUntil = 0, tLogic = 0, tOled = 0, tWifi = 0;

void sendPage() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent_P(PAGE_A);
  server.sendContent_P(PAGE_B);
  server.sendContent_P(PAGE_C);
  server.sendContent("");
}

void hStatus() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", statusJson());
}
void hRoutes() { server.send(200, "application/json", routesJson()); }
void hAlc()    { alcBegin();  server.send(200, "text/plain", "ok"); }
void hGo()     { tripStart(); server.send(200, "text/plain", "ok"); }
void hEnd()    { tripEnd();   server.send(200, "text/plain", "ok"); }
void hSel() {
  if (server.hasArg("r")) selectRoute(server.arg("r").toInt());
  server.send(200, "text/plain", "ok");
}

void oledMsg(const char* l1, const char* l2, int size2) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(l1);
  display.setTextSize(size2);
  display.setCursor(0, 24);
  display.print(l2);
  display.display();
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t = millis();
  int dots = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    String d = "";
    for (int i = 0; i < (dots % 4); i++) d += ".";
    dots++;
    oledMsg("Joining hotspot:", (String(WIFI_SSID) + d).c_str(), 1);
    if (millis() - t > 15000) {            // retry every 15 s
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      t = millis();
    }
  }
  Serial.println();
  Serial.print("Connected. Open this in your laptop browser: http://");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);

  pinMode(BTN_PIN, INPUT_PULLUP);
  encoderBegin();

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found - check wiring");
  }
  computeRoutes();

  connectWifi();
  String ip = WiFi.localIP().toString();
  oledMsg("Open in browser:", ip.c_str(), 2);
  bootUntil = millis() + 8000;             // show the IP for 8 seconds

  if (MDNS.begin("smartdrive")) Serial.println("Also: http://smartdrive.local");

  server.on("/", sendPage);
  server.on("/status", hStatus);
  server.on("/routes", hRoutes);
  server.on("/alc", hAlc);
  server.on("/sel", hSel);
  server.on("/go", hGo);
  server.on("/end", hEnd);
  server.begin();

  tLogic = millis();
}

void loop() {
  server.handleClient();
  readB();
  updateButton();
  updateEncoder();

  unsigned long now = millis();

  if (now - tLogic >= 50) {
    float dt = (now - tLogic) / 1000.0f;
    tLogic = now;
    updateAlcohol();
    updateDrive(dt);
  }

  if (now - tOled >= 100 && now > bootUntil) {
    tOled = now;
    drawOled();
  }

  if (WiFi.status() != WL_CONNECTED && now - tWifi > 5000) {   // hotspot dropped
    tWifi = now;
    WiFi.reconnect();
  }
}
// ===== END OF PART 5 =====