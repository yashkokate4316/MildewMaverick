/*
 ╔══════════════════════════════════════════════════════════════════╗
 ║   MILDEWMAVERICK — MAIN ESP32 CODE  v5.0                        ║
 ║   Team SpectraFarm | Pune Agri Hackathon 2025                   ║
 ╠══════════════════════════════════════════════════════════════════╣
 ║   NEW IN v5.0:                                                   ║
 ║   • Sends row number to ESP32-CAM via 4-bit GPIO signal         ║
 ║   • Treatment Efficacy Score sent to cloud per session          ║
 ║   • Row-level severity tracked and sent to Google Sheets        ║
 ║   • Risk Score per session (HIGH×3 + MEDIUM×2 + LOW×1)         ║
 ║   • Efficacy % = ((last risk - this risk) / last risk) × 100   ║
 ╠══════════════════════════════════════════════════════════════════╣
 ║   WIRING ADDED IN v5.0:                                         ║
 ║   Main ESP32 GPIO 16 → ESP32-CAM GPIO 14 (Row Bit 0)           ║
 ║   Main ESP32 GPIO 17 → ESP32-CAM GPIO 15 (Row Bit 1)           ║
 ║   Main ESP32 GPIO 18 → ESP32-CAM GPIO 2  (Row Bit 2)           ║
 ║   Main ESP32 GPIO 19 → ESP32-CAM GPIO 16 (Row Bit 3)           ║
 ║   (These 4 wires tell the CAM which row the rover is on)        ║
 ╚══════════════════════════════════════════════════════════════════╝
*/

#define BLYNK_TEMPLATE_ID    "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME  "MildewMaverick"
#define BLYNK_AUTH_TOKEN     "YOUR_AUTH_TOKEN"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <HTTPClient.h>

const char* WIFI_SSID       = "YOUR_PHONE_HOTSPOT_NAME";
const char* WIFI_PASS       = "YOUR_HOTSPOT_PASSWORD";
const char* SESSION_LOG_URL = "YOUR_GOOGLE_APPS_SCRIPT_URL";

// ─────────────────────────────────────────────────────────────────
//  YOUR LED TYPE — change depending on which LED you have
//  Set LED_TYPE to 1 if you have a plain 12V LED strip (purchased)
//  Set LED_TYPE to 2 if you have a WS2812B addressable strip
// ─────────────────────────────────────────────────────────────────
#define LED_TYPE 1   // 1 = plain 12V strip via MOSFET, 2 = WS2812B

#if LED_TYPE == 2
  #include <Adafruit_NeoPixel.h>
  Adafruit_NeoPixel strip(12, 15, NEO_GRB + NEO_KHZ800);
#endif

// ─────────────────────────────────────────────────────────────────
//  PINS
// ─────────────────────────────────────────────────────────────────
// Motor Driver #1 — LEFT
#define PWMA  25
#define AIN1  26
#define AIN2  27
// Motor Driver #2 — RIGHT
#define PWMB  32
#define BIN1  33
#define BIN2  13
#define STBY   2

// Sensors
#define TRIG  14
#define ECHO  12

// LED (plain 12V strip — GPIO 15 via MOSFET)
#define LED_PIN     15
#define LED_CH       0
#define LED_FREQ  5000
#define LED_RES      8

// CAM severity signal (INPUT from CAM)
#define CAM_B0   4
#define CAM_B1   5

// Row number OUTPUT to CAM (4-bit — tells CAM which row rover is on)
#define ROW_OUT_B0  16
#define ROW_OUT_B1  17
#define ROW_OUT_B2  18
#define ROW_OUT_B3  19

// ─────────────────────────────────────────────────────────────────
//  TUNING
// ─────────────────────────────────────────────────────────────────
#define ROW_END_CM      18
#define OBS_STOP_CM     22
#define OBS_WARN_CM     40
#define HUMAN_LO       33.5f
#define HUMAN_HI       39.5f
#define AUTO_SPEED      165
#define TURN_SPEED      155
#define REVERSE_MS      600
#define TURN_90_MS      480
#define SIDE_STEP_MS    700

// ─────────────────────────────────────────────────────────────────
//  OBJECTS
// ─────────────────────────────────────────────────────────────────
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
BlynkTimer timer;

// ─────────────────────────────────────────────────────────────────
//  APP CONTROL STATE
// ─────────────────────────────────────────────────────────────────
int   joyY=0, joyX=0;
bool  uvOn=true;
int   appSpeed=200;
bool  autoMode=false;
bool  sessionOn=false;
bool  turning=false;

// ─────────────────────────────────────────────────────────────────
//  SENSOR STATE
// ─────────────────────────────────────────────────────────────────
float tempC=0.0f, distCm=400.0f;
bool  humanFound=false, obsClose=false, obsWarn=false;

// ─────────────────────────────────────────────────────────────────
//  MILDEW STATE
// ─────────────────────────────────────────────────────────────────
int  mildewLvl=0;
int  uvBright=55;

// ─────────────────────────────────────────────────────────────────
//  SESSION DATA
// ─────────────────────────────────────────────────────────────────
int   sessNum=1;
int   rowsDone=0;
int   totalDets=0;
int   highHits=0, medHits=0, lowHits=0;
int   maxSev=0;
int   lastSessionRiskScore=0;  // stored to calculate efficacy
float maxTemp=0.0f, minDist=999.0f;
unsigned long startMs=0;

// ─────────────────────────────────────────────────────────────────
//  ROW-LEVEL SEVERITY TRACKING
//  Stores max severity seen in each row during current session
// ─────────────────────────────────────────────────────────────────
int rowSeverity[16];  // index = row number, value = max severity seen

// LED animation
int  pulseV=0; bool pulseUp=true;
unsigned long flashT=0; bool flashOn=false;

// ═════════════════════════════════════════════════════════════════
//  BLYNK CALLBACKS
// ═════════════════════════════════════════════════════════════════
BLYNK_WRITE(V0)  { joyY=param.asInt(); }
BLYNK_WRITE(V1)  { joyX=param.asInt(); }
BLYNK_WRITE(V2)  { uvOn=param.asInt(); if(!uvOn) setLED(0); }
BLYNK_WRITE(V10) { appSpeed=constrain(param.asInt(),50,255); }

BLYNK_WRITE(V11) {
  autoMode=param.asInt(); turning=false;
  if(autoMode){ if(!sessionOn) startSession(); }
  else motorsStop();
}

BLYNK_WRITE(V14) {
  if(param.asInt()&&sessionOn) endSession();
}

BLYNK_CONNECTED() { Blynk.syncAll(); }
BLYNK_DISCONNECTED() { motorsStop(); }

// ═════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════
void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("=== MildewMaverick Main v5.0 ===");

  initMotors();

  // Sensor pins
  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);
  pinMode(CAM_B0,INPUT); pinMode(CAM_B1,INPUT);

  // Row output pins to CAM
  pinMode(ROW_OUT_B0,OUTPUT); pinMode(ROW_OUT_B1,OUTPUT);
  pinMode(ROW_OUT_B2,OUTPUT); pinMode(ROW_OUT_B3,OUTPUT);
  sendRowToCAM(0);  // start at row 0

  // Clear row severity array
  for(int i=0;i<16;i++) rowSeverity[i]=0;

  // LED init
  initLED();

  // MLX90614
  Wire.begin(21,22);
  if(!mlx.begin()) Serial.println("MLX90614 not found — check 3V3 wiring");
  else Serial.println("MLX90614 OK");

  // Blynk
  Serial.println("Connecting to WiFi...");
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
  Serial.println("Blynk connected");

  // Timer tasks
  timer.setInterval(200L,  readSensors);
  timer.setInterval(50L,   runMotors);
  timer.setInterval(80L,   updateLED);
  timer.setInterval(500L,  updateBlynk);
  timer.setInterval(1000L, tickTimer);

  // Boot LED sequence
  for(int i=0;i<3;i++){
    setLED(255); delay(200); setLED(0); delay(150);
  }
  Serial.println("Ready\n");
}

// ═════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════
void loop(){ Blynk.run(); timer.run(); }

// ═════════════════════════════════════════════════════════════════
//  TASK 1: READ SENSORS (200ms)
// ═════════════════════════════════════════════════════════════════
void readSensors(){
  // MLX90614
  float obj=mlx.readObjectTempC();
  if(obj>-40&&obj<120){
    tempC=obj;
    humanFound=(obj>=HUMAN_LO&&obj<=HUMAN_HI);
    if(obj>maxTemp) maxTemp=obj;
  }

  // HC-SR04
  float d=getDist();
  if(d>1&&d<450) distCm=d;
  if(sessionOn&&distCm<minDist) minDist=distCm;
  obsClose=(distCm<OBS_STOP_CM);
  obsWarn =(distCm<OBS_WARN_CM);

  // CAM severity signal
  int b0=digitalRead(CAM_B0), b1=digitalRead(CAM_B1);
  int newLvl=(b1<<1)|b0;

  // Track row-level severity
  if(newLvl>0&&newLvl>mildewLvl&&sessionOn){
    totalDets++;
    if(newLvl>maxSev) maxSev=newLvl;
    if(newLvl==3) highHits++;
    else if(newLvl==2) medHits++;
    else lowHits++;
    // Update row severity map
    if(rowsDone<16 && newLvl>rowSeverity[rowsDone])
      rowSeverity[rowsDone]=newLvl;
  }
  mildewLvl=newLvl;

  // Map severity to brightness
  const int bMap[]={55,110,190,255};
  uvBright=bMap[constrain(mildewLvl,0,3)];
}

// ═════════════════════════════════════════════════════════════════
//  TASK 2: MOTOR CONTROL (50ms)
// ═════════════════════════════════════════════════════════════════
void runMotors(){
  if(humanFound){ motorsStop(); return; }
  if(autoMode&&!turning) runAuto();
  else if(!autoMode) runManual();
}

void runManual(){
  if(obsClose&&joyY>0){ motorsStop(); return; }
  float sc=appSpeed/255.0f;
  setLeft(constrain((int)((joyY+joyX)*sc),-255,255));
  setRight(constrain((int)((joyY-joyX)*sc),-255,255));
}

void runAuto(){
  if(distCm<ROW_END_CM){ doUTurn(); return; }
  if(obsClose){ motorsStop(); return; }
  setLeft(AUTO_SPEED); setRight(AUTO_SPEED);
}

void doUTurn(){
  turning=true; rowsDone++;
  sendRowToCAM(rowsDone);  // tell CAM about new row
  Blynk.virtualWrite(V12,rowsDone);
  Blynk.virtualWrite(V15,"Row "+String(rowsDone)+" done. Turning...");
  Serial.printf("Row %d done — U-turn\n",rowsDone);

  setLeft(-TURN_SPEED); setRight(-TURN_SPEED); delay(REVERSE_MS);
  motorsStop(); delay(250);
  setLeft(TURN_SPEED);  setRight(-TURN_SPEED); delay(TURN_90_MS);
  motorsStop(); delay(250);
  setLeft(AUTO_SPEED);  setRight(AUTO_SPEED);  delay(SIDE_STEP_MS);
  motorsStop(); delay(250);
  setLeft(TURN_SPEED);  setRight(-TURN_SPEED); delay(TURN_90_MS);
  motorsStop(); delay(250);

  Blynk.virtualWrite(V15,"Now on row "+String(rowsDone+1));
  turning=false;
}

// ─────────────────────────────────────────────────────────────────
//  Send 4-bit row number to ESP32-CAM via GPIO
// ─────────────────────────────────────────────────────────────────
void sendRowToCAM(int row){
  row=constrain(row,0,15);
  digitalWrite(ROW_OUT_B0,(row&0x01)?HIGH:LOW);
  digitalWrite(ROW_OUT_B1,(row&0x02)?HIGH:LOW);
  digitalWrite(ROW_OUT_B2,(row&0x04)?HIGH:LOW);
  digitalWrite(ROW_OUT_B3,(row&0x08)?HIGH:LOW);
}

// ═════════════════════════════════════════════════════════════════
//  TASK 3: LED CONTROL (80ms)
// ═════════════════════════════════════════════════════════════════
void updateLED(){
  if(!uvOn){ setLED(0); return; }
  if(humanFound){ flashLEDFn(); return; }
  if(obsClose){ setLED(130); return; }
  if(turning){ setLED(80); return; }
  setLED(uvBright);
}

void flashLEDFn(){
  if(millis()-flashT>=100){
    flashT=millis(); flashOn=!flashOn;
    setLED(flashOn?200:0);
  }
}

// ═════════════════════════════════════════════════════════════════
//  TASK 4: BLYNK UPDATE (500ms)
// ═════════════════════════════════════════════════════════════════
void updateBlynk(){
  Blynk.virtualWrite(V3,humanFound?255:0);
  Blynk.virtualWrite(V4,obsWarn?255:0);
  Blynk.virtualWrite(V5,String(tempC,1)+" C");
  Blynk.virtualWrite(V6,distCm>=399?"Clear":(String((int)distCm)+" cm"));
  Blynk.virtualWrite(V7,map(uvBright,0,255,0,100));
  Blynk.virtualWrite(V12,rowsDone);
  const char* sl[]={"NONE","LOW","MEDIUM","HIGH"};
  Blynk.virtualWrite(V8,sl[mildewLvl]);
}

// ═════════════════════════════════════════════════════════════════
//  TASK 5: SESSION TIMER (1000ms)
// ═════════════════════════════════════════════════════════════════
void tickTimer(){
  if(!sessionOn) return;
  unsigned long sec=(millis()-startMs)/1000;
  char buf[10]; snprintf(buf,10,"%02lu:%02lu",sec/60,sec%60);
  Blynk.virtualWrite(V13,String(buf));
}

// ═════════════════════════════════════════════════════════════════
//  SESSION START
// ═════════════════════════════════════════════════════════════════
void startSession(){
  rowsDone=0; totalDets=0; highHits=0; medHits=0; lowHits=0;
  maxSev=0; maxTemp=0; minDist=999;
  for(int i=0;i<16;i++) rowSeverity[i]=0;
  sendRowToCAM(0);
  startMs=millis(); sessionOn=true;
  Serial.printf("Session #%d started\n",sessNum);
  Blynk.virtualWrite(V15,"Session #"+String(sessNum)+" started");
}

// ═════════════════════════════════════════════════════════════════
//  SESSION END + CLOUD UPLOAD
// ═════════════════════════════════════════════════════════════════
void endSession(){
  if(!sessionOn) return;
  unsigned long dur=(millis()-startMs)/1000;
  sessionOn=false; autoMode=false; turning=false;
  motorsStop(); sendRowToCAM(0);

  int riskScore = highHits*3 + medHits*2 + lowHits*1;

  // Calculate efficacy vs previous session
  float efficacy=0.0f;
  if(lastSessionRiskScore>0 && riskScore<lastSessionRiskScore){
    efficacy=((float)(lastSessionRiskScore-riskScore)
              / (float)lastSessionRiskScore)*100.0f;
  } else if(lastSessionRiskScore>0 && riskScore>=lastSessionRiskScore){
    efficacy=-((float)(riskScore-lastSessionRiskScore)
               / (float)lastSessionRiskScore)*100.0f;
  }
  lastSessionRiskScore=riskScore;

  // Find worst row
  int worstRow=0, worstSev=0;
  for(int i=0;i<16;i++){
    if(rowSeverity[i]>worstSev){ worstSev=rowSeverity[i]; worstRow=i; }
  }

  // Build report for Blynk V15
  String rpt="=== SESSION #"+String(sessNum)+" ===\n";
  rpt+="Duration: "+String(dur/60)+"m "+String(dur%60)+"s\n";
  rpt+="Rows treated: "+String(rowsDone)+"\n";
  rpt+="Detections: "+String(totalDets)+"\n";
  rpt+="  HIGH: "+String(highHits)+"  MED: "+String(medHits);
  rpt+="  LOW: "+String(lowHits)+"\n";
  rpt+="Worst row: Row "+String(worstRow+1)+
      " (sev "+String(worstSev)+"/3)\n";
  rpt+="Risk score: "+String(riskScore)+"\n";
  if(efficacy>0)
    rpt+="Efficacy: +"+String(efficacy,1)+"% IMPROVEMENT\n";
  else if(efficacy<0)
    rpt+="Efficacy: "+String(efficacy,1)+"% WORSENING\n";
  else
    rpt+="First session — no comparison yet\n";

  Blynk.virtualWrite(V15,rpt);
  Serial.println(rpt);

  // Send to Google Sheets
  sendSessionCloud(dur,riskScore,efficacy,worstRow,worstSev);
  sessNum++;
}

void sendSessionCloud(unsigned long dur, int risk,
                      float efficacy, int worstRow, int worstSev){
  if(WiFi.status()!=WL_CONNECTED) return;
  if(String(SESSION_LOG_URL).startsWith("YOUR")) return;

  // Build row severity JSON array
  String rowArr="[";
  for(int i=0;i<rowsDone&&i<16;i++){
    rowArr+=String(rowSeverity[i]);
    if(i<rowsDone-1&&i<15) rowArr+=",";
  }
  rowArr+="]";

  String pay="{";
  pay+="\"type\":\"session\",";
  pay+="\"session\":"+String(sessNum)+",";
  pay+="\"duration_sec\":"+String(dur)+",";
  pay+="\"rows_completed\":"+String(rowsDone)+",";
  pay+="\"total_detections\":"+String(totalDets)+",";
  pay+="\"high_hits\":"+String(highHits)+",";
  pay+="\"med_hits\":"+String(medHits)+",";
  pay+="\"low_hits\":"+String(lowHits)+",";
  pay+="\"max_severity\":"+String(maxSev)+",";
  pay+="\"risk_score\":"+String(risk)+",";
  pay+="\"efficacy_pct\":"+String(efficacy,1)+",";
  pay+="\"worst_row\":"+String(worstRow+1)+",";
  pay+="\"worst_row_severity\":"+String(worstSev)+",";
  pay+="\"row_severity_array\":"+rowArr+",";
  pay+="\"max_temp\":"+String(maxTemp,2)+",";
  pay+="\"min_dist_cm\":"+String(minDist,1);
  pay+="}";

  HTTPClient h; h.begin(SESSION_LOG_URL);
  h.addHeader("Content-Type","application/json");
  h.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  h.setTimeout(6000);
  int c=h.POST(pay);
  Serial.printf("Session cloud log: HTTP %d\n",c);
  h.end();
}

// ═════════════════════════════════════════════════════════════════
//  LED CONTROL (supports both 12V strip and WS2812B)
// ═════════════════════════════════════════════════════════════════
void initLED(){
  #if LED_TYPE==1
    // Plain 12V strip via MOSFET on GPIO 15
    ledcSetup(LED_CH,LED_FREQ,LED_RES);
    ledcAttachPin(LED_PIN,LED_CH);
    ledcWrite(LED_CH,0);
    Serial.println("12V LED strip (MOSFET) ready");
  #else
    strip.begin(); strip.setBrightness(180); strip.show();
    Serial.println("WS2812B LED strip ready");
  #endif
}

void setLED(int brightness){
  brightness=constrain(brightness,0,255);
  #if LED_TYPE==1
    ledcWrite(LED_CH,brightness);
  #else
    strip.setBrightness(brightness);
    for(int i=0;i<12;i++)
      strip.setPixelColor(i, brightness==0 ? 0 :
        (humanFound ? strip.Color(255,0,0) :
         (obsClose   ? strip.Color(255,80,0) :
          (mildewLvl==3 ? strip.Color(255,255,255) :
           strip.Color(60,0,200)))));
    strip.show();
  #endif
}

// ═════════════════════════════════════════════════════════════════
//  MOTOR FUNCTIONS
// ═════════════════════════════════════════════════════════════════
void initMotors(){
  int pins[]={PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY};
  for(int p:pins) pinMode(p,OUTPUT);
  digitalWrite(STBY,HIGH); motorsStop();
  Serial.println("Motors ready");
}
void setLeft(int s){
  if(s>0){digitalWrite(AIN1,HIGH);digitalWrite(AIN2,LOW);analogWrite(PWMA,s);}
  else if(s<0){digitalWrite(AIN1,LOW);digitalWrite(AIN2,HIGH);analogWrite(PWMA,-s);}
  else{digitalWrite(AIN1,LOW);digitalWrite(AIN2,LOW);analogWrite(PWMA,0);}
}
void setRight(int s){
  if(s>0){digitalWrite(BIN1,HIGH);digitalWrite(BIN2,LOW);analogWrite(PWMB,s);}
  else if(s<0){digitalWrite(BIN1,LOW);digitalWrite(BIN2,HIGH);analogWrite(PWMB,-s);}
  else{digitalWrite(BIN1,LOW);digitalWrite(BIN2,LOW);analogWrite(PWMB,0);}
}
void motorsStop(){ setLeft(0); setRight(0); }

// ═════════════════════════════════════════════════════════════════
//  HC-SR04
// ═════════════════════════════════════════════════════════════════
float getDist(){
  digitalWrite(TRIG,LOW); delayMicroseconds(2);
  digitalWrite(TRIG,HIGH); delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  long d=pulseIn(ECHO,HIGH,30000UL);
  return d==0?400.0f:d*0.01715f;
}
