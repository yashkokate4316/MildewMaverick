/*
 ╔══════════════════════════════════════════════════════════════════╗
 ║   MILDEWMAVERICK — ESP32-CAM CODE  v5.0                                      ║
 ║   Team SpectraFarm | Pune Agri Hackathon 2025                                ║
 ╠══════════════════════════════════════════════════════════════════╣
 ║   NEW IN v5.0:                                                               ║
 ║   • Niphad Grape Leaf Disease Model (Maharashtra-specific)                   ║
 ║   • Frame capture every 8 seconds (no continuous streaming)                  ║
 ║   • Live MJPEG stream on port 81 (view only — no API cost)                   ║
 ║   • Logs row_number with each detection for Row Heatmap                      ║
 ║   • Receives current row from main ESP32 via 4-bit GPIO                      ║
 ║   • Improved camera image quality settings for better detection              ║
 ╠══════════════════════════════════════════════════════════════════╣
 ║   ROBOFLOW MODEL USED:                                                       ║
 ║   Name    : Niphad Grape Leaf Disease Dataset                                ║
 ║   Region  : Niphad, Nashik — Maharashtra grape belt                          ║
 ║   Classes : Powdery_Mildew, Black_Rot, Leaf_Blight, Healthy                  ║
 ║   Find at : universe.roboflow.com → search "Niphad grape"                    ║
 ╠══════════════════════════════════════════════════════════════════╣
 ║   ARDUINO IDE UPLOAD SETTINGS:                                               ║
 ║   Board            : AI Thinker ESP32-CAM                                    ║
 ║   Partition Scheme : Huge APP (3MB No OTA/1MB SPIFFS)                        ║
 ║   PSRAM            : Enabled                                                 ║
 ║   Upload Speed     : 115200                                                  ║
 ║   Hold BOOT button when "Connecting..." appears                              ║
 ╠══════════════════════════════════════════════════════════════════╣
 ║   LIBRARIES NEEDED:                                                          ║
 ║   ArduinoJson by Benoit Blanchon — install via Library Manager.              ║
 ╚══════════════════════════════════════════════════════════════════╝
*/

// ─────────────────────────────────────────────────────────────────
//  FILL IN YOUR DETAILS HERE BEFORE UPLOADING
// ─────────────────────────────────────────────────────────────────
const char* WIFI_SSID        = "YOUR_PHONE_HOTSPOT_NAME";
const char* WIFI_PASS        = "YOUR_HOTSPOT_PASSWORD";

// Roboflow — Niphad Grape Disease Model
// Step 1: Go to universe.roboflow.com
// Step 2: Search "Niphad grape leaf disease"
// Step 3: Click the dataset → click Deploy tab
// Step 4: Copy API key and paste below
const char* ROBOFLOW_KEY     = "YOUR_ROBOFLOW_API_KEY";
const char* ROBOFLOW_MODEL   = "niphad-grape-leaf-disease-datase-abjzp";
const char* ROBOFLOW_VERSION = "1";
const float MIN_CONFIDENCE   = 0.40f;

// Google Apps Script URL (same one used in main ESP32 code)
const char* SHEETS_URL       = "YOUR_GOOGLE_APPS_SCRIPT_URL";

// ─────────────────────────────────────────────────────────────────
//  INCLUDES
// ─────────────────────────────────────────────────────────────────
#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "Base64.h"
#include "time.h"

// ─────────────────────────────────────────────────────────────────
//  AI-THINKER ESP32-CAM PIN MAP — DO NOT CHANGE THESE
// ─────────────────────────────────────────────────────────────────
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ─────────────────────────────────────────────────────────────────
//  SIGNAL PINS — WIRING TO MAIN ESP32
// ─────────────────────────────────────────────────────────────────

// SEVERITY OUTPUT to main ESP32 (2 wires)
// CAM GPIO 12 → Main ESP32 GPIO 4
// CAM GPIO 13 → Main ESP32 GPIO 5
#define SIG_BIT0    12
#define SIG_BIT1    13

// ROW NUMBER INPUT from main ESP32 (4 wires = rows 0 to 15)
// Main GPIO 16 → CAM GPIO 14  (Bit 0)
// Main GPIO 17 → CAM GPIO 15  (Bit 1)
// Main GPIO 18 → CAM GPIO 2   (Bit 2)
// Main GPIO 19 → CAM GPIO 16  (Bit 3)
#define ROW_B0      14
#define ROW_B1      15
#define ROW_B2       2
#define ROW_B3      16

#define FLASH_PIN    4  // Built-in LED — active LOW
#define DETECT_INTERVAL_MS  8000
#define STREAM_PORT          81

// ─────────────────────────────────────────────────────────────────
//  STATE
// ─────────────────────────────────────────────────────────────────
unsigned long  lastDetectMs = 0;
int            curSeverity  = 0;
float          lastConf     = 0.0f;
String         lastClass    = "none";
int            totalDets    = 0;
int            currentRow   = 0;
httpd_handle_t streamSrv    = NULL;

// ─────────────────────────────────────────────────────────────────
//  DISEASE CLASSES FOR NIPHAD MODEL
// ─────────────────────────────────────────────────────────────────
struct Disease {
  const char* label;
  int sev;
  bool treat;
};
const Disease KNOWN[] = {
  { "Powdery_Mildew", 2, true  },
  { "powdery_mildew", 2, true  },
  { "Black_Rot",      3, true  },
  { "black_rot",      3, true  },
  { "Leaf_Blight",    1, true  },
  { "leaf_blight",    1, true  },
  { "mildew",         2, true  },
  { "disease",        1, true  },
  { "infected",       1, true  },
  { "fungal",         2, true  },
  { "Healthy",        0, false },
  { "healthy",        0, false },
};
const int NDIS = sizeof(KNOWN) / sizeof(KNOWN[0]);

// ═════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200); delay(400);
  Serial.println("\n=== MildewMaverick ESP32-CAM v5.0 ===");
  Serial.println("=== Niphad Grape Disease Model     ===");

  pinMode(SIG_BIT0, OUTPUT); pinMode(SIG_BIT1, OUTPUT);
  setSeverity(0);
  pinMode(ROW_B0, INPUT); pinMode(ROW_B1, INPUT);
  pinMode(ROW_B2, INPUT); pinMode(ROW_B3, INPUT);
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, HIGH);

  if (!initCamera()) { delay(2000); ESP.restart(); }
  Serial.println("Camera OK");

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to hotspot");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Stream: http://%s:%d/stream\n",
                  WiFi.localIP().toString().c_str(), STREAM_PORT);
    configTime(19800, 0, "pool.ntp.org"); // IST = UTC+5:30
    startStreamServer();
  } else {
    Serial.println("\nWiFi failed — offline mode");
  }

  for (int i = 0; i < 4; i++) {
    digitalWrite(FLASH_PIN, LOW); delay(80);
    digitalWrite(FLASH_PIN, HIGH); delay(80);
  }
  Serial.println("CAM ready\n");
}

// ═════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════
void loop() {
  currentRow = readRow();
  if (millis() - lastDetectMs >= DETECT_INTERVAL_MS) {
    lastDetectMs = millis();
    runCycle();
  }
}

// ─────────────────────────────────────────────────────────────────
int readRow() {
  return (digitalRead(ROW_B3) << 3) | (digitalRead(ROW_B2) << 2)
       | (digitalRead(ROW_B1) << 1) |  digitalRead(ROW_B0);
}

// ─────────────────────────────────────────────────────────────────
void runCycle() {
  Serial.printf("\n--- Detection | Row %d ---\n", currentRow);
  if (WiFi.status() != WL_CONNECTED) { setSeverity(0); return; }

  digitalWrite(FLASH_PIN, LOW); delay(60);
  digitalWrite(FLASH_PIN, HIGH);

  int sev = captureAndAnalyse();
  curSeverity = sev;
  setSeverity(sev);
  if (sev > 0) logToSheets(sev, lastConf, lastClass, currentRow);

  const char* lbl[] = { "NONE","LOW","MEDIUM","HIGH" };
  Serial.printf("Result: %s (%.0f%%) Row %d\n",
                lbl[sev], lastConf*100.0f, currentRow);
}

// ─────────────────────────────────────────────────────────────────
int captureAndAnalyse() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { Serial.println("Capture failed"); return 0; }
  Serial.printf("Frame: %u bytes\n", fb->len);

  size_t el = Base64.encodedLength(fb->len);
  char*  eb = (char*)malloc(el + 4);
  if (!eb) { esp_camera_fb_return(fb); return 0; }
  Base64.encode(eb, (char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);

  String url = "https://detect.roboflow.com/";
  url += ROBOFLOW_MODEL; url += "/"; url += ROBOFLOW_VERSION;
  url += "?api_key="; url += ROBOFLOW_KEY;
  url += "&name=leaf.jpg&confidence=40";

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.setTimeout(10000);
  int code = http.POST(String(eb));
  free(eb);

  if (code != 200) {
    Serial.printf("API error: %d\n", code);
    http.end(); return 0;
  }
  String body = http.getString();
  http.end();
  return parseResponse(body);
}

// ─────────────────────────────────────────────────────────────────
int parseResponse(const String& json) {
  DynamicJsonDocument doc(8192);
  if (deserializeJson(doc, json)) { return 0; }

  JsonArray preds = doc["predictions"];
  if (preds.size() == 0) { lastConf=0; lastClass="none"; return 0; }

  float  bestConf = 0; String bestCls = "";
  int    cnt=0, maxSev=0;

  for (JsonObject p : preds) {
    float  c = p["confidence"].as<float>();
    String s = p["class"].as<String>();
    Serial.printf("  %s %.0f%%\n", s.c_str(), c*100);
    if (c < MIN_CONFIDENCE) continue;
    for (int i=0; i<NDIS; i++) {
      if (s == KNOWN[i].label && KNOWN[i].treat) {
        cnt++;
        if (KNOWN[i].sev > maxSev) maxSev = KNOWN[i].sev;
        if (c > bestConf) { bestConf=c; bestCls=s; }
        break;
      }
    }
  }

  if (cnt == 0) { lastConf=0; lastClass="none"; return 0; }
  lastConf=bestConf; lastClass=bestCls; totalDets++;

  int sev = maxSev;
  if (cnt>=3 || bestConf>0.85f) sev=3;
  else if (cnt>=2 || bestConf>0.65f) sev=max(sev,2);
  return constrain(sev,1,3);
}

// ─────────────────────────────────────────────────────────────────
void logToSheets(int sev, float conf, const String& cls, int row) {
  if (WiFi.status()!=WL_CONNECTED) return;
  if (String(SHEETS_URL).startsWith("YOUR")) return;

  struct tm t; char ts[32]="unknown";
  if (getLocalTime(&t,1500)) strftime(ts,sizeof(ts),"%Y-%m-%d %H:%M:%S",&t);
  const char* sl[]={"NONE","LOW","MEDIUM","HIGH"};

  String pay="{";
  pay+="\"type\":\"detection\",";
  pay+="\"timestamp\":\""+String(ts)+"\",";
  pay+="\"row_number\":"+String(row)+",";
  pay+="\"severity\":"+String(sev)+",";
  pay+="\"severity_label\":\""+String(sl[sev])+"\",";
  pay+="\"confidence\":"+String(conf,3)+",";
  pay+="\"class\":\""+cls+"\",";
  pay+="\"total_detections\":"+String(totalDets);
  pay+="}";

  HTTPClient h; h.begin(SHEETS_URL);
  h.addHeader("Content-Type","application/json");
  h.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  h.setTimeout(6000);
  int c=h.POST(pay);
  Serial.printf("Sheets: HTTP %d\n",c);
  h.end();
}

// ─────────────────────────────────────────────────────────────────
void setSeverity(int lvl) {
  lvl=constrain(lvl,0,3);
  digitalWrite(SIG_BIT0,(lvl&0x01)?HIGH:LOW);
  digitalWrite(SIG_BIT1,(lvl&0x02)?HIGH:LOW);
}

// ─────────────────────────────────────────────────────────────────
//  MJPEG STREAM
// ─────────────────────────────────────────────────────────────────
#define BD "mfw"
esp_err_t sCb(httpd_req_t* r){
  char h[64]; const char* ct="multipart/x-mixed-replace;boundary="BD;
  const char* bd="\r\n--"BD"\r\n";
  const char* pt="Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
  httpd_resp_set_type(r,ct);
  httpd_resp_set_hdr(r,"Access-Control-Allow-Origin","*");
  while(true){
    camera_fb_t* fb=esp_camera_fb_get(); if(!fb) break;
    size_t hl=snprintf(h,64,pt,fb->len);
    esp_err_t e=httpd_resp_send_chunk(r,bd,strlen(bd));
    e|=httpd_resp_send_chunk(r,h,hl);
    e|=httpd_resp_send_chunk(r,(const char*)fb->buf,fb->len);
    esp_camera_fb_return(fb);
    if(e!=ESP_OK) break; delay(80);
  }
  return ESP_OK;
}

void startStreamServer(){
  httpd_config_t c=HTTPD_DEFAULT_CONFIG();
  c.server_port=STREAM_PORT; c.max_open_sockets=3;
  httpd_uri_t u={"/stream",HTTP_GET,sCb,NULL};
  if(httpd_start(&streamSrv,&c)==ESP_OK){
    httpd_register_uri_handler(streamSrv,&u);
    Serial.printf("Stream on port %d\n",STREAM_PORT);
  }
}

// ─────────────────────────────────────────────────────────────────
//  CAMERA INIT
// ─────────────────────────────────────────────────────────────────
bool initCamera(){
  camera_config_t c={};
  c.ledc_channel=LEDC_CHANNEL_0; c.ledc_timer=LEDC_TIMER_0;
  c.pin_d0=Y2_GPIO_NUM; c.pin_d1=Y3_GPIO_NUM;
  c.pin_d2=Y4_GPIO_NUM; c.pin_d3=Y5_GPIO_NUM;
  c.pin_d4=Y6_GPIO_NUM; c.pin_d5=Y7_GPIO_NUM;
  c.pin_d6=Y8_GPIO_NUM; c.pin_d7=Y9_GPIO_NUM;
  c.pin_xclk=XCLK_GPIO_NUM; c.pin_pclk=PCLK_GPIO_NUM;
  c.pin_vsync=VSYNC_GPIO_NUM; c.pin_href=HREF_GPIO_NUM;
  c.pin_sscb_sda=SIOD_GPIO_NUM; c.pin_sscb_scl=SIOC_GPIO_NUM;
  c.pin_pwdn=PWDN_GPIO_NUM; c.pin_reset=RESET_GPIO_NUM;
  c.xclk_freq_hz=20000000; c.pixel_format=PIXFORMAT_JPEG;
  if(psramFound()){
    c.frame_size=FRAMESIZE_VGA; c.jpeg_quality=12; c.fb_count=2;
  } else {
    c.frame_size=FRAMESIZE_QVGA; c.jpeg_quality=20; c.fb_count=1;
  }
  if(esp_camera_init(&c)!=ESP_OK) return false;
  // Image quality tuning
  sensor_t* s=esp_camera_sensor_get();
  if(s){
    s->set_brightness(s,1); s->set_saturation(s,-1);
    s->set_sharpness(s,1);  s->set_denoise(s,1);
    s->set_whitebal(s,1);   s->set_awb_gain(s,1);
    s->set_exposure_ctrl(s,1);
  }
  return true;
}
