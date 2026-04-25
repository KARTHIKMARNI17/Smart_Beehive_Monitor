

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <time.h>

// ─── USER CONFIG ────────────────────────────────────────────
const char* WIFI_SSID     = "Bangaram";
const char* WIFI_PASSWORD = "bangaram6789";
const char* NTP_SERVER    = "pool.ntp.org";
const long  GMT_OFFSET_SEC   = 0;    // UTC offset in seconds  (e.g. IST = 19800)
const int   DAYLIGHT_OFFSET  = 0;   // DST offset in seconds
// ────────────────────────────────────────────────────────────

// ─── PIN DEFINITIONS ────────────────────────────────────────
#define DHT_PIN         4       // DHT22 data pin
#define DHT_TYPE        DHT22
#define MIC_PIN         34      // MAX9814 analog out
// ────────────────────────────────────────────────────────────

// ─── SIGNAL PROCESSING CONFIG ───────────────────────────────
#define MA_WINDOW        10     // Moving average window size
#define PEAK_THRESHOLD   200    // ADC delta that counts as a spike
#define SOUND_NORM_MIN   0      // Normalised sound floor
#define SOUND_NORM_MAX   1000   // Normalised sound ceiling
#define ADC_RAW_MIN      0
#define ADC_RAW_MAX      4095   // 12-bit ESP32 ADC
// ────────────────────────────────────────────────────────────

// ─── ALERT THRESHOLDS ───────────────────────────────────────
#define TEMP_WARNING_C   33.0f
#define TEMP_CRITICAL_C  38.0f
#define SOUND_WARNING    650
#define SOUND_CRITICAL   850
// ────────────────────────────────────────────────────────────

// ─── TIMING ─────────────────────────────────────────────────
#define SENSOR_INTERVAL_MS   1000   // Read sensors every 1 s
#define WIFI_CHECK_INTERVAL  5000   // Re-check WiFi every 5 s
// ────────────────────────────────────────────────────────────

// ─── OBJECTS ────────────────────────────────────────────────
DHT      dht(DHT_PIN, DHT_TYPE);
WebServer server(80);

// ─── MOVING AVERAGE STATE ────────────────────────────────────
int  soundWindow[MA_WINDOW] = {0};
int  windowIndex             = 0;
bool windowFull              = false;
int  prevSoundRaw            = 0;
bool peakDetected            = false;

// ─── SENSOR SNAPSHOT ─────────────────────────────────────────
struct SensorData {
  float temperature  = 0.0f;
  float humidity     = 0.0f;
  int   sound        = 0;
  bool  peakFlag     = false;
  String status      = "normal";
  String timestamp   = "";
} latest;

// ─── TIMING STATE ────────────────────────────────────────────
unsigned long lastSensorRead  = 0;
unsigned long lastWifiCheck   = 0;

// ═════════════════════════════════════════════════════════════
// MOVING AVERAGE  — push value, return smoothed average
// ═════════════════════════════════════════════════════════════
int movingAverage(int newVal) {
  soundWindow[windowIndex] = newVal;
  windowIndex = (windowIndex + 1) % MA_WINDOW;
  if (windowIndex == 0) windowFull = true;

  int count = windowFull ? MA_WINDOW : windowIndex;
  long sum  = 0;
  for (int i = 0; i < count; i++) sum += soundWindow[i];
  return (int)(sum / count);
}

// ═════════════════════════════════════════════════════════════
// PEAK DETECTION  — true when delta exceeds threshold
// ═════════════════════════════════════════════════════════════
bool detectPeak(int currentRaw) {
  bool spike = abs(currentRaw - prevSoundRaw) > PEAK_THRESHOLD;
  prevSoundRaw = currentRaw;
  return spike;
}

// ═════════════════════════════════════════════════════════════
// NORMALISE  — map raw ADC to 0‥1000 range
// ═════════════════════════════════════════════════════════════
int normalisedSound(int rawADC) {
  int clamped = constrain(rawADC, ADC_RAW_MIN, ADC_RAW_MAX);
  return map(clamped, ADC_RAW_MIN, ADC_RAW_MAX, SOUND_NORM_MIN, SOUND_NORM_MAX);
}

// ═════════════════════════════════════════════════════════════
// STATUS CLASSIFIER
// ═════════════════════════════════════════════════════════════
String classifyStatus(float temp, int sound, bool peak) {
  if (temp >= TEMP_CRITICAL_C || sound >= SOUND_CRITICAL) return "critical";
  if (temp >= TEMP_WARNING_C  || sound >= SOUND_WARNING   || peak)  return "warning";
  return "normal";
}

// ═════════════════════════════════════════════════════════════
// ISO 8601 TIMESTAMP  (requires NTP sync)
// ═════════════════════════════════════════════════════════════
String getISO8601() {
  struct tm ti;
  if (!getLocalTime(&ti)) return "1970-01-01T00:00:00Z";

  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &ti);
  return String(buf);
}

// ═════════════════════════════════════════════════════════════
// READ ALL SENSORS & PROCESS
// ═════════════════════════════════════════════════════════════
void readSensors() {
  // — DHT22 —
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) latest.temperature = t;
  if (!isnan(h)) latest.humidity    = h;

  // — MAX9814 (multi-sample to capture envelope) —
  long peakADC = 0;
  for (int s = 0; s < 64; s++) {
    int v = analogRead(MIC_PIN);
    if (v > peakADC) peakADC = v;
    delayMicroseconds(125);   // ~8 kHz sample rate for 64 samples ≈ 8 ms
  }

  // Signal processing pipeline
  latest.peakFlag = detectPeak((int)peakADC);
  int smoothed    = movingAverage((int)peakADC);
  latest.sound    = normalisedSound(smoothed);

  // Status & timestamp
  latest.status    = classifyStatus(latest.temperature, latest.sound, latest.peakFlag);
  latest.timestamp = getISO8601();

  Serial.printf("[SENSOR] Temp=%.1f°C  Hum=%.1f%%  Sound=%d  Peak=%s  Status=%s\n",
    latest.temperature, latest.humidity, latest.sound,
    latest.peakFlag ? "YES" : "no", latest.status.c_str());
}

// ═════════════════════════════════════════════════════════════
// REST HANDLER  — GET /api/data
// ═════════════════════════════════════════════════════════════
void handleApiData() {
  // CORS headers — allow any origin (dashboard can be served from anywhere)
  server.sendHeader("Access-Control-Allow-Origin",  "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.sendHeader("Cache-Control", "no-cache");

  if (server.method() == HTTP_OPTIONS) {
    server.send(204);
    return;
  }

  // Build JSON response
  StaticJsonDocument<256> doc;
  doc["temperature"] = serialized(String(latest.temperature, 1));
  doc["humidity"]    = serialized(String(latest.humidity, 1));
  doc["sound"]       = latest.sound;
  doc["peak"]        = latest.peakFlag;
  doc["status"]      = latest.status;
  doc["timestamp"]   = latest.timestamp;

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// ═════════════════════════════════════════════════════════════
// REST HANDLER  — GET /api/health
// ═════════════════════════════════════════════════════════════
void handleHealth() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// ═════════════════════════════════════════════════════════════
// REST HANDLER  — 404
// ═════════════════════════════════════════════════════════════
void handleNotFound() {
  server.send(404, "application/json", "{\"error\":\"endpoint not found\"}");
}

// ═════════════════════════════════════════════════════════════
// WiFi CONNECT  (blocking during initial setup)
// ═════════════════════════════════════════════════════════════
void connectWiFi() {
  Serial.printf("\n[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected! IP → http://%s\n", WiFi.localIP().toString().c_str());
    // Sync NTP time
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, NTP_SERVER);
    Serial.println("[NTP] Time synced.");
  } else {
    Serial.println("\n[WiFi] Could not connect — retrying in background.");
  }
}

// ═════════════════════════════════════════════════════════════
// WiFi WATCHDOG  (non-blocking reconnect)
// ═════════════════════════════════════════════════════════════
void maintainWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Disconnected — reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    // NTP will resync automatically on next getLocalTime() call
  }
}

// ═════════════════════════════════════════════════════════════
// SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n╔══════════════════════════════════╗");
  Serial.println("║   Smart Beehive Monitor v1.0     ║");
  Serial.println("╚══════════════════════════════════╝");

  // ADC configuration
  analogReadResolution(12);       // 12-bit → 0..4095
  analogSetAttenuation(ADC_11db); // Full 0-3.3 V range

  // DHT22 init
  dht.begin();
  Serial.println("[DHT22] Sensor initialised on GPIO4");

  // WiFi
  connectWiFi();

  // HTTP routes
  server.on("/api/data",   HTTP_GET,     handleApiData);
  server.on("/api/data",   HTTP_OPTIONS, handleApiData);   // CORS preflight
  server.on("/api/health", HTTP_GET,     handleHealth);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[HTTP] REST server started on port 80");

  // Prime sensor read
  readSensors();
}

// ═════════════════════════════════════════════════════════════
// LOOP
// ═════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // Non-blocking sensor sampling
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    readSensors();
  }

  // Non-blocking WiFi watchdog
  if (now - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
    lastWifiCheck = now;
    maintainWiFi();
  }

  // Handle incoming HTTP requests
  server.handleClient();
}
