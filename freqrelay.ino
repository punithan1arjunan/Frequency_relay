/*************** BLYNK TEMPLATE INFO ****************/
#define BLYNK_TEMPLATE_ID "TMPL3ueDwd5N1"
#define BLYNK_TEMPLATE_NAME "Overvoltage"
#define BLYNK_AUTH_TOKEN "9pHGIIX4DRs-fZLlxbCKdTy1W_loj5Cq"
/****************************************************/

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ZMPT101B.h>

// ------------------- CONFIG -------------------
#define ZMPT_PIN 34
#define VOLTAGE_CALIBRATION 694
#define HYSTERESIS 50
#define SAMPLE_INTERVAL 1000
#define MIN_CROSS_INTERVAL 8
#define AVERAGE_COUNT 5
#define ADC_RES 4095.0
#define RELAY_PIN 25
// ------------------------------------------------

char ssid[] = "Puni";
char pass[] = "1234567890";

ZMPT101B voltageSensor(ZMPT_PIN);

unsigned long freqOutOfRangeStart = 0;
bool tripActive = false;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  voltageSensor.setSensitivity(VOLTAGE_CALIBRATION);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);  // Relay ON initially

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("Blynk connected");
}

void loop() {
  Blynk.run();

  float freqSum = 0, vrmsSum = 0;

  for (int n = 0; n < AVERAGE_COUNT; n++) {
    float Vrms = voltageSensor.getRmsVoltage();

    int zeroCrossCount = 0;
    bool lastPositive = false;
    unsigned long lastCrossTime = 0;
    unsigned long startTime = millis();
    const int OFFSET = 2048;

    while (millis() - startTime < SAMPLE_INTERVAL) {
      int sensorValue = analogRead(ZMPT_PIN);
      unsigned long now = millis();

      if (!lastPositive && sensorValue > (OFFSET + HYSTERESIS)) {
        if (now - lastCrossTime > MIN_CROSS_INTERVAL) {
          zeroCrossCount++;
          lastCrossTime = now;
        }
        lastPositive = true;
      }
      if (sensorValue < (OFFSET - HYSTERESIS))
        lastPositive = false;
    }

    float frequency = zeroCrossCount / (SAMPLE_INTERVAL / 1000.0);
    freqSum += frequency;
    vrmsSum += Vrms;
  }

  float avgFreq = freqSum / AVERAGE_COUNT;
  float avgVrms = vrmsSum / AVERAGE_COUNT;

  Serial.print("Avg Frequency: ");
  Serial.print(avgFreq, 2);
  Serial.print(" Hz\tRMS Voltage: ");
  Serial.print(avgVrms, 2);
  Serial.println(" V");

  // Send data to Blynk
  Blynk.virtualWrite(V0, avgVrms);
  Blynk.virtualWrite(V1, avgFreq);

  // -------- Relay Trip Logic --------
  if (avgFreq < 48.5 || avgFreq > 51.5) {
    if (freqOutOfRangeStart == 0)
      freqOutOfRangeStart = millis();

    if (millis() - freqOutOfRangeStart >= 4000 && !tripActive) {
      digitalWrite(RELAY_PIN, LOW); // Trip relay
      tripActive = true;
      Blynk.virtualWrite(V2, 1);    // Trip status flag
      Serial.println("⚠️ Frequency out of range — RELAY TRIPPED!");
    }
  } else {
    freqOutOfRangeStart = 0;
    if (tripActive) {
      digitalWrite(RELAY_PIN, HIGH); // Reset relay
      tripActive = false;
      Blynk.virtualWrite(V2, 0);
      Serial.println("✅ Frequency back to normal — Relay reset.");
    }
  }

  delay(1000);
}
