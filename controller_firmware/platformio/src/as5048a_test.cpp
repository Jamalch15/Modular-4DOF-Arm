#include <Arduino.h>
#include <SPI.h>

namespace {
constexpr int kPinCs = 5;
constexpr int kPinSck = 18;
constexpr int kPinMiso = 19;
constexpr int kPinMosi = 23;

constexpr uint32_t kSpiClockHz = 1000000;
constexpr uint16_t kReadAngleCommand = 0xFFFF;
constexpr uint16_t kClearErrorCommand = 0x4001;
constexpr uint16_t kAngleMask = 0x3FFF;
constexpr uint16_t kErrorFlag = 0x4000;
constexpr uint32_t kPrintIntervalMs = 100;

SPISettings as5048Settings(kSpiClockHz, MSBFIRST, SPI_MODE1);
uint32_t lastPrintMs = 0;

uint16_t transfer16(uint16_t value) {
  SPI.beginTransaction(as5048Settings);
  digitalWrite(kPinCs, LOW);
  delayMicroseconds(1);
  const uint16_t response = SPI.transfer16(value);
  delayMicroseconds(1);
  digitalWrite(kPinCs, HIGH);
  SPI.endTransaction();
  delayMicroseconds(1);
  return response;
}

uint16_t readAngleRaw(bool& errorFlag) {
  transfer16(kReadAngleCommand);
  const uint16_t response = transfer16(0x0000);
  errorFlag = (response & kErrorFlag) != 0;

  if (errorFlag) {
    transfer16(kClearErrorCommand);
    transfer16(0x0000);
  }

  return response & kAngleMask;
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(kPinCs, OUTPUT);
  digitalWrite(kPinCs, HIGH);
  SPI.begin(kPinSck, kPinMiso, kPinMosi, kPinCs);

  Serial.println();
  Serial.println("AS5048A ESP32-WROVER-E test");
  Serial.println("Wiring: CLK=GPIO12 MISO=GPIO13 MOSI=GPIO14 CS=GPIO15 3V3=3V3 GND=GND");
  Serial.println("Rotate the magnet/shaft slowly and check that raw and deg change smoothly.");
}

void loop() {
  const uint32_t nowMs = millis();
  if (nowMs - lastPrintMs < kPrintIntervalMs) {
    return;
  }
  lastPrintMs = nowMs;

  bool errorFlag = false;
  const uint16_t raw = readAngleRaw(errorFlag);
  const float degrees = static_cast<float>(raw) * 360.0f / 16384.0f;

  Serial.printf("raw=%5u deg=%7.2f error=%d\r\n", raw, degrees, errorFlag ? 1 : 0);
}
