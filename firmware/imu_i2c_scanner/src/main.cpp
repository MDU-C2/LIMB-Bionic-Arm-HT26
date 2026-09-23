#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t SAMPLE_INTERVAL_MS = 50;
constexpr uint32_t RETRY_INTERVAL_MS = 1000;

constexpr uint8_t WHO_AM_I = 0x0F;
constexpr uint8_t WHO_AM_I_VALUE = 0x6C;
constexpr uint8_t CTRL1_XL = 0x10;
constexpr uint8_t CTRL2_G = 0x11;
constexpr uint8_t CTRL3_C = 0x12;
constexpr uint8_t STATUS_REG = 0x1E;
constexpr uint8_t OUT_TEMP_L = 0x20;

constexpr float ACCEL_G_PER_LSB = 0.000122F;  // +/-4 g
constexpr float GYRO_DPS_PER_LSB = 0.00875F; // +/-250 dps

struct PinPair {
  int sda;
  int scl;
};

// The connected AURORA harness uses SDA GPIO8 and SCL GPIO5. The remaining
// pairs keep compatibility with earlier harness documentation.
constexpr PinPair PIN_PAIRS[] = {
    {8, 5}, {5, 8}, {6, 7}, {7, 6}, {4, 5}, {5, 4}, {8, 9}, {9, 8}};

uint8_t imuAddress = 0;
int imuSdaPin = -1;
int imuSclPin = -1;
size_t nextPinPair = 0;
bool imuReady = false;
uint32_t lastSampleMs = 0;
uint32_t lastRetryMs = 0;

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t *data,
                   size_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = Wire.requestFrom(address, length, true);
  if (received != length) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (size_t index = 0; index < length; ++index) {
    data[index] = Wire.read();
  }
  return true;
}

bool readRegister(uint8_t address, uint8_t reg, uint8_t &value) {
  return readRegisters(address, reg, &value, 1);
}

bool configureImu(uint8_t address) {
  uint8_t identity = 0;
  if (!readRegister(address, WHO_AM_I, identity) ||
      identity != WHO_AM_I_VALUE) {
    return false;
  }

  // Reset, then enable block-data-update and register auto-increment.
  if (!writeRegister(address, CTRL3_C, 0x01)) {
    return false;
  }
  const uint32_t resetStarted = millis();
  do {
    delay(2);
    if (!readRegister(address, CTRL3_C, identity)) {
      return false;
    }
  } while ((identity & 0x01) != 0 && millis() - resetStarted < 100);

  if ((identity & 0x01) != 0 || !writeRegister(address, CTRL3_C, 0x44)) {
    return false;
  }

  // Both sensors at 104 Hz. CTRL1_XL keeps the default +/-4 g range;
  // CTRL2_G keeps the default +/-250 dps range.
  if (!writeRegister(address, CTRL1_XL, 0x40) ||
      !writeRegister(address, CTRL2_G, 0x40)) {
    return false;
  }
  delay(40);
  return true;
}

bool findImuOnCurrentBus() {
  constexpr uint8_t addresses[] = {0x6A, 0x6B};
  for (const uint8_t address : addresses) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0) {
      continue;
    }
    if (configureImu(address)) {
      imuAddress = address;
      return true;
    }
  }
  imuAddress = 0;
  return false;
}

bool tryNextImuWiring() {
  const PinPair pins = PIN_PAIRS[nextPinPair];
  nextPinPair = (nextPinPair + 1) % (sizeof(PIN_PAIRS) / sizeof(PIN_PAIRS[0]));

  Wire.end();
  delay(5);
  if (!Wire.begin(pins.sda, pins.scl)) {
    return false;
  }
  Wire.setTimeOut(20);
  Wire.setClock(400000);
  if (findImuOnCurrentBus()) {
    imuSdaPin = pins.sda;
    imuSclPin = pins.scl;
    return true;
  }

  imuAddress = 0;
  imuSdaPin = -1;
  imuSclPin = -1;
  return false;
}

int16_t signedValue(const uint8_t low, const uint8_t high) {
  return static_cast<int16_t>(static_cast<uint16_t>(low) |
                              (static_cast<uint16_t>(high) << 8));
}

void printDeviceStatus(const char *error) {
  Serial.printf(
      "{\"device\":\"%s\",\"uptime_ms\":%lu,\"free_heap_bytes\":%u,"
      "\"imu\":{\"connected\":false,\"error\":\"%s\"}}\n",
      ESP.getChipModel(), static_cast<unsigned long>(millis()),
      ESP.getFreeHeap(), error);
}

bool printImuSample() {
  uint8_t status = 0;
  if (!readRegister(imuAddress, STATUS_REG, status)) {
    return false;
  }
  if ((status & 0x03) != 0x03) {
    return true;
  }

  // Temperature, gyro XYZ, and accelerometer XYZ form one contiguous block.
  uint8_t raw[14] = {};
  if (!readRegisters(imuAddress, OUT_TEMP_L, raw, sizeof(raw))) {
    return false;
  }

  const float temperatureC = 25.0F + signedValue(raw[0], raw[1]) / 256.0F;
  const float gyroX = signedValue(raw[2], raw[3]) * GYRO_DPS_PER_LSB;
  const float gyroY = signedValue(raw[4], raw[5]) * GYRO_DPS_PER_LSB;
  const float gyroZ = signedValue(raw[6], raw[7]) * GYRO_DPS_PER_LSB;
  const float accelX = signedValue(raw[8], raw[9]) * ACCEL_G_PER_LSB;
  const float accelY = signedValue(raw[10], raw[11]) * ACCEL_G_PER_LSB;
  const float accelZ = signedValue(raw[12], raw[13]) * ACCEL_G_PER_LSB;

  Serial.printf(
      "{\"device\":\"%s\",\"uptime_ms\":%lu,\"free_heap_bytes\":%u,"
      "\"imu\":{\"connected\":true,\"model\":\"LSM6DSO32\","
      "\"address\":\"0x%02X\",\"sda_pin\":%d,\"scl_pin\":%d,"
      "\"temperature_c\":%.2f,"
      "\"accel_g\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f},"
      "\"gyro_dps\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}}\n",
      ESP.getChipModel(), static_cast<unsigned long>(millis()),
      ESP.getFreeHeap(), imuAddress, imuSdaPin, imuSclPin, temperatureC,
      accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
  return true;
}
} // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  imuReady = tryNextImuWiring();
  if (!imuReady) {
    printDeviceStatus(
        "LSM6DSO32 not responding; cycling through GPIO pairs 8/5, 6/7, 4/5 and 8/9 both ways");
  }
  lastRetryMs = millis();
}

void loop() {
  const uint32_t now = millis();
  if (!imuReady) {
    if (now - lastRetryMs >= RETRY_INTERVAL_MS) {
      printDeviceStatus(
          "LSM6DSO32 not responding; cycling through GPIO pairs 8/5, 6/7, 4/5 and 8/9 both ways");
      imuReady = tryNextImuWiring();
      lastRetryMs = millis();
    }
    delay(10);
    return;
  }

  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    if (!printImuSample()) {
      imuReady = false;
      printDeviceStatus("lost I2C connection to LSM6DSO32");
    }
  }
  delay(1);
}
