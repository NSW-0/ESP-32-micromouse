#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define I2C_SDA 21
#define I2C_SCL 22

// CHANGE THESE to your working pins (prefer NOT 0,2,4,5,12,15)
#define XSHUT_LEFT   13
#define XSHUT_FRONT  14
#define XSHUT_RIGHT  15

#define ADDR_LEFT   0x30
#define ADDR_FRONT  0x31
#define ADDR_RIGHT  0x32

Adafruit_VL53L0X loxL, loxF, loxR;

void i2cScan(const char* tag) {
  Serial.print("[I2C SCAN] "); Serial.println(tag);
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found 0x"); Serial.println(addr, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  Found NONE");
}

bool beginWithRetries(Adafruit_VL53L0X &lox, uint8_t addr, int tries) {
  for (int i = 1; i <= tries; i++) {
    Serial.print("  Try "); Serial.print(i); Serial.print(": begin(0x");
    Serial.print(addr, HEX); Serial.println(")...");
    if (lox.begin(addr, false, &Wire)) return true;
    delay(80);
  }
  return false;
}

bool bringUpAndSetAddress(Adafruit_VL53L0X &lox, int xshutPin, uint8_t newAddr, const char* name) {
  Serial.println();
  Serial.print("--- "); Serial.print(name); Serial.println(" ---");

  // keep sensor OFF
  digitalWrite(xshutPin, LOW);
  delay(50);

  // turn ON
  digitalWrite(xshutPin, HIGH);
  delay(150); // bigger boot delay helps a lot

  i2cScan("After ON (expect 0x29)");

  // init at default 0x29 (only ONE sensor must be alive now)
  if (!beginWithRetries(lox, 0x29, 6)) {
    Serial.print("❌ "); Serial.print(name); Serial.println(" begin(0x29) FAILED");
    return false;
  }

  // set unique address
  lox.setAddress(newAddr);
  delay(20);

  Serial.print("✅ "); Serial.print(name);
  Serial.print(" set to 0x"); Serial.println(newAddr, HEX);

  // optional: confirm it responds at new address
  // (some libs don’t require re-begin, but it’s a nice check)
  if (!beginWithRetries(lox, newAddr, 3)) {
    Serial.print("❌ "); Serial.print(name);
    Serial.println(" did not respond at new address");
    return false;
  }

  return true;
}

int readMM(Adafruit_VL53L0X &lox) {
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  if (m.RangeStatus != 0) return -1;
  return (int)m.RangeMilliMeter;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nBOOT ✅");

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Force ALL OFF before I2C starts
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // start slow

  i2cScan("ALL XSHUT LOW (expect NONE)");

  Serial.println("\n=== Initializing 3 VL53L0X sensors ===");

  bool okL = bringUpAndSetAddress(loxL, XSHUT_LEFT,  ADDR_LEFT,  "LEFT");
  bool okF = bringUpAndSetAddress(loxF, XSHUT_FRONT, ADDR_FRONT, "FRONT");
  bool okR = bringUpAndSetAddress(loxR, XSHUT_RIGHT, ADDR_RIGHT, "RIGHT");

  if (!(okL && okF && okR)) {
    Serial.println("\n❌ Init failed. Most likely: more than one sensor still alive at 0x29.");
    Serial.println("Fix: verify ALL XSHUT LOW => scan shows NONE. Use safer GPIOs. Re-check XSHUT wiring.");
    while (1) delay(1000);
  }

  Wire.setClock(400000); // speed up after success
  Serial.println("\n✅ All sensors ready!");
}

void loop() {
  int L = readMM(loxL);
  int F = readMM(loxF);
  int R = readMM(loxR);

  Serial.print("L="); (L < 0) ? Serial.print("----") : Serial.print(L);
  Serial.print(" mm  F="); (F < 0) ? Serial.print("----") : Serial.print(F);
  Serial.print(" mm  R="); (R < 0) ? Serial.print("----") : Serial.print(R);
  Serial.println(" mm");

  delay(200);
}
