/*
 * Main.ino
 *
 * WIRING:  WT61 TX -> ESP32 RX2 | WT61 RX -> ESP32 TX2
 *          WT61 VCC -> 3.3V     | WT61 GND -> GND
 *
 * TIMING:
 *   begin() automatically sends the 100Hz command to the sensor
 *   and switches to 115200 baud. After that, you get a new angle
 *   reading every 10ms. The sensor remembers this setting even
 *   after power-off.
 *
 * MULTIPLE INSTANCES — THE FULL ANSWER:
 *
 *   Q: "Can I run multiple instances on the same WT61?"
 *   A: No. 1 sensor sends 1 byte stream on 1 UART. If 2 objects
 *      both call serial.read(), object A grabs some bytes and
 *      object B grabs others — both get corrupted partial packets.
 *      You need exactly 1 WT61IMU object per physical sensor.
 *
 *      BUT you can READ from that 1 object at multiple rates.
 *      The object always has the latest data. You just check it
 *      at whatever interval you need:
 *        - Every 10ms for a PID control loop
 *        - Every 500ms for display/logging
 *        - Every 5000ms for a slow health check
 *      These are NOT separate instances — they're separate timers
 *      reading the same object. This is shown below.
 *
 *   Q: "How many sensors per ESP32?"
 *   A: The ESP32 has 3 hardware UARTs:
 *        Serial  (UART0) = USB connection to your computer
 *        Serial1 (UART1) = available for a sensor
 *        Serial2 (UART2) = available for a sensor
 *      So: maximum 2 physical WT61 sensors per ESP32.
 *      Each gets its own WT61IMU object on its own serial port.
 *
 *      Example for 2 sensors:
 *        WT61IMU imuA(Serial1);  // sensor A on Serial1 pins
 *        WT61IMU imuB(Serial2);  // sensor B on Serial2 pins
 *        In setup(): imuA.begin(); imuB.begin();
 *        In loop():  imuA.update(); imuB.update();
 */

#include "WT61IMU.h"

WT61IMU imu(Serial2);

// Multiple timers reading the SAME object at different rates
unsigned long lastControlLoop = 0;   // 100Hz = every 10ms
unsigned long lastDisplay     = 0;   // 2Hz   = every 500ms

void setup() {
    Serial.begin(115200);

    // Wait until Serial Monitor is actually open and ready.
    // Prints a prompt and waits for you to press Enter (or send
    // any character). This way you never miss the calibration output.
    while (!Serial) { delay(10); }
    delay(500);
    Serial.println();
    Serial.println("=== WT61 IMU Ready ===");
    Serial.println("Place sensor in its ZERO position (flat/still).");
    Serial.println("Press ENTER in Serial Monitor to start calibration...");

    // Flush any leftover input, then wait for a keypress
    while (Serial.available()) Serial.read();
    while (!Serial.available()) { delay(10); }
    while (Serial.available()) Serial.read();  // consume the Enter key

    Serial.println();
    Serial.println("Switching sensor to 100Hz / 115200 baud...");
    Serial.println("Calibrating... keep sensor STILL for 3 seconds!");

    // begin() handles everything:
    //   1. Sends 100Hz command to sensor at both baud rates
    //   2. Switches to 115200
    //   3. Calibrates all 3 axes
    imu.begin();

    if (imu.isCalibrated()) {
        Serial.println("--- Calibration OK ---");
        Serial.print("  Roll offset:  "); Serial.println(imu.getRollOffset(), 2);
        Serial.print("  Pitch offset: "); Serial.println(imu.getPitchOffset(), 2);
        Serial.print("  Yaw offset:   "); Serial.println(imu.getYawOffset(), 2);
        Serial.print("  Packets used: "); Serial.println(imu.getPacketCount());
        Serial.println("  Update rate:  100Hz (10ms per reading)");
    } else {
        Serial.println("--- Calibration FAILED ---");
        Serial.println("  No data. Check wiring & baud rate.");
    }
    Serial.println("=========================");
    Serial.println();
}

void loop() {
    // Always drain the serial buffer. Cheap, just reads bytes.
    imu.update();

    unsigned long now = millis();

    // --- 100Hz CONTROL LOOP (every 10ms) ---
    // This is where you'd feed quaternion data to your Stewart
    // platform's PID controller or inverse kinematics.
    if (now - lastControlLoop >= 10) {
        lastControlLoop = now;

        // Your control code would go here, e.g.:
        // float qx = imu.getQx();
        // float qy = imu.getQy();
        // float qz = imu.getQz();
        // float qw = imu.getQw();
        // updateStewartPlatform(qx, qy, qz, qw);
    }

    // --- 2Hz DISPLAY (every 500ms) ---
    if (now - lastDisplay >= 500) {
        lastDisplay = now;

        Serial.print("Roll: ");
        Serial.print(imu.getRoll(), 2);
        Serial.print("  Pitch: ");
        Serial.print(imu.getPitch(), 2);
        Serial.print("  Yaw: ");
        Serial.print(imu.getYaw(), 2);
        Serial.print("  | q:[");
        Serial.print(imu.getQx(), 4); Serial.print(", ");
        Serial.print(imu.getQy(), 4); Serial.print(", ");
        Serial.print(imu.getQz(), 4); Serial.print(", ");
        Serial.print(imu.getQw(), 4); Serial.print("]");
        Serial.print("  pkts:"); Serial.print(imu.getPacketCount());
        Serial.print(" err:"); Serial.println(imu.getErrorCount());
    }
}
