#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>
#include "common/mavlink.h"  // Adjust this path if needed based on your directory

void sendMavlinkMessage();  // Function prototype

// ====== Configuration ======
#define WIFI_SSID     "Sss"
#define WIFI_PASSWORD "KiChaoBro**"
#define QGC_IP        "255.255.255.255"   // Broadcast to all devices on network
#define QGC_PORT      14550               // QGroundControl UDP port

// ====== Globals ======
WiFiUDP udp;
HardwareSerial SerialGPS(2);  // UART2 for optional GPS
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

unsigned long lastHeartbeat = 0;
unsigned long lastStatus = 0;
unsigned long lastGps = 0;

const int system_id = 1;
const int component_id = MAV_COMP_ID_AUTOPILOT1;

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  delay(1000);  // Give USB some time

  Serial.println("Booting ESP32...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to Wi-Fi");
  Serial.println("Starting UDP...");
  udp.begin(QGC_PORT);
  Serial.println("Init complete.");
}

// ====== Loop ======
void loop() {

  Serial.println("Loop running...");
  delay(1000);
  unsigned long now = millis();

  // 1. Heartbeat @ 1Hz
  if (now - lastHeartbeat > 1000) {
    lastHeartbeat = now;
    mavlink_msg_heartbeat_pack(
      system_id, component_id, &msg,
      MAV_TYPE_QUADROTOR,
      MAV_AUTOPILOT_GENERIC,
      MAV_MODE_GUIDED_ARMED,
      0,
      MAV_STATE_ACTIVE
    );
    sendMavlinkMessage();
  }

  // 2. System Status @ 0.5Hz
  if (now - lastStatus > 2000) {
    lastStatus = now;
    uint32_t sensors = MAV_SYS_STATUS_SENSOR_3D_GYRO |
                       MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                       MAV_SYS_STATUS_SENSOR_GPS;
    mavlink_msg_sys_status_pack(
      system_id, component_id, &msg,
      sensors, sensors, sensors,
      600,        // CPU Load (600 = 60.0%)
      12600,      // Battery Voltage (12.6V in mV)
      -1,         // Current (not supported)
      85,         // Battery %
      0, 0, 0, 0, 0, 0
    );
    sendMavlinkMessage();
  }

  // 3. Fake GPS @ 5Hz
  if (now - lastGps > 200) {
    lastGps = now;
    mavlink_msg_gps_raw_int_pack(
      system_id, component_id, &msg,
      micros(),          // timestamp
      3,                 // fix type: 3D
      340532210,         // lat (34.053221 * 1E7)
      -1182425510,       // lon (-118.242551 * 1E7)
      15000,             // alt (15m * 1000)
      120,               // eph (cm)
      150,               // epv (cm)
      15,                // velocity (cm/s)
      27000,             // course (degrees * 100)
      10                 // satellites visible
    );
    sendMavlinkMessage();
  }

  delay(1);  // Prevent starvation
}

// ====== Send MAVLink Packet Over UDP ======
void sendMavlinkMessage() {
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  udp.beginPacket(QGC_IP, QGC_PORT);
  udp.write(buf, len);
  udp.endPacket();
}
