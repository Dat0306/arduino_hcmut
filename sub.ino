#define START_BYTE '<'
#define END_BYTE '>'
#define MAX_PACKET_LEN 64
#include <Servo.h>
#include <AccelStepper.h>

// Cấu hình chân stepper
#define ENABLE_PIN 
#define DIR_PIN 
#define STEP_PIN 

// -------------------------------
// Global Variables & Constants
// -------------------------------
uint32_t lastSerialSend = 0;            // Timestamp for last serial transmission
const uint16_t SERIAL_SEND_INTERVAL = 15; // ~15ms interval (~66Hz update rate)

const float LINE_FOLLOW_SPEED_FACTOR = 0.6; // Limit for line following speed (60% of max)

char packetBuffer[MAX_PACKET_LEN];
uint8_t packetIndex = 0;
bool packetInProgress = false;
uint8_t currentChecksum = 0;

bool lineFollowingMode = false; // Flag for line following mode

// Stepper
const int MICROSTEPS = 4;
const int STEPS_PER_REV = 200;
const float MM_PER_REV = 4.0; // Trục vít me 4mm/vòng
const float STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / MM_PER_REV;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Servo tay gắp
#define SERVO_PIN 6       // Chân điều khiển servo
#define GRIPPER_OPEN_ANGLE 70
#define GRIPPER_CLOSE_ANGLE 120
Servo gripperServo;       // Đối tượng servo
bool gripperState = false; // false: đóng, true: mở

// -------------------------------
// Function Prototypes
// -------------------------------
void handleSerialComm();
void processPacket(char* data, uint8_t length);
void sendXYRData(int x, int y, int r);
void readLineSensors();
void calculateXYR(int* x, int* y, int* r);
void setMotorSpeeds(int16_t speeds[4]);

// -------------------------------
// Arduino Setup Function
// -------------------------------
void setup() {
  Serial.begin(115200);
  // Initialize sensors and other devices here
  // Stepper
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Kích hoạt driver
  stepper.setMaxSpeed(1000);     // steps/s
  stepper.setAcceleration(500);  // steps/s²
  // Servo tay gắp
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(GRIPPER_CLOSE_ANGLE); // Khởi động ở trạng thái đóng
}

// -------------------------------
// Arduino Main Loop
// -------------------------------
void loop() {
  handleSerialComm(); // Process incoming serial data
  // Duy trì chuyển động nếu đang chạy của stepper
  if (stepper.isRunning()) {
    stepper.run();
  }

  int x = 0, y = 0, r = 0; // Variables for sensor data

  if (lineFollowingMode) {
    // Update sensor readings and compute control values
    readLineSensors();
    calculateXYR(&x, &y, &r);

    // Ensure we send data at a controlled rate
    uint32_t currentMillis = millis();
    if (currentMillis - lastSerialSend >= SERIAL_SEND_INTERVAL) {
      lastSerialSend = currentMillis;

      // Apply speed limitation for line following
      x = round(x * LINE_FOLLOW_SPEED_FACTOR);
      y = round(y * LINE_FOLLOW_SPEED_FACTOR);
      r = round(r * LINE_FOLLOW_SPEED_FACTOR);

      sendXYRData(x, y, r); // Transmit computed data to master
    }
  }
}

// -------------------------------
// Control Functions 
// -------------------------------
// Stepper
void controlStepper(int speed) {
  digitalWrite(ENABLE_PIN, LOW); // Bật driver
  
  if (speed == 0) {
    stepper.stop();
  } else {
    stepper.setSpeed(speed);
    stepper.runSpeed(); // Chạy ở tốc độ cố định
  }
}
// Servo tay gắp
void controlGripper(bool open) {
  if (!gripperServo.attached()) return; // Kiểm tra servo đã kết nối chưa
  
  if (open) {
    gripperServo.write(GRIPPER_OPEN_ANGLE);
    gripperState = true;
    Serial.println("<G:1:OK>"); // Gửi trạng thái về Master
  } else {
    gripperServo.write(GRIPPER_CLOSE_ANGLE);
    gripperState = false;
    Serial.println("<G:0:OK>");
  }
  delay(300); // Đợi servo hoàn thành chuyển động
}

// -------------------------------
// Serial Communication Handler
// -------------------------------
void handleSerialComm() {
  while (Serial.available()) {
    char c = Serial.read();

    // Detect start of packet
    if (c == START_BYTE) {
      packetIndex = 0;
      currentChecksum = 0;
      packetInProgress = true;
      continue;
    }

    // Skip characters if no packet is in progress
    if (!packetInProgress) continue;

    // Detect end of packet and process it
    if (c == END_BYTE) {
      processPacket(packetBuffer, packetIndex);
      packetInProgress = false;
      return;
    }

    // Append character to buffer and update checksum if space allows
    if (packetIndex < MAX_PACKET_LEN - 1) {
      currentChecksum ^= c;
      packetBuffer[packetIndex++] = c;
    }
  }
}

// -------------------------------
// Packet Processing Function
// -------------------------------
void processPacket(char* data, uint8_t length) {
  // Validate checksum
  uint8_t checksum = (uint8_t)data[length - 1];
  uint8_t calcChecksum = currentChecksum ^ data[length - 1];
  
  if (calcChecksum != 0) return; // Discard invalid packet

  // Tokenize command from packet data
  char* command = strtok(data, ",:");
  if (!command) return;

  // Process commands from master
 
  // Additional commands can be handled here
   // Xử lý lệnh stepper
  if (strncmp(data, "S:", 2) == 0) {
    int speed = atoi(data + 2); // Đọc giá trị tốc độ
    controlStepper(speed);
  }
  else if (strncmp(data, "E:", 2) == 0) {
    stepper.stop(); // Dừng khẩn cấp
  }

  // Xử lý lệnh servo
  if (strncmp(data, "G:1", 3) == 0) {
    controlGripper(true);
    Serial.println("<G:1:OK>"); // Phản hồi
  } 
  else if (strncmp(data, "G:0", 3) == 0) {
    controlGripper(false);
    Serial.println("<G:0:OK>");
  }

}

// -------------------------------
// Data Transmission Function
// -------------------------------
void sendXYRData(int x, int y, int r) {
  char payload[20];
  snprintf(payload, sizeof(payload), "%d:%d:%d", x, y, r);
  
  // Calculate checksum for payload
  byte checksum = 0;
  for (int i = 0; payload[i] != '\0'; i++) {
    checksum ^= payload[i];
  }
  
  // Construct and send the packet
  char packet[30];
  snprintf(packet, sizeof(packet), "%c%s:%02X%c", START_BYTE, payload, checksum, END_BYTE);
  Serial.println(packet);
}

// -------------------------------
// Stub Functions (Implement as needed)
// -------------------------------
void readLineSensors() {
  // Implement sensor reading logic here
}

void calculateXYR(int* x, int* y, int* r) {
  // Compute XYR values from sensor data
  *x = 0;   // Default value (update based on actual algorithm)
  *y = 100; // Forward movement with moderate speed
  *r = 0;   // No rotation by default
}

