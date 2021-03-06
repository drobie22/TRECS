

#include "Servo.h" 

#define ESC_PIN         (6)   // PWM pin for signaling ESC
#define DRIVE_PIN       (10)  // Drive pin for power MOSFET

Servo ESC;  
int speed = 0;

void arm(){
  Serial.print("Arming ESC... ");
  digitalWrite(DRIVE_PIN, LOW);   // Disconnect ESC from power
  delay(500);                     // Wait 500ms for ESC to power down
  setSpeed(0);                    // Set speed to 0
  digitalWrite(DRIVE_PIN, HIGH);  // Reconnect ESC to power
  delay(2500);                    // 2.5 second delay for ESC to respond
  Serial.println("Arming complete");
}

/* Callibrate ESC's PWM range for first use */
void callibrate() {
  Serial.print("Calibrating ESC... ");
  digitalWrite(DRIVE_PIN, LOW);   // Disconnect ESC from power
  delay(500);                     // Wait 500ms
  setSpeed(1000);                 // Request full speed
  digitalWrite(DRIVE_PIN, HIGH);  // Reconnect ESC to power
  delay(5000);                    // Wait 5 seconds
  setSpeed(0);                    // Request 0 speed
  delay(8000);                    // Wait 8 seconds
  Serial.println("Calibration complete");
}

void setSpeed(int input_speed){
  //Sets servo positions to different speed
  int us = map(input_speed, 0, 1000, 1000, 2000);
  ESC.writeMicroseconds(us);
}

#define SENSOR_PIN    (A0)
#define BUFFER_SIZE   (2)

int sensorVal;

volatile float filterBuffer[BUFFER_SIZE] = {0};
volatile float filteredVal = 0.0;
volatile int index = 0;

/*
 * Lowpass moving average filter to
 * smooth analog sensor readings
 */
float filter(float value) {
  // Remove oldest value from moving average
  filteredVal -= filterBuffer[index] / BUFFER_SIZE;

  // Add new value to buffer and incrememnt index
  filterBuffer[index++] = value;

  // Add new value to moving average
  filteredVal += value / BUFFER_SIZE;

  // Prevent index out of bounds errors
  index %= BUFFER_SIZE;

  return filteredVal;
}
unsigned long myTime;
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
 // Configure MOSFET drive pin
  pinMode(DRIVE_PIN, OUTPUT);
  digitalWrite(DRIVE_PIN, LOW);

  // Attach ESC to designated pin.
  ESC.attach(ESC_PIN);

  // Arm ESC
  arm();
}

void loop() {
  if (Serial.available()) {
    // Check for calibration request
    if (Serial.peek() == 'c') {
      Serial.println("Callibrating ESC");
      callibrate();
      Serial.read();
    }
    // Otherwise, interpret as new throttle value
    else {
      speed = Serial.parseInt();
      Serial.println(speed);
      setSpeed(speed);
    }
}
 sensorVal = analogRead(SENSOR_PIN);
  myTime = millis();
  Serial.print(myTime);
  Serial.print("          ");
  Serial.print(speed);
  Serial.print("          ");
  Serial.println(filter(-0.3656*sensorVal+185.64), 2);
  delay(10);
}
