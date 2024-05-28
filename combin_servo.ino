#include <LiquidCrystal.h>
#include <Servo.h>
#include <math.h>

#define MQ_PIN A0 
#define LED_PIN 13 

#define TRIGGER_PIN_1 2 // Digital pin for ultrasonic sensor 1 trigger
#define ECHO_PIN_1 3    // Digital pin for ultrasonic sensor 1 echo

#define TRIGGER_PIN_2 4 // Digital pin for ultrasonic sensor 2 trigger
#define ECHO_PIN_2 5    // Digital pin for ultrasonic sensor 2 echo

#define CALIBRATION_SAMPLE_TIMES     (50)   
#define CALIBRATION_SAMPLE_INTERVAL  (500)   
#define READ_SAMPLE_INTERVAL         (50)    
#define READ_SAMPLE_TIMES            (5)     

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 

Servo servoMotor; 

float LPGCurve[3] = {2.3, 0.21, -0.47}; 
#define GAS_LPG 0
#define RL_VALUE 5.0 
#define RO_CLEAN_AIR_FACTOR 9.83 

float Ro = 10.0; 

void setup() {
  Serial.begin(9600);    
  lcd.begin(16, 2);      
  pinMode(LED_PIN, OUTPUT); 
  
  servoMotor.attach(9); // Attach the servo to pin 9

  pinMode(TRIGGER_PIN_1, OUTPUT); // Set trigger pin for ultrasonic sensor 1 as output
  pinMode(ECHO_PIN_1, INPUT);     // Set echo pin for ultrasonic sensor 1 as input
  
  pinMode(TRIGGER_PIN_2, OUTPUT); // Set trigger pin for ultrasonic sensor 2 as output
  pinMode(ECHO_PIN_2, INPUT);     // Set echo pin for ultrasonic sensor 2 as input
}

void loop() {
  float lpg_ppm = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_LPG);
  
  long duration_1 = 0;
  long distance_1 = 0;
  
  digitalWrite(TRIGGER_PIN_1, LOW);   // Ensure pin is low
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN_1, HIGH);  // Send 10 microsecond pulse to trigger
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_1, LOW);

  duration_1 = pulseIn(ECHO_PIN_1, HIGH, 10000); 

  if (duration_1 == 0) {
    distance_1 = 0;
  } else {
    distance_1 = (duration_1 / 2) / 29.1; // Convert the duration to distance (cm)
  }

  long duration_2 = 0;
  long distance_2 = 0;
  
  digitalWrite(TRIGGER_PIN_2, LOW);   // Ensure pin is low
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN_2, HIGH);  // Send 10 microsecond pulse to trigger
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_2, LOW);

  duration_2 = pulseIn(ECHO_PIN_2, HIGH, 10000); 

  if (duration_2 == 0) {
    distance_2 = 0;
  } else {
    distance_2 = (duration_2 / 2) / 29.1; // Convert the duration to distance (cm)
  }

  bool gasDetected = (lpg_ppm > 1000);
  bool objectDetected_1 = (distance_1 == 0 || distance_1 == 100); // Object detected if distance is 0cm or 200cm
  bool objectDetected_2 = (distance_2 < 200 && distance_2 < 26); // Object detected if distance is more than 200cm

  // Output whether the room is vacant or not along with the distance for sensor 1
  if (objectDetected_1) {
    // Room is vacant
    Serial.print("Room is vacant for sensor 1 at distance: ");
    Serial.print(distance_1);
    Serial.println(" cm");
    // Turn on servo motor based on gas detection or object detection for either sensor
    if (gasDetected || objectDetected_2) {
      servoMotor.write(90); // Turn on servo motor
    } else {
      servoMotor.write(0); // Turn off servo motor
    }
  } 
  if (!objectDetected_1) {
    // Room is not vacant
    Serial.print("Room is occupied for sensor 1 at distance: ");
    Serial.print(distance_1);
    Serial.println(" cm");
    if (gasDetected || objectDetected_2) {
      servoMotor.write(0); // Turn off servo motor
    } else {
      servoMotor.write(0); // Turn off servo motor
    }
  } 

  // Output gas concentration and distance
  Serial.print("LPG ppm: ");
  Serial.println(lpg_ppm);
  Serial.print("Distance Sensor 2 cm: ");
  Serial.println(distance_2);

  delay(3000);
}


float MQResistanceCalculation(int raw_adc) {
  return ((float)RL_VALUE * (1023 - raw_adc) / raw_adc);
}

float MQRead(int mq_pin) {
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  switch (gas_id) {
    case GAS_LPG:
      return MQGetPercentage(rs_ro_ratio, LPGCurve);
    default:
      return 0;
  }
}

int MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10, (((log10(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}