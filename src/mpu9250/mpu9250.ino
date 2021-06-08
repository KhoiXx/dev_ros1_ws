
#include <MPU9250_WE.h>
#include <Wire.h>
MPU9250_WE myMPU9250 = MPU9250_WE(0x68);
#define TRIG_PIN 7
#define ECHO_PIN 6
#define TRIG_PIN_1 9
#define ECHO_PIN_1 8
#define TIME_OUT 5000

xyzFloat accValue;
xyzFloat gyrValue;
xyzFloat magValue;
xyzFloat pre_accValue;
float deg2rad = 3.14 / 180; // pi : 180deg
float is_obstacle, is_package;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }

  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done intializing MPU9250!");
  //myMPU9250.setAccOffsets(105.0, 171.0, 452.0, 521.0, 166508.0-16384.0, 166824-16384.0);
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_8HZ);
  delay(100);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
}

void loop() {
  read_imu();
  is_obstacle = check_obstacle(TRIG_PIN, ECHO_PIN);
  is_package = check_obstacle(TRIG_PIN_1, ECHO_PIN_1);
  if (Serial.available()) {
    if (char(Serial.read()) == '1')
      send_imu();
    else if (char(Serial.read()) == '2'){
      myMPU9250.init();
      myMPU9250.initMagnetometer();
      delay(500);
      myMPU9250.autoOffsets();}
  }
//  send_imu();
//  delay(20);
  // Serial.print(accValue.x);;
  // Serial.print("   ");
  // Serial.print(accValue.y);
  // Serial.print("   ");
  // Serial.println(accValue.z);

  // Serial.print(gyrValue.x);
  // Serial.print("   ");
  // Serial.print(gyrValue.y);
  // Serial.print("   ");
  // Serial.println(gyrValue.z);

  // Serial.print(magValue.x);
  // Serial.print("   ");
  // Serial.print(magValue.y);
  // Serial.print("   ");
  // Serial.println(magValue.z);
  // delay(100);

}

void read_imu() {
  pre_accValue = accValue;
  accValue = myMPU9250.getGValues();
  gyrValue = myMPU9250.getGyrValues();
  //myMPU9250.startMagMeasurement(); //only needed for triggered mode
  magValue = myMPU9250.getMagValues();
  accValue.x *= 9.8;
  accValue.y *= 9.8;
  accValue.z *= 9.8;
//  if (abs(accValue.x - pre_accValue.x) < 0.02 && abs(accValue.x <=0.05)){
//    accValue.x = 0.00;
//  }
//  if (abs(accValue.y - pre_accValue.y) < 0.02 && abs(accValue.y <=0.09)){
//    accValue.y = 0.00;
//  }
}

void send_imu() {
  String data = "";
  concatString(data, "", magValue.x, ",");
  concatString(data, "", magValue.y, ",");
  concatString(data, "", magValue.z, ",");
  concatString(data, "", accValue.x, ",");
  concatString(data, "", accValue.y, ",");
  concatString(data, "", accValue.z, ",");
  concatString(data, "", gyrValue.x * deg2rad, ",");
  concatString(data, "", gyrValue.y * deg2rad, ",");
  concatString(data, "", gyrValue.z * deg2rad, ",");
  concatString(data, "", is_obstacle, ",");
  concatString(data, "", is_package, "");
  Serial.println(data);
}

void concatString(String &output, String first, float data, String last) {
  output += first + String(data) + last;
}

float check_obstacle(int trig, int echo)
{
  long duration, distance;
  short boolean;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  duration = pulseIn(echo, HIGH, TIME_OUT);
 
  // convert to distance
  distance = duration / 29.1 / 2;
  if(distance < 15 & distance>0){
    boolean = 1;
  } else boolean = 0;
  return boolean;
}
