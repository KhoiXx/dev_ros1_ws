
#include <MPU9250_WE.h>
#include <Wire.h>
MPU9250_WE myMPU9250 = MPU9250_WE(0x68);

xyzFloat accValue;
xyzFloat gyrValue;
xyzFloat magValue;
float deg2rad = 3.14 / 180; // pi : 180deg

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
  
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_8HZ);
  delay(100);
}

void loop() {
  read_imu();
  if (Serial.available()) {
    if (char(Serial.read()) == '1')
      send_imu();
  }
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
  accValue = myMPU9250.getGValues();
  gyrValue = myMPU9250.getGyrValues();
  //myMPU9250.startMagMeasurement(); //only needed for triggered mode
  magValue = myMPU9250.getMagValues();
}

void send_imu() {
  String data = "";
  concatString(data, "", magValue.x, ",");
  concatString(data, "", magValue.y, ",");
  concatString(data, "", magValue.z, ",");
  concatString(data, "", accValue.x * 9.8, ",");
  concatString(data, "", accValue.y * 9.8, ",");
  concatString(data, "", accValue.z * 9.8, ",");
  concatString(data, "", gyrValue.x * deg2rad, ",");
  concatString(data, "", gyrValue.y * deg2rad, ",");
  concatString(data, "", gyrValue.z * deg2rad, "");
  Serial.println(data);
}

void concatString(String &output, String first, float data, String last) {
  output += first + String(data) + last;
}
