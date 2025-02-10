#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_ICM20948.h>
#include <rn2xx3.h>

#define SDA_PIN 10
#define SCL_PIN 11
#define CS_PIN 14
#define SCK_PIN 13
#define SDO_PIN 15 // MOSI
#define SDI_PIN 12 // MISO
#define INT_PIN 2
#define FAN_PIN 6 
#define RX 4
#define TX 3
#define Reset 5

SoftwareSerial mySerial(RX, TX); // RX, TX
rn2xx3 myLora(mySerial);

Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_ICM20948 icm;

//#define ACCEL_THRESHOLD 10.0
//#define GYRO_THRESHOLD 0.09

const float GYRO_THRESHOLD = 0.05; // Example value, adjust as needed
const float ACCEL_THRESHOLD = 10.0; // Example value, adjust as needed

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(9600);
  mySerial.begin(57600); //serial port to radio
  while (!Serial) delay(10);

  if (!sht31.begin(0x45)) {
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }

  if (!icm.begin_SPI(CS_PIN, SCK_PIN, SDO_PIN, SDI_PIN)) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) { delay(10); }
  }
  Serial.println("ICM20948 Found!");

  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);

  pinMode(INT_PIN, INPUT);
  pinMode(FAN_PIN, OUTPUT); // Set the fan pin as output

  initialize_radio();
}

void loop() {
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  if (!isnan(t)) {
   // Serial.print("Temp *C = "); Serial.println(t);
    if (t > 26) {
      digitalWrite(FAN_PIN, HIGH); 
      Serial.println("FAN ON!");
    } else {
      digitalWrite(FAN_PIN, LOW); 
      Serial.println("FAN OFF!");
    }
  } else {
    Serial.println("Failed to read temperature");
  }

  if (!isnan(h)) {
   // Serial.print("Hum. % = "); Serial.println(h);
  } else {
    Serial.println("Failed to read humidity");
  }

  sensors_event_t accel, gyro, mag;
  icm.getEvent(&accel, &gyro, &mag);


  // Calculate the magnitude of the acceleration vector
  float accelMagnitude = sqrt(pow(accel.acceleration.x, 2) + 
                              pow(accel.acceleration.y, 2) + 
                              pow(accel.acceleration.z, 2));
  
  // Calculate if gyroscopic motion is above the threshold
  bool gyroMotionDetected = (abs(gyro.gyro.x) > GYRO_THRESHOLD || 
                             abs(gyro.gyro.y) > GYRO_THRESHOLD || 
                             abs(gyro.gyro.z) > GYRO_THRESHOLD);

  // Wait for half a second
  //delay(500);

  // Check if either the acceleration or gyroscopic motion exceeds their thresholds
  if (accelMagnitude > ACCEL_THRESHOLD || gyroMotionDetected) {
    Serial.println("Motion detected!");
    String motionType = gyroMotionDetected ? "Gyro" : "Accel";
    String data = "Temp: " + String(t) + ", Hum: " + String(h) + ", MotionType: " + motionType;
    myLora.tx(data);
  } else {
    String data = "Temp: " + String(t) + ", Hum: " + String(h) + ", MotionType: " + String(0);
    myLora.tx(data);
  }
}



void initialize_radio() {
  // Reset rn2483
  pinMode(Reset, OUTPUT);
  digitalWrite(Reset, LOW);
  delay(500);
  digitalWrite(Reset, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  mySerial.flush();

  //Autobaud the rn2483 module to 9600. The default would otherwise be 57600.
  myLora.autobaud();

  //check communication with radio
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(myLora.hweui());
  Serial.println("RN2xx3 firmware version:");
  Serial.println(myLora.sysver());

  //configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;

  /*
   * OTAA: initOTAA(String AppEUI, String AppKey);
   * If you are using OTAA, paste the example code from the TTN console here:
   */
  const char *appEui = "0000000000000000";
  const char *appKey = "92DFAC2AF26C5843CF04F87805BC76D3";

  join_result = myLora.initOTAA(appEui, appKey);

  while(!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(10000); //delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");
}
