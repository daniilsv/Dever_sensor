#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include "DHT.h"        // including the library of DHT11 temperature and humidity sensor
#define DHTTYPE DHT11   // DHT 11

#define dht_dpin D7
DHT dht(dht_dpin, DHTTYPE);

uint8_t IMUAddress = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

const char* ssid     = "DVS";
const char* password = "11212232";

void setup() {
  Wire.begin(D1, D2); // sda, scl
  i2cWrite(0x6B, 0x00); // Disable sleep mode
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {  //Wait for the WiFI connection completion
    delay(500);
    Serial.println("Waiting for connection");
  }
  Serial.println("Connected!!");
  dht.begin();
  delay(1000);
}

int lastTemp = 0, temp = 0;
int lastHum = 0, hum = 0;

bool accel_failed = false;
bool antiddos = true;
int antiddost = 0;
void loop() {
  uint8_t* data = i2cRead(0x3B, 14);
  AcX = ((data[0] << 8) | data[1]);
  AcY = ((data[2] << 8) | data[3]);
  AcZ = ((data[4] << 8) | data[5]);
  Tmp = ((data[6] << 8) | data[7]);
  GyX = ((data[8] << 8) | data[9]);
  GyY = ((data[10] << 8) | data[11]);
  GyZ = ((data[12] << 8) | data[13]);
  int ary = -(180 / 3.141592) * atan(AcX / sqrt(AcY * AcY + AcZ * AcZ));
  int arx =  (180 / 3.141592) * atan(AcY / sqrt(AcX * AcX + AcZ * AcZ));
  AcX = AcX * (2 * 9.8 / 32767) - 1;
  AcY = AcY * (2 * 9.8 / 32767) - 1;
  AcZ = AcZ * (2 * 9.8 / 32767) - 10;


  double accel = (double)sqrt(AcX * AcX + AcY * AcY + AcZ * AcZ);

  if (accel > 30) {
    sendData("O", accel);
    sendTH();
  }
  if (!accel_failed && abs(arx) > 45 || abs(ary) > 45 || (abs(arx) > 25 && abs(ary) > 25)) {
    sendData("A", max(arx, ary));
    sendTH();
    accel_failed = true;
  } else {
    accel_failed = false;
    delay(200);
  }

  delay(50);

  int t = dht.readTemperature();
  if (lastTemp != t) {
    sendData("T", t);
    lastTemp = t;
    delay(25);
  }
  int h = dht.readHumidity();
  if (lastHum != h) {
    sendData("H", h);
    lastHum = h;
    delay(25);
  }
}
void sendTH() {
  int t = dht.readTemperature();
  int h = dht.readHumidity();
  sendData("T", t);
  lastTemp = t;
  sendData("H", h);
  lastHum = h;
}
void i2cWrite(uint8_t registerAddress, uint8_t data) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(); // Send stop
}
uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes];
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++)
    data [i] = Wire.read();
  return data;
}

void sendData(char *type, int data) {
  if (data > 180)
    return;
  if (WiFi.status() == WL_CONNECTED) {
    char *request = (char*)malloc(100 * sizeof(char));
    sprintf(request, "http://dever.itis.team/order/set_state/1/%s/%d", type, data);
    Serial.println(request);
    HTTPClient http;
    http.begin(request);
    int httpCode = http.GET();
        String payload = http.getString();
    Serial.println(payload);
  } else {
    Serial.println("Error in WiFi connection");
  }
}
