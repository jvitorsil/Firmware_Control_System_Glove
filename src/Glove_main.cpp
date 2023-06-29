/**
**************************************************************************************************************
* @file Gloove_main.cpp
* @author João Vitor Silva <joaovitor_s2015@ufu.br>
* @version V0.1.0
* @date 26-Jun-2023
* @brief code for PET project - HOOK 4º ed.
*************************************************************************************************************
*/

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <esp_now.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//#include <Wire.h>
#include "WiFi.h"

/* Constants ---------------------------------------------------------------------------------------------------------*/

static float filteredValue = 0;
float_t softFactor = 0.2;

uint8_t currentButtonState = HIGH;
uint8_t oldButtonState = HIGH;


/* Pin numbers -------------------------------------------------------------------------------------------------------*/
#define SDA_PIN 21
#define SCL_PIN 22
#define Flex_PIN 32
#define Goniometer_PIN 34
#define Pushbutton_PIN 35

/* Private variables -------------------------------------------------------------------------------------------------*/
Adafruit_MPU6050 mpu;
uint8_t broadcastAddress[] = {0x8, 0x3A, 0xF2, 0xBA, 0x64, 0x68}; // {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; Cadastrando o MAC do ESP receptor no ESP remetente
esp_now_peer_info_t peerInfo;

typedef struct struct_message{
  float_t pitch;
  float_t roll;
  uint16_t flexValue;
  uint16_t goniometerValue;
  uint8_t buttonState;

} struct_message;

struct_message myData;

/* Private functions -------------------------------------------------------------------------------------------------*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);


/* Main Application --------------------------------------------------------------------------------------------------*/
void setup() 
{

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  pinMode(Flex_PIN, INPUT);
  pinMode(Pushbutton_PIN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!mpu.begin())
    while (1)
      delay(10);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  if (esp_now_init() != ESP_OK)
    return;
  
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  myData.pitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  myData.roll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  myData.goniometerValue = analogRead(Goniometer_PIN);
  filteredValue = (softFactor * myData.goniometerValue) + ((1 - softFactor) * filteredValue);

  currentButtonState = digitalRead(Pushbutton_PIN);
  if(currentButtonState != oldButtonState && currentButtonState)
    myData.buttonState = !myData.buttonState;

  oldButtonState = currentButtonState;

  myData.flexValue = analogRead(Flex_PIN);

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  Serial.print("Pitch: " + String(myData.pitch) + " | Roll: " + String(myData.roll) + " | Goniometer R: " + String(myData.goniometerValue) + " | Push Button: " + String(myData.buttonState) + " | Flex Sensor: " + String(myData.flexValue));

  delay(500);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}