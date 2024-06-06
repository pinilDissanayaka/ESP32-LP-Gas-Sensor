#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "Dialog 4G"
#define WIFI_PASSWORD "A5LB331H1G1"

// Insert Firebase project API Key
#define API_KEY "AIzaSyCMsRoYBGZnnttlDbtScuMfhHfqYnF57R0"
// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://gassensor-3568d-default-rtdb.asia-southeast1.firebasedatabase.app/" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

byte MQ6_Pin = A0;          /* Define A0 for MQ Sensor Pin */
float Referance_V = 3300.0; /* ESP32 Referance Voltage in mV */
float RL = 1.0;             /* In Module RL value is 1k Ohm */
float Ro = 10.0;            /* The Ro value is 10k Ohm */
float mVolt = 0.0;
const float Ro_clean_air_factor = 10.0;
int LPG_threshold=12;

void setup() {
  Serial.begin(115200);       /* Set baudrate to 9600 */
  pinMode(2, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(4, OUTPUT);

  pinMode(MQ6_Pin, INPUT);  /* Define A0 as a INPUT Pin */
  delay(500);
  Serial.println("Wait for 30 sec warmup");
  delay(30000);             /* Set the warmup delay wich is 30 Sec */
  Serial.println("Warmup Complete");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    digitalWrite(2, HIGH);
    delay(300);
    digitalWrite(2, LOW);
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  digitalWrite(2, HIGH);

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  for(int i=0; i<30; i++){
    mVolt += Get_mVolt(MQ6_Pin);
  }
  mVolt = mVolt /30.0;      /* Get the volatage in mV for 30 Samples */
  Serial.print("Voltage at A0 Pin = ");
  Serial.print(mVolt);
  Serial.println("mVolt");
  Serial.print("Rs = ");
  Serial.println(Calculate_Rs(mVolt));
  Ro = Calculate_Rs(mVolt) / Ro_clean_air_factor;
  Serial.print("Ro = ");
  Serial.println(Ro);
  Serial.println(" ");
  mVolt = 0.0;
  digitalWrite(15, HIGH);
  bot.sendMessage(CHAT_ID, "Bot started up", "");

}

void loop() {
  for(int i=0; i<500; i++){
    mVolt += Get_mVolt(MQ6_Pin);
  }
  mVolt = mVolt/500.0;      /* Get the volatage in mV for 500 Samples */
  Serial.print("Voltage at A0 Pin = ");
  Serial.print(mVolt);      /* Print the mV in Serial Monitor */
  Serial.println(" mV");

  float Rs = Calculate_Rs(mVolt);
  Serial.print("Rs = ");
  Serial.println(Rs);       /* Print the Rs value in Serial Monitor */
  float Ratio_RsRo = Rs/Ro;

  Serial.print("RsRo = ");
  Serial.println(Ratio_RsRo);

  Serial.print("LPG ppm = ");
  unsigned int LPG_ppm = LPG_PPM(Ratio_RsRo);
  Serial.println(LPG_ppm);   /* Print the Gas PPM value in Serial Monitor */
 
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 800 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    Firebase.RTDB.setInt(&fbdo, "Anu/LPG_ppm", LPG_ppm);
    Serial.println("Write on firebase");
  }
  Serial.println("");


  if(LPG_ppm >=LPG_threshold){
    digitalWrite(15, LOW);
    digitalWrite(4, HIGH);
  }
  else{
    digitalWrite(15, HIGH);
  }

  mVolt = 0.0;              /* Set the mVolt variable to 0 */
}


float Calculate_Rs(float Vo) {
/* 
 *  Calculate the Rs value
 *  The equation Rs = (Vc - Vo)*(RL/Vo)
 */
  float Rs = (Referance_V - Vo) * (RL / Vo); 
  return Rs;
}


unsigned int LPG_PPM(float RsRo_ratio) {
/*
 * Calculate the PPM using below equation
 * LPG ppm = [(Rs/Ro)/18.446]^(1/-0.421)
 */
  float ppm;
  ppm = pow((RsRo_ratio/18.446), (1/-0.421));
  return (unsigned int) ppm;
}


float Get_mVolt(byte AnalogPin) {
/* Calculate the ADC Voltage using below equation
 *  mVolt = ADC_Count * (ADC_Referance_Voltage / ADC_Resolution)
 */
  int ADC_Value = analogRead(AnalogPin); 
  delay(1);
  float mVolt = ADC_Value * (Referance_V / 4096.0);
  return mVolt;
}