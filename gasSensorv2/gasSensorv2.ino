#include <SoftwareSerial.h>

byte MQ6_Pin = A0;          /* Define A0 for MQ Sensor Pin */
float Referance_V = 3300.0; /* ESP32 Referance Voltage in mV */
float RL = 1.0;             /* In Module RL value is 1k Ohm */
float Ro = 10.0;            /* The Ro value is 10k Ohm */
float mVolt = 0.0;
const float Ro_clean_air_factor = 10.0;
int LPG_threshold=12;


//Create software serial object to communicate with SIM800L
SoftwareSerial mySerial(3, 2); //SIM800L Tx & Rx is connected to Arduino #3 & #2


void setup() {
  Serial.begin(9600);       /* Set baudrate to 9600 */
  mySerial.begin(9600);  //Begin serial communication with Arduino and SIM800L

  Serial.println("Initializing...");
  delay(1000);

  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  mySerial.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();


  pinMode(2, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(4, OUTPUT);

  pinMode(MQ6_Pin, INPUT);  /* Define A0 as a INPUT Pin */
  delay(500);
  Serial.println("Wait for 30 sec warmup");
  delay(30000);             /* Set the warmup delay wich is 30 Sec */
  Serial.println("Warmup Complete");

  digitalWrite(2, HIGH);

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

}

void loop() {

  updateSerial();

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

  
 

  Serial.println("");


  if(LPG_ppm >=LPG_threshold){
    sendText();
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

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}


void sendText(){
  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  mySerial.println("AT+CMGS=\"+ZZxxxxxxxxxx\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  mySerial.print("Last Minute Engineers | lastminuteengineers.com"); //text content
  updateSerial();
  mySerial.write(26);
}