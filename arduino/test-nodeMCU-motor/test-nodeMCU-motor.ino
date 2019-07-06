// Menggunakan NODEMCU - contoh mengontrol PWM
// dan menggunakan NODEMCU Shield
// Contoh terinspirasi dari
// https://hackaday.io/project/8856-incubator-controller/log/29291-node-mcu-motor-shield



#include <Wire.h> 

#define DIRA 0
#define PWMA 5
#define DIRB 2 
#define PWMB 4

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting...");

  Serial.println("Initialising LCD...");

//  Serial.println("Flashing internal LED");
//  pinMode(BUILTIN_LED, OUTPUT);
//  digitalWrite(BUILTIN_LED, LOW);
//  delay(100);
//  digitalWrite(BUILTIN_LED, HIGH);
//  delay(300);


}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Preparing motor...");
  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
  digitalWrite(DIRA,1);
  digitalWrite(DIRB,1);
  delay(5000);
  
  Serial.println("Starting motor...");
  analogWrite(PWMA,1000);
  analogWrite(PWMB,1000);
  delay(3000);

  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
  delay(3000);

  digitalWrite(DIRA,0);
  digitalWrite(DIRB,0);
  
  analogWrite(PWMA,1000);
  analogWrite(PWMB,1000);
  delay(3000);
  
}
