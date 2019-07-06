// Contoh Penggunaan INTERUPT pada NODE MCU
// untuk ENCODER MH-Sensor-Series
// Inspirasi dari url:
// https://www.electronicwings.com/nodemcu/nodemcu-gpio-interrupts-with-arduino-ide


uint8_t GPIO_Pin = D7;

void setup() {
 Serial.begin(115200);
 attachInterrupt(digitalPinToInterrupt(GPIO_Pin), IntCallback, CHANGE);
 Serial.print("ready");
}

void loop() {
}

void IntCallback(){
 Serial.print("Stamp(ms): ");
 Serial.println(millis());
}
