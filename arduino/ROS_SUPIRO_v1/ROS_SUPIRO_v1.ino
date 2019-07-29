/*
 * rosserial Publisher Example
 * This intend to connect to a Wifi Access Point
 * and a rosserial socket server.
 * You can launch the rosserial socket server with
 * roslaunch rosserial_server socket.launch
 * The default port is 11411
 *
 */
#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <supiro_lite/motorpower.h>
#include <supiro_lite/encoder.h>
#include <supiro_lite/sonar.h>
#
#include <Wire.h> 
#include <Servo.h>


//DEFINE CONFIG
#define DECELERATE 0
#define DEBUG 1

// DEFINE PIN

#define DIRA 0
#define PWMA 5
#define DIRB 2 
#define PWMB 4
#define L_ENC D7
#define R_ENC D6
#define SERVPIN D8

// defines pins numbers SONAR
const int trigPin = 1;
const int echoPin = 3;


double lpwm=0, rpwm=0, lv=0, rv=0, lvt=0, rvt=0;                     // motor pwm output and calculated velocities (lv as tics/sec, lvt as tic/(time interval))
int lmc=0, rmc=0, lmc0=0, rmc0=0, ldir=1, rdir=1, llevel=1, rlevel=1;  // l=left, r=right motors encoder params
double lIn,rIn,lOut,rOut,lSet=0,rSet=0;                                // PID Input Signal, Output command and Setting speed for each wheel 
int period = 50;                                   // PID period in milliseconds
int rcurrenttime, rlasttime, lcurrenttime, llasttime;
os_timer_t myTimer;
double WheelSeparation= 0.135;                              // wheel separation in meters
int servpos=0;                            // wheel separation in meters
// defines variables SONAR
long duration;
int distance;
int cntr = 0;
int lconn = 0;

const char* ssid     = "IRQNET";
const char* password = "zhafirah";
// Set the rosserial socket server IP address
IPAddress server(192,168,30,50);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards


ros::NodeHandle nh;
// Make a encoder publisher

supiro_lite::encoder enc_msg;
ros::Publisher encoder("encoder", &enc_msg);

supiro_lite::sonar sonar_msg;
ros::Publisher sonar("sonar", &sonar_msg);

std_msgs::Int16 int_msg;

void cmd_motor( const supiro_lite::motorpower& CVel){

    lSet = CVel.leftpwr;
    rSet = CVel.rightpwr;
    ldir = CVel.leftdir;
    rdir = CVel.rightdir;
    
}

void cmd_servo( const std_msgs::Int16& CSrv){
  servpos = CSrv.data;
  int pos = servpos;
  myservo.write(pos);              // tell servo to go to position in variable 'pos'
}

ros::Subscriber<supiro_lite::motorpower> Sub("/motorpwr", &cmd_motor );

ros::Subscriber<std_msgs::Int16> Serv("/servo", &cmd_servo );
// Be polite and say hello



void tic(void *pArg) {    // timerCallback, repeat every "period"

  if (lSet!=0) {lpwm = abs(lSet);
                lSet = 0;}
  if (rSet!=0) {rpwm = abs(rSet);
                rSet = 0;}

  motion(lpwm,rpwm, ldir, rdir); 

  if (DECELERATE) {
    if (lpwm>0) {lpwm = lpwm - 20;}
    if (rpwm>0) {rpwm = rpwm - 20;}
  }
  
  if (lpwm<0) {lpwm = 0;}
  if (rpwm<0) {rpwm = 0;}
}

void motion(int lpwm, int rpwm, int ldir, int rdir) {  // move motor at pwm power and change directions flags only when motor cross stop

  analogWrite(PWMA, abs(lpwm));
  analogWrite(PWMB, abs(rpwm));
  digitalWrite(DIRA, ldir);
  digitalWrite(DIRB, rdir);

}


void L_enc_Callback(){
  
  lcurrenttime = millis();
  lvt = 1*1000./(lcurrenttime - llasttime);
  llasttime = lcurrenttime;
  lmc=lmc+1;
}

void R_enc_Callback(){ 
  rcurrenttime = millis();
  rvt = 1*1000./(rcurrenttime - rlasttime);
  rlasttime = rcurrenttime;
  rmc=rmc+1;
}

void setup()
{
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    //cntr = cntr + 1;
    //if (cntr > 50) { cntr = 0;
    //                 ESP.restart();}
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  // Start to be polite
  nh.advertise(encoder);
  nh.advertise(sonar);
  nh.subscribe(Sub);
  nh.subscribe(Serv);


  // initialize PIN
  pinMode(DIRA, OUTPUT); //D0
  pinMode(PWMA, OUTPUT); //D5
  pinMode(DIRB, OUTPUT); //D2
  pinMode(PWMB, OUTPUT); //D4
  attachInterrupt(digitalPinToInterrupt(L_ENC), L_enc_Callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC), R_enc_Callback, CHANGE);
  myservo.attach(SERVPIN);  // attaches the servo on D8 to the servo object
  pinMode(trigPin, OUTPUT); //sonar
  pinMode(echoPin, INPUT);  //sonar
  
  // configure timer
  os_timer_setfn(&myTimer, tic, NULL); 
  os_timer_arm(&myTimer, period, true);   // timer in ms

} 

void loop()
{

  if (nh.connected()) {
    Serial.println("Connected");
    // publish encoder value
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    enc_msg.leftenc = lmc;
    enc_msg.rightenc = rmc;
    encoder.publish( &enc_msg );
    // reset encoder after publish
    lmc = 0;
    rmc = 0;

    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    distance= duration*0.034/2;


    sonar_msg.servopos = servpos;
    sonar_msg.pingval = distance;
    sonar.publish( &sonar_msg );
    
  } else {
    Serial.println("Not Connected");
    cntr = cntr + 1;
    if (cntr > 100) { cntr = 0;
                    ESP.restart();}
  }
  nh.spinOnce();
  // Loop exproximativly at 1Hz
  delay(200);
}
