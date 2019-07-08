/*
 * rosserial Publisher Example
 * Prints "hello world!"
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
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <supiro_lite/motorpower.h>
#include <Wire.h> 
#include <PID_v1.h>

// DEFINE PIN
#define DEBUG 1
#define DIRA 0
#define PWMA 5
#define DIRB 2 
#define PWMB 4
#define L_ENC D7
#define R_ENC D6

double lpwm=0, rpwm=0, lv=0, rv=0, lvt=0, rvt=0;                     // motor pwm output and calculated velocities (lv as tics/sec, lvt as tic/(time interval))
int lmc=0, rmc=0, lmc0=0, rmc0=0, ldir=1, rdir=1, llevel=1, rlevel=1;  // l=left, r=right motors encoder params
double lIn,rIn,lOut,rOut,lSet=0,rSet=0;                                // PID Input Signal, Output command and Setting speed for each wheel 
int period = 50;                                   // PID period in milliseconds
int rcurrenttime, rlasttime, lcurrenttime, llasttime;
os_timer_t myTimer;
double WheelSeparation= 0.135;                              // wheel separation in meters
                            // wheel separation in meters

const char* ssid     = "IRQ-NET";
const char* password = "kupukupu";
// Set the rosserial socket server IP address
IPAddress server(192,168,0,145);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

sensor_msgs::Range range_msg;         // Ultrasonic Range message
std_msgs::Int16 int_msg;

void cmd_velCallback( const supiro_lite::motorpower& CVel){
  //geometry_msgs::Twist twist = twist_msg;   
//    double vel_x = CVel.linear.x;
//    double vel_th = CVel.angular.z;
//    double right_vel = 0.0;
//    double left_vel = 0.0;
//    Serial.println(vel_x);
//    // turning
//    if(vel_x == 0){  
//        right_vel = vel_th * WheelSeparation / 2.0;
//        left_vel = (-1) * right_vel;
//    }
//    // forward / backward
//    else if(vel_th == 0){ 
//        left_vel = right_vel = vel_x;
//    }
//    // moving doing arcs
//    else{ 
//        left_vel = vel_x - vel_th * WheelSeparation / 2.0;
//        right_vel = vel_x + vel_th * WheelSeparation / 2.0;
//    }
//    //write new command speeds to global vars 
//    lSet = left_vel;
//    rSet = right_vel;


    int lpower = CVel.leftpwr;
    int rpower = CVel.rightpwr;
    int ldir = CVel.leftdir;
    int rdir = CVel.rightdir;
    
    if(DEBUG){
      Serial.print("cmd_vel");
      Serial.print(lpower);
      Serial.print(",");
      Serial.println(rpower);
      Serial.println(",");
      Serial.print(ldir);
      Serial.print(",");
      Serial.println(rdir);

    }   
}

ros::Subscriber<supiro_lite::motorpower> Sub("/motorpwr", &cmd_velCallback );

// Be polite and say hello
char hello[13] = "hello world!";


void tic(void *pArg) {    // timerCallback, repeat every "period"

  if (lSet!=0) {lpwm = lSet * 1023;
                lSet = 0;}
  if (rSet!=0) {rpwm = rSet * 1023;
                rSet = 0;}

  motion(lpwm,rpwm); 

  if (lpwm>0) {lpwm = lpwm - 20;}
  if (rpwm>0) {rpwm = rpwm - 20;}

}

void motion(double lpwm, double rpwm) {  // move motor at pwm power and change directions flags only when motor cross stop
  if(abs(lIn)<1){
    if(lOut>=0){
      ldir=1; 
      llevel=HIGH; 
    } else {
      ldir=-1;
      llevel=LOW; 
    }
  }
  if(abs(rIn)<1){
    if(rOut>=0){
      rdir=1; 
      rlevel=HIGH; 
    } else {
      rdir=-1;
      rlevel=LOW; 
    }
  }
  analogWrite(PWMA, abs(lpwm));
  analogWrite(PWMB, abs(rpwm));
  digitalWrite(DIRA, llevel);
  digitalWrite(DIRB, rlevel);

  if(DEBUG){
  Serial.print(lpwm);
  Serial.print(":");
  Serial.print(rpwm);
  Serial.print(":");
  Serial.print(llevel);
  Serial.print(":");
  Serial.println(rlevel);
  }
}


void L_enc_Callback(){
  lcurrenttime = millis();
  lvt = ldir*1000./(lcurrenttime - llasttime);
  llasttime = lcurrenttime;
  lmc=lmc+ldir;
}

void R_enc_Callback(){ 
  rcurrenttime = millis();
  rvt = rdir*1000./(rcurrenttime - rlasttime);
  rlasttime = rcurrenttime;
  rmc=rmc+rdir;
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
  nh.advertise(chatter);
  nh.subscribe(Sub);


  // initialize PIN
  pinMode(DIRA, OUTPUT); //D0
  pinMode(PWMA, OUTPUT); //D5
  pinMode(DIRB, OUTPUT); //D2
  pinMode(PWMB, OUTPUT); //D4
  attachInterrupt(digitalPinToInterrupt(L_ENC), L_enc_Callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC), R_enc_Callback, CHANGE);

  // configure timer
  os_timer_setfn(&myTimer, tic, NULL); 
  os_timer_arm(&myTimer, period, true);   // timer in ms

} 

void loop()
{

  if (nh.connected()) {
    Serial.println("Connected");
    // Say hello
    str_msg.data = hello;
    chatter.publish( &str_msg );
  } else {
    Serial.println("Not Connected");
  }
  nh.spinOnce();
  // Loop exproximativly at 1Hz
  delay(200);
}
