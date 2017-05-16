
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#include "robot_specs.h"

#define LeftMotor  5
#define LeftMotorDir  36
#define LeftMotorStop 22


#define RightMotor  10
#define RightMotorDir  37
#define RightMotorStop 23
      
#define LOOPTIME        100   // PID loop time(ms)
#define SMOOTH      10

#define sign(x) (x > 0) - (x < 0)

#define FORWARD 1
#define BACKWARD 2
#define STOP 0

int direcLeft=0;
int direcRight=0;


ros::NodeHandle nh;


unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMilliPub = 0;

long long rpm_req1 = 0;
double rpm_req2 = 0;

long long rpm_act1 = 0;
long long rpm_act2 = 0;

long long t1_l=0;

int leftEncoder=3;
int rightEncoder=21;


double rpm_req1_smoothed = 0;
double rpm_req2_smoothed = 0;

int directionLeft = FORWARD;
int directionRight = FORWARD;



int PWM_val1 = 0;
int PWM_val2 = 0;

volatile long count1 = 0;          // rev counter
volatile long count2 = 0;

long countAnt1 = 0;
long countAnt2 = 0;

//float Kp =   0.5;
//float Kd =   0;
//float Ki =   0;



int flagL=0 , flagR=0 , fl , fr;


void handle_cmd( const geometry_msgs::Twist& cmd_msg)
{
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  if (z == 0) 
  {     // go straight
    // convert m/s to rpm
    rpm_req1 = x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) 
  {
    // convert rad/s to rpm
    rpm_req2 = z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = -rpm_req2;
  }
  else
  {
    rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
}



ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel_mux/input/teleop", handle_cmd);        

geometry_msgs::Vector3Stamped rpm_msg;

ros::Publisher rpm_pub("rpm", &rpm_msg);

ros::Time current_time;
ros::Time last_time;


char c;

void setup() 
{
     Serial1.begin(9600);
     
     count1 = 0;
     count2 = 0;
     
     countAnt1 = 0;
     countAnt2 = 0;
     
     rpm_req1 = 0;
     rpm_req2 = 0;
     
     rpm_act1 = 0;
     rpm_act2 = 0;
     
     PWM_val1 = 0;
     PWM_val2 = 0;
    
     
     nh.initNode();
     nh.getHardware()->setBaud(57600);
     nh.subscribe(sub);
     nh.advertise(rpm_pub);
     

     pinMode(leftEncoder,INPUT);
     digitalWrite(leftEncoder,HIGH);                                              //for left encoder 3
     attachInterrupt(1, encoderLeft, FALLING);  //attach interrupt on left encoder

     pinMode(rightEncoder,INPUT);
     digitalWrite(rightEncoder,HIGH);                                             //for right encoder 21
     attachInterrupt(2, encoderRight, FALLING); //attach interrupt on right encoder
    
    
    pinMode(LeftMotor , OUTPUT);
    pinMode(LeftMotorDir , OUTPUT);
    pinMode(LeftMotorStop , OUTPUT);
    pinMode(RightMotor , OUTPUT);
    pinMode(RightMotorDir , OUTPUT);
    pinMode(RightMotorStop , OUTPUT);
    
    digitalWrite(LeftMotor , LOW);
    digitalWrite(LeftMotorDir , HIGH);
    digitalWrite(LeftMotorStop , LOW);
    digitalWrite(RightMotor , LOW);
    digitalWrite(RightMotorDir , HIGH);
    digitalWrite(RightMotorStop , LOW);
    
   
}

void loop() 
{
   
    
//    if(Serial1.available())
//    {
//       c = Serial1.read();
//       
//    }
//
//switch(c)
//      {
//      case 'f':
//               motorRun(1,FORWARD,45);
//               motorRun(2,FORWARD,41);
//               directionLeft=FORWARD;
//               directionRight=FORWARD;
//               break;
//    
//      case 'b' : 
//                 motorRun(1,BACKWARD,40);
//                 motorRun(2,BACKWARD,36);
//                 directionLeft=BACKWARD;
//                 directionRight=BACKWARD;
//            
//                break;
//    
//      case 'l' : motorRun(1,BACKWARD,25);
//                 motorRun(2,FORWARD,25);
//                 directionLeft=BACKWARD;
//                 directionRight=FORWARD;
//                 
//                 break; 
//      
//      case 'r' : motorRun(1,FORWARD,25);
//                 motorRun(2,BACKWARD,25);
//                 directionLeft=FORWARD;
//                 directionRight=BACKWARD;
//                 
//                 break;
//    
//       case 's' : motorRun(1,STOP,1);
//                  motorRun(2,STOP,1);
//               break;
//    
//    
//      default : motorRun(1,STOP,0);
//               motorRun(2,STOP,0);
//               break;        
//    }

  nh.spinOnce();
 
  unsigned long time = millis();
  if(time-lastMilli>= 500) 
  {      // enter timed loop
        
        getMotorData(time-lastMilli); 
        
        

//    PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
//    PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);



    PWM_val1 =  constrain((45.0*rpm_req1)/MAX_RPM, -45, 45);
    PWM_val2 = constrain((41.0*rpm_req2)/MAX_RPM, -41, 41);


    if(PWM_val1 > 0) directionLeft = FORWARD;
    else if(PWM_val1 < 0) directionLeft = BACKWARD;
    if (rpm_req1 == 0) directionLeft = STOP;
    if(PWM_val2 > 0) directionRight = FORWARD;
    else if(PWM_val2 < 0) directionRight = BACKWARD;
    if (rpm_req2 == 0) directionRight = STOP;
   
   
    motorRun(1,directionLeft,abs(PWM_val1));
    motorRun(2,directionRight,abs(PWM_val2));
     
    publishRPM(time-lastMilli);   
    lastMilli = time;
  }
    
}


void getMotorData(unsigned long time)  
{
  unsigned long dt = time*30;
  long long  dcount1 = count1-countAnt1;
  long long  dcount2 = count2-countAnt2;
  rpm_act1 = (dcount1*60000)/dt;
  rpm_act2 = (dcount2*60000)/dt;
  if(directionLeft==BACKWARD)
  {
    rpm_act1=-rpm_act1;
  }
   
  if(directionRight==BACKWARD)
  {
    rpm_act2=-rpm_act2;
  }
  
  Serial1.print("LeftRPM: ");
  Serial1.print(long(rpm_act1));
  Serial1.print("\t");
  Serial1.print("RightRPM: ");
  Serial1.println(long(rpm_act2));
  
  countAnt1 = count1;
  countAnt2 = count2;
}


//int updatePid(int id, int command, double targetValue, double currentValue) {
//  double pidTerm = 0;                            // PID correction
//  double error = 0;
//  double new_pwm = 0;
//  double new_cmd = 0;
//  static double last_error1 = 0;
//  static double last_error2 = 0;
//  static double int_error1 = 0;
//  static double int_error2 = 0;
//  
//  error = targetValue-currentValue;
//  if (id == 1) {
//    int_error1 += error;
//    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
//    last_error1 = error;
//  }
//  else {
//    int_error2 += error;
//    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
//    last_error2 = error;
//  }
//  new_pwm = constrain(double(command)*MAX_RPM/255.0 + pidTerm, -MAX_RPM, MAX_RPM);
//  new_cmd = 255.0*new_pwm/MAX_RPM;
//  return int(new_cmd);
//}




void motorRun(int Id , int Direction , int Speed)
{
  switch(Id)
  {
    // left motor
    case 1 : if(Direction == FORWARD)
               {  
                  digitalWrite(LeftMotorStop,0);
                  analogWrite(LeftMotor , Speed);
                  digitalWrite(LeftMotorDir , 0);
                  
                  break;
               }
              
              else if(Direction == BACKWARD)
               { 
                  digitalWrite(LeftMotorStop,0);
                  analogWrite(LeftMotor , Speed);
                  digitalWrite(LeftMotorDir , 1);
               break;
               } 

               else if(Direction == STOP)
               {
                  digitalWrite(LeftMotorStop , 1);
                   
               break;
               }  


                // Right motor
    case 2 : if(Direction == FORWARD)
               {
               digitalWrite(RightMotorStop,0);
               analogWrite(RightMotor , Speed);
               digitalWrite(RightMotorDir , 0);
               break;
               }
              
              else if(Direction == BACKWARD)
                {
                  digitalWrite(RightMotorStop,0);
                  analogWrite(RightMotor , Speed);
                   digitalWrite(RightMotorDir , 1);
                  break;
               }  

               else if(Direction == STOP)
                {
               digitalWrite(RightMotorStop , 1);
               
               break;
               } 

     default : break;
  }
}


void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}


void encoderLeft() 
{
    count1++;
}




void encoderRight() 
{
    count2++;
}
