#include <math.h>
#include <Timer.h>

Timer t;
//variables for robot
double motor_output_R;
double motor_output_L;
double DR = 0, DL =0;
double DC, delta_theta, radius, prev_theta, new_theta;
double x, y, x_new, y_new;
volatile unsigned long encoderR=0, encoderL=0;
unsigned long encoderR_old=0, encoderL_old=0;
int encoderR_dif=0, encoderL_dif=0;
#define motorR 10
#define motorL 11

//positions of sensors
double xsr60=15,ysr60=27.5,thetasr60=3.1418/3;
double xsf=0,ysf=32,thetasf=3.1418/2;
double xsl60=15,ysl60=27.5,thetasl60=3.1418/3;

//varaibles for robot frame
double vsr60_1,vsr60_2,vsr60_3;
double vsf_1,vsf_2,vsf_3;
double vsl60_1,vsl60_2,vsl60_3;

//varaibles for world frame
double vssr60_1,vssr60_2,vssr60_3;
double vssf_1,vssf_2,vssf_3;
double vssl60_1,vssl60_2,vssl60_3;

//compute u
double usr60_1,usr60_2,usr60_3;
double usf_1,usf_2,usf_3;
double usl60_1,usl60_2,usl60_3;

//compute u_ao
double u_ao_1,u_ao_2;
double theta_ao;
//PID
double e_k,e_p,e_i,e_d;
double kp=4,ki=.01,kd=.1;
double EK=0,e_k_1=0;
double omega, velocity, vel_r, vel_l,PID_output,wheel_radius;
double vel_r_in_pulses, vel_l_in_pulses, vel_rl_max, vel_rl_min;
int v_max = 3.06;
int w_max = 0.3;

//sensors
unsigned long echo = 0;
int ultraSoundSignal = 6; // Ultrasound signal pin
int echoSignal = 7; // echo signal pin
unsigned long ultrasoundValue = 0;

unsigned long echo2 = 0;
int ultraSoundSignal2 = 8; // Ultrasound signal pin
int echoSignal2 = 9; // echo signal pin
unsigned long ultrasoundValue2 = 0;

unsigned long echo3 = 0;
int ultraSoundSignal3 = A3; // Ultrasound signal pin
int echoSignal3 = A2; // echo signal pin
unsigned long ultrasoundValue3 = 0;


int get_x_y_new_Event;

void setup()
{
   motor_output_R = 0;
  motor_output_L = 0;
  Serial.begin(9600);
  pinMode(ultraSoundSignal,OUTPUT);
  pinMode(ultraSoundSignal2,OUTPUT);
  pinMode(ultraSoundSignal3,OUTPUT);

pinMode(motorR,OUTPUT);
  analogWrite(motorR,motor_output_R);
  
  pinMode(motorL,OUTPUT);
  analogWrite(motorL,motor_output_L);

}

unsigned long ping()
{
  pinMode(ultraSoundSignal, OUTPUT); // Switch signalpin to output
  digitalWrite(ultraSoundSignal, LOW); // Send low pulse
  delayMicroseconds(4); // Wait for 2 microseconds
  digitalWrite(ultraSoundSignal, HIGH); // Send high pulse
  delayMicroseconds(10); // Wait for 5 microseconds
  digitalWrite(ultraSoundSignal, LOW); // Holdoff
  
  pinMode(echoSignal, INPUT); // Switch signalpin to input
  digitalWrite(echoSignal, HIGH); // Turn on pullup resistor
  // please note that pulseIn has a 1sec timeout, which may
  // not be desirable. Depending on your sensor specs, you
  // can likely bound the time like this -- marcmerlin
  // echo = pulseIn(ultraSoundSignal, HIGH, 38000)
  echo = pulseIn(echoSignal, HIGH, 38000); //Listen for echo
  ultrasoundValue = (echo / 57); //convert to CM then to inches
  return ultrasoundValue;
}

unsigned long ping2()
{
  pinMode(ultraSoundSignal2, OUTPUT); // Switch signalpin to output
  digitalWrite(ultraSoundSignal2, LOW); // Send low pulse
  delayMicroseconds(4); // Wait for 2 microseconds
  digitalWrite(ultraSoundSignal2, HIGH); // Send high pulse
  delayMicroseconds(10); // Wait for 5 microseconds
  digitalWrite(ultraSoundSignal2, LOW); // Holdoff
  
  pinMode(echoSignal2, INPUT); // Switch signalpin to input
  digitalWrite(echoSignal2, HIGH); // Turn on pullup resistor
  // please note that pulseIn has a 1sec timeout, which may
  // not be desirable. Depending on your sensor specs, you
  // can likely bound the time like this -- marcmerlin
  // echo = pulseIn(ultraSoundSignal, HIGH, 38000)
  echo2 = pulseIn(echoSignal2, HIGH, 38000); //Listen for echo
  ultrasoundValue2 = (echo2 / 57); //convert to CM then to inches
  return ultrasoundValue2;
}

unsigned long ping3()
{
  pinMode(ultraSoundSignal3, OUTPUT); // Switch signalpin to output
  digitalWrite(ultraSoundSignal3, LOW); // Send low pulse
  delayMicroseconds(4); // Wait for 2 microseconds
  digitalWrite(ultraSoundSignal3, HIGH); // Send high pulse
  delayMicroseconds(10); // Wait for 5 microseconds
  digitalWrite(ultraSoundSignal3, LOW); // Holdoff
  
  pinMode(echoSignal3, INPUT); // Switch signalpin to input
  digitalWrite(echoSignal3, HIGH); // Turn on pullup resistor
  // please note that pulseIn has a 1sec timeout, which may
  // not be desirable. Depending on your sensor specs, you
  // can likely bound the time like this -- marcmerlin
  // echo = pulseIn(ultraSoundSignal, HIGH, 38000)
  echo3 = pulseIn(echoSignal3, HIGH, 38000); //Listen for echo
  ultrasoundValue3 = (echo3 / 57); //convert to CM then to inches
  return ultrasoundValue3;
}

void get_x_y_new_direct()
{
  velocity=3;
  // every 50 ms
  encoderR_dif = encoderR - encoderR_old;
  encoderR_old = encoderR;
  encoderL_dif = encoderL - encoderL_old;
  encoderL_old = encoderL;

  DR = 47.1 * encoderR_dif/928;     // DR = 31.0 * encoderR_dif/420.0;
  DL = 47.1 * encoderL_dif/928;      //DL = 31.0 * encoderL_dif/420.0;
 /* Serial.print("DR= ");
  Serial.print(DR);
  Serial.print("   DL= ");
  Serial.println(DL);*/
  DC = (DR+DL)/2.0;
  delta_theta = (DL-DR)/30;   ///(DR-DL)
  /*Serial.print("   DC= ");
  Serial.println(DC);
  Serial.print("delta_theta= ");
  Serial.print(delta_theta);
  Serial.print("   prev_theta= ");
  Serial.println(prev_theta);*/
  x_new = x + DC * cos(prev_theta);
  y_new = y + DC * sin(prev_theta);
  new_theta = prev_theta + delta_theta;
  /*Serial.print("x_new= ");
  Serial.print(x_new);
  Serial.print("   y_new= ");
  Serial.println(y_new);*/
  x = x_new;
  y = y_new;
  prev_theta = new_theta;
} 

void doEncoder_0()
{
  encoderR++;
 
}

void doEncoder_1()
{
  encoderL++;
 
}
void unicycle_to_diff_drive()
{
  // 2*pi*R=31
  wheel_radius = 47.1/(2*3.1418);   //31/(2*3.1418);  
  vel_r = (2*velocity+omega*30)/(2*wheel_radius);    //vel_r = (2*velocity+omega*35)/(2*wheel_radius);
  vel_l = (2*velocity-omega*30)/(2*wheel_radius);    //vel_l = (2*velocity-omega*35)/(2*wheel_radius);
 /* Serial.print("   omega= ");
  Serial.println(omega);
  Serial.print("vel_r= ");
  Serial.print(vel_r);
  Serial.print("   vel_l= ");
  Serial.println(vel_l);*/
}

void output_shaping()
{
  vel_rl_max = max(vel_r, vel_l);
  vel_rl_min = min(vel_r, vel_l);
  if(vel_rl_max > v_max)
  {
    vel_r = vel_r - (vel_rl_max-1.75);
    vel_l = vel_l - (vel_rl_max-1.75);
  }
  if(vel_rl_min < 0)
  {
    vel_r = vel_r + (0 - vel_rl_min);
    vel_l = vel_l + (0 - vel_rl_min);
  }
  else
  {
    vel_r = vel_r;
    vel_l = vel_l;
  }
}

void loop()
{
  unsigned long dsr60,dsf,dsl60;
  dsr60=ping(); 
  dsf=ping2();
  dsl60=ping3();
  Serial.println(dsr60);
  Serial.println(dsf);
  Serial.println(dsl60);
  t.update();
  get_x_y_new_Event = t.every(50, get_x_y_new_direct);

  //robot frame transformation
  vsr60_1=cos(thetasr60)*dsr60+xsr60;    ///cos(60)
  vsr60_2=sin(thetasr60)*dsr60+ysr60;
  vsr60_3=1;
  vsf_1=cos(thetasf)*dsf+xsf;    ///cos(60)
  vsf_2=sin(thetasf)*dsf+ysf;
  vsr60_3=1;
  vsl60_1=cos(thetasl60)*dsl60+xsl60;    ///cos(60)
  vsl60_2=sin(thetasl60)*dsl60+ysl60;
  vsr60_3=1;

  //world fram transformation
  vssr60_1=cos(prev_theta)*vsr60_1-sin(prev_theta)*vsr60_2+x;
  vssr60_2=sin(prev_theta)*vsr60_1+cos(prev_theta)*vsr60_2+y;
  vssr60_3=1;
  vssf_1=cos(prev_theta)*vsf_1-sin(prev_theta)*vsf_2+x;
  vssf_2=sin(prev_theta)*vsf_1+cos(prev_theta)*vsf_2+y;
  vssf_3=1;
  vssl60_1=cos(prev_theta)*vsl60_1-sin(prev_theta)*vsl60_2+x;
  vssl60_2=sin(prev_theta)*vsl60_1+cos(prev_theta)*vsl60_2+y;
  vssl60_3=1;

  //compute u
  usr60_1=vssr60_1-x;
  usr60_2=vssr60_2-y;
  usf_1=vssf_1-x;
  usf_2=vssf_2-y;
  usl60_1=vssl60_1-x;
  usl60_2=vssl60_2-y;
  
  u_ao_1=usr60_1+usf_1+usl60_1;
  u_ao_2=usr60_2+usf_2+usl60_2;
  
  theta_ao=atan2(u_ao_2,u_ao_1);

  //PID
  e_k=theta_ao-prev_theta;
  e_k=atan2(sin(e_k),cos(e_k));
  e_p=e_k;
  e_i=EK+e_k;//*dt;
  e_d=e_k-e_k_1;
  PID_output=kp*e_p+ki*e_i+kd*e_d;

  EK=e_i;
  e_k_1=e_k;
  omega = PID_output;
  if(omega > w_max)
    omega = w_max;
  else if(omega < -w_max)
    omega = -w_max;

  unicycle_to_diff_drive();
  output_shaping();
  motor_output_R = map(vel_r*100, 0, v_max*100, 0, 255);
  motor_output_L = map(vel_l*100, 0, v_max*100, 0, 255);

  analogWrite(motorR, motor_output_R);
  analogWrite(motorL, motor_output_L);
  Serial.println(motor_output_R);
  Serial.println(motor_output_L);
  //Serial.println(cos(3.1418/3));
}


