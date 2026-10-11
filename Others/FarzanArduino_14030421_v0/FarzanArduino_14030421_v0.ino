//..........................................................................................

#include <math.h>
#include <Wire.h>
#include "MPU9250.h"
#include <AutoPID.h>



// Timers
unsigned long timer = 0;
float timeStep = 0.01, sign;

//Ultrasonics
const int ultrasonic_trig[4] = { 0, 22, 24, 26 };
const int ultrasonic_echo[4] = { 0, 23, 25, 27 };

//IRs
const int IRpin[4] = { 0, 34, 35, 36 };

int IRstate[4];     //Using for IR values
int distanceCm[4];  //using for Ultrasonics
int duration[4];

int IRs = 0;
int Ultras = 0;

//Motors
const int motor1In1 = 4;
const int motor1In2 = 5;
const int motor2In1 = 6;
const int motor2In2 = 7;
const int motor3In1 = 8;
const int motor3In2 = 9;

//Encoders
// const int encoder_ChannelA[4] = { 0, A8, 19, 2 };
// const int encoder_ChannelB[4] = { 0, A9, 18, 3 };
const int encoder_ChannelA[4] = { 0, 2, 18, A8 };
const int encoder_ChannelB[4] = { 0, 3, 19, A9 };
double pi = 3.14159;
double r = 0.029;
double L = 0.215;  //0.191;

double t, t_00;
long pulse1, pulse2, pulse3 = 0;
long TimeI, t1=1, t2=1, t3=1; //micro second
long TimeI_p, t1_p, t2_p, t3_p;
long dTimeI, dt1=1, dt2=1, dt3=1; //micro second

unsigned long TimeII, TimeII_p, dt_TimeII;

#define OUTPUT_MIN_SPEED -200
#define OUTPUT_MAX_SPEED +200

#define KP1_speed 0.5
#define KI1_speed 10
#define KD1_speed 0.0

#define KP2_speed 0.5
#define KI2_speed 10
#define KD2_speed 0.0

#define KP3_speed 0.5
#define KI3_speed 10
#define KD3_speed 0.0

#define OUTPUT_MIN_POSITION -80
#define OUTPUT_MAX_POSITION +80

#define KP3_c 1
#define KI3_c 2
#define KD3_c 0
// float t;
double theta1_dot_RPM = 0, theta2_dot_RPM = 0, theta3_dot_RPM = 0, thetaMAX_dot;
double theta1_dot_RPM_p, theta2_dot_RPM_p, theta3_dot_RPM_p;

double theta1_deg, theta2_deg, theta3_deg;
double theta1_rad, theta2_rad, theta3_rad;

double x_dot, y_dot, phi_dot, xd_dot, yd_dot, phid_dot;
double xd, yd, phid, xd_p, yd_p, phid_p;
double xd_g_past, yd_g_past, phid_past; 

double xd_dot_g, yd_dot_g;
double x_dot_g, y_dot_g, x_g, y_g, phi_g, xd_g = 0, yd_g=0, phid_g = 0, phi_g_imu, phi_g_enc = 0, phi_dot_enc, e_phi;

double theta1_dot, theta2_dot, theta3_dot;
double theta1d_dot, theta2d_dot, theta3d_dot;
#define delta_theta_dot 0.1  //0.15
#define delta_VmotorPWM 50   //25//15


double VmotorPWM1, VmotorPWM2, VmotorPWM3;
//input/output variables passed by reference, so they are updated automatically
AutoPID motor1_PID_speed(&theta1_dot, &theta1d_dot, &VmotorPWM1, OUTPUT_MIN_SPEED, OUTPUT_MAX_SPEED, KP1_speed, KI1_speed, KD1_speed);
AutoPID motor2_PID_speed(&theta2_dot, &theta2d_dot, &VmotorPWM2, OUTPUT_MIN_SPEED, OUTPUT_MAX_SPEED, KP2_speed, KI2_speed, KD2_speed);
AutoPID motor3_PID_speed(&theta3_dot, &theta3d_dot, &VmotorPWM3, OUTPUT_MIN_SPEED, OUTPUT_MAX_SPEED, KP3_speed, KI3_speed, KD3_speed);
// AutoPID Function (feedback , desierd , output , minimumInterwall , maximumInterwall , kp , ki , kd)
// AutoPID phiMyPIDPosition(&phi_g, &phid, &e_phi, OUTPUT_MIN_POSITION, OUTPUT_MAX_POSITION, KP3_c, KI3_c, KD3_c);

double position_PWM1, position_PWM2, position_PWM3;

// AutoPID xMyPIDPosition(&x_g, &xd, &e_x, OUTPUT_MIN_POSITION, OUTPUT_MAX_POSITION, KP1_c, KI1_c, KD1_c);
// AutoPID yMyPIDPosition(&y_g, &yd, &e_y, OUTPUT_MIN_POSITION, OUTPUT_MAX_POSITION, KP2_c, KI2_c, KD2_c);
// AutoPID phiMyPIDPosition(&phi_g, &phid, &e_phi, OUTPUT_MIN_POSITION, OUTPUT_MAX_POSITION, KP3_c, KI3_c, KD3_c);

// AutoPID motor1_PID_position(&theta1_dot, &theta1d_dot, &position_PWM1, OUTPUT_MIN_POSITION, OUTPUT_MAX_POSITION, KP1_position, KI1_position, KD1_position);
// AutoPID motor2_PID_position(&theta2_dot, &theta2d_dot, &position_PWM2, OUTPUT_MIN_POSITION, OUTPUT_MAX_POSITION, KP2_position, KI2_position, KD2_position);
// AutoPID motor3_PID_position(&theta3_dot, &theta3d_dot, &position_PWM3, OUTPUT_MIN_POSITION, OUTPUT_MAX_POSITION, KP3_position, KI3_position, KD3_position);
// double signal, signal_dot;
// AutoPID Diff_PID(0.0, &signal, &signal_dot, -1000000, 1000000, 0, 0, 1);



//bluetooth
const byte numChars = 64;//32;
char receivedChars[numChars];
char temAndroidhars[numChars];  // temporary array for use when parsing

// variables to hold the parsed data
char messageFromAndroid[numChars] = { 0 };

int intFromAndroid = 0.0;
double floatFromAndroid1 = 0.0;//x
double floatFromAndroid2 = 0.0;//y
double floatFromAndroid3 = 0.0;//phi
double floatFromAndroid4 = 0.0;//x_dot
double floatFromAndroid5 = 0.0;//y_dot
double floatFromAndroid6 = 0.0;//phi_dot
double floatFromAndroid7 = 0.0;//
double floatFromAndroid8 = 0.0;//
double floatFromAndroid9 = 0.0;//



boolean newData = false;
//..........................................................................................................................................................................................

void setup() {
  Serial.begin(9600);
  // Serial2.begin(115200);

  pinMode(motor1In1, OUTPUT);
  pinMode(motor1In2, OUTPUT);
  pinMode(motor2In1, OUTPUT);
  pinMode(motor2In2, OUTPUT);
  pinMode(motor3In1, OUTPUT);
  pinMode(motor3In2, OUTPUT);

  digitalWrite(motor1In1, LOW);
  digitalWrite(motor1In2, LOW);
  digitalWrite(motor2In1, LOW);
  digitalWrite(motor2In2, LOW);
  digitalWrite(motor3In1, LOW);
  digitalWrite(motor3In2, LOW);

  pinMode(ultrasonic_trig[1], OUTPUT);
  pinMode(ultrasonic_echo[1], INPUT);
  pinMode(ultrasonic_trig[2], OUTPUT);
  pinMode(ultrasonic_echo[2], INPUT);
  pinMode(ultrasonic_trig[3], OUTPUT);
  pinMode(ultrasonic_echo[3], INPUT);

  pinMode(encoder_ChannelA[1], INPUT);
  pinMode(encoder_ChannelB[1], INPUT);
  pinMode(encoder_ChannelA[2], INPUT);
  pinMode(encoder_ChannelB[2], INPUT);
  pinMode(encoder_ChannelA[3], INPUT);
  pinMode(encoder_ChannelB[3], INPUT);

  // motor1_PID_speed.setBangBang(50);
  // motor2_PID_speed.setBangBang(50);
  // motor3_PID_speed.setBangBang(50);

  // motor1_PID_position.setBangBang(10);
  // motor2_PID_position.setBangBang(10);
  // motor3_PID_position.setBangBang(10);

  // xMyPIDPosition.setBangBang(10);
  // yMyPIDPosition.setBangBang(10);
  // phiMyPIDPosition.setBangBang(10);

  motor1_PID_speed.setTimeStep(50);
  motor2_PID_speed.setTimeStep(50);
  motor3_PID_speed.setTimeStep(50);

  // motor1_PID_position.setTimeStep(50);
  // motor2_PID_position.setTimeStep(50);
  // motor3_PID_position.setTimeStep(50);

  // xMyPIDPosition.setTimeStep(50);
  // yMyPIDPosition.setTimeStep(50);
  // phiMyPIDPosition.setTimeStep(50);

  attachInterrupt(digitalPinToInterrupt(encoder_ChannelA[3]), ai3, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder_ChannelA[2]), ai2, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder_ChannelA[1]), ai1, FALLING);

  // phi_g_d = (mpu.getYaw()+44)*(-90)/32;
  // mpu.calibrateMag();
  t_00=millis();
}

void loop() {
  //* Recieve data
  //-----------------
  recvWithStartEndMarkers();
   if (newData == true) {
  strcpy(temAndroidhars, receivedChars);
  parseData();
  }

    
  //-----------------
  // recieve_virtual(); 


  // imu();


//SPEED COMMAND
  if (intFromAndroid == 0) {
    xd_g = 0;
    yd_g = 0;
    phid = 0;
    xd_dot_g = (floatFromAndroid4) ;// / 10;
    yd_dot_g = (floatFromAndroid5) ;// / 10;
    phid_dot = (floatFromAndroid6) ;// / 10;

    // xd_dot_g=-0.5*(xd_g-x_g);
    // phid_dot=100*(phid_g-phi_g_imu);


    speedLoop();
  }

//POSITION-TRAJECTORY COMMAND
  else if (intFromAndroid == 1) {

    
    xd_g = floatFromAndroid1; // 100;
    yd_g = floatFromAndroid2; // 100;
    phid = floatFromAndroid3;// 100;
    
    xd_dot_g = floatFromAndroid4;//  (xd_g-xd_g_past)/dTimeI* 1e6;
    yd_dot_g = floatFromAndroid5;//  (yd_g-yd_g_past)/dTimeI* 1e6;
    phid_dot = floatFromAndroid6;//  (phid-phid_past)/dTimeI* 1e6;


    speedLoop();

  }

  


  // encoder data (Speed and position)
  TimeII_p = TimeII;
  TimeII = micros();
  dt_TimeII = TimeII - TimeII_p;

  x_g = x_dot_g * dt_TimeII * 1e-6 + x_g; //[m]
  y_g = y_dot_g * dt_TimeII * 1e-6 + y_g; //[m]
  phi_g_enc = phi_dot_enc * dt_TimeII * 1e-6 + phi_g_enc; //[rad]

  phi_g= phi_g_enc; //phi_g_imu;
  

  // x_dot = r * (-theta1_dot - theta3_dot + 2 * theta2_dot) / 3;
  // y_dot = ((1 / sqrt(3)) * theta1_dot - (1 / sqrt(3)) * theta3_dot) * r;
  // phi_dot_enc = ((1 / (3 * L)) * theta1_dot + (1 / (3 * L)) * theta2_dot + (1 / (3 * L)) * theta3_dot) * r;

  x_dot = r * (theta1_dot + theta2_dot - 2 * theta3_dot) / 3; //[m/s]
  y_dot = (-(1 / sqrt(3)) * theta1_dot + (1 / sqrt(3)) * theta2_dot) * r; //[m/s]
  phi_dot_enc = -(((1 / (3 * L)) * theta1_dot + (1 / (3 * L)) * theta2_dot + (1 / (3 * L)) * theta3_dot) * r); //[rad/s]

  phi_dot= phi_dot_enc;

  x_dot_g = cos(phi_g) * x_dot - sin(phi_g) * y_dot; //[m/s]
  y_dot_g = sin(phi_g) * x_dot + cos(phi_g) * y_dot; //[m/s]

  theta1_dot_RPM_p =(60 * 1e6) / (dt1 * 100 * 20.25);
  if (theta1_dot_RPM_p < 200 && theta1_dot_RPM_p > -200)
    theta1_dot_RPM = theta1_dot_RPM_p;
  theta1_dot = theta1_dot_RPM * (2 * pi / 60); //[rad/s]
  theta1_deg = ((pulse1 * 3.6) / 20.25);  theta1_rad = (theta1_deg * pi) / 180;

  theta2_dot_RPM_p = (60 * 1e6) / (dt2 * 100 * 20.25);
  if (theta2_dot_RPM_p < 200 && theta2_dot_RPM_p > -200)
    theta2_dot_RPM = theta2_dot_RPM_p;
  theta2_dot = theta2_dot_RPM * (2 * pi / 60);//[rad/s]
  theta2_deg = ((pulse2 * 3.6) / 20.25);  theta2_rad = (theta2_deg * pi) / 180;

  theta3_dot_RPM_p = (60 * 1e6) / (dt3 * 100 * 20.25);
  if (theta3_dot_RPM_p < 200 && theta3_dot_RPM_p > -200)
    theta3_dot_RPM = theta3_dot_RPM_p;
  theta3_dot = theta3_dot_RPM * (2 * pi / 60);//[rad/s]
  theta3_deg = ((pulse3 * 3.6) / 20.25);  theta3_rad = (theta3_deg * pi) / 180;

  // Ultras = floatFromAndroid4;
  // IRs = floatFromAndroid5;
  newData = false;

  // ultrasonic();

  //* Send data
  //------------------
  DatatoRaspberry(); 
  //------------------
}

void recieve_virtual(){

  intFromAndroid=0;
  floatFromAndroid1 = 0.0;
  floatFromAndroid2 = 0.0;
  floatFromAndroid3 = 0.0;
  floatFromAndroid4 = 0.0;
  floatFromAndroid5 = 0.0;//min(0.25*1e-3*(millis()-t_00),3.0)
  floatFromAndroid6 = 0.0;
  floatFromAndroid7 = 0.0;
  floatFromAndroid8 = 0.0;
  floatFromAndroid9 = 0.0;

}


void speedLoop(){



  xd_dot = cos(phid) * xd_dot_g + sin(phid) * yd_dot_g;
  yd_dot = -sin(phid) * xd_dot_g + cos(phid) * yd_dot_g;

  // xd_dot =xd_dot_g;
  // yd_dot =yd_dot_g;

  // theta1d_dot = ((-0.5) * xd_dot + (sqrt(3) / 2) * yd_dot + L * phid_dot) / r;
  // theta3d_dot = (xd_dot + L * phid_dot) / r;
  // theta2d_dot = ((-0.5) * xd_dot - (sqrt(3) / 2) * yd_dot + L * phid_dot) / r;

  theta1d_dot = ((0.5) * xd_dot - (sqrt(3) / 2) * yd_dot - L * phid_dot) / r;
  theta2d_dot = ((0.5) * xd_dot + (sqrt(3) / 2) * yd_dot - L * phid_dot) / r;
  theta3d_dot = (-xd_dot - L * phid_dot) / r;

  thetaMAX_dot = max(max(abs(theta1d_dot), abs(theta2d_dot)), abs(theta3d_dot));

  if (thetaMAX_dot > 10) {
    theta1d_dot = 10 / thetaMAX_dot * theta1d_dot;
    theta2d_dot = 10 / thetaMAX_dot * theta2d_dot;
    theta3d_dot = 10 / thetaMAX_dot * theta3d_dot;
  }



  motor1_PID_speed.run();
  motor2_PID_speed.run();
  motor3_PID_speed.run();


  //  AutoPID motor1_PID_speed(&theta1_dot, &theta1d_dot, &VmotorPWM1, OUTPUT_MIN_SPEED, OUTPUT_MAX_SPEED, KP1_speed, KI1_speed, KD1_speed);

  if (abs(theta1d_dot - theta1_dot) <= delta_theta_dot && abs(VmotorPWM1) <= delta_VmotorPWM) { VmotorPWM1 = 0; }
  if (abs(theta2d_dot - theta2_dot) <= delta_theta_dot && abs(VmotorPWM2) <= delta_VmotorPWM) { VmotorPWM2 = 0; }
  if (abs(theta3d_dot - theta3_dot) <= delta_theta_dot && abs(VmotorPWM3) <= delta_VmotorPWM) { VmotorPWM3 = 0; }

  motor(VmotorPWM1, VmotorPWM2, VmotorPWM3);
  // motor(00, 100, 00);
  // Serial.print(pulse1);Serial.print(',');
  // Serial.print(pulse2);Serial.print(',');
  // Serial.println(pulse3);

}


void recvWithStartEndMarkers() {  //data from raspberry
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    // Serial.print("rc=[");Serial.print(rc);Serial.println(']');

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        // Serial.print("receivedChars=[");Serial.print(receivedChars);Serial.println(']');
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
          // Serial.print("ndx=[");Serial.print(ndx);Serial.println(']');

        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;

      }
    }
    
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}
// //============

void parseData() {  // split the one unit data into its parts and read them seperately

  char* strtokIndx;  // this is used as an index vector from one part to next part

  strtokIndx = strtok(temAndroidhars, ",");  // get the first part - the string
  //strcpy(messageFromAndroid, strtokIndx); // copy it to messageFromAndroid

  intFromAndroid = atoi(strtokIndx);  //flag - 0.velocity or 1.location

  strtokIndx = strtok(NULL, ",");        // this continues where the previous call left off
  floatFromAndroid1 = atof(strtokIndx);  //float - xd_g

  strtokIndx = strtok(NULL, ",");        // this continues where the previous call left off
  floatFromAndroid2 = atof(strtokIndx);  //float - yd_g

  strtokIndx = strtok(NULL, ",");        // this continues where the previous call left off
  floatFromAndroid3 = atof(strtokIndx);  //float - phid

  strtokIndx = strtok(NULL, ",");        // this continues where the previous call left off
  floatFromAndroid4 = atof(strtokIndx);  //float - xd_dot_g

  strtokIndx = strtok(NULL, ",");        // this continues where the previous call left off
  floatFromAndroid5 = atof(strtokIndx);  //float - yd_dot_g

  strtokIndx = strtok(NULL, ",");        // this continues where the previous call left off
  floatFromAndroid6 = atof(strtokIndx);  //float - phid_dot

  strtokIndx = strtok(NULL, ",");        // this continues where the previous call left off
  floatFromAndroid7 = atof(strtokIndx);  //flag - Ultrasonic sensors should be off or on?

  strtokIndx = strtok(NULL, ",");        // this continues where the previous call left off
  floatFromAndroid8 = atof(strtokIndx);  //flag - ir sensors should be off or on?

  strtokIndx = strtok(NULL, ",");        // this continues where the previous call left off
  floatFromAndroid9 = atof(strtokIndx);  //flag - ? sensors should be off or on?

}

void motor(float PWM1, float PWM2, float PWM3) {

  if (PWM1 >= 0) {
    analogWrite(motor1In1, PWM1);
    analogWrite(motor1In2, 0);
  } else {
    analogWrite(motor1In1, 0);
    analogWrite(motor1In2, abs(PWM1));
  }

  if (PWM2 >= 0) {
    analogWrite(motor2In1, PWM2);
    analogWrite(motor2In2, 0);
  } else {
    analogWrite(motor2In1, 0);
    analogWrite(motor2In2, abs(PWM2));
  }

  if (PWM3 >= 0) {
    analogWrite(motor3In1, PWM3);
    analogWrite(motor3In2, 0);
  } else {
    analogWrite(motor3In1, 0);
    analogWrite(motor3In2, abs(PWM3));
  }
  // if (PWM1 = 0) {
  //   analogWrite(motor1In1, LOW);
  //   analogWrite(motor1In2, LOW); }
  // if (PWM2 = 0) {
  //   analogWrite(motor2In1, LOW);
  //   analogWrite(motor2In2, LOW); }
  // if (PWM3 = 0) {
  //   analogWrite(motor3In1, LOW);
  //   analogWrite(motor3In2, LOW); }
}



//Encoder
void ai1() {

  t1 = micros();
  if (digitalRead(encoder_ChannelB[1])) {
    pulse1++;
    dt1 = t1 - t1_p;
  } else {
    pulse1--;
    dt1 = t1_p - t1;
  }
  t1_p = t1;
}
void ai2() {

  t2 = micros();
  if (digitalRead(encoder_ChannelB[2])) {
    pulse2++;
    dt2 = t2 - t2_p;
  } else {
    pulse2--;
    dt2 = t2_p - t2;
  }
  t2_p = t2;
}
void ai3() {

  t3 = micros();
  if (digitalRead(encoder_ChannelB[3])) {
    pulse3++;
    dt3 = t3 - t3_p;
  } else {
    pulse3--;
    dt3 = t3_p - t3;
  }
  t3_p = t3;
}


void ulntrasonic() {
  int k = 1;
  while (k < 4) {
    digitalWrite(ultrasonic_trig[k], LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonic_trig[k], HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonic_trig[k], LOW);
    duration[k] = pulseIn(ultrasonic_echo[k], HIGH, 10);
    distanceCm[k] = duration[k] * 0.034 / 2;
    k++;
  }
}


void DatatoRaspberry() {
  Serial.print("<OK,");

  // standard feedback for raspberry/PC comunication_uncomment at final trial 
//-----------------------------------------------------------------------------
  // Serial.print("0");Serial.print(",");
  // Serial.print("0");Serial.print(",");//Ultrasonic2 = 0
  // Serial.print("0");Serial.print(",");//Ultrasonic3 = 0
  // Serial.print(x_g);Serial.print(",");
  // Serial.print(y_g);Serial.print(",");
  // Serial.print(phi_g);Serial.print(",");
  // Serial.println(">");//delay(1000);
//-----------------------------------------------------------------------------


  // Serial.print(theta1d_dot-theta1_dot);Serial.print(",");
  // Serial.print(theta2d_dot-theta2_dot);Serial.print(",");//Ultrasonic2 = 0
  // Serial.print(theta3d_dot-theta3_dot);Serial.print(",");//Ultrasonic3 = 0
  // Serial.print(VmotorPWM1);Serial.print(",");
  // Serial.print(VmotorPWM2);Serial.print(",");
  // Serial.print(VmotorPWM3);Serial.print(",");
  // Serial.println(">");//delay(1000);

  Serial.print(theta1d_dot);Serial.print(",");
  Serial.print(theta1_dot);Serial.print(",");
  Serial.print(theta2d_dot);Serial.print(",");
  Serial.print(theta2_dot);Serial.print(",");
  Serial.print(theta3d_dot);Serial.print(",");
  Serial.print(theta3_dot);Serial.print(",");
  Serial.println(">");  //delay(1000);

  // Serial.print(theta1d_dot);Serial.print(",");
  // Serial.print(theta2d_dot);Serial.print(",");
  // Serial.print(theta3d_dot);Serial.print(",");
  // Serial.print(x_g);Serial.print(", ");
  // Serial.print(y_g);Serial.print(",");
  // Serial.print(phi_g);Serial.print(", ");
  // Serial.println(">");  //delay(1000);

  // Serial.print(TimeI);Serial.print(",");
  // Serial.print(TimeI_p);Serial.print(",");
  // Serial.print(xd_g);Serial.print(",");
  // Serial.print(xd_g_past);Serial.print(",");
  // Serial.print(xd_dot_g);Serial.print(",");
  // Serial.print(yd_dot_g);Serial.print(",");
  // Serial.println(">");  //delay(1000);
  
  // Serial.print(theta1_dot_RPM_p);Serial.print(",");
  // Serial.print(dt1);Serial.print(",");
  // Serial.print(theta2_dot_RPM_p);Serial.print(",");
  // Serial.print(dt2);Serial.print(",");
  // Serial.print(phi_g_enc);Serial.print(",");
  // Serial.print(phi_g);Serial.print(",");
  // Serial.println(">");  //delay(1000);

  // Serial.print(x_g);Serial.print(",");
  // Serial.print(y_g);Serial.print(",");
  // Serial.print(phi_g);Serial.print(",");
  // Serial.print(x_dot_g);Serial.print(",");
  // Serial.print(y_dot_g);Serial.print(",");
  // Serial.print(phi_dot);Serial.print(",");
  // Serial.println(">");  //delay(1000);

  // Serial.print(xd_g);Serial.print(",");
  // Serial.print(yd_g);Serial.print(",");
  // Serial.print(phid);Serial.print(","); 
  // Serial.print(x_g);Serial.print(",");
  // Serial.print(y_g);Serial.print(",");
  // Serial.print(phi_g);Serial.print(",");
  // Serial.println(">");  //delay(1000);

  // Serial.print(x_g,4);Serial.print(",");
  // Serial.print(y_g,4);Serial.print(",");
  // Serial.print(phi_g,4);Serial.print(","); 
  // Serial.print(x_dot_g,4);Serial.print(",");
  // Serial.print(y_dot_g,4);Serial.print(",");
  // Serial.print(phi_dot,4);Serial.print(",");
  // Serial.println(">");  //delay(1000);

  // Serial.print(xd_g,4);Serial.print(",");
  // Serial.print(yd_g,4);Serial.print(",");
  // Serial.print(phid,4);Serial.print(",");
  // Serial.print(xd_dot_g,4);Serial.print(",");  
  // Serial.print(yd_dot_g,4);Serial.print(","); 
  // Serial.print(phid_dot,4);Serial.print(",");
  // Serial.println(">");  //delay(1000);

  // Serial.print(floatFromAndroid1,3);Serial.print(",");
  // Serial.print(floatFromAndroid2,3);Serial.print(",");
  // Serial.print(floatFromAndroid3,3);Serial.print(","); 
  // Serial.print(floatFromAndroid4,3);Serial.print(",");
  // Serial.print(floatFromAndroid5,3);Serial.print(",");
  // Serial.print(floatFromAndroid6,3);Serial.print(",");
  // Serial.println(">");  //delay(1000);


}
