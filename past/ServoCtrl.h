#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <ArduinoQueue.h>

TwoWire IIC = TwoWire(0);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, IIC);

#define SERVOMIN  263 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  463 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_RANGE 90
#define M_PI 3.1415926

// 0  Forward 6
// 1  -1|^|3- 7
//<2> --|^|-- <8>
//      |||
//<3> --|^|-- <9>
// 4  -2|^|4- 10
// 5  --|^|-- 11





// ------[S-W] means wave-servo, which controls the y axis
//   [S-W]---L-W---O
//                 |
//                 |
//                 |
//                 |
//                 |
//               Ground
double Linkage_W = 19.15;   // The distance between wiggle servo and the plane of the leg linkages. 

//     <<<[S-A][S-B]<<<
//         /     |
//       L-A    L-A
//       /       |
//      O        O
//      |       /
//     L-B    L-C
//      |     /
//      |    /
//      |   /
//      |  /
//      | /
//      O
//     /  
//   L-D
//   /
//  <---90°
//      .L-E
//           .
// ------------------------
double Linkage_S = 12.2;    // The distance between two servos.
double Linkage_A = 40.0;    // The linkage that connected with the servo.
double Linkage_B = 40.0;    // The linkage that limit the direction.
double Linkage_C = 39.8153; // The upper part of the leg.
double Linkage_D = 31.7750; // The lower part of the leg.
double Linkage_E = 30.8076; // The foot.

double WALK_HEIGHT_MAX  = 130;
double WALK_HEIGHT_MIN  = 50;
double WALK_HEIGHT      = 90;
double WALK_LIFT        = 9; // WALK_HEIGHT + WALK_LIFT <= WALK_HEIGHT_MAX.
double WALK_RANGE       = 40;
double WALK_ACC         = 5;

double WALK_EXTENDED_X  = 0;      // turning
// double WALK_EXTENDED_X  = 15;   

double WALK_EXTENDED_Z  = 25;      //  control the extend in y axis, do not been modified also
double WALK_SIDE_MAX    = 30;
double WALK_MASS_ADJUST = 10;
double STAND_HEIGHT     = 95;

uint8_t WALK_STATUS     = 0;
float WALK_CYCLE_GLOBAL = 0;    // 0-1.
float WALK_LIFT_PROP    = 0.25; // WALK_LIFT_TIME<1.This should be the stance config

//YAO, 2/6/2024
double WALK_EXTENDED_Y  = 0;
float ybuffer = 0;
float zbuffer = 0; 
float xbuffer = 0;
//YAO, 2/7/2024, control the angle bias
// float deg_bias = 210;
float deg_bias = 0;
int lastb=0;
int lasta=0;
float YBUFFER[4] = {0,0,0,0};

//YAO, 2/13/2024, climb the stair
float adjust_pitch = 0;
float mean_pitch[20];
ArduinoQueue<float> Queue;
ArduinoQueue<float> Queue_roll;
ArduinoQueue<float> Queue_pitch;
ArduinoQueue<float> Queue_yaw;
float pitchSum = 0;
float rollSum = 0;
float yawSum = 0;
// ------balance parameter---------

//-------------------------------下面这两个参数也没设置过初始值，这一点有点奇怪
float BALANCE_PITCHU_BUFFER;
float BALANCE_ROLL_BUFFER;


float BALANCE_PITCHU_BASE = 0;
float BALANCE_ROLL_BASE   = 0;
float BALANCE_P = 0.00018;

float GLOBAL_STEP  = 0;
int   STEP_DELAY   = 2;
// int   STEP_DELAY   = 2;
float STEP_ITERATE = 0.04;
// float STEP_ITERATE = 0.01;

int SERVO_MOVE_EVERY = 0;
int MAX_TEST = 125;

#define LEG_A_FORE 8
#define LEG_A_BACK 9
#define LEG_A_WAVE 10

#define LEG_B_WAVE 13
#define LEG_B_FORE 14
#define LEG_B_BACK 15

#define LEG_C_FORE 7
#define LEG_C_BACK 6
#define LEG_C_WAVE 5

#define LEG_D_WAVE 2
#define LEG_D_FORE 1
#define LEG_D_BACK 0


extern int ServoMiddlePWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                                 MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                                 MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                                 MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};

int legPosBuffer[12] = { WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z,
                        -WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z,
                         WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z,
                        -WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z};

int GoalPWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};

int LastPWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};

int ServoDirection[16] = {-1,  1,  1,  1,
                           1, -1, -1,  1,
                          -1,  1,  1,  1,
                           1, -1, -1,  1};

double linkageBuffer[32] = {0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0};


// pre render.
float LAxLA = Linkage_A*Linkage_A;
float LBxLB = Linkage_B*Linkage_B;
float LWxLW = Linkage_W*Linkage_W;
float LExLE = Linkage_E*Linkage_E;
float LAxLA_LBxLB = LAxLA - LBxLB;
float LBxLB_LAxLA = LBxLB - LAxLA;
float L_CD = (Linkage_C+Linkage_D)*(Linkage_C+Linkage_D);
float LAx2  = 2 * Linkage_A;
float LBx2  = 2 * Linkage_B;
float E_PI  = 180 / M_PI;
float LSs2  = Linkage_S/2;
float aLCDE = atan((Linkage_C + Linkage_D)/Linkage_E);
float sLEDC = sqrt(Linkage_E*Linkage_E + (Linkage_D+Linkage_C)*(Linkage_D+Linkage_C));
float O_WLP = 1 - WALK_LIFT_PROP;
float WALK_ACCx2 = WALK_ACC*2;
float WALK_H_L = WALK_HEIGHT - WALK_LIFT;

//YAO, 2/7/2024, control the parameter of pitch and roll, PID Controller
// PID constants
double Kp_pitch = 0.5;
double Ki_pitch = 0.1;
double Kd_pitch = 0.01;
double Kp_roll = 0.5;
double Ki_roll = 0.1;
double Kd_roll = 0.01;

// Pitch variables
double last_pitch = 0.0;      // Previous pitch angle
double error_pitch = 0.0;     // Pitch error
double integral_pitch = 0.0;  // Integral of pitch error
double derivative_pitch = 0.0;// Derivative of pitch error


// Roll variables (similar to pitch)
double last_roll = 0.0;
double error_roll = 0.0;
double integral_roll = 0.0;
double derivative_roll = 0.0;


// 2/14/2024, YAO, climb the stairs
int liedown = 0;
float climb_height[5] = {95,95,95,95,95};
float climb_lift[5] = {20,20,20,20,20};

// linearCtrl() function is a simple example, which shows how besselCtrl works.
float besselCtrl(float numStart, float numEnd, float rateInput){
  float numOut;
  numOut = (numEnd - numStart)*((cos(rateInput*M_PI-M_PI)+1)/2) + numStart;
  return numOut;
}

float plainCtrl(float numStart, float numEnd, float rateInput){
  float numOut;
  numOut = (numEnd - numStart)*(rateInput) + numStart;
  return numOut;
}

//expected postion of the COM
// This is OO'
double PosCom[3][1] = {{1}, {1}, {1}};
double PosIMU[3] = {0,0,0};



//climb stairs
bool isup = false;

// upper level control
float turning_direction=0.0;
float last_turning_direction=0.0;
float temp_turning_direction=0.0;
float adjust_yaw = 0.0;
int crab_gait; 

//turningdegree

double global_yaw=0.0;
////////////////////////////YAO END/////////////////////////////////

void ServoSetup(){
  IIC.begin(S_SDA, S_SCL, 26000000);
  pwm.begin();
  pwm.setOscillatorFrequency(26000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  Wire.setClock(100000);
  // Serial.println("16 channel PWM Setup!");
  delay(10);
}


extern void initPosAll(){
  for(int i = 0; i < 16; i++){
    pwm.setPWM(i, 0, MiddlePosition);
    CurrentPWM[i] = MiddlePosition;
    Serial.print(CurrentPWM[i]);
    Serial.print(" ");
    delay(SERVO_MOVE_EVERY);
  }
  Serial.println(" ");
}


extern void middlePosAll(){
  for(int i = 0; i < 16; i++){
    pwm.setPWM(i, 0, ServoMiddlePWM[i]);
    CurrentPWM[i] = ServoMiddlePWM[i];
    Serial.print(CurrentPWM[i]);
    Serial.print(" ");
    delay(SERVO_MOVE_EVERY);
  }
  Serial.println(" ");
}


extern void servoDebug(byte servoID, int offset){
  CurrentPWM[servoID] += offset;
  pwm.setPWM(servoID, 0, CurrentPWM[servoID]);  
}


void GoalPosAll(){
  for(int i = 0; i < 16; i++){
    pwm.setPWM(i, 0, GoalPWM[i]);
  }
}


void goalPWMSet(uint8_t servoNum, double angleInput){
  int pwmGet;
  if (angleInput == 0){pwmGet = 0;}
  else{pwmGet = round((SERVOMAX - SERVOMIN) * angleInput / SERVO_RANGE);}
  pwmGet = pwmGet * ServoDirection[servoNum] + ServoMiddlePWM[servoNum];
  GoalPWM[servoNum] = pwmGet;
}


// Simple Linkage IK:
// input the position of the end and return angle.
//   O----O
//  /
// O
// ---------------------------------------------------
// |       /beta           /delta                    |
//        O----LB---------X------                    |
// |     /       omega.   |       \LB                |
//      LA        .                < ----------------|
// |alpha     .          bIn         \LB -EP  <delta |
//    /psi.                           \LB -EP        |
// | /.   lambda          |                          |
// O- - - - - aIn - - - - X -                        |
// ---------------------------------------------------

// alpha, beta > 0 ; delta <= 0 ; aIn, bIn > 0
// simpleLinkageIK(Linkage_A, Linkage_B, linkageBuffer[yPosBuffer], (linkageBuffer[xPosBuffer]-Linkage_S/2), betaOut, betaB, betaC);

void simpleLinkageIK(double LA, double LB, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputBeta, uint8_t outputDelta){
  double psi;
  double alpha;
  double omega;
  double beta;
  double L2C;
  double LC;
  double lambda;
  double delta;
  //
  //float LAxLA_LBxLB = LAxLA - LBxLB;
  //float LBxLB_LAxLA = LBxLB - LAxLA;
  //
  if(bIn == 0){
    psi   = acos((LAxLA_LBxLB + aIn * aIn)/(LAx2 * aIn)) * E_PI;
    alpha = 90 - psi;
    omega = acos((aIn * aIn + LBxLB_LAxLA)/(LBx2 * aIn)) * E_PI;
    beta  = psi + omega;
  }
  else{
    L2C = aIn * aIn + bIn * bIn;
    LC  = sqrt(L2C);
    lambda = atan(bIn/aIn) * 180 / M_PI;
    psi    = acos((LAxLA_LBxLB + L2C)/(2 * LA * LC)) * E_PI;
    alpha  = 90 - lambda - psi;
    omega  = acos((LBxLB_LAxLA + L2C)/(2 * LC * LB)) * E_PI;
    beta   = psi + omega;
  }
  delta = 90 - alpha - beta;
  linkageBuffer[outputAlpha] = alpha;
  linkageBuffer[outputBeta]  = beta;
  linkageBuffer[outputDelta] = delta;
}

// ||| ||| ||| ||| ||| |||
// void simpleLinkageIK(double LA, double LB, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputBeta, uint8_t outputDelta){
//   double psi;
//   double alpha;
//   double omega;
//   double beta;
//   double L2C;
//   double LC;
//   double lambda;
//   double delta;
//   if(bIn == 0){
//     psi   = acos((LA * LA + aIn * aIn - LB * LB)/(2 * LA * aIn)) * 180 / M_PI;
//     alpha = 90 - psi;
//     omega = acos((aIn * aIn + LB * LB - LA * LA)/(2 * aIn * LB)) * 180 / M_PI;
//     beta  = psi + omega;
//   }
//   else{
//     L2C = aIn * aIn + bIn * bIn;
//     LC  = sqrt(L2C);
//     lambda = atan(bIn/aIn) * 180 / M_PI;
//     psi    = acos((LA * LA + L2C - LB * LB)/(2 * LA * LC)) * 180 / M_PI;
//     alpha = 90 - lambda - psi;
//     omega = acos((LB * LB + L2C - LA * LA)/(2 * LC * LB)) * 180 / M_PI;
//     beta  = psi + omega;
//   }
//   delta = 90 - alpha - beta;
//   linkageBuffer[outputAlpha] = alpha;
//   linkageBuffer[outputBeta]  = beta;
//   linkageBuffer[outputDelta] = delta;
// }


// Wiggle Plane IK:
// input the position of the end and return angle.
// O-----X
//       |
//       |
//       O
// ------------------------------
//     X                        |
//    /    .                    
//  LA         .                |
//  /alpha         .LB         
// O- - - - - - - - - -.- - - -X|
//                         .  bIn
// ------------aIn-------------X|
// ------------------------------
// alpha, aIn, bIn > 0
// wigglePlaneIK(Linkage_W, zPos, yPos, wiggleAlpha, wiggleLen);
void wigglePlaneIK(double LA, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputLen){
  double LB;
  double L2C;
  double LC;
  double alpha;
  double beta;
  double lambda;
  double psi;
  if(bIn > 0){
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan(aIn/bIn) * E_PI;
    psi = acos(LA/LC) * E_PI;
    LB = sqrt(L2C - LWxLW);
    alpha = psi + lambda - 90;
  }
  else if(bIn == 0){
    alpha = asin(LA/aIn) * E_PI;
    L2C = aIn * aIn + bIn * bIn;
    LB = sqrt(L2C);
  }
  else if(bIn < 0){
    bIn = -bIn;
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan(aIn/bIn) * E_PI;
    psi = acos(LA/LC) * E_PI;
    LB = sqrt(L2C - LWxLW);
    alpha = 90 - lambda + psi;
  }
  linkageBuffer[outputAlpha] = alpha;
  linkageBuffer[outputLen]  = LB;
}

// ||| ||| ||| ||| ||| |||
// void wigglePlaneIK(double LA, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputLen){
//   double LB;
//   double L2C;
//   double LC;
//   double alpha;
//   double beta;
//   double lambda;
//   double psi;
//   if(bIn > 0){
//     L2C = aIn * aIn + bIn * bIn;
//     LC = sqrt(L2C);
//     lambda = atan(aIn/bIn) * 180 / M_PI;
//     psi = acos(LA/LC) * 180 / M_PI;
//     LB = sqrt(L2C - LA * LA);
//     alpha = psi + lambda - 90;
//   }
//   else if(bIn == 0){
//     alpha = asin(LA/aIn) * 180 / M_PI;
//     L2C = aIn * aIn + bIn * bIn;
//     LB = sqrt(L2C);
//   }
//   else if(bIn < 0){
//     bIn = -bIn;
//     L2C = aIn * aIn + bIn * bIn;
//     LC = sqrt(L2C);
//     lambda = atan(aIn/bIn) * 180 / M_PI;
//     psi = acos(LA/LC) * 180 / M_PI;
//     LB = sqrt(L2C - LA * LA);
//     alpha = 90 - lambda + psi;
//   }
//   linkageBuffer[outputAlpha] = alpha;
//   linkageBuffer[outputLen]  = LB;
// }


// Ctrl single leg plane IK:
// input the args of the leg, return angle and position.
//  SE--------LS(O)-----SE
//             |        |  \.
//                      |   LA
//             |        |beta\.
//       \.                   \.
//        \.   |               O
//         \.              .
//          LB |       LC
//           \.    .
//             O
//         LD
//     .       |
// F
//  \.         |
//   LE       yIn
//    \.       |
//     \.--xIn-X  
// beta > 0 ; xIn, yIn > 0


//   singleLegPlaneIK(Linkage_S, Linkage_A, Linkage_C, Linkage_D, Linkage_E, xPos, linkageBuffer[wiggleLen], alphaOut, xPosBuffer, yPosBuffer);
void singleLegPlaneIK(double LS, double LA, double LC, double LD, double LE, double xIn, double yIn, uint8_t outputBeta, uint8_t outputX, uint8_t outputY){
  double bufferS = sqrt((xIn + LSs2)*(xIn + LSs2) + yIn*yIn);
  double lambda = acos(((xIn + LSs2)*(xIn + LSs2) + yIn*yIn + LAxLA - L_CD - LExLE)/(2*bufferS*LA));
  double delta = atan((xIn + LSs2)/yIn);
  double beta = lambda - delta;
  double betaAngle = beta * E_PI;

  double theta = aLCDE;
  double omega = asin((yIn - cos(beta)*LA)/sLEDC);
  double nu = M_PI - theta - omega;
  double dFX = cos(nu)*LE;
  double dFY = sin(nu)*LE;

  double mu = M_PI/2 - nu;
  double dEX = cos(mu)*LD;
  double dEY = sin(mu)*LD;

  double positionX = xIn + dFX - dEX;
  double positionY = yIn - dFY - dEY;
  
  linkageBuffer[outputBeta] = betaAngle;
  linkageBuffer[outputX]  = positionX;
  linkageBuffer[outputY]  = positionY;
}

// ||| ||| ||| ||| ||| |||
// void singleLegPlaneIK(double LS, double LA, double LC, double LD, double LE, double xIn, double yIn, uint8_t outputBeta, uint8_t outputX, uint8_t outputY){
//   double bufferS = sqrt((xIn + LS/2)*(xIn + LS/2) + yIn*yIn);
//   double lambda = acos(((xIn + LS/2)*(xIn + LS/2) + yIn*yIn + LA*LA - (LC+LD)*(LC+LD) - LE*LE)/(2*bufferS*LA));
//   double delta = atan((xIn + LS/2)/yIn);
//   double beta = lambda - delta;
//   double betaAngle = beta * 180 / M_PI;

//   double theta = atan((LC+LD)/LE);
//   double omega = asin((yIn - cos(beta)*LA)/sqrt(LE*LE + (LD+LC)*(LD+LC)));
//   double nu = M_PI - theta - omega;
//   double dFX = cos(nu)*LE;
//   double dFY = sin(nu)*LE;

//   double mu = M_PI/2 - nu;
//   double dEX = cos(mu)*LD;
//   double dEY = sin(mu)*LD;

//   double positionX = xIn + dFX - dEX;
//   double positionY = yIn - dFY - dEY;
  
//   linkageBuffer[outputBeta] = betaAngle;
//   linkageBuffer[outputX]  = positionX;
//   linkageBuffer[outputY]  = positionY;
// }


// Ctrl a single leg of WAVEGO, it moves in a plane.
// input (x,y) position and return angle alpha and angle beta.
//     O  X  O
//    /         .
//   /    |        O
//  O     y     .
//   \.   |  .
//    \.  .
//     O  |
//  .
//   \.   |
//    \-x-X
// ------------------
// x, y > 0
void singleLegCtrl(uint8_t LegNum, double xPos, double yPos, double zPos){
  uint8_t alphaOut;
  uint8_t xPosBuffer;
  uint8_t yPosBuffer;
  uint8_t betaOut;
  uint8_t betaB;
  uint8_t betaC;
  uint8_t NumF;
  uint8_t NumB;
  uint8_t wiggleAlpha;
  uint8_t wiggleLen;
  uint8_t NumW;
  if(LegNum == 1){
    NumF = LEG_A_FORE;
    NumB = LEG_A_BACK;
    NumW = LEG_A_WAVE;
    alphaOut   = 0;
    xPosBuffer = 1;
    yPosBuffer = 2;
    betaOut = 3;
    betaB   = 4;
    betaC   = 5;
    wiggleAlpha = 6;
    wiggleLen   = 7;
  }
  else if(LegNum == 2){
    NumF = LEG_B_FORE;
    NumB = LEG_B_BACK;
    NumW = LEG_B_WAVE;
    alphaOut   = 8;
    xPosBuffer = 9;
    yPosBuffer = 10;
    betaOut = 11;
    betaB   = 12;
    betaC   = 13;
    wiggleAlpha = 14;
    wiggleLen   = 15;
  }
  else if(LegNum == 3){
    NumF = LEG_C_FORE;
    NumB = LEG_C_BACK;
    NumW = LEG_C_WAVE;
    alphaOut   = 16;
    xPosBuffer = 17;
    yPosBuffer = 18;
    betaOut = 19;
    betaB   = 20;
    betaC   = 21;
    wiggleAlpha = 22;
    wiggleLen   = 23;
  }
  else if(LegNum == 4){
    NumF = LEG_D_FORE;
    NumB = LEG_D_BACK;
    NumW = LEG_D_WAVE;
    alphaOut   = 24;
    xPosBuffer = 25;
    yPosBuffer = 26;
    betaOut = 27;
    betaB   = 28;
    betaC   = 29;
    wiggleAlpha = 30;
    wiggleLen   = 31;
  }

  wigglePlaneIK(Linkage_W, zPos, yPos, wiggleAlpha, wiggleLen);
  singleLegPlaneIK(Linkage_S, Linkage_A, Linkage_C, Linkage_D, Linkage_E, xPos, linkageBuffer[wiggleLen], alphaOut, xPosBuffer, yPosBuffer);
  simpleLinkageIK(Linkage_A, Linkage_B, linkageBuffer[yPosBuffer], (linkageBuffer[xPosBuffer]-Linkage_S/2), betaOut, betaB, betaC);

  goalPWMSet(NumW, linkageBuffer[wiggleAlpha]);
  goalPWMSet(NumF, (90 - linkageBuffer[betaOut]));
  goalPWMSet(NumB, linkageBuffer[alphaOut]);
}


void standUp(double cmdInput){
  singleLegCtrl(1, WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
  singleLegCtrl(2, -WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
  singleLegCtrl(3, WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
  singleLegCtrl(4, -WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
}


// Ctrl the gait of a single leg with the variable cycleInput change between 0-1.
// when the directionInput > 0, the front point in the gait cycle move away from the middle line of the robot.
// use the extendedX and extendedZ to adjust the position of the middle point in a wiggle cycle.
// statusInput used to reduce the WALK_RANGE.
void singleGaitCtrl(uint8_t LegNum, uint8_t statusInput, float cycleInput, float directionInput, double extendedX, double extendedZ){
  double rDist;
  double xGait;
  double yGait;
  double zGait;
  double rDiection = directionInput * M_PI / 180;

  // WALK_LIFT_PROP = 0.25
  // WALK_ACC = 5
  // WALK_RANGE = 40

  if(cycleInput < (1 - WALK_LIFT_PROP))
  {
    if(cycleInput <= (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      yGait = (WALK_HEIGHT - WALK_LIFT) + cycleInput/  (1-WALK_LIFT_PROP-((WALK_ACC+WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))  *WALK_LIFT;
    }
    
    else if(cycleInput > (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput <= ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      yGait = WALK_HEIGHT;
    }
    
    else if(cycleInput > ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput < ((WALK_ACC*2 + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      yGait = WALK_HEIGHT - ((cycleInput-((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))/((WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)))*WALK_LIFT;
    }

    rDist = (WALK_RANGE*statusInput/2 + WALK_ACC) - (cycleInput/(1 - WALK_LIFT_PROP))*(WALK_RANGE*statusInput + WALK_ACC*2);
  }


  else if(cycleInput >= (1 - WALK_LIFT_PROP)){
    yGait = WALK_HEIGHT - WALK_LIFT;
    rDist = - (WALK_RANGE*statusInput/2 + WALK_ACC) + ((cycleInput-(1-WALK_LIFT_PROP))/WALK_LIFT_PROP)*(WALK_RANGE*statusInput + WALK_ACC*2);
  }

  xGait = cos(rDiection) * rDist;
  zGait = sin(rDiection) * rDist;
  singleLegCtrl(LegNum, (xGait + extendedX), yGait, (zGait + extendedZ));
}


//YAO , 2/6/2024, change the gait to make at least 3 foots on the ground
void notsingleGaitCtrl(uint8_t LegNum, uint8_t statusInput, float cycleInput, float directionInput, double extendedX, double extendedZ){
  double rDist;
  double xGait;
  double yGait;
  double zGait;
  double rDiection = directionInput * M_PI / 180;

  // WALK_LIFT_PROP = 0.125
  // WALK_ACC = 5
  // WALK_RANGE = 70

  //adjust the cycle input to make the second acc at the beginning
  cycleInput += ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP);
  if(cycleInput>1){cycleInput--;}

  if(cycleInput < (1 - WALK_LIFT_PROP))
  {
    if(cycleInput <= (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      yGait = (WALK_HEIGHT - WALK_LIFT) + cycleInput/  (1-WALK_LIFT_PROP-((WALK_ACC+WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))  *WALK_LIFT;
    }
    
    else if(cycleInput > (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput <= ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      yGait = WALK_HEIGHT;
    }
    
    else if(cycleInput > ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput < ((WALK_ACC*2 + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      yGait = WALK_HEIGHT - ((cycleInput-((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))/((WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)))*WALK_LIFT;
    }

    rDist = (WALK_RANGE*statusInput/2 + WALK_ACC) - (cycleInput/(1 - WALK_LIFT_PROP))*(WALK_RANGE*statusInput + WALK_ACC*2);
  }

  //double WALK_HEIGHT      = 95;
  //double WALK_LIFT        = 9; // WALK_HEIGHT + WALK_LIFT <= WALK_HEIGHT_MAX.
  else if(cycleInput >= (1 - WALK_LIFT_PROP)){
    yGait = WALK_HEIGHT - WALK_LIFT;
    rDist = - (WALK_RANGE*statusInput/2 + WALK_ACC) + ((cycleInput-(1-WALK_LIFT_PROP))/WALK_LIFT_PROP)*(WALK_RANGE*statusInput + WALK_ACC*2);
  }

  xGait = cos(rDiection) * rDist;
  zGait = sin(rDiection) * rDist;
  singleLegCtrl(LegNum, (xGait + extendedX), yGait, (zGait + extendedZ));
}





// YAO, 2/6/2024, self-balancing gait
void autosingleGaitCtrl(uint8_t LegNum, uint8_t statusInput, float cycleInput, float directionInput, float target_pitch, float target_roll){
  double rDist;
  double xGait;
  double yGait;
  double zGait;
  double rDiection = directionInput * M_PI / 180;

  double upper_limit = 0;
  double lower_limit = 0;

  zbuffer = 0;
  //deal with IMU data
  pr_update();
  Serial.print("pitch: ");
  Serial.print(pitch);
  Serial.print("  ");
  Serial.print("roll: ");
  Serial.println(roll);
  // Serial.println("Min:-10,Max:10");


  Serial.print("target_pitch: ");
  Serial.print(target_pitch);
  Serial.print("  ");
  Serial.print("target_roll: ");
  Serial.println(target_roll);


  // Serial.print(atan2(pitch,roll)/M_PI*180);
  // Serial.print(" ");
  // Serial.println(atan2(target_pitch,target_roll)/M_PI*180);

  // WALK_LIFT_PROP = 0.125
  // WALK_ACC = 5
  // WALK_RANGE = 70

  //adjust the cycle input to make the second acc at the beginning
  cycleInput += ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP);
  if(cycleInput>1){cycleInput--;}

  if(cycleInput < (1 - WALK_LIFT_PROP))
  {
    if(cycleInput <= (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      yGait = (WALK_HEIGHT - WALK_LIFT) + cycleInput/  (1-WALK_LIFT_PROP-((WALK_ACC+WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))  *WALK_LIFT;
    }
    
    //Contact Phase
    else if(cycleInput > (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput <= ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {

      //PID Controller
      error_pitch = target_pitch - pitch;
      integral_pitch += error_pitch;
      derivative_pitch = pitch - last_pitch;
      double PID_pitch = Kp_pitch * error_pitch + Ki_pitch * integral_pitch + Kd_pitch * derivative_pitch;
      last_pitch = pitch;  // Update last pitch for the next iteration

      error_roll = target_roll - roll;
      integral_roll += error_roll;
      derivative_roll = roll - last_roll;
      double PID_roll = Kp_roll * error_roll + Ki_roll * integral_roll + Kd_roll * derivative_roll;
      last_roll = roll;  // Update last roll for the next iteration


      //adjust Y
      // if(LegNum == 1){YBUFFER[LegNum-1]  = (YBUFFER[LegNum-1] + PID_pitch + PID_roll);}
      // else if(LegNum == 2){YBUFFER[LegNum-1]  = YBUFFER[LegNum-1] + PID_pitch - PID_roll;}
      // else if(LegNum == 3){YBUFFER[LegNum-1]  = (YBUFFER[LegNum-1] - PID_pitch + PID_roll);}
      // else if(LegNum == 4){YBUFFER[LegNum-1]  = YBUFFER[LegNum-1] - PID_pitch - PID_roll;}
      // if( WALK_HEIGHT +YBUFFER[LegNum-1] > WALK_HEIGHT_MAX){YBUFFER[LegNum-1] = WALK_HEIGHT_MAX - WALK_HEIGHT;}
      // if( WALK_HEIGHT +YBUFFER[LegNum-1] < WALK_HEIGHT_MIN){YBUFFER[LegNum-1] = WALK_HEIGHT_MIN - WALK_HEIGHT;}
      // yGait = WALK_HEIGHT + YBUFFER[LegNum-1];


      // if(LegNum == 1){YBUFFER[LegNum-1]  = (WALK_HEIGHT + PID_pitch + PID_roll*0.3);}
      // else if(LegNum == 2){YBUFFER[LegNum-1]  = WALK_HEIGHT + PID_pitch - PID_roll*0.3;}
      // else if(LegNum == 3){YBUFFER[LegNum-1]  = (WALK_HEIGHT - PID_pitch + PID_roll*0.3);}
      // else if(LegNum == 4){YBUFFER[LegNum-1]  = WALK_HEIGHT - PID_pitch - PID_roll*0.3;}
      // if( YBUFFER[LegNum-1] > WALK_HEIGHT_MAX){YBUFFER[LegNum-1] = WALK_HEIGHT_MAX;}
      // if( YBUFFER[LegNum-1] < WALK_HEIGHT_MIN){YBUFFER[LegNum-1] = WALK_HEIGHT_MIN;}
      // yGait = YBUFFER[LegNum-1];

      if(LegNum == 1){YBUFFER[LegNum-1]  += (PID_pitch + PID_roll*0.3);}
      else if(LegNum == 2){YBUFFER[LegNum-1]  += PID_pitch - PID_roll*0.3;}
      else if(LegNum == 3){YBUFFER[LegNum-1]  += (PID_pitch + PID_roll*0.3);}
      else if(LegNum == 4){YBUFFER[LegNum-1]  += PID_pitch - PID_roll*0.3;}
      


      //limit each YBUFFER in range(-20,20)
      float threshold_h = float(10);
      for(int i=0;i<=3;i++)
      {
        YBUFFER[i] = min(YBUFFER[i],threshold_h);
        YBUFFER[i] = max(YBUFFER[i],-threshold_h);
      }



      //limit the average height within (upper_limit,lower_limit) - (20,-20)
      double total_height_buffer;
      total_height_buffer = YBUFFER[0] + YBUFFER[1] + YBUFFER[2] + YBUFFER[3];
      if(total_height_buffer >= upper_limit * 4)
      {
        YBUFFER[0] -= (total_height_buffer/4 - upper_limit);
        YBUFFER[1] -= (total_height_buffer/4 - upper_limit);
        YBUFFER[2] -= (total_height_buffer/4 - upper_limit);
        YBUFFER[3] -= (total_height_buffer/4 - upper_limit);
      }

      if(total_height_buffer <= lower_limit * 4)
      {
        YBUFFER[0] -= (total_height_buffer/4 - lower_limit);
        YBUFFER[1] -= (total_height_buffer/4 - lower_limit);
        YBUFFER[2] -= (total_height_buffer/4 - lower_limit);
        YBUFFER[3] -= (total_height_buffer/4 - lower_limit);
      }



      // Serial.print(YBUFFER[0]);
      // Serial.print("  ");
      // Serial.print(YBUFFER[1]);
      // Serial.print("  ");
      // Serial.print(YBUFFER[2]);
      // Serial.print("  ");
      // Serial.println(YBUFFER[3]);

      yGait = WALK_HEIGHT+YBUFFER[LegNum-1];
      yGait = min(yGait, WALK_HEIGHT_MAX);
      yGait = max(yGait, WALK_HEIGHT_MIN);

      //adjust Z
      if(LegNum == 1){zbuffer = -PID_roll;}
      else if(LegNum == 2){zbuffer = -PID_roll;}
      else if(LegNum == 3){zbuffer = PID_roll;}
      else if(LegNum == 4){zbuffer = PID_roll;}
      if(zbuffer > WALK_SIDE_MAX){zbuffer = WALK_SIDE_MAX;}
      if(zbuffer < -WALK_SIDE_MAX){zbuffer = -WALK_SIDE_MAX;}

    }
    
    else if(cycleInput > ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput < ((WALK_ACC*2 + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      yGait = WALK_HEIGHT - ((cycleInput-((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))/((WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)))*WALK_LIFT;
    }

    rDist = (WALK_RANGE*statusInput/2 + WALK_ACC) - (cycleInput/(1 - WALK_LIFT_PROP))*(WALK_RANGE*statusInput + WALK_ACC*2);
  }

  //double WALK_HEIGHT      = 95;
  //double WALK_LIFT        = 9; 
  // WALK_HEIGHT + WALK_LIFT <= WALK_HEIGHT_MAX.
  else if(cycleInput >= (1 - WALK_LIFT_PROP)){
    yGait = WALK_HEIGHT - WALK_LIFT;
    rDist = - (WALK_RANGE*statusInput/2 + WALK_ACC) + ((cycleInput-(1-WALK_LIFT_PROP))/WALK_LIFT_PROP)*(WALK_RANGE*statusInput + WALK_ACC*2);
  }

  // xGait = cos(rDiection) * rDist;
  xGait = 0;
  zGait = sin(rDiection) * rDist;
  if(LegNum==1||LegNum==3){singleLegCtrl(LegNum, (xGait + WALK_EXTENDED_X), yGait, (zGait + WALK_EXTENDED_Z + zbuffer));}
  else{singleLegCtrl(LegNum, (xGait - WALK_EXTENDED_X), yGait, (zGait + WALK_EXTENDED_Z + zbuffer));}

  // Serial.print("yGait: ");
  // Serial.print(yGait);
  // Serial.print("  ");
  // Serial.print("zbuffer: ");
  // Serial.println(zbuffer);
}


// YAO, 2/12/2024, self-balancing gait without IMU
void roundsingleGaitCtrl(uint8_t LegNum, float kinput, float cycleInput, float directionInput, float target_pitch, float target_roll){
  double rDist;
  double xGait;
  double yGait;
  double zGait;
  double rDiection = directionInput * M_PI / 180;

  double upper_limit = 0;
  double lower_limit = 0;


  float statusInput = 1.0;
  zbuffer = 0;
  float distance_k = kinput;
  // WALK_LIFT_PROP = 0.125;
  // WALK_ACC = 5;
  // WALK_RANGE = 70;

  //adjust the cycle input to make the second acc at the beginning
  cycleInput += ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP);
  if(cycleInput>1){cycleInput--;}

  if(cycleInput < (1 - WALK_LIFT_PROP))
  {
    if(cycleInput <= (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      // Serial.print("Lift leg ");
      // Serial.println(LegNum);
      yGait = (climb_height[LegNum] - climb_lift[LegNum]) + cycleInput/  (1-WALK_LIFT_PROP-((WALK_ACC+WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))  *climb_lift[LegNum];
    }
    
    //Contact Phase
    else if(cycleInput > (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput <= ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      if(liedown==0||liedown==1)
      {
        //adjust Y
        if(LegNum == 1){YBUFFER[LegNum-1]  = climb_height[LegNum] + (target_pitch + target_roll*0.3);}
        else if(LegNum == 2){YBUFFER[LegNum-1]  = climb_height[LegNum] + (target_pitch - target_roll*0.3);}
        // else if(LegNum == 2){YBUFFER[LegNum-1]  = WALK_HEIGHT + (target_pitch - target_roll*0.3) + adjust_pitch;}
        else if(LegNum == 3){YBUFFER[LegNum-1]  = climb_height[LegNum] - (target_pitch - target_roll*0.3);}
        else if(LegNum == 4){YBUFFER[LegNum-1]  = climb_height[LegNum] - (target_pitch + target_roll*0.3) ;}
        // else if(LegNum == 4){YBUFFER[LegNum-1]  = WALK_HEIGHT - (target_pitch + target_roll*0.3) + adjust_pitch;}

        if( YBUFFER[LegNum-1] > WALK_HEIGHT_MAX){YBUFFER[LegNum-1] = WALK_HEIGHT_MAX;}
        if( YBUFFER[LegNum-1] < WALK_HEIGHT_MIN){YBUFFER[LegNum-1] = WALK_HEIGHT_MIN;}
        yGait = YBUFFER[LegNum-1];
      }
      //in the climbing phase, we only adjust the z axis
      else
      {
        yGait = climb_height[LegNum];
      }

      //adjust Z
      //this 0.5 works when x=0 
      // float asso = 0.5;


      float asso = 0.3;
      if(LegNum == 1){zbuffer = -target_roll * asso;}
      else if(LegNum == 2){zbuffer = -target_roll * asso;}
      else if(LegNum == 3){zbuffer = target_roll * asso;}
      else if(LegNum == 4){zbuffer = target_roll * asso;}
      if(zbuffer > WALK_SIDE_MAX){zbuffer = WALK_SIDE_MAX;}
      if(zbuffer < -WALK_SIDE_MAX){zbuffer = -WALK_SIDE_MAX;}

    }
    
    else if(cycleInput > ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput < ((WALK_ACC*2 + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      // Serial.print("Lift leg ");
      // Serial.println(LegNum);
      yGait = climb_height[LegNum] - ((cycleInput-((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))/((WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)))*climb_lift[LegNum];
    }

    rDist = (WALK_RANGE*distance_k/2 + WALK_ACC) - (cycleInput/(1 - WALK_LIFT_PROP))*(WALK_RANGE*distance_k + WALK_ACC*2);
  }

  //double WALK_HEIGHT      = 95;
  //double WALK_LIFT        = 9; 
  // WALK_HEIGHT + WALK_LIFT <= WALK_HEIGHT_MAX.
  else if(cycleInput >= (1 - WALK_LIFT_PROP)){
    yGait = climb_height[LegNum] - climb_lift[LegNum];
    rDist = - (WALK_RANGE*distance_k/2 + WALK_ACC) + ((cycleInput-(1-WALK_LIFT_PROP))/WALK_LIFT_PROP)*(WALK_RANGE*distance_k + WALK_ACC*2);
  }

  // if(LegNum==1){Serial.println(rDist);}
  xGait = cos(rDiection) * rDist;
  // xGait = 0;
  zGait = sin(rDiection) * rDist;

  if(LegNum==1 || LegNum==2){xGait = xGait*0.8;}

  if(LegNum==1||LegNum==3){singleLegCtrl(LegNum, max((xGait + WALK_EXTENDED_X),-30.0), yGait- adjust_pitch, (zGait + WALK_EXTENDED_Z + zbuffer));}
  else{singleLegCtrl(LegNum, max((xGait - WALK_EXTENDED_X),-30.0), yGait, (zGait + WALK_EXTENDED_Z + zbuffer));}




  // Serial.print(YBUFFER[0]);
  // Serial.print("  ");
  // Serial.print(YBUFFER[1]);
  // Serial.print("  ");
  // Serial.print(YBUFFER[2]);
  // Serial.print("  ");
  // Serial.println(YBUFFER[3]);

  // Serial.print("zbuffer: ");
  // Serial.print(zbuffer);
  // Serial.print("zGait: ");
  // Serial.print(zGait);
  // Serial.print("WALK_EXTENDED_Z: ");
  // Serial.print(WALK_EXTENDED_Z);
}

//YAO, 2/16/2024, steady-state gait

void walksingleGaitCtrl(uint8_t LegNum, float kinput, float cycleInput, float directionInput, float target_pitch, float target_roll){
  double rDist;
  double xGait;
  double yGait;
  double zGait;
  double rDiection = directionInput * M_PI / 180;

  double upper_limit = 0;
  double lower_limit = 0;


  float statusInput = 1.0;
  zbuffer = 0;
  float distance_k = kinput;


  //adjust the cycle input to make the second acc at the beginning
  cycleInput += ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP);
  if(cycleInput>1){cycleInput--;}

  if(cycleInput < (1 - WALK_LIFT_PROP))
  {
    if(cycleInput <= (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      // Serial.print("Lift leg ");
      // Serial.println(LegNum);
      yGait = (climb_height[LegNum] - climb_lift[LegNum]) + cycleInput/  (1-WALK_LIFT_PROP-((WALK_ACC+WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))  *climb_lift[LegNum];
    }
    
    //Contact Phase
    else if(cycleInput > (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput <= ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      if(liedown==0||liedown==1)
      {
        //adjust Y
        if(LegNum == 1){YBUFFER[LegNum-1]  =   (target_pitch + target_roll*0.3);}
        else if(LegNum == 2){YBUFFER[LegNum-1]  =   (target_pitch - target_roll*0.3);}
        else if(LegNum == 3){YBUFFER[LegNum-1]  =  - (target_pitch - target_roll*0.3);}
        else if(LegNum == 4){YBUFFER[LegNum-1]  =  - (target_pitch + target_roll*0.3) ;}
        // if( YBUFFER[LegNum-1] > 6){YBUFFER[LegNum-1] = 6;}
        // if( YBUFFER[LegNum-1] < WALK_HEIGHT_MIN){YBUFFER[LegNum-1] = WALK_HEIGHT_MIN;}
        yGait = YBUFFER[LegNum-1]+climb_height[LegNum];
      }

      Serial.print(YBUFFER[0]);
      Serial.print("  ");
      Serial.print(YBUFFER[1]);
      Serial.print("  ");
      Serial.print(YBUFFER[2]);
      Serial.print("  ");
      Serial.println(YBUFFER[3]);
      Serial.println("Min:-20,Max:20");


      //adjust Z
      //this 0.5 works when x=0 
      // float asso = 0.5;

      // float asso = 0.3;
      // if(LegNum == 1){zbuffer = -target_roll * asso;}
      // else if(LegNum == 2){zbuffer = -target_roll * asso;}
      // else if(LegNum == 3){zbuffer = target_roll * asso;}
      // else if(LegNum == 4){zbuffer = target_roll * asso;}
      // if(zbuffer > WALK_SIDE_MAX){zbuffer = WALK_SIDE_MAX;}
      // if(zbuffer < -WALK_SIDE_MAX){zbuffer = -WALK_SIDE_MAX;}

    }
    
    else if(cycleInput > ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput < ((WALK_ACC*2 + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))
    {
      yGait = climb_height[LegNum] - ((cycleInput-((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))/((WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)))*climb_lift[LegNum];
    }

    rDist = (WALK_RANGE*distance_k/2 + WALK_ACC) - (cycleInput/(1 - WALK_LIFT_PROP))*(WALK_RANGE*distance_k + WALK_ACC*2);
  }

  //double WALK_HEIGHT      = 95;
  //double WALK_LIFT        = 9; 
  // WALK_HEIGHT + WALK_LIFT <= WALK_HEIGHT_MAX.
  else if(cycleInput >= (1 - WALK_LIFT_PROP)){
    yGait = climb_height[LegNum] - climb_lift[LegNum];
    rDist = - (WALK_RANGE*distance_k/2 + WALK_ACC) + ((cycleInput-(1-WALK_LIFT_PROP))/WALK_LIFT_PROP)*(WALK_RANGE*distance_k + WALK_ACC*2);
  }

  xGait = cos(rDiection) * rDist;
  // xGait = 0;
  zGait = sin(rDiection) * rDist;

  if(LegNum==1||LegNum==3){singleLegCtrl(LegNum, (xGait + WALK_EXTENDED_X+xbuffer), yGait, (zGait + WALK_EXTENDED_Z + zbuffer));}
  else{singleLegCtrl(LegNum, (xGait - WALK_EXTENDED_X+xbuffer), yGait, (zGait + WALK_EXTENDED_Z + zbuffer));}




  // Serial.print(YBUFFER[0]);
  // Serial.print("  ");
  // Serial.print(YBUFFER[1]);
  // Serial.print("  ");
  // Serial.print(YBUFFER[2]);
  // Serial.print("  ");
  // Serial.println(YBUFFER[3]);

  // Serial.print("zbuffer: ");
  // Serial.print(zbuffer);
  // Serial.print("zGait: ");
  // Serial.print(zGait);
  // Serial.print("WALK_EXTENDED_Z: ");
  // Serial.print(WALK_EXTENDED_Z);
}

//YAO, 2/21/2024, wirte my own gait control function

//first let's add matrix add and multiplication function
const int rows=3, cols=1;
// 3*1 matrix + 3*1 matrix
void matadd(double A[rows][cols], double B[rows][cols], double result[rows][cols]) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}
//3*3 matrix multiplies 3*1 matrix 
// this is used for rotation matrix multiply OA' vector 
void matmul(double A[rows][rows], double B[rows][cols], double result[rows][cols]) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[i][j] = 0;
            for (int k = 0; k < rows; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// 3*3 matrix multiplies 3*3 matrix
// this is used to calculate the rotation matrix, rotx(Roll) * roty(Pitch) * rotz(Yaw)
void matmulXX(double A[rows][rows], double B[rows][rows], double result[rows][rows]) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < rows; ++j) {
            result[i][j] = 0;
            for (int k = 0; k < rows; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

////////////////////////////////////////////define the matrix in calculating the pos //////////////////////////
//the vector from joint to foot
///////// This is what we want ///////////
double TransPos[4][3][1] = {{{1}, {1}, {1}},   //leg1
                           {{1}, {1}, {1}},   //leg2
                           {{1}, {1}, {1}},   //leg3
                           {{1}, {1}, {1}}};  //leg4

double StartPos[4][3][1] = {{{1}, {1}, {1}},   //leg1
                           {{1}, {1}, {1}},   //leg2
                           {{1}, {1}, {1}},   //leg3
                           {{1}, {1}, {1}}};  //leg4

double CurrentPos[4][3][1] = {{{1}, {1}, {1}},   //leg1
                           {{1}, {1}, {1}},   //leg2
                           {{1}, {1}, {1}},   //leg3
                           {{1}, {1}, {1}}};  //leg4

// TransCoord: this is the result of M*O'A1
double TransCoord[3][1] = {{1}, {1}, {1}};
double addtemp[3][1] = {{1}, {1}, {1}};

/////////////////////////////////////// This is fixed /////////////////////////
// Com_to_Leg1: This is O'A1
// Proj_to_leg: This is OD1
double Com_to_Leg1[3][1] = {{130.0/2}, {55.0/2}, {0.0}};
double Proj_to_Leg1[3][1] = {{130.0/2+WALK_EXTENDED_X}, {55.0/2+WALK_EXTENDED_Z}, {0.0}};

double Com_to_Leg2[3][1] = {{-130.0/2}, {55.0/2}, {0.0}};
double Proj_to_Leg2[3][1] = {{-130.0/2-WALK_EXTENDED_X}, {55.0/2+WALK_EXTENDED_Z}, {0.0}};

double Com_to_Leg3[3][1] = {{130.0/2}, {-55.0/2}, {0.0}};
double Proj_to_Leg3[3][1] = {{130.0/2+WALK_EXTENDED_X}, {-55.0/2-WALK_EXTENDED_Z}, {0.0}};

double Com_to_Leg4[3][1] = {{-130.0/2}, {-55.0/2}, {0.0}};
double Proj_to_Leg4[3][1] = {{-130.0/2-WALK_EXTENDED_X}, {-55.0/2-WALK_EXTENDED_Z}, {0.0}};
////////////////////////////////////////////////////////////////////////////////


/// @brief calculate rotation matrix /////////////////
double Rmat[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
double tempR[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
// using the WALK_LIFT_PROP as the swing phase

// the Transition matrix is stored in Rmat
void calculate_transmatrix_M(double roll, double pitch, double yaw)
{
  double rotx[3][3] = {
    {1, 0,         0},
    {0, cos(roll), -sin(roll)},
    {0, sin(roll), cos(roll)}
  };
  double roty[3][3] = {
    {cos(pitch),  0,  sin(pitch)},
    {0,           1,  0},
    {-sin(pitch), 0,  cos(pitch)}
  };
  double rotz[3][3] = {
    {cos(yaw), -sin(yaw), 0},
    {sin(yaw), cos(yaw),  0},
    {0,        0,         1}
  };

  //calculate the rotation matrix
  matmulXX(rotz,roty,tempR);
  matmulXX(tempR,rotx,Rmat);
  //result is stored in Rmat
  return;
}

/////////////////////// Rotation matrix end /////////////////////////////

double (*makeArrayNegative(double array[3][1]))[1] {
  for (int i = 0; i < 3; i++) {
    array[i][0] = -array[i][0];
  }
  return array;
}


//////############################################################################################################
//
                                                                          // this is poscom[3][1]
void newsinglegait(uint8_t LegNum, float cycleInput, float directionInput, double target_IMU[3], double target_COM[3][1], bool crabwalk){

  double swing_height;
  double x_distance,z_distance,r_distance;
  double rDirection = directionInput*M_PI/180;


  //The first phase is the swing phase
  if(cycleInput < WALK_LIFT_PROP)
  {
    //define the proportion of the WALK_LIFT_PROP, WLP ranging from 0 to 1
    /////////////////////////// calculate the swing height ////////////////////////////
    float WLP = cycleInput/WALK_LIFT_PROP;
    if(WLP<0.5){ swing_height = (sin( (WLP / 0.5)*(M_PI / 2) + M_PI*3/2) + 1) * WALK_LIFT; }
    else{ swing_height = (sin( (WLP - 0.5)/0.5 * (M_PI / 2 + M_PI)) + 1 ) * WALK_LIFT; }
    r_distance = sin(WLP*M_PI/2) * WALK_RANGE - WALK_RANGE/2;
  }
  else
  {
    swing_height = 0;
    double walk_prop = (cycleInput-WALK_LIFT_PROP)/(1-WALK_LIFT_PROP);
    r_distance = WALK_RANGE/2 - WALK_RANGE*walk_prop;
  }



  //calculate the Rmat
  ////////////////////////roll,         pitch,          yaw /////////////
  calculate_transmatrix_M(target_IMU[0], target_IMU[1], target_IMU[2]);
  if(LegNum == 1)
  {
    matmul(Rmat,Com_to_Leg1,TransCoord);
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg1,TransPos[LegNum-1]);
    //the final result is stored in TransPos[i]
  }
  if(LegNum == 2)
  {
    matmul(Rmat,Com_to_Leg2,TransCoord);
    //result is stored in TransCoord
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg2,TransPos[LegNum-1]);

    //result is stored in TransCoord
    // Serial.print(TransCoord[0][0]);
    // Serial.print(" ");
    // Serial.print(TransCoord[1][0]);
    // Serial.print(" ");
    // Serial.print(TransCoord[2][0]);
    // Serial.println(" ");
    // Serial.print(addtemp[0][0]);
    // Serial.print(" ");
    // Serial.print(addtemp[1][0]);
    // Serial.print(" ");
    // Serial.print(addtemp[2][0]);
    // Serial.println(" ");
    // Serial.print(TransPos[LegNum-1][0][0]);
    // Serial.print(" ");
    // Serial.print(TransPos[LegNum-1][1][0]);
    // Serial.print(" ");
    // Serial.print(TransPos[LegNum-1][2][0]);
    // Serial.println(" ");
    // Serial.println(" ");

    //the final result is stored in TransPos[i]
  }
  if(LegNum == 3)
  {
    matmul(Rmat,Com_to_Leg3,TransCoord);
    //result is stored in TransCoord
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg3,TransPos[LegNum-1]);
    //the final result is stored in TransPos[i]
  }
  if(LegNum == 4)
  {
    matmul(Rmat,Com_to_Leg4,TransCoord);
    //result is stored in TransCoord
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg4,TransPos[LegNum-1]);
    //the final result is stored in TransPos[i]
  }
  // limit z_distance in range(-15,15) degree
  // r_distance = 0; //for testing, 原地踏步
  z_distance = max(min(sin(rDirection) * r_distance,15.0),-15.0);
  x_distance = cos(rDirection) * r_distance;
  if(LegNum==4||LegNum==3){singleLegCtrl(LegNum, TransPos[LegNum-1][0][0]+x_distance, -TransPos[LegNum-1][2][0]-swing_height, -TransPos[LegNum-1][1][0]+z_distance);}
  else{singleLegCtrl(LegNum, TransPos[LegNum-1][0][0]+x_distance, -TransPos[LegNum-1][2][0]-swing_height, TransPos[LegNum-1][1][0]+z_distance);}
  
  // for (int i = 0; i < 4; ++i) {
  //   for (int j = 0; j < 3; ++j) {
  //       for (int k = 0; k < 1; ++k) {
  //           Serial.print(TransPos[i][j][k]);
  //           Serial.print(" ");
  //       }
  //   }
  //   Serial.println();  // Move to the next line after printing the array
  // }
  // Serial.println();  // Move to the next line after printing the array
}

//////############################################################################################################
//
                                                                          // this is poscom[3][1]
void newclimbgait(uint8_t LegNum, float cycleInput, double target_IMU[3], double target_COM[3][1]){
  
  double swing_height;
  double x_distance;
  //The first phase is the swing phase
  if(cycleInput < WALK_LIFT_PROP)
  {
    //define the proportion of the WALK_LIFT_PROP, WLP ranging from 0 to 1
    /////////////////////////// calculate the swing height ////////////////////////////
    float WLP = cycleInput/WALK_LIFT_PROP;
    //       ..
    //      : ..
    //     :    ..
    //    :       ..
    //   :          ..
    //  :             ..
    // :                 .. .. 
    // if(WLP<0.5){ swing_height = (sin( (WLP / 0.5)*(M_PI / 2) + M_PI*3/2) + 1) * WALK_LIFT; }
    // else{ swing_height = (sin( (WLP - 0.5)/0.5 * (M_PI / 2 + M_PI)) + 1 ) * WALK_LIFT; }
    // x_distance = sin(WLP*M_PI/2) * WALK_RANGE - WALK_RANGE/2;

        //   .....................
    //      :                     :
    //     :                        :
    //    :                          :
    //   :                            :
    //  :                              :
    // :                                :
    if(WLP<=(1.0/6)){swing_height = (WLP/(1.0/6))*WALK_LIFT;}
    if(WLP>(1.0/6)&&WLP<=(5.0/6)){swing_height = WALK_LIFT;}
    if(WLP>(5.0/6)&&WLP<=1){swing_height = WALK_LIFT-((WLP-(5.0/6))/(1.0/6))*WALK_LIFT;}
    x_distance = WLP* WALK_RANGE - WALK_RANGE/2;
    // Serial.println(swing_height);
    //         ..
    //        :  .
    //      :     .
    //     :       .
    //   :          .
    //  :            .
    // :              . 
    // if(WLP<0.5){ swing_height = (cos( (WLP * M_PI) + M_PI*3/2)) * WALK_LIFT; }
    // else{ swing_height = (cos( (WLP - 0.5)* M_PI)) * WALK_LIFT; }
    // x_distance = (WLP*WLP) * WALK_RANGE - WALK_RANGE/2;
  }
  else
  {
    swing_height = 0;
    double walk_prop = (cycleInput-WALK_LIFT_PROP)/(1-WALK_LIFT_PROP);
    x_distance = WALK_RANGE/2 - WALK_RANGE*walk_prop;
  }

  //calculate the Rmat
  ////////////////////////roll,         pitch,          yaw /////////////
  calculate_transmatrix_M(target_IMU[0], target_IMU[1], target_IMU[2]);
  if(LegNum == 1)
  {
    matmul(Rmat,Com_to_Leg1,TransCoord);
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg1,TransPos[LegNum-1]);
  }
  if(LegNum == 2)
  {
    matmul(Rmat,Com_to_Leg2,TransCoord);
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg2,TransPos[LegNum-1]);
  }
  if(LegNum == 3)
  {
    matmul(Rmat,Com_to_Leg3,TransCoord);
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg3,TransPos[LegNum-1]);
  }
  if(LegNum == 4)
  {
    matmul(Rmat,Com_to_Leg4,TransCoord);
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg4,TransPos[LegNum-1]);
  }

  if(LegNum==4||LegNum==3){singleLegCtrl(LegNum, TransPos[LegNum-1][0][0]+x_distance, -TransPos[LegNum-1][2][0]-swing_height, -TransPos[LegNum-1][1][0]);}
  else{singleLegCtrl(LegNum, TransPos[LegNum-1][0][0]+x_distance, -TransPos[LegNum-1][2][0]-swing_height, TransPos[LegNum-1][1][0]);}
  
}
//########################################################################################################
                                                                          // this is poscom[3][1]
void turningsinglegait(uint8_t LegNum, float cycleInput, double target_IMU[3], double target_COM[3][1], bool is_turn){

  double swing_height;
  double x_pos, z_pos;
  //The first phase is the swing phase
  if(cycleInput < WALK_LIFT_PROP)
  {
    //define the proportion of the WALK_LIFT_PROP, WLP ranging from 0 to 1
    /////////////////////////// calculate the swing height ////////////////////////////
    float WLP = cycleInput/WALK_LIFT_PROP;
    if(WLP<0.5){ swing_height = (sin( (WLP / 0.5)*(M_PI / 2) + M_PI*3/2) + 1) * WALK_LIFT; }
    else{ swing_height = (sin( (WLP - 0.5)/0.5 * (M_PI / 2 + M_PI)) + 1 ) * WALK_LIFT; }
  }
  else
  {
    swing_height = 0;
  }

  //calculate the Rmat
  ////////////////////////roll,         pitch,          yaw /////////////
  calculate_transmatrix_M(target_IMU[0], target_IMU[1], target_IMU[2]);
  if(LegNum == 1)
  {
    matmul(Rmat,Com_to_Leg1,TransCoord);
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg1,TransPos[LegNum-1]);
    //the final result is stored in TransPos[i]
  }
  if(LegNum == 2)
  {
    matmul(Rmat,Com_to_Leg2,TransCoord);
    //result is stored in TransCoord
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg2,TransPos[LegNum-1]);
    //the final result is stored in TransPos[i]
  }
  if(LegNum == 3)
  {
    matmul(Rmat,Com_to_Leg3,TransCoord);
    //result is stored in TransCoord
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg3,TransPos[LegNum-1]);
    //the final result is stored in TransPos[i]
  }
  if(LegNum == 4)
  {
    matmul(Rmat,Com_to_Leg4,TransCoord);
    //result is stored in TransCoord
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg4,TransPos[LegNum-1]);
    //the final result is stored in TransPos[i]
  }

  if(is_turn)  //if is turning, using transpos matrix to turn
  {
    if(LegNum==4||LegNum==3){singleLegCtrl(LegNum, TransPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, -TransPos[LegNum-1][1][0]);}
    else{singleLegCtrl(LegNum, TransPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, TransPos[LegNum-1][1][0]);}
  }
  else    //else, use currentpos matrix to calculate and store current position
  {
    if(cycleInput < WALK_LIFT_PROP)
    {
      float WLP = cycleInput/WALK_LIFT_PROP;
      CurrentPos[LegNum-1][0][0] = plainCtrl(StartPos[LegNum-1][0][0],TransPos[LegNum-1][0][0],WLP);
      CurrentPos[LegNum-1][1][0] = plainCtrl(StartPos[LegNum-1][1][0],TransPos[LegNum-1][1][0],WLP);

      if(LegNum==4||LegNum==3){singleLegCtrl(LegNum, CurrentPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, -CurrentPos[LegNum-1][1][0]);}
      else{singleLegCtrl(LegNum, CurrentPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, CurrentPos[LegNum-1][1][0]);}
    }
    else
    {
      if(LegNum==4||LegNum==3){singleLegCtrl(LegNum, CurrentPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, -CurrentPos[LegNum-1][1][0]);}
      else{singleLegCtrl(LegNum, CurrentPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, CurrentPos[LegNum-1][1][0]);}
    }
  }





}
//########################################################################################################
                                                                          // this is poscom[3][1]
void turningdegreegait(uint8_t LegNum, float cycleInput, double target_IMU[3], double target_COM[3][1], bool is_turn){

  double swing_height;
  double x_pos, z_pos;
  //The first phase is the swing phase
  if(cycleInput < WALK_LIFT_PROP)
  {
    //define the proportion of the WALK_LIFT_PROP, WLP ranging from 0 to 1
    /////////////////////////// calculate the swing height ////////////////////////////
    float WLP = cycleInput/WALK_LIFT_PROP;
    if(WLP<0.5){ swing_height = (sin( (WLP / 0.5)*(M_PI / 2) + M_PI*3/2) + 1) * WALK_LIFT; }
    else{ swing_height = (sin( (WLP - 0.5)/0.5 * (M_PI / 2 + M_PI)) + 1 ) * WALK_LIFT; }
  }
  else
  {
    swing_height = 0;
  }

  //calculate the Rmat
  ////////////////////////roll,         pitch,          yaw /////////////
  calculate_transmatrix_M(target_IMU[0], target_IMU[1], target_IMU[2]);
  if(LegNum == 1)
  {
    matmul(Rmat,Com_to_Leg1,TransCoord);
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg1,TransPos[LegNum-1]);
  }
  if(LegNum == 2)
  {
    matmul(Rmat,Com_to_Leg2,TransCoord);
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg2,TransPos[LegNum-1]);
  }
  if(LegNum == 3)
  {
    matmul(Rmat,Com_to_Leg3,TransCoord);
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg3,TransPos[LegNum-1]);
  }
  if(LegNum == 4)
  {
    matmul(Rmat,Com_to_Leg4,TransCoord);
    matadd(target_COM,makeArrayNegative(TransCoord),addtemp);
    matadd(addtemp,Proj_to_Leg4,TransPos[LegNum-1]);
  }

  if(LegNum==4||LegNum==3){singleLegCtrl(LegNum, TransPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0], -TransPos[LegNum-1][1][0]);}
  else{singleLegCtrl(LegNum, TransPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0], TransPos[LegNum-1][1][0]);}
  
  
  // if(is_turn)  //if is turning, using transpos matrix to turn
  // {
  //   if(LegNum==4||LegNum==3){singleLegCtrl(LegNum, TransPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, -TransPos[LegNum-1][1][0]);}
  //   else{singleLegCtrl(LegNum, TransPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, TransPos[LegNum-1][1][0]);}
  // }
  // else    //else, use currentpos matrix to calculate and store current position
  // {
  //   if(cycleInput < WALK_LIFT_PROP)
  //   {
  //     float WLP = cycleInput/WALK_LIFT_PROP;
  //     CurrentPos[LegNum-1][0][0] = plainCtrl(StartPos[LegNum-1][0][0],TransPos[LegNum-1][0][0],WLP);
  //     CurrentPos[LegNum-1][1][0] = plainCtrl(StartPos[LegNum-1][1][0],TransPos[LegNum-1][1][0],WLP);

  //     if(LegNum==4||LegNum==3){singleLegCtrl(LegNum, CurrentPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, -CurrentPos[LegNum-1][1][0]);}
  //     else{singleLegCtrl(LegNum, CurrentPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, CurrentPos[LegNum-1][1][0]);}
  //   }
  //   else
  //   {
  //     if(LegNum==4||LegNum==3){singleLegCtrl(LegNum, CurrentPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, -CurrentPos[LegNum-1][1][0]);}
  //     else{singleLegCtrl(LegNum, CurrentPos[LegNum-1][0][0], -TransPos[LegNum-1][2][0]-swing_height, CurrentPos[LegNum-1][1][0]);}
  //   }
  // }


}


// a simple gait to ctrl the robot.
// GlobalInput changes between 0-1.
// use directionAngle to ctrl the direction.
// turnCmd-> -1:left 1:right
void simpleGait(float GlobalInput, float directionAngle, int turnCmd){
  float Group_A;
  float Group_B;

  Group_A = GlobalInput;
  Group_B = GlobalInput+0.5;
  if(Group_B>1){Group_B--;}

  if(!turnCmd){
    singleGaitCtrl(1, 1, Group_A, directionAngle,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(4, 1, Group_A, -directionAngle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);

    singleGaitCtrl(2, 1, Group_B, directionAngle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(3, 1, Group_B, -directionAngle, WALK_EXTENDED_X, WALK_EXTENDED_Z);
  }
  else if(turnCmd == -1){
    singleGaitCtrl(1, 1.5, Group_A, 90,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(4, 1.5, Group_A, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);

    singleGaitCtrl(2, 1.5, Group_B, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(3, 1.5, Group_B, -90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
  }
  else if(turnCmd == 1){
    singleGaitCtrl(1, 1.5, Group_A, -90,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(4, 1.5, Group_A, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);

    singleGaitCtrl(2, 1.5, Group_B, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(3, 1.5, Group_B, 90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
  }
}


// Triangular gait generater.
void triangularGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;

  float aInput = 0;
  float bInput = 0;
  float adProp;

  StepB = GlobalInput;
  StepC = GlobalInput + 0.25;
  StepD = GlobalInput + 0.5;
  StepA = GlobalInput + 0.75;

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}

  if(GlobalInput <= 0.25){
    adProp = GlobalInput;
    aInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
    bInput = -WALK_MASS_ADJUST;
  }
  else if(GlobalInput > 0.25 && GlobalInput <= 0.5){
    adProp = GlobalInput-0.25;
    aInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
    bInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
  }
  else if(GlobalInput > 0.5 && GlobalInput <= 0.75){
    adProp = GlobalInput-0.5;
    aInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
    bInput =  WALK_MASS_ADJUST;
  }
  else if(GlobalInput > 0.75 && GlobalInput <= 1){
    adProp = GlobalInput-0.75;
    aInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
    bInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
  }

  if(!turnCmd){
    singleGaitCtrl(1, 1, StepA,  directionAngle,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(4, 1, StepD, -directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

    singleGaitCtrl(2, 1, StepB,  directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(3, 1, StepC, -directionAngle,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  }
  
  else if(turnCmd == -1){
    singleGaitCtrl(1, 1.5, StepA,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(4, 1.5, StepD,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

    singleGaitCtrl(2, 1.5, StepB, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(3, 1.5, StepC, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  }
  else if(turnCmd == 1){
    singleGaitCtrl(1, 1.5, StepA, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(4, 1.5, StepD, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

    singleGaitCtrl(2, 1.5, StepB,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(3, 1.5, StepC,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  }
}

// YAO 6/2/2024, add a custom climb gait
void climbGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;

  float aInput = 0;
  float bInput = 0;
  float adProp;
  float phi;
  float kb = 1;

 

////////////////////////////////////////////calculate gesture sequence/////////////////////////
  //the term is A->B->C->D->A
  StepA = GlobalInput;
  StepB = GlobalInput + 0.75;
  StepC = GlobalInput + 0.5;
  StepD = GlobalInput + 0.25;

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}

  //when 1st leg lift
  if(GlobalInput <= 0.25){
    adProp = GlobalInput;
    phi = (adProp/0.25)*M_PI/2;    //phi going from zero to pi/2
    aInput = WALK_MASS_ADJUST * sin(phi);
    bInput = WALK_MASS_ADJUST * cos(phi) * kb;
  }
  //when 2nd leg lift
  else if(GlobalInput > 0.25 && GlobalInput <= 0.5){
    adProp = GlobalInput-0.25;
    phi = (adProp/0.25)*M_PI/2;    //phi going from zero to pi/2
    aInput = WALK_MASS_ADJUST * cos(phi);
    bInput = -WALK_MASS_ADJUST * sin(phi) * kb;
  }
  //when 3rd leg lift
  else if(GlobalInput > 0.5 && GlobalInput <= 0.75){
    adProp = GlobalInput-0.5;
    phi = (adProp/0.25)*M_PI/2;    //phi going from zero to pi/2
    aInput = -WALK_MASS_ADJUST * sin(phi);
    bInput = -WALK_MASS_ADJUST * cos(phi) * kb;
  }
  // when 4th leg lift
  else if(GlobalInput > 0.75 && GlobalInput <= 1){
    adProp = GlobalInput-0.75;
    phi = (adProp/0.25)*M_PI/2;    //phi going from zero to pi/2
    aInput = -WALK_MASS_ADJUST * cos(phi);
    bInput = WALK_MASS_ADJUST * sin(phi) * kb;
  }

////////////////////////////////////////////////////////////calculate gesture end //////////////////////
  
  // Serial.print("aIn: ");
  // Serial.print(aInput);
  // Serial.print("  ");
  // Serial.print("bIn: ");
  // Serial.println(bInput);
  


  if(!turnCmd){
    notsingleGaitCtrl(1, 1, StepA,  directionAngle,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    notsingleGaitCtrl(3, 1, StepB,  directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    notsingleGaitCtrl(4, 1, StepC, -directionAngle,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
    notsingleGaitCtrl(2, 1, StepD, -directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

  }

  
  else if(turnCmd == -1){
    singleGaitCtrl(1, 1.5, StepA,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(4, 1.5, StepD,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

    singleGaitCtrl(2, 1.5, StepB, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(3, 1.5, StepC, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  }
  else if(turnCmd == 1){
    singleGaitCtrl(1, 1.5, StepA, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(4, 1.5, StepD, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

    singleGaitCtrl(2, 1.5, StepB,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(3, 1.5, StepC,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  }
}


// YAO 6/2/2024, add a auto balacing climb gait
void autoGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;
  float Tpitch, Troll;

////////////////////////////////////////////calculate gesture sequence/////////////////////////
  float aInput = 0;
  float bInput = 0;
  float adProp;
  float phi;
  float kb = 1;

  //the term is A->B->C->D->A
  StepA = GlobalInput;
  StepB = GlobalInput + 0.75;
  StepC = GlobalInput + 0.5;
  StepD = GlobalInput + 0.25;

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}

///////////////////////////////////////////Calculate the target pitch and roll///////////////
  // phi = (GlobalInput)*M_PI*2 + deg_bias/180*M_PI;    //phi going from zero to pi/2
  // //when 1st leg lift

  // // circle mass
  // aInput = WALK_MASS_ADJUST * sin(phi);
  // bInput = WALK_MASS_ADJUST * cos(phi);

  // // Tpitch = -bInput;
  // // Troll = -aInput;

  // float k=1.2;
  // if (aInput>=0&&bInput>=0){aInput = WALK_MASS_ADJUST*k;bInput = WALK_MASS_ADJUST*k;}
  // if (aInput>=0&&bInput<0){aInput = WALK_MASS_ADJUST*k;bInput = -WALK_MASS_ADJUST*k;}
  // if (aInput<0&&bInput>=0){aInput = -WALK_MASS_ADJUST;bInput = WALK_MASS_ADJUST;}
  // if (aInput<0&&bInput<0){aInput = -WALK_MASS_ADJUST;bInput = -WALK_MASS_ADJUST;}

  // Tpitch = -bInput;
  // Troll = -aInput;

  // // if (aInput>0&&bInput>0){Serial.println(4);}
  // // if (aInput>0&&bInput<0){Serial.println(2);}
  // // if (aInput<0&&bInput>0){Serial.println(3);}
  // // if (aInput<0&&bInput<0){Serial.println(1);}
/////////////////////////////////////////////////////// use smoother rectangular ///////////////////////////////
  //GlobalInput ranges from 0 to 1
  float Clock_X,Clock_Y;
  float TX,TY;

  Clock_X = GlobalInput + 0.875;
  if(Clock_X>1){Clock_X--;}
  Clock_Y = Clock_X + 0.75;
  if(Clock_Y>1){Clock_Y--;}

  if(Clock_X>0&&Clock_X<=0.25){TX = besselCtrl(-1,1,Clock_X*4);}
  if(Clock_X>0.25&&Clock_X<=0.5){TX = 1;}
  if(Clock_X>0.5&&Clock_X<=0.75){TX = besselCtrl(1,-1,(Clock_X-0.5)*4);}
  if(Clock_X>0.75&&Clock_X<=1){TX = -1;}

  if(Clock_Y>0&&Clock_Y<=0.25){TY = besselCtrl(-1,1,Clock_Y*4);}
  if(Clock_Y>0.25&&Clock_Y<=0.5){TY = 1;}
  if(Clock_Y>0.5&&Clock_Y<=0.75){TY = besselCtrl(1,-1,(Clock_Y-0.5)*4);}
  if(Clock_Y>0.75&&Clock_Y<=1){TY = -1;}
  
  
  // Serial.print(TX);
  // Serial.print("  ");
  // // Serial.print("roll: ");
  // Serial.println(TY);
  // Serial.println("Min:-5,Max:5");

  Tpitch = TX*  WALK_MASS_ADJUST;
  Troll = TY*  WALK_MASS_ADJUST;

//clear the integral and derivative error when changing target COM
  // if(lastb!=bInput||lasta!=aInput)
  // {
  //   integral_roll = 0;
  //   integral_pitch = 0;
  //   derivative_roll = 0;
  //   derivative_pitch = 0;
    // YBUFFER[0] = 0;
    // YBUFFER[1] = 0;
    // YBUFFER[2] = 0;
    // YBUFFER[3] = 0;
  //}
  // lastb = bInput;
  // lasta = aInput;

////////////////////////////////////////////////////////////calculate gesture end //////////////////////

  if(!turnCmd){
    autosingleGaitCtrl(1, 1, StepA,  directionAngle, Tpitch, Troll);
    autosingleGaitCtrl(3, 1, StepB,  directionAngle, Tpitch, Troll);
    autosingleGaitCtrl(4, 1, StepC, -directionAngle, Tpitch, Troll);
    autosingleGaitCtrl(2, 1, StepD, -directionAngle, Tpitch, Troll);
  }
  
  // else if(turnCmd == -1){
  //   singleGaitCtrl(1, 1.5, StepA,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
  //   singleGaitCtrl(4, 1.5, StepD,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

  //   singleGaitCtrl(2, 1.5, StepB, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
  //   singleGaitCtrl(3, 1.5, StepC, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  // }
  // else if(turnCmd == 1){
  //   singleGaitCtrl(1, 1.5, StepA, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
  //   singleGaitCtrl(4, 1.5, StepD, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

  //   singleGaitCtrl(2, 1.5, StepB,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
  //   singleGaitCtrl(3, 1.5, StepC,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  // }
}


//YAO, 2/12/2024, add a round gait to control the leg without using IMU
void roundGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;
  float Tpitch, Troll;

////////////////////////////////////////////calculate gesture sequence/////////////////////////
  float adProp;
  float phi;

  //the term is A->B->C->D->A
  StepA = GlobalInput;
  StepB = GlobalInput + 0.75;
  StepC = GlobalInput + 0.5;
  StepD = GlobalInput + 0.25;

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}


/////////////////////////////////////////////////////// use smoother rectangular ///////////////////////////////
  //GlobalInput ranges from 0 to 1
  float Clock_X,Clock_Y;
  float TX,TY;

  float LocalInput = GlobalInput;
  // Clock_X = LocalInput + 0.875; this works but not so stable
  Clock_X = LocalInput+0.9;
  if(Clock_X>1){Clock_X--;}
  Clock_Y = Clock_X + 0.75;
  if(Clock_Y>1){Clock_Y--;}

  if(Clock_X>0&&Clock_X<=0.25){TX = besselCtrl(-1,1,Clock_X*4);}
  if(Clock_X>0.25&&Clock_X<=0.5){TX = 1;}
  if(Clock_X>0.5&&Clock_X<=0.75){TX = besselCtrl(1,-1,(Clock_X-0.5)*4);}
  if(Clock_X>0.75&&Clock_X<=1){TX = -1;}

  if(Clock_Y>0&&Clock_Y<=0.25){TY = besselCtrl(-1,1,Clock_Y*4);}
  if(Clock_Y>0.25&&Clock_Y<=0.5){TY = 1;}
  if(Clock_Y>0.5&&Clock_Y<=0.75){TY = besselCtrl(1,-1,(Clock_Y-0.5)*4);}
  if(Clock_Y>0.75&&Clock_Y<=1){TY = -1;}

  Tpitch = -TX*  WALK_MASS_ADJUST;
  Troll = -TY*  WALK_MASS_ADJUST;

  // pr_update();

  ///////// calculate mean pitch ////////////////
  // Queue.enqueue(pitch);
  // if(Queue.itemCount() > 5){pitchSum -= Queue.dequeue();}
  // pitchSum += pitch;
  // float meanpitch = pitchSum / Queue.itemCount();
  // Serial.println(meanpitch);

  ///////////////////////////////////////

  // if(meanpitch<-15)
  // {
  //   liedown = 1;
  //   WALK_MASS_ADJUST = 10;
  //   climb_height[1] = WALK_HEIGHT - 15;
  //   climb_height[2] = WALK_HEIGHT + 15;
  //   climb_height[3] = WALK_HEIGHT - 15;
  //   climb_height[4] = WALK_HEIGHT + 15;
  //   climb_lift[1] = 5;
  //   climb_lift[1] = 20;
  //   climb_lift[1] = 5;
  //   climb_lift[1] = 20;
  // }

  // Tpitch = 0;
  // Troll = 0;

  //uint8_t statusInput 用于控制腿部的摆动幅度的系数，实际幅度 = WALK_RANGE * statusInput。
  if(!turnCmd){
    if(liedown == 1)
    {
      roundsingleGaitCtrl(1, 0.5, StepA,  directionAngle, Tpitch, Troll);
      roundsingleGaitCtrl(3, 0.5, StepB,  directionAngle, Tpitch, Troll);
      roundsingleGaitCtrl(4, 0.5, StepC, -directionAngle, Tpitch, Troll);
      roundsingleGaitCtrl(2, 0.5, StepD, -directionAngle, Tpitch, Troll);
    }
    else
    {
      walksingleGaitCtrl(1, 0.5, StepA,  directionAngle, Tpitch, Troll);
      walksingleGaitCtrl(3, 0.5, StepB,  directionAngle, Tpitch, Troll);
      walksingleGaitCtrl(4, 0.5, StepC, -directionAngle, Tpitch, Troll);
      walksingleGaitCtrl(2, 0.5, StepD, -directionAngle, Tpitch, Troll);
    }
  }

}


//YAO, 2/26/2024, new gait control using newsinglegait()
void newGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;
  float Tpitch, Troll;

////////////////////////////////////////////calculate gesture sequence/////////////////////////

  //the term is A->B->C->D->A
  StepA = GlobalInput - 0.15 + 1;
  StepB = GlobalInput - 0.25 + 1;
  StepC = GlobalInput - 0.65 + 1;
  StepD = GlobalInput - 0.75 + 1;

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}

  // pr_update();
  ///////// calculate mean pitch ////////////////
  // Queue.enqueue(pitch);
  // if(Queue.itemCount() > 5){pitchSum -= Queue.dequeue();}
  // pitchSum += pitch;
  // float meanpitch = pitchSum / Queue.itemCount();
  // Serial.println(meanpitch);

  
  // if(GlobalInput>=0&&GlobalInput<0.25){target_x = besselCtrl(-1,0.75,(GlobalInput)/0.25);PosIMU[1] = besselCtrl(5*M_PI/180,-5*M_PI/180,(GlobalInput)/0.25);}
  // if(GlobalInput>=0.25&&GlobalInput<0.5){target_x = 0.75;PosIMU[1] = -5*M_PI/180;}
  // if(GlobalInput>=0.5&&GlobalInput<0.75){target_x = besselCtrl(0.75,-1,(GlobalInput-0.5)/0.25);PosIMU[1] = besselCtrl(-5*M_PI/180,5*M_PI/180,(GlobalInput-0.5)/0.25);}
  // if(GlobalInput>=0.75&&GlobalInput<1){target_x = -1;PosIMU[1] = 5*M_PI/180;}
  
  double target_x;
  if(GlobalInput>=0&&GlobalInput<0.125){target_x = plainCtrl(0,0.75,(GlobalInput)/0.125);}
  if(GlobalInput>=0.125&&GlobalInput<0.375){target_x = 0.75;}
  if(GlobalInput>=0.375&&GlobalInput<0.625){target_x = plainCtrl(0.75,-1,(GlobalInput-0.375)/0.25);}
  if(GlobalInput>=0.625&&GlobalInput<0.875){target_x = -1;}
  if(GlobalInput>=0.875&&GlobalInput<1){target_x = plainCtrl(-1,0,(GlobalInput-0.875)/0.125);}

  target_x = target_x * 35;
  //roll pitch yaw
  PosIMU[0] = 0;
  PosIMU[1] = 0;
  // PosIMU[2] = 0;
  PosIMU[2] = adjust_yaw*M_PI/180;
  // // PosIMU[1] = 30*M_PI/180;
  // PosIMU[2] = target_yaw;

    //O-OA
  PosCom[0][0] = target_x;
  PosCom[1][0] = 0;
  PosCom[2][0] = WALK_HEIGHT;
  //this step is necessary
  makeArrayNegative(PosCom);
  
  // if turnCmd = 1, do crab walk
  newsinglegait(4, StepA, -directionAngle, PosIMU, PosCom, turnCmd);
  newsinglegait(2, StepB, directionAngle, PosIMU, PosCom, turnCmd);
  newsinglegait(1, StepC, directionAngle, PosIMU, PosCom, turnCmd);
  newsinglegait(3, StepD, -directionAngle, PosIMU, PosCom, turnCmd);

}



//YAO, 2/26/2024, new turning gait control using newsinglegait()
float LastInput = 0.0;
void newTurningGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;

////////////////////////////////////// record the last gesture before re-turning ///////////////////
  if(LastInput<0.2&&GlobalInput>=0.2)
  {
    for(int i=0;i<=3;i++)
    {
      for(int j=0;j<=2;j++)
      {
        StartPos[i][j][0] = TransPos[i][j][0];
        CurrentPos[i][j][0] = TransPos[i][j][0];
      }
    }
  }
  
  // for (int i = 0; i < 4; ++i) {
  //   for (int j = 0; j < 3; ++j) {
  //       for (int k = 0; k < 1; ++k) {
  //           Serial.print(StartPos[i][j][k]);
  //           Serial.print(" ");
  //       }
  //   }
  //   Serial.println();  // Move to the next line after printing the array
  // }
  // Serial.println();  // Move to the next line after printing the array

  LastInput = GlobalInput;
////////////////////////////////////////////calculate gesture sequence/////////////////////////
  // float adProp;
  // float phi;
  // bool isturn = false;
  // //the term is A->B->C->D->A
  // StepA = GlobalInput + 0.8;  //start at 0.2s
  // StepB = GlobalInput + 0.6;  //start at 0.4s
  // StepC = GlobalInput + 0.4;  //start at 0.6s
  // StepD = GlobalInput + 0.2;  //start at 0.8s

  // if(StepA>1){StepA--;}
  // if(StepB>1){StepB--;}
  // if(StepC>1){StepC--;}
  // if(StepD>1){StepD--;}
//////////////////////////////////////////// turning using trot /////////////////////////
  float adProp;
  float phi;
  bool isturn = false;
  //the term is A->B->C->D->A
  StepA = GlobalInput + 0.8;  //start at 0.2s
  StepB = GlobalInput + 0.8;  //start at 0.2s
  StepC = GlobalInput + 0.4;  //start at 0.6s
  StepD = GlobalInput + 0.4;  //start at 0.6s

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}

  WALK_LIFT_PROP = 0.4;

/////////////////////////////////////////////////////// use smoother rectangular ///////////////////////////////

  double target_yaw = 0;

  // turning direction in 0-0.2s
  if(GlobalInput<0.2) { target_yaw = plainCtrl(15*M_PI/180,-15*M_PI/180,GlobalInput/0.2); isturn = true;}
  else{ target_yaw = 15*M_PI/180;}
 
  //roll pitch yaw
  PosIMU[0] = 0;
  PosIMU[1] = 1*M_PI/180;
  // PosIMU[2] = 0;
  // // PosIMU[1] = 30*M_PI/180;
  PosIMU[2] = target_yaw;


    //O-OA    x , z , y
  PosCom[0][0] = 3;  //adjust COM when turning
  PosCom[1][0] = 4;
  PosCom[2][0] = WALK_HEIGHT;
  //this step is necessary
  makeArrayNegative(PosCom);


  //uint8_t statusInput 用于控制腿部的摆动幅度的系数，实际幅度 = WALK_RANGE * statusInput。
  if(!turnCmd){
    turningsinglegait(3, StepA, PosIMU, PosCom, isturn);
    turningsinglegait(2, StepB, PosIMU, PosCom, isturn);
    turningsinglegait(4, StepC, PosIMU, PosCom, isturn);
    turningsinglegait(1, StepD, PosIMU, PosCom, isturn);
  }

}

//YAO, 3/4/2024, PID Gait control to walk using roll,pitch,roll
void PIDGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;

////////////////////////////////////////////calculate gesture sequence/////////////////////////
  float adProp;
  float phi;

  //using trot gait
  //the term is A->B->C->D->A
  StepA = GlobalInput;  //start at 0s
  StepB = GlobalInput;  //start at 0s
  StepC = GlobalInput + 0.5;  //start at 5s
  StepD = GlobalInput + 0.5;  //start at 5s

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}
  

  ////////////////////////////////////////// IMU data MEAN value filter /////////////////////////////////
  pr_update();
  Serial.print("pitch: ");
  Serial.print(pitch);
  Serial.print("  ");
  Serial.print("roll: ");
  Serial.println(roll);

  int filter_number = 5;
  /////// calculate mean pitch ////////////////
  Queue_pitch.enqueue(pitch);
  if(Queue_pitch.itemCount() > filter_number){pitchSum -= Queue_pitch.dequeue();}
  pitchSum += pitch;
  float meanpitch = pitchSum / Queue_pitch.itemCount();

  Serial.print(meanpitch);
  Serial.print(' ');
  
  /////// calculate mean roll ////////////////
  Queue_roll.enqueue(roll);
  if(Queue_roll.itemCount() > filter_number){rollSum -= Queue_roll.dequeue();}
  rollSum += roll;
  float meanroll = rollSum / Queue_roll.itemCount();

  Serial.println(meanroll);

  /////// calculate mean yaw ////////////////
  // NO YAW RIGHT NOW !
  // Queue_yaw.enqueue(yaw);
  // if(Queue_yaw.itemCount() > filter_number){yawSum -= Queue_yaw.dequeue();}
  // yawSum += yaw;
  // float meanyaw = yawSum / Queue_yaw.itemCount();
  //Serial.println(meanyaw);
  ////////////////////////////////////////// IMU END ////////////////////////////////////

  //roll pitch yaw
  // PosIMU[0] = -meanroll*M_PI/180*0.1;
  // PosIMU[1] = -meanpitch*M_PI/180*0.1;
  PosIMU[0] = 0;
  PosIMU[1] = 0;
  PosIMU[2] = 0;


    //O-OA
  PosCom[0][0] = 0;
  PosCom[1][0] = 0;
  PosCom[2][0] = WALK_HEIGHT;
  //this step is necessary
  makeArrayNegative(PosCom);

  newsinglegait(4, StepA, -directionAngle, PosIMU, PosCom, turnCmd);
  newsinglegait(1, StepB, directionAngle, PosIMU, PosCom, turnCmd);
  newsinglegait(2, StepC, directionAngle, PosIMU, PosCom, turnCmd);
  newsinglegait(3, StepD, -directionAngle, PosIMU, PosCom, turnCmd);

}

//YAO, 3/6/2024, use new gait to climb stairs
void newclimbGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;
  float Tpitch, Troll;

////////////////////////////////////////////calculate gesture sequence/////////////////////////
  float adProp;
  float phi;

  //the term is A->B->C->D->A
  StepA = GlobalInput - 0.1 + 1;
  StepB = GlobalInput - 0.3 + 1;
  StepC = GlobalInput - 0.6 + 1;
  StepD = GlobalInput - 0.8 + 1;

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}
  

/////////////////////////////////////////////////////// use smoother rectangular ///////////////////////////////
  //GlobalInput ranges from 0 to 1
  float Clock_X,Clock_Y;
  float TX,TY;


  double target_yaw = 0;

  if(GlobalInput<0.5) 
  {
    target_yaw = besselCtrl(-10*M_PI/180,10*M_PI/180,GlobalInput/0.5);
  }
  else
  {
    target_yaw = besselCtrl(10*M_PI/180,-10*M_PI/180,(GlobalInput-0.5)/0.5);
  }

  pr_update();

  /////// calculate mean pitch ////////////////
  Queue.enqueue(pitch);
  if(Queue.itemCount() > 20){pitchSum -= Queue.dequeue();}
  pitchSum += pitch;
  float meanpitch = pitchSum / Queue.itemCount();
  Serial.print(0);
  Serial.print(' ');
  Serial.print(10);
  Serial.print(' ');
  Serial.print(meanpitch);


  double target_x;
  double ff=1,bb=-1;
  // if(GlobalInput>=0&&GlobalInput<0.05){target_x = besselCtrl(bb,ff,(GlobalInput)/0.05);PosIMU[1] = besselCtrl(5*M_PI/180,-5*M_PI/180,(GlobalInput)/0.05);}
  // if(GlobalInput>=0.05&&GlobalInput<0.5){target_x = ff;PosIMU[1] = -5*M_PI/180;}
  // if(GlobalInput>=0.5&&GlobalInput<0.55){target_x = besselCtrl(ff,bb,(GlobalInput-0.5)/0.05);PosIMU[1] = besselCtrl(-5*M_PI/180,5*M_PI/180,(GlobalInput-0.5)/0.05);}
  // if(GlobalInput>=0.55&&GlobalInput<1){target_x = bb;PosIMU[1] = 5*M_PI/180;}

  if(GlobalInput>=0&&GlobalInput<0.05){target_x = besselCtrl(bb,ff,(GlobalInput)/0.05);}
  if(GlobalInput>=0.05&&GlobalInput<0.5){target_x = ff;}
  if(GlobalInput>=0.5&&GlobalInput<0.55){target_x = besselCtrl(ff,bb,(GlobalInput-0.5)/0.05);}
  if(GlobalInput>=0.55&&GlobalInput<1){target_x = bb;}


  target_x = target_x * 35;
  // Serial.println(target_yaw);
  //roll pitch yaw
  PosIMU[0] = 0;
  // PosIMU[1] = 0;
  PosIMU[2] = 0;
  // PosIMU[1] = 15*M_PI/180;    
  // PosIMU[2] = target_yaw;
  if(meanpitch<-5){isup = true;}
  if(meanpitch>5){isup = false;}
  if(isup){PosIMU[1] = min(PosIMU[1]+0.5*M_PI/180,8.5*M_PI/180);}
  if(!isup){PosIMU[1] = max(PosIMU[1]-0.5*M_PI/180,0.0);}

  Serial.print(' ');
  Serial.println(PosIMU[1]);
    //O-OA
  PosCom[0][0] = target_x;
  PosCom[1][0] = 0;
  PosCom[2][0] = WALK_HEIGHT;
  //this step is necessary
  makeArrayNegative(PosCom);


  //uint8_t statusInput 用于控制腿部的摆动幅度的系数，实际幅度 = WALK_RANGE * statusInput。
  if(!turnCmd){
    newclimbgait(4, StepA, PosIMU, PosCom);
    newclimbgait(2, StepB, PosIMU, PosCom);
    newclimbgait(1, StepD, PosIMU, PosCom);
    newclimbgait(3, StepC, PosIMU, PosCom);
  }

}

//YAO, 3/16/2024, adjust turning gait to make degree adjustable and only once
void adjustTurningGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;

////////////////////////////////////// record the last gesture before re-turning ///////////////////
  if(LastInput<0.2&&GlobalInput>=0.2)
  {
    for(int i=0;i<=3;i++)
    {
      for(int j=0;j<=2;j++)
      {
        StartPos[i][j][0] = TransPos[i][j][0];
        CurrentPos[i][j][0] = TransPos[i][j][0];
      }
    }
  }
  
  LastInput = GlobalInput;
//////////////////////////////////////////// turning using trot /////////////////////////
  float adProp;
  float phi;
  bool isturn = false;
  //the term is A->B->C->D->A
  StepA = GlobalInput + 0.8;  //start at 0.2s
  StepB = GlobalInput + 0.8;  //start at 0.2s
  StepC = GlobalInput + 0.4;  //start at 0.6s
  StepD = GlobalInput + 0.4;  //start at 0.6s

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}

  WALK_LIFT_PROP = 0.4;

/////////////////////////////////////////////////////// use smoother rectangular ///////////////////////////////

  double target_yaw = 0;

  // turning direction in 0-0.2s
  if(GlobalInput<0.2) 
  { 
    if(crab_gait==1){target_yaw = plainCtrl(0*M_PI/180,15*M_PI/180,GlobalInput/0.2);}
    else{target_yaw = plainCtrl(0*M_PI/180,-15*M_PI/180,GlobalInput/0.2);}
    isturn = true;
  }
  else{ target_yaw = 0*M_PI/180;}
 
  //roll pitch yaw
  PosIMU[0] = 0;
  PosIMU[1] = 1*M_PI/180;
  // PosIMU[2] = 0;
  // // PosIMU[1] = 30*M_PI/180;
  PosIMU[2] = target_yaw;


    //O-OA    x , z , y
  PosCom[0][0] = 3;  //adjust COM when turning
  PosCom[1][0] = 4;
  PosCom[2][0] = WALK_HEIGHT;
  //this step is necessary
  makeArrayNegative(PosCom);


  //uint8_t statusInput 用于控制腿部的摆动幅度的系数，实际幅度 = WALK_RANGE * statusInput。
  if(!turnCmd){
    turningsinglegait(3, StepA, PosIMU, PosCom, isturn);
    turningsinglegait(2, StepB, PosIMU, PosCom, isturn);
    turningsinglegait(4, StepC, PosIMU, PosCom, isturn);
    turningsinglegait(1, StepD, PosIMU, PosCom, isturn);
  }

}



//YAO, 3/19/2024, new turning gait control using newsinglegait()
void TurningDegreeGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;



//////////////////////////////////////////// turning using trot /////////////////////////
  float adProp;
  float phi;
  bool isturn = false;
  //the term is A->B->C->D->A
  StepA = GlobalInput + 0.8;  //start at 0.2s
  StepB = GlobalInput + 0.8;  //start at 0.2s
  StepC = GlobalInput + 0.4;  //start at 0.6s
  StepD = GlobalInput + 0.4;  //start at 0.6s

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}

  WALK_LIFT_PROP = 0.4;

/////////////////////////////////////////////////////// use smoother rectangular ///////////////////////////////

 
  //roll pitch yaw
  PosIMU[0] = 0;
  PosIMU[1] = 1*M_PI/180;
  // PosIMU[2] = 0;
  // // PosIMU[1] = 30*M_PI/180;
  PosIMU[2] = adjust_yaw*M_PI/180;


    //O-OA    x , z , y
  PosCom[0][0] = 0;  //adjust COM when turning
  PosCom[1][0] = 0;
  PosCom[2][0] = WALK_HEIGHT;
  //this step is necessary
  makeArrayNegative(PosCom);


  //uint8_t statusInput 用于控制腿部的摆动幅度的系数，实际幅度 = WALK_RANGE * statusInput。
  if(!turnCmd){
    turningdegreegait(3, StepA, PosIMU, PosCom, isturn);
    turningdegreegait(2, StepB, PosIMU, PosCom, isturn);
    turningdegreegait(4, StepC, PosIMU, PosCom, isturn);
    turningdegreegait(1, StepD, PosIMU, PosCom, isturn);
  }

}





// Gait Select
void gaitTypeCtrl(float GlobalStepInput, float directionCmd, int turnCmd){
  if(GAIT_TYPE == 0){
    simpleGait(GlobalStepInput, directionCmd, turnCmd);
  }
  else if(GAIT_TYPE == 1){
    triangularGait(GlobalStepInput, directionCmd, turnCmd);
  }
  else if(GAIT_TYPE == 2){
    climbGait(GlobalStepInput, directionCmd, turnCmd);
  }
  else if(GAIT_TYPE == 3){
    autoGait(GlobalStepInput, directionCmd, turnCmd);
  }
  else if(GAIT_TYPE == 4){
    roundGait(GlobalStepInput, directionCmd, turnCmd);
  }
  else if(GAIT_TYPE == 5){
    newGait(GlobalStepInput, directionCmd, turnCmd);
  }  
  else if(GAIT_TYPE == 6){
    newTurningGait(GlobalStepInput, directionCmd, turnCmd);
  }  
  else if(GAIT_TYPE == 7){
    PIDGait(GlobalStepInput, directionCmd, turnCmd);
  }  
  else if(GAIT_TYPE == 8){
    newclimbGait(GlobalStepInput, directionCmd, turnCmd);
  }  
  else if(GAIT_TYPE == 9){
    adjustTurningGait(GlobalStepInput, directionCmd, turnCmd);
  }  
  else if(GAIT_TYPE == 10){
    TurningDegreeGait(GlobalStepInput, directionCmd, turnCmd);
  }  
}


// Stand and adjust mass center.
//    |
//    a
//    |
// -b-M  a,b > 0
void standMassCenter(float aInput, float bInput){
  singleLegCtrl(1, ( WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z - bInput));
  singleLegCtrl(2, (-WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z - bInput));

  singleLegCtrl(3, ( WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z + bInput));
  singleLegCtrl(4, (-WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z + bInput));
}


// ctrl pitch yaw and roll.
// pitchInput (-, +), if > 0, look up.
// yawInput   (-, +), if > 0, look right.
// rollInput  (-, +), if > 0, lean right.
void pitchYawRoll(float pitchInput, float yawInput, float rollInput){
  legPosBuffer[1]  = STAND_HEIGHT + pitchInput + rollInput;
  legPosBuffer[4]  = STAND_HEIGHT - pitchInput + rollInput;
  legPosBuffer[7]  = STAND_HEIGHT + pitchInput - rollInput;
  legPosBuffer[10] = STAND_HEIGHT - pitchInput - rollInput;

  if(legPosBuffer[1] > WALK_HEIGHT_MAX){legPosBuffer[1] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[4] > WALK_HEIGHT_MAX){legPosBuffer[4] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[7] > WALK_HEIGHT_MAX){legPosBuffer[7] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[10] > WALK_HEIGHT_MAX){legPosBuffer[10] = WALK_HEIGHT_MAX;}
  
  if(legPosBuffer[1] < WALK_HEIGHT_MIN){legPosBuffer[1] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[4] < WALK_HEIGHT_MIN){legPosBuffer[4] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[7] < WALK_HEIGHT_MIN){legPosBuffer[7] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[10] < WALK_HEIGHT_MIN){legPosBuffer[10] = WALK_HEIGHT_MIN;}

  legPosBuffer[2]  = WALK_EXTENDED_Z + yawInput - rollInput;
  legPosBuffer[5]  = WALK_EXTENDED_Z - yawInput - rollInput;
  legPosBuffer[8]  = WALK_EXTENDED_Z - yawInput + rollInput;
  legPosBuffer[11] = WALK_EXTENDED_Z + yawInput + rollInput;

  if(legPosBuffer[2] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[5] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[8] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[11] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}

  if(legPosBuffer[2] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[5] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[8] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[11] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}

  singleLegCtrl(1,  WALK_EXTENDED_X, legPosBuffer[1] , legPosBuffer[2]);
  singleLegCtrl(2, -WALK_EXTENDED_X, legPosBuffer[4] , legPosBuffer[5]);
  singleLegCtrl(3,  WALK_EXTENDED_X, legPosBuffer[7] , legPosBuffer[8]);
  singleLegCtrl(4, -WALK_EXTENDED_X, legPosBuffer[10], legPosBuffer[11]);
}


void pitchYawRollHeightCtrl(float pitchInput, float yawInput, float rollInput, float heightInput){
  legPosBuffer[1]  = STAND_HEIGHT + pitchInput + rollInput + heightInput;
  legPosBuffer[4]  = STAND_HEIGHT - pitchInput + rollInput + heightInput;
  legPosBuffer[7]  = STAND_HEIGHT + pitchInput - rollInput + heightInput;
  legPosBuffer[10] = STAND_HEIGHT - pitchInput - rollInput + heightInput;

  if(legPosBuffer[1] > WALK_HEIGHT_MAX){legPosBuffer[1] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[4] > WALK_HEIGHT_MAX){legPosBuffer[4] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[7] > WALK_HEIGHT_MAX){legPosBuffer[7] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[10] > WALK_HEIGHT_MAX){legPosBuffer[10] = WALK_HEIGHT_MAX;}
  
  if(legPosBuffer[1] < WALK_HEIGHT_MIN){legPosBuffer[1] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[4] < WALK_HEIGHT_MIN){legPosBuffer[4] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[7] < WALK_HEIGHT_MIN){legPosBuffer[7] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[10] < WALK_HEIGHT_MIN){legPosBuffer[10] = WALK_HEIGHT_MIN;}

  legPosBuffer[2]  = WALK_EXTENDED_Z + yawInput - rollInput;
  legPosBuffer[5]  = WALK_EXTENDED_Z - yawInput - rollInput;
  legPosBuffer[8]  = WALK_EXTENDED_Z - yawInput + rollInput;
  legPosBuffer[11] = WALK_EXTENDED_Z + yawInput + rollInput;

  if(legPosBuffer[2] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[5] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[8] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[11] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}

  if(legPosBuffer[2] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[5] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[8] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[11] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}

  singleLegCtrl(1,  WALK_EXTENDED_X, legPosBuffer[1] , legPosBuffer[2]);
  singleLegCtrl(2, -WALK_EXTENDED_X, legPosBuffer[4] , legPosBuffer[5]);
  singleLegCtrl(3,  WALK_EXTENDED_X, legPosBuffer[7] , legPosBuffer[8]);
  singleLegCtrl(4, -WALK_EXTENDED_X, legPosBuffer[10], legPosBuffer[11]);
}


// BALANCE
void balancing(){
  // float BALANCE_P = 0.00018;
  BALANCE_PITCHU_BUFFER += ACC_Y * BALANCE_P;
  BALANCE_ROLL_BUFFER -= ACC_X * BALANCE_P;

  if(BALANCE_PITCHU_BUFFER > 21){BALANCE_PITCHU_BUFFER = 21;}
  if(BALANCE_PITCHU_BUFFER < -21){BALANCE_PITCHU_BUFFER = -21;}

  if(BALANCE_ROLL_BUFFER > 21){BALANCE_ROLL_BUFFER = 21;}
  if(BALANCE_ROLL_BUFFER < -21){BALANCE_ROLL_BUFFER = -21;}
  
  pitchYawRoll(BALANCE_PITCHU_BUFFER, 0, BALANCE_ROLL_BUFFER);
}


//YAO, 2/6/2024
void walk_balancing(){
  // float BALANCE_P = 0.00018;
  BALANCE_PITCHU_BUFFER += ACC_Y * BALANCE_P;
  BALANCE_ROLL_BUFFER -= ACC_X * BALANCE_P;

  if(BALANCE_PITCHU_BUFFER > 21){BALANCE_PITCHU_BUFFER = 21;}
  if(BALANCE_PITCHU_BUFFER < -21){BALANCE_PITCHU_BUFFER = -21;}

  if(BALANCE_ROLL_BUFFER > 21){BALANCE_ROLL_BUFFER = 21;}
  if(BALANCE_ROLL_BUFFER < -21){BALANCE_ROLL_BUFFER = -21;}
  
  pitchYawRoll(BALANCE_PITCHU_BUFFER, 0, BALANCE_ROLL_BUFFER);
}


// mass center adjust test.
void massCenerAdjustTestLoop(){
  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(-20+i, i);
    GoalPosAll();
    delay(0);
  }

  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(i, 20-i);
    GoalPosAll();
    delay(0);
  }

  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(20-i, -i);
    GoalPosAll();
    delay(0);
  }

  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(-i, -20+i);
    GoalPosAll();
    delay(0);
  }
}


// pitch yaw roll test.
void pitchYawRollTestLoop(){
  for(int i = -23; i<23; i++){
    pitchYawRoll(0, 0, i);
    GoalPosAll();
    delay(10);
  }
  for(int i = 23; i>-23; i--){
    pitchYawRoll(0, 0, i);
    GoalPosAll();
    delay(10);
  }

  for(int i = -23; i<23; i++){
    pitchYawRoll(i, 0, 0);
    GoalPosAll();
    delay(10);
  }
  for(int i = 23; i>-23; i--){
    pitchYawRoll(i, 0, 0);
    GoalPosAll();
    delay(10);
  }
}


// key frames ctrl.
// 0 <= rateInput <= 1
// use it fuction to ctrl a number change from numStart to numEnd.
float linearCtrl(float numStart, float numEnd, float rateInput){
  float numOut;
  numOut = (numEnd - numStart)*rateInput + numStart;
  return numOut;
}




// Functions.
void functionStayLow(){
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i));
    GoalPosAll();
    delay(1);
  }
  delay(300);
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT_MAX, i));
    GoalPosAll();
    delay(1);
  }
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT, i));
    GoalPosAll();
    delay(1);
  }
}


void functionHandshake(){
  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1,  besselCtrl(WALK_EXTENDED_X, 0, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MAX, i), besselCtrl(WALK_EXTENDED_Z, -15, i));
    singleLegCtrl(3,  besselCtrl(WALK_EXTENDED_X, 0, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MAX, i), WALK_EXTENDED_Z);

    singleLegCtrl(2,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN-10, i), besselCtrl(WALK_EXTENDED_Z, 2*WALK_EXTENDED_Z, i));
    singleLegCtrl(4,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN-10, i), besselCtrl(WALK_EXTENDED_Z, 2*WALK_EXTENDED_Z, i));

    GoalPosAll();
    delay(1);
  }


  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(3,  besselCtrl(0, WALK_RANGE/2+WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT_MIN, i), besselCtrl(WALK_EXTENDED_Z, 0, i));

    GoalPosAll();
    delay(1);
  }

  for(int shakeTimes = 0; shakeTimes < 3; shakeTimes++){
    for(float i = 0; i<=1; i+=0.03){
      singleLegCtrl(3,  WALK_RANGE/2+WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT_MIN+30, i), 0);

      GoalPosAll();
      delay(1);
    }
    for(float i = 0; i<=1; i+=0.03){
      singleLegCtrl(3,  WALK_RANGE/2+WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN+30, WALK_HEIGHT_MIN, i), 0);

      GoalPosAll();
      delay(1);
    }
  }

  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1,  besselCtrl(0, WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT, i), besselCtrl(-15, WALK_EXTENDED_Z, i));
    singleLegCtrl(3,  besselCtrl(WALK_RANGE/2+WALK_EXTENDED_X, WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), besselCtrl(0, WALK_EXTENDED_Z, i));

    singleLegCtrl(2,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN-10, WALK_HEIGHT, i), besselCtrl(2*WALK_EXTENDED_Z, WALK_EXTENDED_Z, i));
    singleLegCtrl(4,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN-10, WALK_HEIGHT, i), besselCtrl(2*WALK_EXTENDED_Z, WALK_EXTENDED_Z, i));

    GoalPosAll();
    delay(1);
  }
}


void functionJump(){
  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(2,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(3, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(4,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    GoalPosAll();
    delay(1);
  }

  singleLegCtrl(1, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(2,-WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(3, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(4,-WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  GoalPosAll();
  delay(70);

  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(2,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(3, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(4,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    GoalPosAll();
    delay(1);
  }
}


// void functionActionA(){
//   WALK_LIFT = 9;
//   WALK_LIFT_PROP = 0.25;
//   STEP_ITERATE = 0.04;
//   GAIT_TYPE = 0;
// }

void functionActionA(){
  WALK_LIFT = 9;
  
  WALK_LIFT_PROP = float(3)/8;
  WALK_ACC = 5;
  WALK_RANGE = 40;

  STEP_ITERATE = 0.01;
  STEP_DELAY = 0.001;
  GAIT_TYPE = 0;
}

void functionActionB(){
  WALK_LIFT = 20;
  WALK_LIFT_PROP=0.18;
  STEP_ITERATE = 0.01;

  WALK_MASS_ADJUST = 3;
  STEP_DELAY = 10;
  GAIT_TYPE = 1;
}

//YAO, 6/2/2024
// add the climb gait as fuction C
void functionActionC(){
  //controls the portion of lift 
  WALK_LIFT_PROP = 0.125;
  WALK_ACC = 5;
  WALK_RANGE = 70;

  WALK_LIFT = 40;


  //controls the speed
  STEP_ITERATE = 0.01;
  STEP_DELAY = 40;

  WALK_MASS_ADJUST = 10;
  GAIT_TYPE = 2;
}

// add the auto gait as function D
void functionActionD(){
  //controls the portion of lift 
  WALK_LIFT_PROP = 0.125;
  WALK_ACC = 5;
  WALK_RANGE = 70;

  WALK_LIFT = 20;

  //control the balance
  // Kp_pitch = 0.4;
  // Kp_roll = 0.4;
  
  // Ki_pitch = 0.000;
  // Ki_roll = 0.000;

  // Kd_pitch = 0.05;
  // Kd_roll = 0.01;
  Kp_pitch = 0.1;
  Kp_roll = 0.1;
  
  Ki_pitch = 0.000;
  Ki_roll = 0.000;

  Kd_pitch = 0.01;
  Kd_roll = 0.01;
  deg_bias = 200;
  WALK_MASS_ADJUST = 3;

  //controls the speed
  STEP_ITERATE = 0.01;
  STEP_DELAY = 40;
  GAIT_TYPE = 3;
}


void functionActionE(){
  //controls the portion of lift 
  WALK_LIFT_PROP = 0.125;
  WALK_ACC = 5;
  WALK_RANGE = 70;

  //controls the walk height
  WALK_HEIGHT_MAX  = 115;
  WALK_HEIGHT_MIN  = 50;
  WALK_HEIGHT      = 95;

  /// for testing
  // WALK_HEIGHT      = 90;
  // WALK_HEIGHT_MAX  = 110;

  WALK_EXTENDED_Z = 30;

  WALK_LIFT = 20;

  deg_bias = 200;
  WALK_MASS_ADJUST = 10;

  //controls the speed
  STEP_ITERATE = 0.01;
  // STEP_DELAY = 5;
  STEP_DELAY = 10;
  GAIT_TYPE = 4;


  //climb stair
  climb_height[1] = WALK_HEIGHT;
  climb_height[2] = WALK_HEIGHT;
  climb_height[3] = WALK_HEIGHT;
  climb_height[4] = WALK_HEIGHT;
  climb_lift[1] = WALK_LIFT;
  climb_lift[2] = WALK_LIFT;
  climb_lift[3] = WALK_LIFT;
  climb_lift[4] = WALK_LIFT;

}


//YAO, 2/20/2024, using round gait to test friction
// using walksinglegait
void functionActionF(){
  //controls the portion of lift 
  WALK_LIFT_PROP = 0.125;
  WALK_ACC = 5/2;
  WALK_RANGE = 70/2;

  //controls the walk height
  WALK_HEIGHT_MAX  = 115;
  WALK_HEIGHT_MIN  = 50;
  WALK_HEIGHT      = 95;

  WALK_EXTENDED_Z = 30;
  WALK_EXTENDED_X  = 12;
  xbuffer = -10;
  WALK_LIFT = 20;

  WALK_MASS_ADJUST = 7.5;
  //controls the speed
  STEP_ITERATE = 0.01;
  STEP_DELAY = 10;
  GAIT_TYPE = 4;


  //climb stair
  climb_height[1] = WALK_HEIGHT;
  climb_height[2] = WALK_HEIGHT+3;
  climb_height[3] = WALK_HEIGHT;
  climb_height[4] = WALK_HEIGHT+3;
  climb_lift[1] = WALK_LIFT;
  climb_lift[2] = WALK_LIFT;
  climb_lift[3] = WALK_LIFT;
  climb_lift[4] = WALK_LIFT;

}


//YAO, 2/26/2024, using newgait
// using newsinglegait
void functionActionG(){

  //controls the portion of lift 
  WALK_LIFT_PROP = 0.1;
  WALK_RANGE = 30;

  //controls the walk height
  WALK_HEIGHT_MAX  = 115;
  WALK_HEIGHT_MIN  = 50;
  WALK_HEIGHT      = 85;


  WALK_EXTENDED_Z = 35;
  WALK_EXTENDED_X  = -20;
  WALK_LIFT = 15;

  WALK_MASS_ADJUST = 7.5;
  //controls the speed
  STEP_ITERATE = 0.0025;
  STEP_DELAY = 2;

  //Gait type is 5 means using new gait
  GAIT_TYPE = 5;
}

// rotation gait
void functionActionH(){

  //controls the portion of lift 
  WALK_LIFT_PROP = 0.2;
  WALK_RANGE = 30;

  //controls the walk height
  WALK_HEIGHT_MAX  = 115;
  WALK_HEIGHT_MIN  = 50;
  WALK_HEIGHT      = 85;
/////////////////////// success ////////////////////////////
//WALK_EXTENDED_X  = 0;      // turning
//WALK_EXTENDED_Z  = 25;      //  control the extend in y axis, do not been modified also
////////////////////////////////////////////////////
  
  //not stable
  WALK_EXTENDED_X  = -10.0;
  WALK_EXTENDED_Z = 20.0;

  Proj_to_Leg1[0][0] = 130.0/2+WALK_EXTENDED_X; 
  Proj_to_Leg1[1][0] = 55.0/2+WALK_EXTENDED_Z;
  Proj_to_Leg1[2][0] = 0.0;
  Proj_to_Leg2[0][0] = -130.0/2-WALK_EXTENDED_X; 
  Proj_to_Leg2[1][0] = 55.0/2+WALK_EXTENDED_Z;
  Proj_to_Leg2[2][0] = 0.0;
  Proj_to_Leg3[0][0] = 130.0/2+WALK_EXTENDED_X; 
  Proj_to_Leg3[1][0] = -55.0/2-WALK_EXTENDED_Z;
  Proj_to_Leg3[2][0] = 0.0;
  Proj_to_Leg4[0][0] = -130.0/2-WALK_EXTENDED_X; 
  Proj_to_Leg4[1][0] = -55.0/2-WALK_EXTENDED_Z;
  Proj_to_Leg4[2][0] = 0.0;



  WALK_LIFT = 7.5;

  WALK_MASS_ADJUST = 7.5;
  //controls the speed
  // STEP_ITERATE = 0.01;
  STEP_ITERATE = 0.015;                                      
  STEP_DELAY = 10;

  //Gait type is 5 means using new gait
  GAIT_TYPE = 6;
}

void functionActionI(){

  //controls the portion of lift 
  WALK_LIFT_PROP = 0.5;
  WALK_RANGE = 30;

  //controls the walk height
  WALK_HEIGHT_MAX  = 115;
  WALK_HEIGHT_MIN  = 50;
  WALK_HEIGHT   = 85;

  WALK_EXTENDED_Z = 35;
  WALK_EXTENDED_X  = -20;
  WALK_LIFT = 20;

  WALK_MASS_ADJUST = 7.5;
  //controls the speed
  // STEP_ITERATE = 0.01;
  STEP_ITERATE = 0.06;                             
  STEP_DELAY = 10;

  pitchSum = 0.0;
  rollSum = 0.0;
  yawSum = 0.0;

  //Gait type is 5 means using new gait
  GAIT_TYPE = 7;
}

//YAO, 2/26/2024, using newgait to climb stairs
// using newsinglegait
void functionActionJ(){

  //controls the portion of lift 
  WALK_LIFT_PROP = 0.2;
  WALK_RANGE = 30;

  //controls the walk height
  WALK_HEIGHT_MAX  = 115;
  WALK_HEIGHT_MIN  = 50;
  WALK_HEIGHT      = 90;


  WALK_EXTENDED_Z = 35;
  WALK_EXTENDED_X  = -20;
  WALK_LIFT = 30;

  //controls the speed
  STEP_ITERATE = 0.0025;
  STEP_DELAY = 15;

  pitchSum = 0.0;
  isup = false;

  //Gait type is 5 means using new gait
  GAIT_TYPE = 8;
}

// same as turning gait
void functionActionK(){

  //controls the portion of lift 
  WALK_LIFT_PROP = 0.2;
  WALK_RANGE = 30;

  //controls the walk height
  WALK_HEIGHT_MAX  = 115;
  WALK_HEIGHT_MIN  = 50;
  WALK_HEIGHT      = 85;
/////////////////////// success ////////////////////////////
//WALK_EXTENDED_X  = 0;      // turning
//WALK_EXTENDED_Z  = 25;      //  control the extend in y axis, do not been modified also
////////////////////////////////////////////////////
  
  //not stable
  WALK_EXTENDED_X  = -10.0;
  WALK_EXTENDED_Z = 20.0;

  Proj_to_Leg1[0][0] = 130.0/2+WALK_EXTENDED_X; 
  Proj_to_Leg1[1][0] = 55.0/2+WALK_EXTENDED_Z;
  Proj_to_Leg1[2][0] = 0.0;
  Proj_to_Leg2[0][0] = -130.0/2-WALK_EXTENDED_X; 
  Proj_to_Leg2[1][0] = 55.0/2+WALK_EXTENDED_Z;
  Proj_to_Leg2[2][0] = 0.0;
  Proj_to_Leg3[0][0] = 130.0/2+WALK_EXTENDED_X; 
  Proj_to_Leg3[1][0] = -55.0/2-WALK_EXTENDED_Z;
  Proj_to_Leg3[2][0] = 0.0;
  Proj_to_Leg4[0][0] = -130.0/2-WALK_EXTENDED_X; 
  Proj_to_Leg4[1][0] = -55.0/2-WALK_EXTENDED_Z;
  Proj_to_Leg4[2][0] = 0.0;



  WALK_LIFT = 7.5;

  WALK_MASS_ADJUST = 7.5;
  //controls the speed
  // STEP_ITERATE = 0.01;
  STEP_ITERATE = 0.015;                                      
  STEP_DELAY = 10;

  //Gait type is 5 means using new gait
  GAIT_TYPE = 9;
}


void functionActionL(){

  //controls the portion of lift 
  WALK_LIFT_PROP = 0.2;
  WALK_RANGE = 30;

  //controls the walk height
  WALK_HEIGHT_MAX  = 115;
  WALK_HEIGHT_MIN  = 50;
  WALK_HEIGHT      = 85;
/////////////////////// success ////////////////////////////
//WALK_EXTENDED_X  = 0;      // turning
//WALK_EXTENDED_Z  = 25;      //  control the extend in y axis, do not been modified also
////////////////////////////////////////////////////
  
  //not stable
  WALK_EXTENDED_X  = -10.0;
  WALK_EXTENDED_Z = 20.0;

  Proj_to_Leg1[0][0] = 130.0/2+WALK_EXTENDED_X; 
  Proj_to_Leg1[1][0] = 55.0/2+WALK_EXTENDED_Z;
  Proj_to_Leg1[2][0] = 0.0;
  Proj_to_Leg2[0][0] = -130.0/2-WALK_EXTENDED_X; 
  Proj_to_Leg2[1][0] = 55.0/2+WALK_EXTENDED_Z;
  Proj_to_Leg2[2][0] = 0.0;
  Proj_to_Leg3[0][0] = 130.0/2+WALK_EXTENDED_X; 
  Proj_to_Leg3[1][0] = -55.0/2-WALK_EXTENDED_Z;
  Proj_to_Leg3[2][0] = 0.0;
  Proj_to_Leg4[0][0] = -130.0/2-WALK_EXTENDED_X; 
  Proj_to_Leg4[1][0] = -55.0/2-WALK_EXTENDED_Z;
  Proj_to_Leg4[2][0] = 0.0;



  WALK_LIFT = 7.5;

  WALK_MASS_ADJUST = 7.5;
  //controls the speed
  // STEP_ITERATE = 0.01;
  STEP_ITERATE = 0.015;                                      
  STEP_DELAY = 10;

  //Gait type is 5 means using new gait
  GAIT_TYPE = 10;
}


// base move ctrl.
void robotCtrl(){

  // move ctrl.
  if(!debugMode){
    if(moveFB == 0 && moveLR == 0 && STAND_STILL == 0){
      standMassCenter(0, 0);
      GoalPosAll();
      STAND_STILL = 1;
      GLOBAL_STEP = 0;
      delay(STEP_DELAY);
    }
    else if(moveFB == 0 && moveLR == 0 && STAND_STILL == 1){
      GoalPosAll();
      delay(STEP_DELAY);
    }
    else{                 //moveFB = 1
      Serial.print(GLOBAL_STEP);
      Serial.println(" ");

      STAND_STILL = 0;
      gestureUD = 0;
      gestureLR = 0;

      // to make sure change turning direction only in contact phase
      if(turning_direction!=last_turning_direction)  //if the current turning direction not equal to the lasting turning direction, means the command changed the direction
      {
        temp_turning_direction = turning_direction;  // store the current(changed) direction in temp 
        turning_direction = last_turning_direction;  // remain the lasting direction till global step > 1
      }
      if(GLOBAL_STEP > 1)
      {
        GLOBAL_STEP = 0;
        if(turning_direction!=temp_turning_direction) //check if there is a different temp direction
        {
          turning_direction=temp_turning_direction;   //change every direction to current(commanded) direction
          last_turning_direction = temp_turning_direction;
        }
        //turning only once
        if(GAIT_TYPE==9)
        {
          funcMode = 13;
        }
      }
      // YAO, 3/9/2024  ///////////
      //upper level control
      if(upper_flag && GAIT_TYPE==5 ){gaitTypeCtrl(GLOBAL_STEP, turning_direction, crab_gait);}
      else if(moveFB == 1 && moveLR == 0){gaitTypeCtrl(GLOBAL_STEP, 0, 0);}
      else if(moveFB == -1 && moveLR == 0){gaitTypeCtrl(GLOBAL_STEP, 180, 0);}
      else if(moveFB == 1 && moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 30, 0);}
      else if(moveFB == 1 && moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, -30, 0);}
      else if(moveFB == -1 && moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, -120, 0);}
      else if(moveFB == -1 && moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 120, 0);}
      else if(moveFB == 0 && moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 0, -1);}
      else if(moveFB == 0 && moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, 0, 1);}
      GoalPosAll();
      GLOBAL_STEP += STEP_ITERATE;
      delay(STEP_DELAY);
    }
  }
  // function ctrl.
  if(!debugMode && funcMode && GLOBAL_STEP == STEP_ITERATE){
    delay(1000);
    if(funcMode == 1){
      accXYZUpdate();
      balancing();
      GoalPosAll();
    }
    else if (funcMode == 2){
      Serial.println("stayLow");
      functionStayLow();
      funcMode = 0;
    }
    else if (funcMode == 3){
      Serial.println("handshake");
      functionHandshake();
      funcMode = 0;
    }
    else if (funcMode == 4){
      Serial.println("Jump");
      functionJump();
      funcMode = 0;
    }
    else if (funcMode == 5){
      Serial.println("ActionA");
      functionActionA();
      funcMode = 0;
    }
    else if (funcMode == 6){
      Serial.println("ActionB");
      functionActionB();
      funcMode = 0;
    }
    else if (funcMode == 7){
      Serial.println("ActionC");
      functionActionC();
      funcMode = 0;
    }
    else if (funcMode == 10){
      Serial.println("ActionD");
      functionActionD();
      funcMode = 0;
    }
    else if (funcMode == 11){
      Serial.println("ActionE");
      functionActionE();
      funcMode = 0;
    }
    else if (funcMode == 12){
      Serial.println("ActionF");
      functionActionF();
      funcMode = 0;
    }
    else if (funcMode == 13){
      Serial.println("ActionG: NEW GAIT");
      functionActionG();
      funcMode = 0;
    }   
    else if (funcMode == 14){
      Serial.println("ActionH: TURNING GAIT");
      functionActionH();
      funcMode = 0;
    }   
    else if (funcMode == 15){
      Serial.println("ActionI: BALANCE GAIT");
      functionActionI();
      funcMode = 0;
    }   
    else if (funcMode == 16){
      Serial.println("ActionJ: BALANCE NEW GAIT");
      functionActionJ();
      funcMode = 0;
    }   
    else if (funcMode == 17){
      Serial.println("ActionK: TURNING Once");
      functionActionK();
      funcMode = 0;
    }
    else if (funcMode == 18){
      Serial.println("ActionL: TURNING Degree");
      functionActionL();
      funcMode = 0;
    }
    else if (funcMode == 8){
      Serial.println("InitPos");
      initPosAll();
    }
    else if (funcMode == 9){
      Serial.println("MiddlePos");
      middlePosAll();
    }
    setSingleLED(0,matrix.Color(0, 128, 255));
    setSingleLED(1,matrix.Color(0, 128, 255));
  }
  else if(debugMode){
    setSingleLED(0,matrix.Color(255, 64, 0));
    setSingleLED(1,matrix.Color(255, 64, 0));
    delay(100);
  }
}


// if the level of IO12 is HIGH, go into debugMode.
void wireDebugDetect(){
  if(digitalRead(WIRE_DEBUG) == HIGH){
    initPosAll();
    debugMode = 1;

    setSingleLED(0,matrix.Color(255, 64, 0));
    setSingleLED(1,matrix.Color(255, 64, 0));

    while(digitalRead(WIRE_DEBUG) == HIGH){
      delay(100);
    }
    delay(1000);
  }
}