/*******************************************************************/
/*******************************************************************
 * Created January 2024 by Nick Livingston
 * Edited April 2024 by Nick Livingston
 * 
 * This is baseline code for getting a TinyDuino based two-wheeled
 * vehicle to drive using proportional drive.  Proportional drive
 * in this context means, the left and right motors (servo or DC)
 * are controlled through a drive() function that specifies how
 * much of the maximum possible power should be used to drive
 * each motor, and for how long.  The left_proportion and
 * right_proportion parameters for the drive() function may take
 * values between -1 and 1, where -1 means that motor should be
 * driven at full power "backwards", and +1 means the motor should
 * be driven at full power "forwards".
 *
 * This code has been tested and verified for the TinyScreen+ and
 * RobotZero controllers, and works with both DC and servo motors.
 * The constant integer, "MOTOR_TYPE" is used to determine which
 * motor is used in the rest of the code.  "DC_MOTOR" means that
 * a direct current motor will be used, either in the DC motor port
 * on the RobotZero, or in a motor shield for the TinyScreen+.
 * "TS_SERVO" stands for "TinyScreen" servo, and will indicate that
 * a ServoDriver object should be created and initialized with the
 * value NO_R_REMOVED, which comes from the ServoDriver library, and
 * resolves to the value 0.  "RZ_SERVO" stands for "RobotZero" servo
 * and indicates that a ServoDriver object should be created and
 * initialized with the value 15, which is the argument needed for
 * the ServoDriver constructor if a servo is being used with the
 * RobotZero controller.
 * Thus, to use this code, you must know which controller you are
 * using if you are going to drive using servos, or you must set
 * MOTOR_TYPE to DC_MOTOR if you are using a DC motor.  This code
 * can serve as a starting point for more sophisticated robotics
 * projects.  Add code for your sensors and controlling logic, and
 * your two wheeled robot should be on its way!
 *******************************************************************/
/*******************************************************************/
#include <Wire.h>
#include <ServoDriver.h>
#include <MotorDriver.h>

#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

//Meta Motor
#define DC_MOTOR 1
#define TS_SERVO NO_R_REMOVED
#define RZ_SERVO 15
const int MOTOR_TYPE = DC_MOTOR;

//Servo Motor
const int SERVO_HALT = 1500;
const int MAX_SERVO_OFFSET = 500;
const int LEFT_PORT = 1;
const int RIGHT_PORT = 2;

//DC Motor
int maxPWM = 10000;
int steps = 300;
int stepSize = maxPWM / steps;

//If you are not using a DC motor, the value of MOTOR_TYPE
//should be set to "TS_SERVO" if you are using the TinyScreen+ processor
//or "RZ_SERVO" if you are using the RobotZero.  Each of these constants
//has the value needed to pass to the ServoDriver constructor in order
//to properly initialize the servo object for the corresponding controller.
#if MOTOR_TYPE != DC_MOTOR
ServoDriver servo(MOTOR_TYPE);// Value passed is the address- RobotZero is always address 15
#endif 

void setup() 
{
  SerialMonitorInterface.begin(9600);
  driveInit();
}
/*******************************************************************/
void loop() 
{
  //Example code for driving motors.
  float driveProp = 0.0;
  //This will drive the left and right motors in opposite directions
  //ranging from -60% of the maximum power (backward) to 60% (foward)
  //and back.  The robot will oscillate back and forth in place.
  for(driveProp = -0.6; driveProp < 0.6; driveProp += 0.1)
  {
    SerialMonitorInterface.print("driveProp: ");
    SerialMonitorInterface.println(driveProp);
    drive(driveProp, -driveProp, 0.5);
  }
  for(driveProp = 0.6; driveProp > -0.6; driveProp -= 0.1)
  {
    SerialMonitorInterface.print("driveProp: ");
    SerialMonitorInterface.println(driveProp);
    drive(driveProp, -driveProp, 0.5);
  }
}
/*******************************************************************/
void driveInit() {
  if (MOTOR_TYPE == DC_MOTOR)
  {
    dcMotorInit();
  }
  else
  {
    servoInit();
  }
}
/*******************************************************************/
void servoInit()
{
  Wire.begin();
  // while(!SerialMonitorInterface)//This will block until the Serial Monitor is opened on TinyScreen+/TinyZero platform!
  pinMode(9, OUTPUT);//Pin 9 is the reset pin for the servo controller TinyShield
  digitalWrite(9, LOW);
  delay(10);
  digitalWrite(9, HIGH);
  delay(100);
  
  
  if(servo.begin(20000))      //Set the period to 20000us or 20ms, correct for driving most servos
  {
    SerialMonitorInterface.println("Motor driver not detected!");
    while(1);
  }
  //The failsafe turns off the PWM output if a command is not sent in a certain amount of time.
  //Failsafe is set in milliseconds- comment or set to 0 to disable
  servo.setFailsafe(1000);
}
/*******************************************************************/
void dcMotorInit()
{
  //SerialMonitorInterface.begin(115200);
  Wire.begin();
  if (MOTOR_TYPE == DC_MOTOR) {
    DcMotorInit(maxPWM);
  } else {
    stepperInit();
  }
  delay(100);
  setMotorCurrent(100);
}
/*******************************************************************/
void drive(float left_proportion, float right_proportion, float delay_seconds)
{
  int left_speed;
  int right_speed;

  if(MOTOR_TYPE == DC_MOTOR)
  {
    left_speed  = -left_proportion * maxPWM;
    right_speed = right_proportion * maxPWM;

    SerialMonitorInterface.print(", left_speed: ");
    SerialMonitorInterface.print(left_speed);
    SerialMonitorInterface.print(", right_speed: ");
    SerialMonitorInterface.println(right_speed);

    setDCMotor(LEFT_PORT, left_speed);
    setDCMotor(RIGHT_PORT, right_speed);
    delay(delay_seconds * 1000);
  }
  else
  {
    left_speed  = SERVO_HALT - left_proportion * MAX_SERVO_OFFSET;
    right_speed = SERVO_HALT + right_proportion * MAX_SERVO_OFFSET;
    
    SerialMonitorInterface.print(", left_speed: ");
    SerialMonitorInterface.print(left_speed);
    SerialMonitorInterface.print(", right_speed: ");
    SerialMonitorInterface.println(right_speed);

    servo.setServo(LEFT_PORT, left_speed);
    servo.setServo(RIGHT_PORT, right_speed);     
    delay(delay_seconds * 1000);
  }
  
  
}
