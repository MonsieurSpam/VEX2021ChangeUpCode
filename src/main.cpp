/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LIntake              motor         12              
// RIntake              motor         19              
// LRoller              motor         13              
// RRoller              motor         20              
// Drivetrain           drivetrain    1, 2, 18, 9, 21 
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h" 

using namespace vex; 
using namespace std;

// A global instance of competition
competition Competition; 
int driveFunctions();
int intakeControl();
int rollerControl();
int faceButton(); 
int screenTask();
// define your global instances of motors and other devices here
float speedVar=1.0f; 
float circRatio=12.5663706144f;



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();  
  LeftDriveSmart.setPosition(0,vex::rotationUnits::raw);
  RightDriveSmart.setPosition(0,vex::rotationUnits::raw);
  TurnGyroSmart.calibrate();
  wait(2,seconds);
  task StartScreenTask(screenTask);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ... 

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void stopIntakes(){
  LIntake.stop(brake);
  RIntake.stop(brake);
}

void runIntake(){
  LIntake.spin(reverse,100,pct);
  RIntake.spin(reverse,100,pct);
}

void shootBall(float time){
  LRoller.spin(fwd,100,pct);
  RRoller.spin(fwd,100,pct);
  wait(time,seconds);
  LRoller.stop();
  RRoller.stop();

}

void removeBall(){
  LRoller.spin(vex::directionType::rev,100,pct); 
  RRoller.spin(vex::directionType::rev,100,pct);
} 

void resetDriveEncoders(){
  LeftDriveSmart.setPosition(0,vex::rotationUnits::raw);
  RightDriveSmart.setPosition(0,vex::rotationUnits::raw);
}

void PIDTurn(double desiredDeg,double kp, double kd,double timeLimit){ 
  double startTime=Brain.timer(seconds);
  double lastError=0; 
  while( fabs(desiredDeg-TurnGyroSmart.rotation(degrees))>1 && Brain.timer(seconds)-startTime<timeLimit){
    double error= desiredDeg-TurnGyroSmart.rotation(degrees);
    double proportional=error*kp;
    double DValue=error-lastError;
    double derivative=DValue*kd; 
    double Power=proportional+derivative;
    Drivetrain.turn(right,Power,vex::velocityUnits::pct); 
    lastError=error; 
    vex::task::sleep(10);
  }
  Drivetrain.stop();
  resetDriveEncoders();
} 



void PIDDrive(double desiredDist,double kp, double kd,double dKp,double dKd){
  double desiredTicks=desiredDist*(1/circRatio)*(900);
  double lastError=0;  
  double lastDriveError=0;
  double driveKp=dKp;
  double driveKd=dKd; 

  while(fabs(desiredTicks-LeftDriveSmart.rotation(vex::rotationUnits::raw))>10){
    double error=desiredTicks-LeftDriveSmart.rotation(vex::rotationUnits::raw);
    double proportional=error*kp;
    double DValue=error-lastError;
    double derivative=DValue*kd;
    double Power=proportional+derivative; 
    
    
    double driveError=RightDriveSmart.rotation(vex::rotationUnits::raw)-LeftDriveSmart.rotation(vex::rotationUnits::raw);
 
    double rightPower=Power-(fabs(driveError*driveKp))-(fabs((driveError-lastDriveError)*driveKd));

    if(rightPower<Power*.556){
      rightPower=Power;
    }

    LeftDriveSmart.spin(fwd,Power,vex::velocityUnits::pct);
    RightDriveSmart.spin(fwd,rightPower,vex::velocityUnits::pct);
    lastDriveError=driveError;
    lastError=error;
    vex::task::sleep(10);
  }

  LeftDriveSmart.stop(brake);
  RightDriveSmart.stop(brake);
  resetDriveEncoders();
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................  
  //PIDTurn(90,0.75,1);
  //wait(1.5,seconds);
  //Drivetrain.driveFor(15,inches);
  runIntake();
  PIDDrive(20,0.2,0.3,0.6,0.2);
  wait(1.5,seconds);
  PIDTurn(152,0.5,.4,.8);
  stopIntakes();
  Drivetrain.setDriveVelocity(75,pct);
  Drivetrain.drive(fwd);
  wait(1.3,seconds);
  Drivetrain.stop();
  shootBall(1.8);
  runIntake();
  resetDriveEncoders();

  PIDDrive(-18,0.2,0.3,0.1,0.018);
  wait(0.6,seconds);
  
  PIDTurn(270,0.5,0.4,1);
  wait(0.5,seconds);
  stopIntakes();
  shootBall(0.2);
  Drivetrain.setDriveVelocity(45,pct);
  Drivetrain.drive(reverse);

  wait(1.7,seconds);
  Drivetrain.stop();
  wait(0.5,seconds);
  TurnGyroSmart.setHeading(0,degrees);
  resetDriveEncoders();
  wait(0.5,seconds);
  PIDDrive(63,0.1,0.3,0.6,0.2);
  wait(1.5,seconds);
  PIDTurn(184,0.6,.4,1);
  Drivetrain.drive(fwd);
  wait(0.8,seconds);
  Drivetrain.stop();
  shootBall(1.8);
  PIDDrive(-15,0.2,0.3,0.1,0.018);
  wait(1.5,seconds);
  PIDTurn(0,0.5,.4,.8);
  /*shootBall(.5);  
  
  
  wait(0.75,seconds);
  wait(1.5,seconds);
 
  wait(1.5,seconds);
  shootBall(1.0);
  */
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) { 
    task StartDriveTask(driveFunctions);
    task StartIntakeTask(intakeControl);
    task StartRollerTask(rollerControl);
    task StartButtonTask(faceButton);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}


int driveFunctions(){
  motor LFDrive = motor(PORT1, ratio18_1, false);
  motor LRDrive = motor(PORT2, ratio18_1, true);
  motor RFDrive = motor(PORT18, ratio18_1, true);
  motor RRDrive = motor(PORT9, ratio18_1, false); 

  while (1) { 
    if(Controller1.ButtonUp.pressing()){
      speedVar=1.0f;
    }

    if(Controller1.ButtonDown.pressing()){
      speedVar=0.5f;
    }
    //Drive Control Axis 1 controls forward & back, Axis 3 controls left & right
    if (abs(Controller1.Axis1.value())>>5 || abs(Controller1.Axis3.value()>>5)){
      LFDrive.spin(reverse, (Controller1.Axis3.value()+0.75*Controller1.Axis1.value())*speedVar,pct);
      LRDrive.spin(reverse, (Controller1.Axis3.value()+0.75*Controller1.Axis1.value())*speedVar,pct); 
      RFDrive.spin(reverse, (Controller1.Axis3.value()-0.75*Controller1.Axis1.value())*speedVar,pct);
      RRDrive.spin(reverse, (Controller1.Axis3.value()-0.75*Controller1.Axis1.value())*speedVar,pct);
    }else{
      LFDrive.stop(brake);
      LRDrive.stop(brake);
      RFDrive.stop(brake);
      RRDrive.stop(brake);
    }
    vex::task::sleep(10);
  } 

} 

/* Task to control intake
   L1-intake roller
   R1-Outtake rollers
*/
int intakeControl(){ 
  while(1){
    if (Controller1.ButtonL1.pressing()){
      LIntake.spin(reverse,100,pct);
      RIntake.spin(reverse,100,pct);

    }else if(Controller1.ButtonR1.pressing()){
      LIntake.spin(fwd,100,pct);
      RIntake.spin(fwd,100,pct);
    }else{
      LIntake.stop(brake);
      RIntake.stop(brake);
    }
    vex::task::sleep(10);
  }
}  


//Task to control rollers
int rollerControl(){ 
  while(1){
    if(Controller1.ButtonL2.pressing()){
      LRoller.spin(fwd,100,pct);
      RRoller.spin(fwd,100,pct);
    }else if(Controller1.ButtonR2.pressing()){
      LRoller.spin(reverse,100,pct);
      RRoller.spin(reverse,100,pct);
    }else{
      LRoller.stop();
      RRoller.stop();
    } 
    vex::task::sleep(10);
  }
}


int faceButton(){
  while(1){
    if(Controller1.ButtonA.pressing()){
      autonomous();
    }
    vex::task::sleep(10);
  }
}  

int screenTask(){
  while(1){
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,2);
    Controller1.Screen.print(TurnGyroSmart.rotation(degrees)); 
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print(LeftDriveSmart.rotation(vex::rotationUnits::raw));
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print(RightDriveSmart.rotation(vex::rotationUnits::raw));
    Controller1.Screen.setCursor(1,8);
    Controller1.Screen.print(Brain.timer(seconds));
    vex::task::sleep(5);
  }
}
 
