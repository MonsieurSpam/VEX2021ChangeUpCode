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
// LFDrive              motor         1               
// LRDrive              motor         2               
// RFDrive              motor         18              
// RRDrive              motor         9               
// LIntake              motor         12              
// RIntake              motor         19              
// LRoller              motor         13              
// RRoller              motor         20              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h" 

using namespace vex; 
using namespace std;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
float speedVar=1.0f; 
double rotRatio=0.17857142857;
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
void deploySequence(float time){
  LIntake.spin(fwd,100,pct);
  RIntake.spin(fwd,100,pct); 
  wait(time,seconds);
  LIntake.stop();
  RIntake.stop();

}  

void stopIntakes(){
  LIntake.stop(brake);
  RIntake.stop(brake);
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

void rightEncoderTurn(float rotations){
    RFDrive.rotateFor(vex::directionType::rev,rotations,rev,false);
    RRDrive.rotateFor(vex::directionType::rev,rotations,rev,false);
    LFDrive.rotateFor(vex::directionType::fwd,rotations,rev,false); 
    LRDrive.rotateFor(vex::directionType::fwd,rotations,rev,false);
}
void driveFor(float rotations,string direction){ 
  if(direction=="forward"){ 
    RFDrive.rotateFor(vex::directionType::fwd,rotations,rev,false);
    RRDrive.rotateFor(vex::directionType::fwd,rotations,rev,false);
    LFDrive.rotateFor(vex::directionType::fwd,rotations,rev,false); 
    LRDrive.rotateFor(vex::directionType::fwd,rotations,rev,false);
   
  }else if(direction=="reverse"){
    RFDrive.rotateFor(vex::directionType::rev,rotations,rev,false);
    RRDrive.rotateFor(vex::directionType::rev,rotations,rev,false);
    LFDrive.rotateFor(vex::directionType::rev,rotations,rev,false); 
    LRDrive.rotateFor(vex::directionType::rev,rotations,rev,false);
  }
} 


void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // .......................................................................... 
  shootBall(.5);
  deploySequence(1.5);
  driveFor(0.5,"forward");
  wait(0.75,seconds);
  rightEncoderTurn(0.75); 
  wait(1.5,seconds);
  driveFor(.95,"forward");
  wait(1.5,seconds);
  shootBall(1.0);
  driveFor(1.0,"reverse");
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
  while (1) { 
    if(Controller1.ButtonUp.pressing()){
      speedVar=1.0f;
    }

    if(Controller1.ButtonDown.pressing()){
      speedVar=0.5f;
    }
    //Drive Control Axis 1 controls forward & back, Axis 3 controls left & right
    if (abs(Controller1.Axis1.value())>>5 || abs(Controller1.Axis3.value()>>5)){
      LFDrive.spin(fwd, (Controller1.Axis3.value()+Controller1.Axis1.value())*speedVar,pct);
      LRDrive.spin(fwd, (Controller1.Axis3.value()+Controller1.Axis1.value())*speedVar,pct); 
      RFDrive.spin(fwd, (Controller1.Axis3.value()-Controller1.Axis1.value())*speedVar,pct);
      RRDrive.spin(fwd, (Controller1.Axis3.value()-Controller1.Axis1.value())*speedVar,pct);
    }else{
      LFDrive.stop(brake);
      LRDrive.stop(brake);
      RFDrive.stop(brake);
      RRDrive.stop(brake);
    }

  
 
   if(Controller1.ButtonL1.pressing()){
   
      LRoller.spin(fwd,100,pct);
      RRoller.spin(fwd,100,pct); 
     
    }
    else if(Controller1.ButtonL2.pressing()){ 
     
      LIntake.spin(reverse, 100,pct);
      RIntake.spin(reverse,100,pct);  

   
    }else if(Controller1.ButtonR1.pressing()){
     LIntake.spin(fwd,100,pct);
      RIntake.spin(fwd,100,pct);

      
    }else if(Controller1.ButtonR2.pressing()){
      LRoller.spin(reverse,100,pct);
      RRoller.spin(reverse,100,pct); 

    }else{
      LIntake.stop(brake);
      RIntake.stop(brake); 
      LRoller.stop();
      RRoller.stop();
    } 



    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
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
