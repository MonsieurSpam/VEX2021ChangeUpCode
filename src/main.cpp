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
// TopBackRoller        motor         12              
// BottomBackRoller     motor         7               
// Controller1          controller                    
// TopRightDrive        motor         4               
// BottomRightDrive     motor         2               
// TopLeftDrive         motor         1               
// BottomLeftDrive      motor         3               
// RightIntake          motor         8               
// LeftIntake           motor         9               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex; 
using namespace std;

double speedVar=0.5;

// A global instance of competition
competition Competition; 

//define tasks  

int controllerStuff(); 
//int autoScorer();

// define your global instances of motors and other devices here

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
 // task StartMyTask(autoScorer);

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

//##########################################AUTONOMOUS DRIVE FUNCTIONS#################################################
 

 /*Allows our bot to drive forward and backwards autonomously. Takes a string and rotation argument that is used to determine whether or not the 
 bot will go forwards or backwards and how far*/
  
void driveFwdBkwd(string direction, int rotations){
  if(direction=="forward"){
    TopRightDrive.spinFor(fwd,rotations,rev);
    BottomRightDrive.spinFor(reverse,rotations,rev); 
    TopLeftDrive.spinFor(reverse,rotations,rev);
    BottomLeftDrive.spinFor(fwd,rotations,rev);
  }else if(direction=="backward"){
    TopRightDrive.spinFor(reverse,rotations,rev);
    BottomRightDrive.spinFor(fwd,rotations,rev); 
    TopLeftDrive.spinFor(fwd,rotations,rev);
    BottomLeftDrive.spinFor(reverse,rotations,rev);
  }
}  

/*Allows us to control our rollers. Takes an integer and a double(decimal number) that determines how fast and how long we want the rollers to run. 
We only have the rollers run one way because our intake system will do all the sorting*/
void rollers(int speed,double timeUnit){ 
  LeftIntake.setVelocity(speed,pct);
  RightIntake.setVelocity(speed,pct); 

  LeftIntake.spinFor(fwd,timeUnit,sec);
  RightIntake.spinFor(reverse,timeUnit,sec);
} 

//Allows us to control our intake system. Takes a string and a double that determines whether we intake our outake, and for how long
void intakeOutake(string direction, double timeUnit){
  if(direction=="intake"){
    TopBackRoller.spinFor(reverse,timeUnit,sec);  
    BottomBackRoller.spinFor(fwd,timeUnit,sec);
  }else if(direction=="outake"){
    TopBackRoller.spinFor(fwd,timeUnit,sec);  
    BottomBackRoller.spinFor(fwd,timeUnit,sec);
  }
}
 


void autonomous(void) {
  // ..........................................................................
  // Notice how we can simply call in the functions that were defined above in this autonomous function. 
  //The reason we split it up into separate functions is to make the code extremely compact here
  // .......................................................................... 
  driveFwdBkwd("forward",5); 
  rollers(75,4.5);
  intakeOutake("intake",3); 

  /*^^^Notice that these three lines of code call in all the code that was written above. If we didn't choose to make our own functions  
  we would have to write the code above every single time we wanted to drive forwards and backwards or use our rollers and intake.*/


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

  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................


    //Button A controls intake. Button B controls outtake
    if(Controller1.ButtonA.pressing()){
      TopBackRoller.spin(reverse,100,pct); 
      BottomBackRoller.spin(fwd,100,pct);
    }else if(Controller1.ButtonB.pressing()){
      TopBackRoller.spin(fwd,100,pct);
      BottomBackRoller.spin(fwd,100,pct);
    }else{
      TopBackRoller.stop(coast);
      BottomBackRoller.stop(coast);
    } 

    //Button Up sets drive to 100% velocity. Button Down sets drive to 50% velocity
    if(Controller1.ButtonUp.pressing()){
      speedVar=1.0;
    }

    if(Controller1.ButtonDown.pressing()){
      speedVar=0.5;
    }

    if (Controller1.ButtonL1.pressing()){
      RightIntake.spin(fwd,100,pct);
      LeftIntake.spin(reverse,100,pct);
    }else if(Controller1.ButtonL2.pressing()){
      RightIntake.spin(reverse,100,pct);
      LeftIntake.spin(fwd,100,pct);
    }else{
      RightIntake.stop(coast);
      LeftIntake.stop(coast);
    }
    //Drive base code. Axis 3 Controls fwd & bkwd. Axis1 Controls left & right
    if(abs(Controller1.Axis3.value())>5 || abs(Controller1.Axis1.value())>5){
      TopRightDrive.spin(reverse,speedVar*(Controller1.Axis3.value()-(Controller1.Axis1.value())),pct);
      BottomRightDrive.spin(fwd,speedVar*(Controller1.Axis3.value()-(Controller1.Axis1.value())),pct);

      TopLeftDrive.spin(fwd,speedVar*(Controller1.Axis3.value()+(Controller1.Axis1.value())),pct);
      BottomLeftDrive.spin(reverse,speedVar*(Controller1.Axis3.value()+(Controller1.Axis1.value())),pct);
    }else{
      TopRightDrive.stop(brake);
      BottomRightDrive.stop(brake);
      TopLeftDrive.stop(brake);
      BottomLeftDrive.stop(brake);
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

/*int autoSorter(){
  while(1){
      if(Controller1.ButtonB.pressing()){
        TopBackRoller.spinFor(reverse,2,sec);  
        BottomBackRoller.spinFor(fwd,2,sec);
      }else if(Controller1.ButtonX.pressing()){
        TopBackRoller.spinFor(fwd,2,sec);  
        BottomBackRoller.spinFor(fwd,2,sec);
      }

  } 
   

}*/





