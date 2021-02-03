#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor TopBackRoller = motor(PORT12, ratio18_1, false);
motor BottomBackRoller = motor(PORT7, ratio18_1, false);
controller Controller1 = controller(primary);
motor TopRightDrive = motor(PORT4, ratio18_1, false);
motor BottomRightDrive = motor(PORT2, ratio18_1, false);
motor TopLeftDrive = motor(PORT1, ratio18_1, false);
motor BottomLeftDrive = motor(PORT3, ratio18_1, false);
motor RightIntake = motor(PORT8, ratio18_1, false);
motor LeftIntake = motor(PORT9, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}