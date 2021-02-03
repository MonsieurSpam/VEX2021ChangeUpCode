using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor TopBackRoller;
extern motor BottomBackRoller;
extern controller Controller1;
extern motor TopRightDrive;
extern motor BottomRightDrive;
extern motor TopLeftDrive;
extern motor BottomLeftDrive;
extern motor RightIntake;
extern motor LeftIntake;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );