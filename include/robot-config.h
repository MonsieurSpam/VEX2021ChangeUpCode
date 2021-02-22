using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LFDrive;
extern motor LRDrive;
extern motor RFDrive;
extern motor RRDrive;
extern motor LIntake;
extern motor RIntake;
extern motor LRoller;
extern motor RRoller;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );