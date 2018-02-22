#pragma systemFile

int coneCount = 0;

bool inStackLoop = false;

const int armPosAuto[16] = {3400, 3000, 2700, 2400, 2100, 1700, 1300, 2770, 2850, 3000, 4000, 3000, 3000, 3000, 3000, 3000};
#define FOURBAR_STACK 2100
#define FOURBAR_PICK 300
#define LIFT_DOWN 3400

int waitBeforeAuton = 0;

#define wheelDiameter 1

const float ticsPerDegree = 0.9506;

const int slits = 90;
const float wheelDiameterMeters = 0.1016;
const float metersPerTick = wheelDiameterMeters*PI/slits;

const float basewheelcirc = 12.99;

const float baseToInches = 25.94

#define testedDriveTimeWaitConst (2)
#define testedTurnTimeWaitConst (3)

#define wheelCircum (wheelDiameter)

#define driveTimeConst round(testedDriveTimeWaitConst*wheelCircum)
#define turnTimeConst round(testedTurnTimeWaitConst*wheelCircum)
#define mogoTimeConst 5
#define fourBarTimeConst 1
#define liftTimeConst 2

#define driveSlowSpeed (127-30)

#define getdriveEnc SensorValue[encDrive]
#define getHeading SensorValue[gyro]
#define getFourBar SensorValue[potFourBar]
#define getLiftLeft (SensorValue[potLeftLift]-175)
#define getLiftRight SensorValue[potRightLift]
#define getMogoPos SensorValue[potMogo]

#define driveStraightStationaryCountThresh 50
#define turnStraightStationaryCountThresh 10



bool isMove = true;
bool mogoIsFront = false;
bool breaker = false;
bool loopIsRunning = false;

int drivePosTarget = 0;
int targetHeading = 0;
