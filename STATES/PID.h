#define getDriveEnc SensorValue[encDrive]


#pragma systemfile


typedef struct{

float kP, kI, kD;

int Error, Integral, Derivitive, prevError, integralLimit;

int maxIntegral, maxOutput, minSpeed;

} PID;

void resetPID(
PID controller)
{

controller.Error = 0;
controller.Integral = 0;
controller.Derivitive = 0;
controller.prevError = 0;

}

void setMinSpeedPID(
PID controller,
int minSpeed
)
{
controller.minSpeed = minSpeed;
}

void initPID(
PID controller,
float kP,
float kI,
float kD)
{

controller.kP = kP;
controller.kI = kI;
controller.kD = kD;

controller.maxIntegral = 0;
controller.maxOutput = 127;
controller.minSpeed = 0;
controller.integralLimit = 100;

resetPID(controller);

}

//////////////////////////////////////////////////////////Drive PID///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////Drive PID///////////////////////////////////////////////////////////
int drivePIDCalculation(
int targetDrivePos,
PID controller)
{

controller.Error = targetDrivePos - getDriveEnc;

if(controller.integralLimit > abs(controller.Error) &&
	controller.integralLimit != 0 &&
controller.Integral < 10000)
controller.Integral += controller.Error;
else
	controller.Integral = 0;


if (controller.maxIntegral != 0)
{
	if (controller.Integral > controller.maxIntegral)
		controller.Integral = controller.maxIntegral;

	if (controller.Integral < -controller.maxIntegral)
		controller.Integral = -controller.maxIntegral;
}

controller.Derivitive = controller.Error - controller.prevError;
controller.prevError = controller.Error;

int returnValue = ((int)abs((controller.Error*controller.kP +
controller.Integral*controller.kI -
controller.Derivitive*controller.kD)) > controller.minSpeed)
?
(int)(controller.Error*controller.kP +
controller.Integral*controller.kI -
controller.Derivitive*controller.kD)
:
controller.minSpeed*sgn((int)(controller.Error*controller.kP + controller.Integral*controller.kI - controller.Derivitive*controller.kD));

if (controller.maxOutput != 0)
{
	if (returnValue > controller.maxOutput)
		returnValue = controller.maxOutput;

	if (returnValue < -controller.maxOutput)
		returnValue = -controller.maxOutput;
}
return returnValue;
}



int turnToHeadingCalucation(int targetHeading, PID controller){

controller.Error = targetHeading - (getHeading)*(ticsPerDegree);

if(abs(controller.Error) < 100){
	controller.Integral += controller.Error;
}

if(controller.Error == 0 || abs(controller.Error) < 10){
	controller.Integral = 0;
}
	controller.Derivitive = controller.prevError - controller.error;


controller.prevError = controller.error;
int returnValue = controller.Error * controller.kP + controller.Integral * controller.kI - controller.Derivitive * controller.kD;

return returnValue;
}

///////////////////// MOGO
int mogoPIDCalculation(
int targetMogoPos,
PID controller)
{

controller.Error = targetMogoPos - getMogoPos;

if(controller.integralLimit > abs(controller.Error) &&
	controller.integralLimit != 0 &&
controller.Integral < 10000)
controller.Integral += controller.Error;
else
	controller.Integral = 0;


if (controller.maxIntegral != 0)
{
	if (controller.Integral > controller.maxIntegral)
		controller.Integral = controller.maxIntegral;

	if (controller.Integral < -controller.maxIntegral)
		controller.Integral = -controller.maxIntegral;
}

controller.Derivitive = controller.Error - controller.prevError;
controller.prevError = controller.Error;

int returnValue = ((int)abs((controller.Error*controller.kP +
controller.Integral*controller.kI -
controller.Derivitive*controller.kD)) > controller.minSpeed)
?
(int)(controller.Error*controller.kP +
controller.Integral*controller.kI -
controller.Derivitive*controller.kD)
:
controller.minSpeed*sgn((int)(controller.Error*controller.kP + controller.Integral*controller.kI - controller.Derivitive*controller.kD));

if (controller.maxOutput != 0)
{
	if (returnValue > controller.maxOutput)
		returnValue = controller.maxOutput;

	if (returnValue < -controller.maxOutput)
		returnValue = -controller.maxOutput;
}
return returnValue;
}

///////////////////////////////// LIFT
int liftPIDCalculation(
int targetLiftPos,
PID controller,
int liftPot)
{

controller.Error = targetLiftPos - liftPot;

if(controller.integralLimit > abs(controller.Error) &&
	controller.integralLimit != 0 &&
controller.Integral < 10000)
controller.Integral += controller.Error;
else
	controller.Integral = 0;


if (controller.maxIntegral != 0)
{
	if (controller.Integral > controller.maxIntegral)
		controller.Integral = controller.maxIntegral;

	if (controller.Integral < -controller.maxIntegral)
		controller.Integral = -controller.maxIntegral;
}

controller.Derivitive = controller.Error - controller.prevError;
controller.prevError = controller.Error;

int returnValue = ((int)abs((controller.Error*controller.kP +
controller.Integral*controller.kI -
controller.Derivitive*controller.kD)) > controller.minSpeed)
?
(int)(controller.Error*controller.kP +
controller.Integral*controller.kI -
controller.Derivitive*controller.kD)
:
controller.minSpeed*sgn((int)(controller.Error*controller.kP + controller.Integral*controller.kI - controller.Derivitive*controller.kD));

if (controller.maxOutput != 0)
{
	if (returnValue > controller.maxOutput)
		returnValue = controller.maxOutput;

	if (returnValue < -controller.maxOutput)
		returnValue = -controller.maxOutput;
}
return returnValue;
}

///////////////////// FOURBAR
int fourBarPIDCalculation(
int targetMogoPos,
PID controller)
{

controller.Error = targetMogoPos - getFourBar;

if(controller.integralLimit > abs(controller.Error) &&
	controller.integralLimit != 0 &&
controller.Integral < 10000)
controller.Integral += controller.Error;
else
	controller.Integral = 0;


if (controller.maxIntegral != 0)
{
	if (controller.Integral > controller.maxIntegral)
		controller.Integral = controller.maxIntegral;

	if (controller.Integral < -controller.maxIntegral)
		controller.Integral = -controller.maxIntegral;
}

controller.Derivitive = controller.Error - controller.prevError;
controller.prevError = controller.Error;

int returnValue = ((int)abs((controller.Error*controller.kP +
controller.Integral*controller.kI -
controller.Derivitive*controller.kD)) > controller.minSpeed)
?
(int)(controller.Error*controller.kP +
controller.Integral*controller.kI -
controller.Derivitive*controller.kD)
:
controller.minSpeed*sgn((int)(controller.Error*controller.kP + controller.Integral*controller.kI - controller.Derivitive*controller.kD));

if (controller.maxOutput != 0)
{
	if (returnValue > controller.maxOutput)
		returnValue = controller.maxOutput;

	if (returnValue < -controller.maxOutput)
		returnValue = -controller.maxOutput;
}
return returnValue;
}

void driveKeepHeading(int encoderCount, int gyroHeading = targetHeading, int addedTime = 0, int stationaryCount = driveStraightStationaryCountThresh, bool display = false)
{

	PID driveController;
	initPID(driveController, 0.75, 0.000, 0.55);

	PID turnController;
	initPID(turnController, 0.6, 0.00, 0.1);

	int statCount = 0;
	time1[T1] = 0;
	int estimatedTime = abs(driveTimeConst*(encoderCount-getdriveEnc));

	int driveTurnPower = 0;
	int drivePower = 0;

	while( ((time1[T1]-addedTime) < estimatedTime) && (statCount < stationaryCount))
	{

		drivePower = limit(drivePIDCalculation(encoderCount, driveController), 127);

		if(abs(targetHeading-getHeading)>10)
			driveTurnPower = turnToHeadingCalucation(gyroHeading, turnController);
		else
			driveTurnPower = 0;

		drive(drivePower - driveTurnPower, drivePower + driveTurnPower);

		if(	abs(targetHeading-getHeading) < 5 && (abs(drivePosTarget-getDriveEnc) < 30 )
		{
			statCount++;
		}
		else
		{
			statCount = 0;
		}

		delay(5);
	}
	drive(0,0);
}

void driveTurn (int targetHeading, int addedTime = 0, float kPTurn = 0.30, int stationaryCount = turnStraightStationaryCountThresh, bool display = false)
{
		PID turnController;
		//initPID(turnController, kPTurn, 0.0001, 11.8);
		initPID(turnController, kPTurn, 0.0005, 3.8);
		time1[T2] = 0;

		int statCount = 0;

		int estimatedTime = ( (turnTimeConst) * abs((targetHeading-getHeading)) );

		while((time1[T2]-addedTime) < (estimatedTime) && statCount < stationaryCount)
		{

			int driveTurnPower = turnToHeadingCalucation(targetHeading, turnController);

			drive( -driveTurnPower ,  driveTurnPower );

			if(display)
				writeDebugStreamLine("statCount: %d, estimatedTime: %d, (time1[T2]-addedTime): %d, getHeading: %d, targetHeading: %d", statCount, estimatedTime, (time1[T2]-addedTime), getHeading, targetHeading);

			if(	abs(targetHeading - (getHeading)*(ticsPerDegree)) < 17 )
			{
				statCount++;
			}
		 	else
			{
				statCount = 0;
			}
			delay(5);
		}
		drive(0,0);
}

void driveSmoothTurn ( int medianSpeed,  int targetHeadings, int addedTime = 0 ,int stationaryCount = 200, bool display = false)
{
	PID turnController;
	initPID(turnController, 0.1, 0.00, 1000);

	time1[T3] = 0;

	int estimatedTime = abs(turnTimeConst*(targetHeadings-getHeading));
	int statCount = 0;
	int driveTurnPower = 1;


	while( (time1[T3]-addedTime) < estimatedTime && statCount < stationaryCount)
	{
		driveTurnPower = turnToHeadingCalucation(targetHeadings, turnController);
		drive(medianSpeed + driveTurnPower*sgn(medianSpeed), medianSpeed - driveTurnPower*sgn(medianSpeed));

		if(display)
			writeDebugStreamLine("statCount: %d, estimatedTime: %d, (time1[T2]-addedTime): %d, getHeading: %d, targetHeading: %d", statCount, estimatedTime, (time1[T2]-addedTime), getHeading, targetHeadings);


		if(	abs(targetHeadings-getHeading) < 20 )
		{
			statCount++;
		}
		else
		{
			statCount = 0;
		}

	}
	drive(0,0);
}

void liftPID(int potCount, int addedTime = 0, int stationaryCount = driveStraightStationaryCountThresh, float kP = 0.13 ,bool display = false)
{

	PID liftController;
	initPID(liftController, kP, 0.000, 0.01);

	int statCount = 0;
	time1[T3] = 0;
	int estimatedTime = abs(liftTimeConst*(potCount-(getLiftRight+getLiftRight)/2));


	int liftLeftPower = 0;
	int liftRightPower = 0;

	while( ((time1[T3]-addedTime) < estimatedTime) && (statCount < stationaryCount))
	{

		liftLeftPower = limit(liftPIDCalculation(potCount, liftController, getLiftLeft), 127);
		liftRightPower = limit(liftPIDCalculation(potCount, liftController, getLiftRight), 127);

	  powLift(-liftLeftPower, -liftRightPower);


		if(	abs(potCount-getLiftLeft) < 100 && (abs(potCount-getLiftRight) < 100 )
		{
			statCount++;
		}
		else
		{
			statCount = 0;
		}

		delay(5);
	}
	powLift(0,0);
}

void mogoPID(int potCount, int addedTime = 0, int stationaryCount = driveStraightStationaryCountThresh, bool display = false)
{

	PID mogoController;
	initPID(mogoController, 0.75, 0.000, 0.55);

	int statCount = 0;
	time1[T2] = 0;
	int estimatedTime = abs(mogoTimeConst*(potCount-getMogoPos));


	int powerMogo = 0;

	while( ((time1[T2]-addedTime) < estimatedTime) && (statCount < stationaryCount))
	{

		powerMogo = limit(mogoPIDCalculation(potCount, mogoController), 127);

	  powMogo(powerMogo);


		if(	abs(potCount-getMogoPos) < 10 )
		{
			statCount++;
		}
		else
		{
			statCount = 0;
		}

		delay(5);
	}
	powMogo(0);
}

void fourBarPID(int potCount, int addedTime = 0, int stationaryCount = driveStraightStationaryCountThresh, bool display = false)
{

	PID fourBarController;
	initPID(fourBarController, 0.2, 0.000, 0.0);

	int statCount = 0;
	time1[T1] = 0;
	int estimatedTime = abs(fourBarTimeConst*(potCount-(getFourBar)));


	int powerFourBar = 0;

	while( ((time1[T1]-addedTime) < estimatedTime) && (statCount < stationaryCount))
	{

		powerFourBar = limit(fourBarPIDCalculation(potCount, fourBarController), 127);

	  powFourBar(powerFourBar);


		if(	abs(potCount-getFourBar) < 50 )
		{
			statCount++;
		}
		else
		{
			statCount = 0;
		}

		delay(5);
	}
	powFourBar(0,0);
}

void fourBarUp()
{
	while(getFourBar<FOURBAR_STACK-100)
	{
		powFourBar(127);
	}
	powFourBar(20);
}

void fourBarDown()
{
	while(getFourBar>FOURBAR_PICK+100)
	{
		powFourBar(-127);
	}
	powFourBar(-20);
}


task liftPIDAuto
{
	while(vexRT[Btn6U] == 1)
	{
		powLift(127, 127);
	}
	powLift(0,0);
}

task inDownUpOut
{
	fourBarUp();
	powRoller(-127);
	powLift(-127, -127);
	wait1Msec(300);
	powLift(127, 127);
	wait1Msec(250);
	powLift(10,10);
	fourBarDown();
	powRoller(127);
	liftPID(LIFT_AUTOSTACK_DOWN);
	while(getLiftAv > (LIFT_AUTOSTACK_DOWN+100)){}
	inStackLoop = false;
}



void go2Stack(){

inStackLoop = true;

LIFT_AUTOSTACK_DOWN = getLiftAv;

fourBarDown();
startTask(liftPIDAuto);

while(vexRT[Btn6U] == 1) {}

startTask(InDownUpOut);


}

/*
task autoStack(){
while(1){
	while(!breaker){
		loopIsRunning = false;
		if(vexRT[Btn5U] == 0){
			powRoller(90);
			if(vexRT[Btn6U] == 1)
				powFourBar(127);
			if(vexRT[Btn6D] == 1)
				powFourBar(-127);
		}

		else{
			loopIsRunning = true;
			while(!breaker && loopIsRunning){
				powRoller(20);
				wait1Msec(100);
				go2Stack();
				wait1Msec(200);
				powRoller(127);
				wait1Msec(100);
				coneCount++;
				if(coneCount > 16 || coneCount <0){
					coneCount=16;
				}
				loopIsRunning = false;
			}
		}
	}
}
}
*/

void holdStack_orient()
{
	isStackHeld = true;
	inStackLoop = true;

	powRoller(5);

	fourBarUp();

	powLift(-80, -80);
	wait1Msec(200);
	powLift(0, 0);
	inStackLoop = false;
}

void normal_orient()
{
	isStackHeld = false;
	inStackLoop = true;

	powRoller(-127);
	powLift(-90, -90);
	wait1Msec(250);
	powLift(10, 10);
	fourBarDown();

	powLift(0, 0);

	inStackLoop = false;

}

task singTask
{

		LIFT_AUTOSTACK_DOWN = getLiftAv;
		powRoller(80);
		powLift(127, 127);
		while(vexRT[Btn6U] == 1) {}
		powLift(10, 10);

		fourBarDown();
		while(vexRT[Btn5U] == 0 || vexRT[Btn5D] == 0) {}
		powLift(-110, -110);
		while(vexRT[Btn5U] == 1 || vexRT[Btn5D] == 1) {}
		powRoller(-127);
		powLift(10,10);
		wait1Msec(300);

		while(vexRT[Btn5U] == 0 || vexRT[Btn5D] == 0) {}
		powLift(127, 127);
		while(vexRT[Btn5U] == 1 || vexRT[Btn5D] == 1) {}
		powLift(10,10);
		fourBarUp();
		powRoller(127);
		liftPID(LIFT_AUTOSTACK_DOWN);
		while(getLiftAv > (LIFT_AUTOSTACK_DOWN+100)){}
		powRoller(5);
		inStackLoop = false;
}

void singapore()
{
	inStackLoop = true;
	startTask(singTask);

}
