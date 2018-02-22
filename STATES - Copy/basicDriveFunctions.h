#pragma systemFile
void drive(int powerL, int powerR){
	motor[driveLeftBack] = motor[driveLeftFront]  = powerL;
	motor[driveRightBack] = motor[driveRightFront]  = powerR;
}
