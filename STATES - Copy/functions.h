#pragma systemFile

//Functions
void powRoller(int power){
motor[roller] = power;
}

void powMogo (int power){
motor[Mogo] = power;
}

void powLift(int powerL, int powerR = powerL)
{
motor[liftLeft] = powerL;
motor[liftRight] = powerR;
}

void powFourBar(int powerL, int powerR = powerL)
{
	motor[fourBarLeft] = powerL;
	motor[fourBarRight] = powerR;
}

void clearGyro()
{
getHeading = 0;
}

void clearDriveEnc(){
getdriveEnc = 0;
}

int limit(int number, int max)
{

	if(abs(max)<abs(number))
		return sgn(number)*abs(max);
	else
		return number;
}
