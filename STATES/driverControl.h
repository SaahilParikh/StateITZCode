void driverControl()
{

if(vexRT[Btn6U] == 1 && !(buttonHasBeenHeld6U==vexRT[Btn6U])
{
	if(isStackHeld)
		singapore();
	else
		go2Stack();

}

if(!inStackLoop)
{
	powLift( (vexRT[Btn5U] - vexRT[Btn5D])*127 );
	powFourBar((vexRT[Btn8L] - vexRT[Btn8R])*127 );
}
drive( vexRT[Ch3], vexRT[Ch2] );
powMogo( (vexRT[Btn8U] - vexRT[Btn6D])*127 );


if(vexRT[Btn7D] == 1)
	powRoller(-60*speedChanger);
else if(vexRT[Btn7U] == 1)
	powRoller(60*speedChanger);
else if(vexRT[Btn7L] == 1)
	powRoller(0);
else if(vexRT[Btn7R] == 1 && !(buttonHasBeenHeld7R==vexRT[Btn7R])
{
	if(speedChanger == 1)
		speedChanger = 2;
	else
		speedChanger = 1;
}

if(vexRT[Btn8D] == 1 && !(buttonHasBeenHeld8D==vexRT[Btn8D])
{
	if(isStackHeld)
		normal_Orient();
	else
		holdStack_Orient();

}




}
