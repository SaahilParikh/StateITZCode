void driverControl()
{

if(vexRT[Btn6U] == 1)
	go2Stack();

//if(!inStackLoop)
//	powLift( (vexRT[Btn5U] - vexRT[Btn5D])*127 );

drive( vexRT[Ch3], vexRT[Ch2] );
powMogo( (vexRT[Btn8U] - vexRT[Btn6D])*127 );










}
