
#include "masnet_Messages.h";

#define MOTEID 3  // 11 is base station
#define W2WLENGTH 0.087 // wheel to wheel length, unit meter
#define MAXSPEED 0.3
#define SAMPLEINTERVEL 4 // unit ms  4
#define CONTROLINTERVEL 10 // unit ms 10
#define SPOKEWIDTH 0.19635 //rad 
#define RADIUS 0.034 // radius of the wheel


module robotMainM {

  provides {
    interface StdControl;
  }

  uses {
  
    interface Leds;

    interface Timer as SensorTimer;
	interface Timer as SendTimer;
	interface Timer as ControlTimer;

    interface ADC as leftEncodeADC;
    interface ADC as rightEncodeADC;

    interface ADC as frontLeftIRADC;
    interface ADC as frontRightIRADC;
	interface ADC as backIRADC;

    interface ADC as frontPhotoADC;
    interface ADC as backPhotoADC;
    interface ADCControl;


    interface StdControl as CommControl;

    interface SendMsg as SensorMsg;
	interface SendMsg as DebugMsg;
	interface SendMsg as AckMsg;

	interface ReceiveMsg as DestinationMsg;
	interface ReceiveMsg as MoveMsg;
	interface ReceiveMsg as CalibrationMsg;
	interface ReceiveMsg as ControlMsg;


  }
}

implementation { 


struct sensorStr

{

	uint16_t leftEncode;
	uint16_t rightEncode;

	uint16_t frontLeftIR;
	uint16_t frontRightIR;
	uint16_t backIR;

	uint16_t frontPhoto;
	uint16_t backPhoto;


} sensorData;

struct sensorStr *pSensor;

struct stateStr
{
    
    int16_t leftD;
    int16_t rightD;

	double x;
	double y;
	double theta;
	

	//add timeStamp, powerLevel, other mote info here

  }robotState;


struct stateStr * pState;

//controller

struct controlStr
{
	   
    double wkp;
	double wki;

	uint16_t threshold;
  
  	double integErr;

	int16_t leftEState; 
	int16_t rightEState;

    int16_t leftPulse;
	int16_t rightPulse;

	int16_t leftIntegP;
	int16_t rightIntegP;

	int16_t action;

	int16_t angleCount;

	uint16_t  sequence;
       
}controller;


struct controlStr *pControl;


struct taskStr
{ 
   double x1;
   double y1;
   double theta1;
   uint16_t sequence;

};


struct taskStr taskList[8];

struct taskStr *pFront, *pBack;




//output message buffer

TOS_Msg sensor[2];
uint8_t currentSensor;

TOS_Msg debug[2];
uint8_t currentDebug;

TOS_Msg ack[2];
uint8_t currentAck;




//input message buffer

TOS_Msg receiveBuffer[4];
TOS_MsgPtr ppmsg[4];

//debug

double debugV[6];



void sensorInit()

{

	pSensor=&sensorData;

    pSensor->leftEncode=0;
    pSensor->rightEncode=0;

	pSensor->frontLeftIR=0;
	pSensor->frontRightIR=0;
	pSensor->backIR=0;

	pSensor->frontPhoto=0;
	pSensor->backPhoto=0;

//	kk1=0;
//	kk2=0;



}


 void robotStatusInit()
{
	
    pState=&robotState;

    pState->leftD=0;
    pState->rightD=0;
	
	pState->x=0;
	pState->y=0;
	pState->theta=0;



}



void controlInfoInit()
{
     pControl=&controller;
			   
     pControl->wkp=20;
	 pControl->wki=1;

	 pControl->threshold=80;
	

     pControl->integErr=0;

  
     pControl->leftEState=0;
	 pControl->rightEState=0;

     pControl->leftPulse=0;
	 pControl->rightPulse=0;

     pControl->leftIntegP=0;
	 pControl->rightIntegP=0;

	 pControl->action=0;

	 pControl->angleCount=0;

	 pControl->sequence=0;


}





void motorInit(){
    sbi(DDRC,0); //enable output dirver
    sbi(DDRC,1); //enable output dirver,0,1 for left driver
    sbi(DDRC,2); //enable output dirver
    sbi(DDRC,3); //enable output dirver,2,3, for right driver
    sbi(DDRB,6); //enable output dirver
    sbi(DDRB,5); //enable output dirver
    OCR1A=0;//clear output compare register
    OCR1B=0;//clear output compare register
    sbi(TCCR1A, COM1A1); // COM1A1 = 1
    cbi(TCCR1A, COM1A0); // COM1A0 = 0
    sbi(TCCR1A, COM1B1); // COM1B1 = 1
    cbi(TCCR1A, COM1B0); // COM1B0 = 0
    cbi(TCCR1A, WGM10);  // Phase correct, SET ICR1
    sbi(TCCR1A, WGM11);
    cbi(TCCR1B, WGM12);
    sbi(TCCR1B, WGM13);
    cbi(TCCR1B, CS12);   // Set prescaler to CK/8
    sbi(TCCR1B, CS11);
    cbi(TCCR1B, CS10);
    atomic  ICR1=10000;// 1000- 2ms, 5000 - 10ms, 10000-20ms
}


void robotInit() {

	int16_t i;

   	motorInit();   
	sensorInit();      
	robotStatusInit();
	controlInfoInit();


	pBack=&taskList[0];
	pFront=&taskList[0];

      for(i=0;i<8;i++)
      {
		taskList[i].x1=0;
        taskList[i].y1=0;
		taskList[i].theta1=0;

     }

      for(i=0;i<6;i++)
      {
			debugV[i]=0;
 

     }

}



void setRightMotorDirection(int16_t direction)
 {
     switch(direction)
    {
        //1 forward, -1 backward, 0 stop
	    case -1: sbi(PORTC,PORTC2);cbi(PORTC,PORTC3);break;
        case 1: cbi(PORTC,PORTC2);sbi(PORTC,PORTC3);break;
        case 0: cbi(PORTC,PORTC2);cbi(PORTC,PORTC3);

    }
 
  }


 void setLeftMotorDirection(int16_t direction) 
 {

   switch(direction)
    {
        //1 forward, -1 backward, 0 stop
	    case -1: sbi(PORTC,PORTC0);cbi(PORTC,PORTC1);break;
        case 1: cbi(PORTC,PORTC0);sbi(PORTC,PORTC1);break;
        case 0: cbi(PORTC,PORTC0);cbi(PORTC,PORTC1);

    }
  
  }

  void setLeftMotorDutyCycle(int16_t cycle) 
  {
 
	//saturation
	if (cycle > 100 ) { cycle=100;} 
	else if( cycle < -100 ) {cycle=-100;}
	

	
	if(cycle > 0)
	{

	pState->leftD=1;
	
	
	}

	else if(cycle < 0)

	{	pState->leftD=-1;}


    setLeftMotorDirection(pState->leftD);
	OCR1A=ICR1/100*abs(cycle);
   
  }


void setRightMotorDutyCycle(int16_t cycle)
 {
	//saturation    
	if (cycle >100 ) { cycle=100;} 
	else if( cycle < -100 ) {cycle=-100;}
	

	
	if(cycle > 0)
	{

	pState->rightD=1;
	
	
	}

	else if(cycle < 0)

	{	pState->rightD=-1;}

    setRightMotorDirection(pState->rightD);
    OCR1B=ICR1/100*abs(cycle);
  
  }




  void setLeftMotorVelocity( double velocity)

  {	
	int16_t cycle;
	cycle= (int16_t)  (velocity*100)/MAXSPEED;
	setLeftMotorDutyCycle( cycle );

  }


  void setRightMotorVelocity( double velocity)

  {	
 	int16_t cycle;
	cycle= (int16_t)  (velocity*100)/MAXSPEED;
	setRightMotorDutyCycle( cycle );
  }

//theat confined to -pi to pi
double setUniTheta( double theta)
  {
	double temptheta;

	temptheta=fmod( theta, 6.2832);
    if ( temptheta < -3.1416 ) temptheta=temptheta+6.2832;
	if ( temptheta > 3.1416) temptheta=temptheta-6.2832;
	return (double) temptheta;
  
  }



void wpid(double err, struct controlStr *p)

	{
	
     int16_t outputL, outputR;
     double av;   





      p->integErr=p->integErr+err;

      p->integErr=setUniTheta(p->integErr);

	  p->integErr=setUniTheta(p->integErr);

      av=p->wkp*err+p->wki*p->integErr;


	  if (av < -20 ) av=-20;
	  if (av > 20)   av=20;


	  outputL=(int16_t) (p->threshold-av);
	  outputR=(int16_t) (p->threshold+av);

   	  setLeftMotorDutyCycle( outputL);
	  setRightMotorDutyCycle( outputR );

}



 task void sendSensor() {


   struct  MoteSensorMsg_st *pack;

    atomic {
      pack = ( struct MoteSensorMsg_st *)sensor[currentSensor].data;
    }


     pack->leftEncode=(uint16_t) (pSensor->leftEncode);
     pack->rightEncode=	(uint16_t) (pSensor->rightEncode);
//	 pack->frontIR1=(uint16_t) (pSensor->frontLeftIR);
//	 pack->frontIR2=(uint16_t) (pSensor->frontRightIR);
//	 pack->backIR=(uint16_t) (pSensor->backIR);
	 pack->frontPhoto=(uint16_t) (pSensor->frontPhoto);
	 pack->backPhoto=(uint16_t) (pSensor->backPhoto);
	 pack->x=(uint16_t) (1000* (pState->x));
	 pack->y=(uint16_t) (1000*(pState->y));
	 pack->angle=(float) (pState->theta);



    if (call SensorMsg.send(11, sizeof(struct MoteSensorMsg_st),
                  &sensor[currentSensor]))
      {
    atomic {
      currentSensor ^= 0x1;
    }

      }

	  return;
}



task void sendDebug()

{

	 struct DebugMsg_st *pmess;

    atomic {
      pmess = (struct DebugMsg_st* ) debug[currentDebug].data;
    }


     pmess->debugv[0]=(float)debugV[0];
     pmess->debugv[1]=(float)debugV[1];
	 pmess->debugv[2]=(float)debugV[2];
	 pmess->debugv[3]=(float)debugV[3];
	 pmess->debugv[4]=(float)debugV[4];
	 pmess->debugv[5]=(float)debugV[5];



    if ( call DebugMsg.send(11, sizeof( struct DebugMsg_st),
                  &debug[currentDebug]) )
      {
    atomic {
      currentDebug ^= 0x1;
    }

      }

	  return;

  }



task void sendAck()


{
	struct  CmdCompleteMsg_st *packAck;

    atomic {
      packAck = (struct CmdCompleteMsg_st *)ack[currentAck].data;
	       }


     packAck->seqno=pControl->sequence;


    if (call AckMsg.send(11, sizeof(struct CmdCompleteMsg_st),
                  &ack[currentAck]))
      {
    atomic {
      currentAck ^= 0x1;
    }

      }

	  return;
}



  task void  cmdDestination()
  
  {
     
     uint16_t override;
     struct  taskStr *p;
	  struct DestinationMsg_st * cmd = (struct  DestinationMsg_st*) ppmsg[2]->data;

    atomic
	{
	override=(uint8_t)cmd->header.override;
	}

	if(override==1)

	{

	pBack=&taskList[0];
	pFront=&taskList[0];
    pControl->action=0;
	
	}



    if(pBack==&taskList[7])
    {
	p=taskList;
 
    }

    else 
    { 
	
	p=pBack+1;
  
    } 
    

      if(p==pFront)
    {

	return ;
    }    

    
    else {

    pBack->x1=((double)cmd->new_x)/1000;
    pBack->y1=((double)cmd->new_y) /1000;
	pBack->theta1=(double)cmd->new_angle;
	pBack->sequence=cmd->header.seqno;
    pBack=p;

    }

     return ;
  }




	task void  cmdMove()
  
  {
     double v;
	 double velocityL;
	 double velocityR;
	 double thetaV;

	 struct VWControlMsg_st * cmd = ( struct VWControlMsg_st*) ppmsg[1]->data;

	 v=(double)cmd->v;
	 thetaV=(double)cmd->w;

	 velocityL=v-W2WLENGTH*thetaV/2;
	 velocityR=v+W2WLENGTH*thetaV/2;


	 setLeftMotorVelocity( velocityL);
	 setRightMotorVelocity( velocityR);

  }

  
	task void  cmdCalibration()
  
  {
     struct GpsCalibrationMsg_st * cmd = (struct GpsCalibrationMsg_st*) ppmsg[2]->data;
		

	atomic
	{
	pState->x=((double)cmd->x)/1000;
	pState->y=((double)cmd->y)/1000;
	pState->theta=(double)cmd->angle;
	}
     


  }


   task void  cmdControl()
  
  {
   struct    ControlMsg_st * cmd = (struct ControlMsg_st*) ppmsg[3]->data;

	 atomic
	 {
	 pControl->wkp=(double)cmd->wkp;
	 pControl->wki=(double)cmd->wki;
//	 pControl->threshold=cmd->threshold;
	 
	 }

  }




  command result_t StdControl.init() {
   
    int i;

	robotInit();
 	call ADCControl.init();   
    call CommControl.init();
    call Leds.init(); 

      atomic{
      currentSensor=0;
      currentDebug=0;
	  currentAck=0;
     }

	  for(i=0;i<=3;i++)
	  {     ppmsg[i]=&receiveBuffer[i];

	  }
  

    return SUCCESS;

  }


  command result_t StdControl.start() {
 
 
//	call Leds.redOn();
	call CommControl.start();

	call SensorTimer.start(TIMER_REPEAT, SAMPLEINTERVEL);
	call SendTimer.start(TIMER_REPEAT, 1000);
	call ControlTimer.start(TIMER_REPEAT, CONTROLINTERVEL);

    return SUCCESS;
 
   }

 
  command result_t StdControl.stop() 
  {
     call SensorTimer.stop();
	 call SendTimer.stop();
	 call ControlTimer.stop();
     call CommControl.stop();
     return SUCCESS;
  }


event result_t SensorTimer.fired() {
 

   call leftEncodeADC.getData();

   return SUCCESS;

}

event result_t SendTimer.fired() {

  
  post sendSensor();
//   
  
   return SUCCESS;

}

event result_t ControlTimer.fired() {

//control loop
double temp;
double xErr;
double yErr;

double dErr,err,aErr;
double deltaA;


	
int16_t i;

    i=0;


  if(pFront!=pBack)

    {  
	   switch(pControl->action)
	   
	   { 
	   
	   //0 initial state
	   case 0: 

			{
				pControl->action=1;

				pControl->leftPulse=0;
				pControl->rightPulse=0;

				pControl->leftIntegP=0;

				xErr=pFront->x1-pState->x;



			
				yErr=pFront->y1-pState->y;

				deltaA=atan2(yErr,xErr)-pState->theta;
			
				deltaA=setUniTheta(deltaA);

			 

				temp=SPOKEWIDTH*RADIUS*2/W2WLENGTH;

				temp=deltaA/temp;

				pControl->angleCount=(int16_t) temp;

				debugV[1]=(int16_t)pControl->angleCount;


				if(pControl->angleCount > 0 )
				{

				setLeftMotorVelocity( -0.3);
				setRightMotorVelocity( 0.3);
				

		
				}

				else if (pControl->angleCount < 0 )
				{
				setLeftMotorVelocity( 0.3);
				setRightMotorVelocity( -0.3);
				
				}

				else

				{pControl->action=2;}
 

			break;			
			}


		//1 turn angle to the route
		case 1:
			{
				pControl->leftIntegP=pControl->leftPulse;
				
				temp=(double) (abs(pControl->leftIntegP)-abs(pControl->angleCount));

            
			    debugV[2]++;

				debugV[3]=pControl->leftIntegP;
		   
				if( temp > 0.1 )  
				{

				 pControl->action=2;
				 pState->theta=pState->theta+pControl->angleCount*(SPOKEWIDTH*RADIUS*2/W2WLENGTH);
				
			     pState->theta=setUniTheta(pState->theta);	
					
				 pControl->leftIntegP=0;

				 pControl->leftPulse=0;
			     pControl->rightPulse=0;


				 }
				
				break;

			}

		//2 try to go a line on the route
        case 2:	
		
			{

			
				{
				temp=(pControl->rightPulse - pControl->leftPulse )*SPOKEWIDTH*RADIUS/W2WLENGTH;
				pState->theta=pState->theta+temp;
	
				pState->theta=setUniTheta(pState->theta);

				temp=(pControl->leftPulse+pControl->rightPulse)*SPOKEWIDTH*RADIUS*0.5;

				pState->x=pState->x+temp*cos( pState->theta );
				pState->y=pState->y+temp*sin( pState->theta );

				pControl->leftPulse=0;
				pControl->rightPulse=0;
			
				}


		
	
			xErr=pFront->x1-pState->x;
			yErr=pFront->y1-pState->y; 
			temp= atan2( yErr,xErr);
			err=setUniTheta(temp-pState->theta);
			dErr=sqrt( xErr*xErr+yErr*yErr);
		   

		   if ( dErr < 0.04 )
   
			{  
         
			 	  setLeftMotorDutyCycle( 0);
				  setRightMotorDutyCycle( 0 );

				    	

				  pControl->action=3;
			 }

		   else

		   {
				wpid( err,  pControl);
			

	   	   
		   }





		   break;

	    }
	  //judge turn to angle or turn to line
	  case 3:

		{

		    	  if( (pBack==pFront+1) || ( (pBack== & taskList[0]) && (pFront== & taskList[7]) ) )

				  {

						pControl->action=4;

				  }
				  
				   else
				   
				   {
				   
					  pControl->sequence=pFront->sequence;
					  
					  if( pFront== & taskList[7])
					  {pFront=taskList;}
					  else
					  {pFront++;}
   
					  pControl->action=0;


					  post sendAck();
				   
				   }

				   break;

		  }
	  //turn to angle;
	   case 4:

			{
				pControl->action=5;
				deltaA=pFront->theta1-pState->theta;
				deltaA=setUniTheta(deltaA);
		
				temp=SPOKEWIDTH*RADIUS*2/W2WLENGTH;
				temp=deltaA/temp;

				pControl->angleCount=(int16_t) temp;

				pControl->leftIntegP=0;
			    pControl->leftPulse=0;
			    pControl->rightPulse=0;


				if(pControl->angleCount> 0 )
			{

				setLeftMotorVelocity( -0.3);
				setRightMotorVelocity( 0.3);
		
			}

				else if (pControl->angleCount < 0 )
			{
				setLeftMotorVelocity( 0.3);
				setRightMotorVelocity( -0.3);
			}

				else

				{pControl->action=0;}
 

			break;		
	  
	  
	  
			}

     case 5:

			{
				pControl->leftIntegP=pControl->leftPulse;
				
				temp=(double)(abs(pControl->leftIntegP)-abs(pControl->angleCount));

				if( temp > 0.1 )  

				{pControl->action=0;

				 pState->theta=pFront->theta1;

				 pControl->leftIntegP=0;
				 pControl->leftPulse=0;
			     pControl->rightPulse=0;

				 pControl->sequence=pFront->sequence;
				 post sendAck();

				setLeftMotorVelocity( 0);
				setRightMotorVelocity( 0);
				
				
		    	  if( pFront== & taskList[7])
			     {pFront=taskList;}
			     else
			     {pFront++;}
                }
			  break;
		    }
	  
	     }
	 }
    debugV[0]=pControl->action;

   return SUCCESS;
}



 async event result_t leftEncodeADC.dataReady(uint16_t data) 

{ 
    uint8_t inputE;
	
	
	pSensor->leftEncode=data;
    call rightEncodeADC.getData();
  
      
   if(data > 100 ) 
   inputE=1 ;
   
   else
   {inputE=0;
    }

   if( inputE!=pControl->leftEState  )
   
   {  
     pControl->leftEState=inputE;

      
     atomic 
	{
		    if(pState->leftD ==1 )
			pControl->leftPulse++;
            else if (pState->leftD ==-1)
            pControl->leftPulse--;

	}


	
 }

//kk1++;

	return SUCCESS;

  }

 async  event result_t rightEncodeADC.dataReady(uint16_t data) 

{ 
   uint8_t inputE;

   pSensor->rightEncode=data;  
   call frontLeftIRADC.getData();

   if(data > 100 ) 
   inputE=1 ;
    else
   {inputE=0;
    }

   if( inputE!=pControl->rightEState  )
   
   {   
     atomic 
	{
		pControl->rightEState=inputE;
		
            if(pState->rightD ==1 )
			pControl->rightPulse++;
            else if (pState->rightD ==-1)
            pControl->rightPulse--;

	}

}
	return SUCCESS;
  }

async event result_t frontLeftIRADC.dataReady(uint16_t data) {


	pSensor->frontLeftIR=data;
    call frontRightIRADC.getData(); 
    return SUCCESS;
  }

async event result_t frontRightIRADC.dataReady(uint16_t data) {


	pSensor->frontRightIR=data;
    call backIRADC.getData(); 
    return SUCCESS;
  }

async event result_t backIRADC.dataReady(uint16_t data) {

	pSensor->backIR=data;
    call frontPhotoADC.getData(); 
    return SUCCESS;
  }



 async event result_t frontPhotoADC.dataReady(uint16_t data) {
    pSensor->frontPhoto=data;
    call backPhotoADC.getData();
    return SUCCESS;
  }

   async event result_t backPhotoADC.dataReady(uint16_t data) {

   pSensor->backPhoto=data;
   return SUCCESS;
  }

   
    event result_t SensorMsg.sendDone(TOS_MsgPtr sent, result_t success) {
    return SUCCESS;
  }


   
    event result_t AckMsg.sendDone(TOS_MsgPtr sent, result_t success) {
    return SUCCESS;
  }


   
    event result_t DebugMsg.sendDone(TOS_MsgPtr sent, result_t success) {
    return SUCCESS;
  }


   event TOS_MsgPtr DestinationMsg.receive(TOS_MsgPtr pmsg) {

    ppmsg[2] = pmsg;
    call Leds.redToggle();
    post cmdDestination();
    return pmsg;

  }

   event TOS_MsgPtr MoveMsg.receive(TOS_MsgPtr pmsg) {


    ppmsg[1] = pmsg;
  
    post cmdMove();
    return pmsg;

  }

    event TOS_MsgPtr CalibrationMsg.receive(TOS_MsgPtr pmsg) {

    ppmsg[0] = pmsg;
    post cmdCalibration(); 
	
    return pmsg;
 
  }
 
 
    event TOS_MsgPtr ControlMsg.receive(TOS_MsgPtr pmsg) {
 
    ppmsg[3] = pmsg;
	call Leds.greenToggle();

    post cmdControl();
    return pmsg;
 
  }


}