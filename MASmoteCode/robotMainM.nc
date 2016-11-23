
#include "masnet_Messages.h"

#define MOTEID 3  // 11 is base station
#define W2WLENGTH 0.088 // wheel to wheel length, unit meter
#define MAXSPEED 0.3
#define CONTROLINTERVEL 150 // unit ms  50
#define SPOKEWIDTH M_PI/32 //rad 
#define RADIUS 0.0335 // radius of the wheel



module robotMainM {

  provides {
    interface StdControl;
  }

  uses {
  
    interface Leds;

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

	int16_t leftEncode;
	int16_t rightEncode;
	uint16_t frontLeftIR;
	uint16_t frontRightIR;
	uint16_t backIR;
	int16_t frontPhoto;
	int16_t backPhoto;

////////////////////////    
    int16_t leftD;
    int16_t rightD;

	double x;
	double y;
	double angle;
	
/////////////////////////

	   
    double wkp;
	double wki;

	int16_t threshold;
   	double integErr;

	int16_t leftEState; 
	int16_t rightEState;

    int16_t leftPulse;
	int16_t rightPulse;


	int16_t action;
	int16_t angleCount;
	int16_t  sequence;

	int16_t deltaL,deltaR;

    double objx, objy,objangle;

	int16_t beginTurn;
	int16_t beginRun;
/////////////////////////////////
	double tempx,tempy;
	double tempangle;
       
	double deltaT;

	int16_t backB;

///////////////////////////

uint16_t calx,caly;

struct taskStr
{ 
   double x1;
   double y1;
   double angle1;
   int16_t sequence;

};

struct taskStr taskList[8];

struct taskStr *pFront, *pBack;
////////////////////////



//output message buffer

TOS_Msg sensor[2];
int16_t currentSensor;

TOS_Msg debug[2];
int16_t currentDebug;

TOS_Msg ack[2];
int16_t currentAck;


//input message buffer

TOS_Msg receiveBuffer[4];
TOS_MsgPtr ppmsg[4];

//debug

double debugV[6];


void sensorInit()

{

    leftEncode=0;
    rightEncode=0;
	frontLeftIR=0;
	frontRightIR=0;
	backIR=0;
	frontPhoto=0;
	backPhoto=0;

}


 void robotStatusInit()
{
	
    
    leftD=0;
    rightD=0;
	x=0;
	y=0;
	angle=0;

}



void controlInfoInit()
{
     
			   
     wkp=20;
	 wki=1;
	 threshold=80;
     integErr=0;
     leftEState=0;
	 rightEState=0;

     leftPulse=0;
	 rightPulse=0;

	 action=0;
	 angleCount=0;
	 sequence=0;

	 deltaL=0;
	 deltaR=0;

	 objx=0;
	 objy=0;
	 objangle=0;

	 beginTurn=0;
	 beginRun=0;

	 tempx=0;
	 tempy=0;
	 tempangle=0;

	 deltaT=0;

	 backB=0;


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
      ICR1=10000;// 1000- 2ms, 5000 - 10ms, 10000-20ms
}


void robotInit() {

	int16_t i;

   	motorInit();   
	sensorInit();      
	robotStatusInit();
	controlInfoInit();


	  pFront=taskList;
      pBack=taskList;

      for(i=0;i<8;i++)
      {
		taskList[i].x1=0;
        taskList[i].y1=0;
		taskList[i].angle1=0;

     }

      for(i=0;i<6;i++)
      {
			debugV[i]=0;
 

     }

	 calx=70;
	 caly=70;


	
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
	{	leftD=1;	}

	else if(cycle < 0)

	{	leftD=-1;}


    setLeftMotorDirection(leftD);
	OCR1A=ICR1/100*abs(cycle);
   
  }


void setRightMotorDutyCycle(int16_t cycle)
 {
	//saturation    
	if (cycle >100 ) { cycle=100;} 
	else if( cycle < -100 ) {cycle=-100;}
	
	if(cycle > 0)

	{ rightD=1; }

	else if(cycle < 0)

	{rightD=-1;}

    setRightMotorDirection(rightD);
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


double setUniangle( double xangle)
  {
	double temp;

	temp=fmod( xangle, M_PI*2);
    if ( temp < -1*M_PI ) temp=temp+M_PI*2;
	if ( temp > M_PI) temp=temp-M_PI*2;
	return (double) temp;
  
  }



void wpid(double err)

	{
	
     int16_t outputL, outputR;
     int16_t av;   

      integErr=integErr+err;

      av=(int16_t)wkp*err+wki*integErr;


	  if (av < -19 ) av=-19;
	  if (av > 19) av=19;


	  outputL= threshold-av;
	  outputR= threshold+av;

   	  setLeftMotorDutyCycle( outputL);
	  setRightMotorDutyCycle( outputR );

}



 task void sendSensor() {

 struct  MoteSensorMsg_st *pack;
 double temp1,temp2;

    atomic {
      pack = ( struct MoteSensorMsg_st *)sensor[currentSensor].data;
    }


     pack->leftEncode=(uint16_t) (leftEncode);
     pack->rightEncode=	(uint16_t) (rightEncode);
	 pack->frontIR1=(uint16_t) (frontLeftIR);
	 pack->frontIR2=(uint16_t) (frontRightIR);
	 pack->backIR=(uint16_t) (backIR);
	 pack->frontPhoto=(uint16_t) (frontPhoto);
	 pack->backPhoto=(uint16_t) (backPhoto);



/*	if(action==2){

        
		temp1=tempx+x*cos(tempangle);
		temp2=tempy+y*sin(tempangle);


 	 pack->x=(uint16_t) (1000* (temp1));
	 pack->y=(uint16_t) (1000* (temp2));
	 pack->angle=(float) (tempangle);
	



	}
	else
	{
	
 	 pack->x=(uint16_t) ( x);
	 pack->y=(uint16_t) ( y);
	 pack->angle=(float) (angle);
	
//	}  */  

 	 pack->x=calx;
	 pack->y=caly;

 pack->mote=MOTEID;








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
     pmess->mote=MOTEID;


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


     packAck->seqno=sequence;
	 packAck->mote=MOTEID;


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

     override=(int16_t)cmd->header.override;

   
    

	if(override==1)

	{

	pBack=&taskList[0];
	pFront=&taskList[0];
    action=0;
	
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
    pBack->y1=((double)cmd->new_y)/1000;
	pBack->angle1=(double)cmd->new_angle;
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
     return;
  


  }

  
	task void  cmdCalibration()
  
   {     
   
    double tx,ty,ta,temp;
    struct GpsCalibrationMsg_st * cmd = (struct GpsCalibrationMsg_st*) ppmsg[2]->data;
	/*int16_t msgnum = ppmsg[2]->length / sizeof(struct GpsCalibrationMsg_st) ;
	int16_t i=0;
	for(i=0; i < msgnum;i++){
		if(cmd[i].id == 3) break;
	}*/

/*	if(action==2){
		tx=((double)cmd->x)/1000-tempx;
		ty=((double)cmd->y)/1000-tempy;

	   temp=sqrt(tx*tx+ty*ty);

	   ta=(double)cmd->angle-tempangle;

	   x=temp*cos(ta);
	   y=temp*sin(ta);
	   angle=ta;

	}
	else
	{


*/   
	
 	call Leds.yellowToggle();

     if(  (cmd->x == 10)  &&  (cmd->y == 10)  ) 
	 call  Leds.redToggle();


      calx=cmd->x;
	  caly=cmd->y;

		
	x=((double)cmd->x)/((double)1000);
	y=((double)cmd->y)/((double)1000);
	angle=(double)cmd->angle;


}


   task void  cmdControl()
  
  {
   struct    ControlMsg_st * cmd = (struct ControlMsg_st*) ppmsg[3]->data;

	 atomic
	 {
	 wkp=(double)cmd->wkp;
	 wki=(double)cmd->wki;
//	 threshold=cmd->threshold;
	 
	 }

   return;

  }




  command result_t StdControl.init() {
   
    int16_t i;

	robotInit();
 	call ADCControl.init();   
    call CommControl.init();
    call Leds.init(); 

      {
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
 
 

	call CommControl.start();

	call SendTimer.start(TIMER_REPEAT, 1000);
	call ControlTimer.start(TIMER_REPEAT, CONTROLINTERVEL);
    call leftEncodeADC.getData();
    return SUCCESS;
 
   }

 
  command result_t StdControl.stop() 
  {
 	 call SendTimer.stop();
	 call ControlTimer.stop();
     call CommControl.stop();
     return SUCCESS;
  }



event result_t SendTimer.fired() {
  
   post sendSensor();
   return SUCCESS;

}

void case0()

{

double temp;
double xErr;
double yErr,dErr;


				action=1;
				xErr=pFront->x1-x;
				yErr=pFront->y1-y;

				deltaT=atan2(yErr,xErr)-angle;
				deltaT=setUniangle(deltaT);

			 

				temp=SPOKEWIDTH*RADIUS*2/W2WLENGTH;
				temp=deltaT/temp;

				angleCount=(int16_t) (temp+0.5);

				if(angleCount > 0 )
				{
				setLeftMotorVelocity( -0.3);
				setRightMotorVelocity( 0.3);
				}
				else if (angleCount < 0 )
				{
				setLeftMotorVelocity( 0.3);
				setRightMotorVelocity( -0.3);
			
				}
				
				else
				{

				action=2;

				objx=pFront->x1;
				objy=pFront->y1;

				tempx=x;
				tempy=y;

         
		    	xErr=pFront->x1-x;
		    	yErr=pFront->y1-y; 
		 		
				x=0;
				y=0;
				angle=0;
		    	
				dErr=sqrt( xErr*xErr+yErr*yErr);

				pFront->x1=dErr;
				pFront->y1=0;
			//	pFront->angle1=0;

				deltaL=0;
				deltaR=0;

				beginRun=1;

				 }

                 debugV[5]=0.1;

				if(angleCount!=0)
				{ 
 				  beginTurn=1;
				   debugV[5]=0.2;
				  call ControlTimer.stop();
				}


}


void caseNext()

{



double xErr;
double yErr;

double dErr;


		        setLeftMotorVelocity( 0);
				setRightMotorVelocity( 0);
				
                beginTurn=0;
                tempangle=angle+deltaT;
			    tempangle=setUniangle(tempangle);

				
                leftPulse=0;
				rightPulse=0;

			    if(action==1)

				{
				action=	2;

				objx=pFront->x1;
				objy=pFront->y1;
		//		objangle=pFront->angle1;

				tempx=x;
				tempy=y;


		    	xErr=pFront->x1-x;
		    	yErr=pFront->y1-y; 
		 		
				x=0;
				y=0;
				angle=0;
		    	
				dErr=sqrt( xErr*xErr+yErr*yErr);

				pFront->x1=dErr;
				pFront->y1=0;
			//	pFront->angle1=0;

				deltaL=0;
				deltaR=0;

				beginRun=1;
				 

				
				   debugV[5]=1.2;
				}

				if(action==4)
				{
				action=5;
				debugV[5]=1.3;
				}
			    call ControlTimer.start(TIMER_REPEAT, CONTROLINTERVEL);
}

void case2()
{

double temp;
double xErr;
double yErr;

double dErr,err;
int16_t tempL, tempR;



			    tempL=leftPulse-deltaL;
				tempR=rightPulse-deltaR;

				deltaL=leftPulse;
				deltaR=rightPulse;

  				temp= ((double) (tempR - tempL ))*SPOKEWIDTH*RADIUS/W2WLENGTH;
	
				angle=angle+temp;
				angle=setUniangle(angle);

				temp=((double) (tempR + tempL ))*SPOKEWIDTH*RADIUS*0.5;

				x=x+temp*cos( angle );
				y=y+temp*sin( angle );
  
		    	xErr=pFront->x1-x;
		    	yErr=0-y; 
		    //	temp= atan2( yErr,xErr);

		    	err=setUniangle(0-angle);
		    	dErr=sqrt( xErr*xErr+yErr*yErr);
		   

			
				   debugV[5]=2.1;

		   if ( dErr < 0.1 )
   
			{  
         
			 	  setLeftMotorDutyCycle( 0);
				  setRightMotorDutyCycle( 0 );
				  beginRun=0;
				  action=3;
			
			 }

		   else

		   {
				wpid( err);
				debugV[5]=2.2;
	
		   }
}

void case3()

{
		    	 
				 

				 x=objx;
				 y=objy;
				 angle=tempangle;


                 
	//			 pFront->angle1=objangle;
				 
				  if( (pBack==pFront+1) || ( (pBack== & taskList[0]) && (pFront== & taskList[7]) ) )

				  {	action=4;  }
				  
				   else
				   
				   {
				   	  debugV[5]=3.2;
					  sequence=pFront->sequence;

			    	  if( pFront== & taskList[7])
					  {pFront=taskList;}
					  else
					  {
						pFront++;
						
						}

						if(leftPulse==0)
   						{action=0;
   						post sendAck();
						debugV[5]=3.3;
						}
				   }

}


void case4()

{

double temp;


				deltaT=pFront->angle1-angle;
				deltaT=setUniangle(deltaT);
		
				temp=SPOKEWIDTH*RADIUS*2/W2WLENGTH;
				temp=deltaT/temp;

				angleCount=(int16_t) (temp+0.5);
				
				debugV[5]=4.1;
				
				if(angleCount > 0 )
				{
				setLeftMotorVelocity( -0.3);
				setRightMotorVelocity( 0.3);
				}
				else if (angleCount < 0 )
				{
				setLeftMotorVelocity( 0.3);
				setRightMotorVelocity( -0.3);
			
				}
				
				else
				{action=5;}


				if(angleCount!=0)
				{ 
			  
    			  if(leftPulse==0)
				  {
				  beginTurn=1;
				  call ControlTimer.stop();
				  						   debugV[5]=4.2;
				  }
				}

}

void case5()
{

				
				 angle=pFront->angle1;
                 						   debugV[5]=5.3;
				  sequence=pFront->sequence;
		    	  if( pFront== & taskList[7])
				  {pFront=taskList;
				  						   debugV[5]=5.5;}
				  else
					  {
						pFront++;
   						action=0;
   						post sendAck();
											   debugV[5]=5.8;
				      }

}    
 

event result_t ControlTimer.fired() {


/*
if(pFront!=pBack)

    {  
	   switch(action)
	   
	   { 
	   case 0:  {  case0();  break;	}
	   case 2:	{  case2();  break; }
 	   case 3:	{  case3();  break; }
	   case 4:	{  case4();  break; }
	   case 5:  {  case5();  break; }
	   }

				 debugV[0]=action;
				 debugV[1]=x;
				 debugV[2]=y;
    			 debugV[3]=angle;
				 debugV[4]=leftPulse;
	 

       


   }

   */
     return SUCCESS;
}



 async event result_t leftEncodeADC.dataReady(uint16_t data) 

{ 
  
    int16_t inputE;
	leftEncode=data;
    call rightEncodeADC.getData();


    if(data > 100 ) 
     inputE=1;
    else
     inputE=0; 

   if( inputE!=leftEState  )
   
   {  
        leftEState=inputE;
      
        if(beginTurn==1 || beginRun==1)

		{
		    if(leftD ==1 )
			leftPulse++;
            else if (leftD ==-1)
            leftPulse--;

				
        }
}
	 if(beginTurn==0 && beginRun==0)
		{ 

		  leftPulse=0;
	
		}
   




			   if( abs(leftPulse) == abs(angleCount)&& beginTurn==1 ) 
			    
				{
               
            	        caseNext();
		
                }

				 
 


	return SUCCESS;

  }

 async  event result_t rightEncodeADC.dataReady(uint16_t data) 

{ 
 
uint16_t inputE;

   rightEncode=data;  
   call frontLeftIRADC.getData();

   if(data > 100 ) 
   inputE=1 ;
    else
   {inputE=0;
    }

   if( inputE!=rightEState  )
   
   {  
        rightEState=inputE;
      
        if(beginTurn==1 || beginRun==1)

		{
		    if(rightD ==1 )
			rightPulse++;
            else if (rightD ==-1)
            rightPulse--;
        }

	
	}
		
		if(beginTurn==0 && beginRun==0)
		{ 

		  rightPulse=0;
		
   }

       __asm volatile ("nop"); 
	   __asm volatile ("nop");
	   __asm volatile ("nop");  

	return SUCCESS;

  }

async event result_t frontLeftIRADC.dataReady(uint16_t data) {


	frontLeftIR=data;
    call frontRightIRADC.getData(); 
    return SUCCESS;
  }

async event result_t frontRightIRADC.dataReady(uint16_t data) {


	frontRightIR=data;
    call backIRADC.getData(); 
    return SUCCESS;
  }

async event result_t backIRADC.dataReady(uint16_t data) {

	backIR=data;
    call frontPhotoADC.getData(); 
    return SUCCESS;
  }



 async event result_t frontPhotoADC.dataReady(uint16_t data) {
    frontPhoto=data;
    call backPhotoADC.getData();
    return SUCCESS;
  }

   async event result_t backPhotoADC.dataReady(uint16_t data) {

   backPhoto=data;

    call leftEncodeADC.getData();
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
  	call Leds.greenToggle();
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









