

includes masnet_Messages;

configuration robotMain {
}
implementation {

  components Main,robotMainM,LedsC,TimerC,ADCC,GenericComm as Comm;
  
  
  Main.StdControl -> robotMainM.StdControl;
  Main.StdControl -> TimerC;	
  
  robotMainM.Leds -> LedsC;
 
//  robotMainM.SensorTimer -> TimerC.Timer[unique("Timer")];
  robotMainM.SendTimer -> TimerC.Timer[unique("Timer")];
  robotMainM.ControlTimer -> TimerC.Timer[unique("Timer")];
//robotMainM.ClockTimer -> TimerC.Timer[unique("Timer")];

  robotMainM.ADCControl -> ADCC;

  robotMainM.leftEncodeADC -> ADCC.ADC[3];
  robotMainM.rightEncodeADC -> ADCC.ADC[1];
  robotMainM.frontLeftIRADC -> ADCC.ADC[2];
  robotMainM.frontRightIRADC -> ADCC.ADC[4];
  robotMainM.backIRADC -> ADCC.ADC[7];

  robotMainM.frontPhotoADC -> ADCC.ADC[6];
  robotMainM.backPhotoADC -> ADCC.ADC[5];
  


  robotMainM.CommControl -> Comm;

  robotMainM.SensorMsg -> Comm.SendMsg[AM_MOTE_SENSOR_MSG];
  robotMainM.AckMsg -> Comm.SendMsg[AM_CMD_COMPLETE_MSG];
  robotMainM.DebugMsg -> Comm.SendMsg[AM_DEBUG_MSG];



  robotMainM.DestinationMsg -> Comm.ReceiveMsg[AM_DESTINATION_MSG];
  robotMainM.MoveMsg -> Comm.ReceiveMsg[AM_VW_CONTROL_MSG];
  robotMainM.CalibrationMsg -> Comm.ReceiveMsg[AM_GPS_CALIBRATION_MSG];
  robotMainM.ControlMsg -> Comm.ReceiveMsg[AM_CONTROL_MSG];
  




}
