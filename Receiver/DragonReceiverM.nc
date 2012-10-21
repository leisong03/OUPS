module DragonReceiverM{ 
    provides interface StdControl;
    uses {
	interface Leds;
//	interface ReadTemp;
	interface UltrasoundControl;
	interface StdControl as RadioControl;
	interface ReceiveMsg;
	interface BareSendMsg;
//	interface CanBus;
	interface RadioCoordinator as RadioReceiveCoord;       
	interface Serial;
//	interface AllSync;
//	interface ADC;
//	interface ADCControl;

	//interface Timer as SyncWDimer;// watch dog for Sync
	//interface SysTime;
	//interface Timer as TmpeTimer;
//	interface Timer as TestTimer;
	async command uint8_t GetRxBitOffset();
    }
}

implementation {
    uint8_t CanReported=1;
    uint16_t count=0;
    uint8_t USIEnable=0;
    uint8_t USgain = 255;
    uint8_t Vol = 128;
    uint8_t TYPEOffset = 0;
    uint8_t received = 0;
    uint16_t uscount = 0;
    uint16_t detector_is_on = 0;
    uint16_t rfstrength = 0;
    uint8_t RID;
    //TOS_Msg msgbuf;
    TOS_MsgPtr rMsg;
    CanMsg CMsg;

    //US detection
    uint8_t listing_count = 0;
    uint8_t got_id = 0;
    uint8_t reported = 1;
    uint16_t last_time = 0;
    uint8_t USI=0;
    uint8_t SyncED = 0;
    uint8_t TypeSyncED = 0;
    uint8_t myTagID;
    uint16_t myEmitIdx;
    uint16_t PacketIndex;


    // RF bit receive compensation 
    uint8_t bitOffset = 0;
    uint16_t us_time = 0;

    float Realtmpe = 0.0;

    // Sync time
    int RFSyncTime = 0;
    int SyncLineTime = 0;

    task void ReportRFReceived()
    {
	char OutputMsg[100];
	atomic{
	    sprintf(OutputMsg, "RSSI  %d, TagID %d, EmtIDx %d \r\n", rfstrength, CMsg.TagID, CMsg.EmitIdx);
	}
	UARTOutput(DBG_ERROR,"%s", &OutputMsg);
    }

    command result_t StdControl.init()
    { 
      uint8_t val;
      char OutputMsg[100];


	call Leds.init();
	call Leds.redOn();
	call Leds.greenOn();
	call Leds.yellowOn();
	// Temperature init
	//call ReadTemp.init();
	// Ultrasound intensity
//	call ADCControl.init();
//	call ADCControl.setSamplingRate(0);

	// RS232 connection
	TOSH_CLR_US_IN_EN_PIN();
	TOSH_SET_BAT_MON_PIN();

	TOSH_uwait(150);

	// Init serial port
	call Serial.SetStdoutSerial();
	call RadioControl.init();
	//init US detection
	call UltrasoundControl.SetGain(USgain);
	TYPEOffset = offsetof(struct TOS_Msg,type) + sizeof(((struct TOS_Msg*)0)->type);
	//RID=(uint8_t)(TOS_LOCAL_ADDRESS&0xFF);
//	sprintf(OutputMsg,"Level=%u\n",val);
//	UARTOutput(DBG_ERROR,"%s", &OutputMsg);
	return SUCCESS;
    }

    command result_t StdControl.start()
    {
	call Leds.redOff();
	call Leds.yellowOff();
	call Leds.greenOff();
	call RadioControl.start();
//	call ADC.getData();
//	call TmpeTimer.start(TIMER_REPEAT, 10000);
//	call TestTimer.start(TIMER_REPEAT, 1000);
//	call SyncWDimer.start(TIMER_ONE_SHOT, SYNC_WD_TIMER);
	return SUCCESS;
    }

    command result_t StdControl.stop()
    {
	call RadioControl.stop();
	return SUCCESS;
    }

    void ReceiverReset()
    {
	atomic {
	    count=0;
	    reported = 1;
	    received=0;
	    bitOffset = 0;
	}
    }

    task void ReportPulse()
    {
	uint8_t  comp;
	int     ajusted_timer = 0;
	uint16_t timer;
	float distance = 0.0;
	char OutputMsg[255];
	float speed_of_sound;
	int32_t  td,biga,bigb;
	

	atomic {
	    //biga=(int32_t)SyncLineTime;
	    //bigb=(int32_t)RFSyncTime;
	    //td=biga-bigb;
	    comp = bitOffset;
	    timer = last_time;
	}
	//PacketIndex=0;
	sprintf(OutputMsg, "Rep:ID=%u EIdx=%u timer=%u comp=%u\n\r",myTagID,myEmitIdx,timer ,comp);
	UARTOutput(DBG_ERROR,"%s", &OutputMsg);
	ReceiverReset();
    }

    async event result_t UltrasoundControl.PulseDetected(uint16_t timer)
    {
	atomic{
	    received=1;
	    
	    if (us_time == 0) {
		us_time = timer;
	    }	
	}
	call Leds.redToggle();
	return SUCCESS;
    }

    void try_post_report() {
	//	if (got_id && !detector_is_on && reported) {
	if (!detector_is_on && reported) {
	    last_time = us_time;
	    post ReportPulse();
	    reported = 0;
	}
    }

    async event void RadioReceiveCoord.startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff) {};
    async event void RadioReceiveCoord.blockTimer() {};

    async event void RadioReceiveCoord.byte(TOS_MsgPtr msg, uint8_t cnt){
	if (cnt == TYPEOffset && msg->type == TARGETMSG) {
	    bitOffset = call GetRxBitOffset();
	   // RFSyncTime = call SysTime.getTime16();
	    call Leds.greenToggle();
	    call UltrasoundControl.StartDetector(60000);
	    detector_is_on=1;
	    us_time=0;
	}
    }
  

    async event result_t UltrasoundControl.DetectorTimeout()
    {
	atomic {
	    detector_is_on = 0;
	}
	try_post_report();
	return SUCCESS;
    }

    event TOS_MsgPtr ReceiveMsg.receive(TOS_MsgPtr Pmsg)
    {
	struct TargetMsgType *ptr = (struct TargetMsgType *)Pmsg->data;
	//call Leds.greenToggle();

	if(Pmsg->type!=TARGETMSG||Pmsg->crc!=1){
	    return Pmsg;
	}
	//atomic{
	//rfstrength = Pmsg -> strength;
	myTagID = ptr -> TargetId;
	myEmitIdx = ptr -> EmitIdx;
	//}
	got_id = 1;
	try_post_report();
	return Pmsg;
    }

     event result_t BareSendMsg.sendDone(TOS_MsgPtr rmsg, result_t success) {
	return SUCCESS;
    }

    event result_t Serial.Receive(char * buf, uint8_t len)
    {
	return SUCCESS;
    }
}
