includes TagMsg;
module TagM{
    provides interface StdControl;
    uses{
	interface Leds;
	interface StdControl as RadioControl;
	interface BareSendMsg as RadioSend;
	interface ReceiveMsg as RadioReceive;
	interface RadioCoordinator as RadioSendCoord;
	interface UltrasoundControl;
	interface Timer;
	async command uint8_t GetRxBitOffset();
    }
}
implementation{
    TOS_Msg TagMsg;
    uint16_t EmitIndex;
    uint8_t fixoffset;
    async event void RadioSendCoord.startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff) {}
    async event void RadioSendCoord.blockTimer() {}
    async event void RadioSendCoord.byte(TOS_MsgPtr msg, uint8_t cnt)  {
	if(fixoffset+2==cnt){
	    call UltrasoundControl.SendPulse();
	    call Leds.redOn();
	    call Timer.start(TIMER_ONE_SHOT,500);
	}
    }
    event TOS_MsgPtr RadioReceive.receive(TOS_MsgPtr data) {
	return data;
    }
    command result_t StdControl.init() {
	call Leds.init();
	call RadioControl.init();
	call Leds.redOn();
	call Leds.greenOn();
	call Leds.yellowOn();
	EmitIndex=0;
	return SUCCESS;
    }
    command result_t StdControl.start() {
	call Timer.start(TIMER_ONE_SHOT,90);
	call RadioControl.start();
	call Leds.redOff();
	call Leds.greenOff();
	call Leds.yellowOff();
	fixoffset=offsetof(struct TOS_Msg,data);
	return SUCCESS;
    }
    command result_t StdControl.stop() {
	call RadioControl.stop();
	call Timer.stop();
	return SUCCESS;
    }
    event result_t RadioSend.sendDone(TOS_MsgPtr rmsg, result_t success){
	return success;
    }
    async event result_t UltrasoundControl.PulseDetected(uint16_t timer){
	return SUCCESS;
    }
    async event result_t UltrasoundControl.DetectorTimeout(){
	return SUCCESS;
    }
    event result_t Timer.fired(){
	TagMsgData* DataPtr;
	call Leds.yellowToggle();
	call Leds.redOff();
	TagMsg.length=sizeof(TagMsgData);
	TagMsg.type=TAGMSG;
	TagMsg.addr = TOS_BCAST_ADDR;
	DataPtr=(TagMsgData*)TagMsg.data;
	DataPtr->TagId=TOS_LOCAL_ADDRESS;
	DataPtr->TagIndex=EmitIndex;
	call RadioSend.send(&TagMsg);
	EmitIndex=(EmitIndex+1)%(0XFFFF);
	return SUCCESS;
    }
}
