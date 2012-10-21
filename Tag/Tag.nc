includes TagMsg;
configuration Tag{
}
implementation{
    components Main,TagM,TimerC,LedsC;
    components RadioCRCPacket as Comm, CC1000RadioIntM as Radio;
    components UltrasoundControlM, CC1000ControlM;

    Main.StdControl->TagM;
    TagM.Leds->LedsC;
    TagM.Timer->TimerC.Timer[unique("Timer")];
    TagM.RadioControl->Comm;
    TagM.RadioSend->Comm;
    TagM.RadioReceive->Comm;
    TagM.GetRxBitOffset->Radio.GetRxBitOffset;
    TagM.RadioSendCoord->Radio.RadioSendCoordinator;
    TagM.UltrasoundControl->UltrasoundControlM;
}
