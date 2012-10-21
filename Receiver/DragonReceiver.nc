
includes Receiver;

configuration DragonReceiver{
}

implementation {
   components Main, DragonReceiverM, LedsC, UltrasoundControlM, SerialM, HPLUARTC;
   components RadioCRCPacket as Comm, CC1000RadioIntM as Radio, SysTimeC,TimerC;//CC1000RadioC as Radio;// 
   
   //components CanBusM, AllSyncM, ReadTempM;//ADCC;

   Main.StdControl -> DragonReceiverM;
   //Main.StdControl -> Comm;
   Main.StdControl -> TimerC;
   
   DragonReceiverM.RadioControl ->Comm;
   DragonReceiverM.GetRxBitOffset -> Radio.GetRxBitOffset;   
   DragonReceiverM.ReceiveMsg -> Comm;
   DragonReceiverM.BareSendMsg -> Comm;
   DragonReceiverM.RadioReceiveCoord -> Radio.RadioReceiveCoordinator;
   DragonReceiverM.Leds -> LedsC;
   DragonReceiverM.UltrasoundControl -> UltrasoundControlM;
   //DragonReceiverM.SysTime -> SysTimeC;
   DragonReceiverM.Serial -> SerialM;
   //DragonReceiverM.TmpeTimer -> TimerC.Timer[unique("Timer")];
   //DragonReceiverM.TestTimer -> TimerC.Timer[unique("Timer")];
   //DragonReceiverM.CanBus -> CanBusM;
   //DragonReceiverM.AllSync -> AllSyncM;
   //DragonReceiverM.ReadTemp -> ReadTempM;
 //  DragonReceiverM.ADC->ADCC.ADC[5];
 //  DragonReceiverM.ADCControl->ADCC;

   //DragonReceiverM.SyncWDimer -> TimerC.Timer[unique("Timer")];
   //AllSyncM.Timer->TimerC.Timer[unique("Timer")];
   //ReadTempM.Timer -> TimerC.Timer[unique("Timer")];
   SerialM.Leds -> LedsC;
   SerialM.HPLUART -> HPLUARTC;
}

