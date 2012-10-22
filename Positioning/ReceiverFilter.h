#ifndef _ReceiverFilter_h
#define _ReceiverFilter_h

//function:filter the receivers whose distance is not valid
void OutlierRejection(float LowT, float HighT,const RefNode refNodes[],PSample &data);


#endif

