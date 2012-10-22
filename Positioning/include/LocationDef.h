//定位结果
#ifndef _LocationDef_H
#define _LocationDef_H

#define max_rx_num 20
#define max_tag_num 20

#define INVALIDVALUE 5000
#define PK_INTERVAL 200
//control the output of debug info 
//#define DEBUG

typedef struct PositionData{
    double x;
    double y;
    double z;
    float  belief;
    int TargetID;
}PositionData;

//参考点信息
typedef struct{
    double x;
    double y;
    double z;
    int RID;
    float  belief;//the lit,the more confidence
}RefNode;

typedef struct{
    char slice_id[15];
    double Dist[max_rx_num];
    int ReceiverId[max_rx_num];
    //int validity[max_rx_num];
    int SampleCount;
    int TargetID;
    long TimeStamp;
}PSample;

#endif
