//#include <LocationDef.h>
#include <stddef.h>
#include <newekf.h>
#include <matrix.h>  

static class NEWEKFfilter* solver[max_tag_num]={NULL,NULL,NULL,NULL,NULL};

//stop the ekf processing about the tag indicated by tagid
void stopEKF(int tagid){
    if(tagid>0&&tagid<max_tag_num&&solver[tagid]){
        delete solver[tagid];
        solver[tagid] = NULL;
    }
}

PositionData EKF(const RefNode refNodes[], PSample &data,PositionData LsqPos)
{
    class NEWEKFfilter *thisfilter;
    
    PositionData EKFPos;
    //if(data.SampleCount<1){
    if(data.SampleCount<3){//------------changed by wl
        EKFPos.belief=0;
        EKFPos.TargetID=-1;
    }else{
        if(solver[data.TargetID]==NULL){
#ifdef DEBUG
            cout<<"new ekf"<<endl;
#endif
            solver[data.TargetID]=new class NEWEKFfilter(true);//三态卡尔曼滤波
            solver[data.TargetID]->resetEKF(LsqPos);
            EKFPos=LsqPos;
            return EKFPos;

        }

#ifdef DEBUG
        cout<<"old ekf no "<<data.TargetID<<endl;
#endif
        thisfilter=solver[data.TargetID];
        //double fPHcov = 0;
        //PositionData ekfPHpos;
        EKFPos=thisfilter->ekfprocess(refNodes,data);
        if(EKFPos.TargetID==-1){
#ifdef DEBUG
            cout<<"reset EKF"<<endl;
#endif
            thisfilter->resetEKF(LsqPos);
            EKFPos=LsqPos;
        }
#ifdef DEBUG
        cout<<EKFPos.TargetID<<endl;
#endif
    }
    return EKFPos;
}
