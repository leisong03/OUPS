
#include <signal.h>
#include <matrix.h>  
#include <Mymath.h>  
#include <LocationDef.h>  
#include <ReceiverFilter.h>
#include <sys/time.h>
#include "NodePosition.h"
//#include <stdio.h>
//#include <string.h>

//PositionData PositionEstimateTriangulation(double x[],double y[],double z[],double dist[],double lastz,int size,int TargetID);
PositionData PositionEstimateTriangulation(const RefNode refNodes[], PSample &data,double lastz);
PositionData EKF(const RefNode refNodes[], PSample &data,PositionData LsqPos);

//a simple positioning algrithem
//PositionData simple_gps(const RefNode refNodes[], PSample &data);

//int calc_offset(const RefNode refNodes[],PSample &saved_data,PositionData pos_result);
extern int calc_offset(const RefNode refNodes[],PSample &saved_data,PositionData pos_result);
//get data from file stream
bool getData(PSample &data,FILE *fp);
bool check_x_y_z(PSample &data,FILE *fp,RefNode refNodes[],int rx_num);
int PosInfoQuality(const RefNode refNodes[], PSample data,PositionData Pos);
int main (int argc,char *argv[])
{
    PositionData lsq_pos;
    PositionData ekf_pos;
    
    int rx_num,i;
    FILE *conf_fp;
    double lastz=70;
    double x[max_rx_num];
    double y[max_rx_num];
    double z[max_rx_num];
    RefNode refNodes[max_rx_num];
    PSample data;
    int conf_mode = 0;

#define KF 1
#define TRI 2
#define BOTH 0
    int outputType= BOTH;
    if(argc>=2){
        if(!strcmp(argv[1],"EKF")||!strcmp(argv[1],"ekf")){
            outputType= KF;
        }else if(!strcmp(argv[1],"CONFIG")||!strcmp(argv[1],"config")){
            outputType= KF;
            conf_mode = 1;
        }else{
            outputType= TRI;
        }
    }

    if(conf_mode){//enter configuration mode
        conf_fp=fopen("./rx_coord_new.conf","wt");
        if(!conf_fp){
            cout<<"open ./rx_coord.conf failed"<<endl;
            exit(0);
        }
        rx_num= 0;
        char buffer[1024] = "";

        nodes_position_calc(refNodes,max_rx_num,true);
        for(i=0;i<(int)(sizeof(refNodes)/sizeof(RefNode));i++){
            if(refNodes[i].RID>0&&refNodes[i].RID<=max_rx_num){
                sprintf(&buffer[strlen(buffer)-1],"%2d:%.1f %.1f %.1f\n",(int)refNodes[i].RID,
                        (float)refNodes[i].x,(float)refNodes[i].y,(float)refNodes[i].z);
                rx_num++;
            }
        }
        
        if(rx_num>0){
            fprintf(stdout,"%3d\n%s",rx_num,buffer);//output the info to screen
            fprintf(conf_fp,"%3d\n%s",rx_num,buffer);//output the info to file
        }else{
            cout<<"configure failed!"<<endl;
        }
        fclose(conf_fp);
        exit(0);
    }else{//normal mode
        conf_fp=fopen("./rx_coord.conf","rt");
        if(!conf_fp){
            cout<<"open ./rx_coord.conf failed"<<endl;
            exit(0);
        }

        fscanf(conf_fp,"%d\n",&rx_num);
        for(i=0;i<rx_num;i++){
            int idx;
            fscanf(conf_fp,"%d:",&idx);
            fscanf(conf_fp,"%lf %lf %lf\n",&(x[idx]),&(y[idx]),&(z[idx]));
            refNodes[idx].x=x[idx];
            refNodes[idx].y=y[idx];
            refNodes[idx].z=z[idx];
            refNodes[idx].RID = idx;
        }
    }
    
    while(1){
        bool success = true;

        //usleep((unsigned int)500*1000);
        //fprintf(stderr,"-----loop %d---------\n",loop++);
        success = getData(data,stdin);
        //success = check_x_y_z(data,stdin,refNodes,rx_num);

        if(success==false){
            if(!feof(stdin)){
                cout<<"error:Negative data"<<endl;
                continue;
            }
            return 0;
        }
        else{
            PSample saved_data,org_data;
            int calc_again = 0;

            org_data = data;
            //LowThreshhold = 50.0, HighThreshold = 1200.0
            OutlierRejection(50.0, 1200.0,refNodes,data);
            saved_data = data;
            lsq_pos = PositionEstimateTriangulation(refNodes,data,lastz);
            
            while(calc_again--){//get a good lsq result 
#ifdef DEBUG
                fprintf(stderr,"lsq_again:%d\n",calc_again);
#endif
                lsq_pos = PositionEstimateTriangulation(refNodes,data,lastz);
                if(!calc_offset(refNodes,saved_data,lsq_pos))
                    break;
                data = saved_data;
            }
            int with_slice_id = 0;
            //int pk_type = 0;

            if(data.slice_id[0]){
                with_slice_id = 1;
            }

            if(outputType==BOTH||outputType==TRI){
                if(outputType==BOTH)
                    printf("LSQ:");
                if(!with_slice_id){
                    printf("B 0 %d %lf %lf %lf\n",lsq_pos.TargetID,lsq_pos.x,lsq_pos.y,lsq_pos.z);
                }else{
                    printf("B 2 %d %lf %lf %lf %s\n",lsq_pos.TargetID,lsq_pos.x,lsq_pos.y,lsq_pos.z,data.slice_id);
                }
            }

            if(outputType==BOTH||outputType==KF){
                ekf_pos=EKF(refNodes,data,lsq_pos);
                if(outputType==BOTH)
                    printf("EKF:");

               
                if(!with_slice_id){
                    printf("B 0 %d %lf %lf %lf\n",ekf_pos.TargetID,ekf_pos.x,ekf_pos.y,ekf_pos.z);
                }else{
                    printf("B 2 %d %lf %lf %lf %s\n",ekf_pos.TargetID,ekf_pos.x,ekf_pos.y,ekf_pos.z,data.slice_id);
                }

                //PosInfoQuality(refNodes,org_data,ekf_pos);
            }
            fflush(stdout);


        }
    }
    return 0;
}


bool getData(PSample &data,FILE *fp)
{
    int ret;
    int dist_num;
    int TargetID;

    //ret = fscanf(stdin,"%d",&dist_num);
    //cout<<"dist_num"<<dist_num<<" "<<"ret "<<ret<<endl;
    //return false;
    struct timeval time_now;

    //sleep(1);
    memset(&data,0,sizeof(data));//init

    if(gettimeofday(&time_now,NULL)){
        fprintf(stderr,"get time error!\n");
        data.TimeStamp = -1;
    }else{
        data.TimeStamp = (long)(time_now.tv_sec*1000+(long)time_now.tv_usec/1000);//get the time measured by ms
        
        /*
        if(data.TimeStamp<0)
            data.TimeStamp = 0;
        else
            data.TimeStamp += 1300;
        */
    }
    //fprintf(stderr,"--------get time %ld\n",data.TimeStamp);
    //fflush(stderr);

    data.SampleCount = 0;
    ret = fscanf(fp,"%d %d",&TargetID,&dist_num);
    /*
    TargetID= 0;
    ret = fscanf(fp,"%d",&dist_num);
    ret = 2;
    */
   // fprintf(stderr,"%d targetid=%d, distnum=%d\n",ret,TargetID,dist_num);

    //cout<<"dist_num="<<dist_num<<" ret = "<<ret<<endl;
    
    int ch;
    while(ret!=2){
        do{
            ch = fgetc(fp);
            //printf("%d\n",(int)ch);
        }while(ch!=-1&&ch!='\n');

        if(ch==-1){
            ret = -1;
            break;
        }
        ret = fscanf(fp,"%d %d",&TargetID,&dist_num);
        if(ret == -1)
            break;
    }
    if(ret==-1){
        return false;
    }

    data.SampleCount=dist_num;
    data.TargetID=TargetID;

    int error_flag=0;
    int j;
    //fprintf(stderr,"%d %d ",TargetID,dist_num);
    fprintf(stderr,"B 1 %d %d ",TargetID,dist_num);
    for(j=0;j<data.SampleCount;j++){
        int idx;
        double t;
        if(fscanf(fp,"%d:%lf",&idx,&t)!=-1){
            if(t<0){
                error_flag=1;
                break;
            }
            data.ReceiverId[j]=idx;
            data.Dist[j]=t;
            fprintf(stderr,"%d:%.1f ",idx,t);
        }
    }

    j=0;
    memset(data.slice_id,0,sizeof(data.slice_id));
    do{
        ch = fgetc(fp);
        if(ch>=(int)'0'&&ch<=(int)'9'&&j<(int)sizeof(data.slice_id)-1){
            data.slice_id[j++] = (char)ch;
        }
        //printf("%d\n",(int)ch);
    }while(ch!=-1&&ch!='\n');

    //fprintf(stderr,"slice_id %s\n",data.slice_id);//output for debug 
    
    fprintf(stderr,"\n");
    fflush(stderr);

    return (error_flag? false:true);
}


bool check_x_y_z(PSample &data,FILE *fp,RefNode refNodes[],int rx_num)
{
    int ret;
    double x,y,z;

    //const int node_num = 14;

    //ret = fscanf(stdin,"%d",&dist_num);
    //cout<<"dist_num"<<dist_num<<" "<<"ret "<<ret<<endl;
    //return false;
    data.SampleCount = 0;

    ret = fscanf(fp,"%lf %lf %lf",&x,&y,&z);

    if(ret==3)
        printf("set x=%.1f,y=%.1f,z=%.1f\n",x,y,z);

    //cout<<"dist_num="<<dist_num<<" ret = "<<ret<<endl;
    
    while(ret!=3){
        int ch;
        do{
            ch = fgetc(fp);
            //printf("%d\n",(int)ch);
        }while(ch!=-1&&ch!='\n');

        if(ch==-1){
            ret = -1;
            break;
        }
        ret = fscanf(fp,"%lf %lf %lf",&x,&y,&z);
        if(ret == -1)
            break;
    }
    if(ret==-1)
        return false;

    memset(&data,0,sizeof(data));
    data.SampleCount=rx_num;

    printf("%d ",rx_num);

    for(int j=1;j<=data.SampleCount;j++){
        double diffx,diffy,diffz;
        double distance;

        diffx = refNodes[j].x-x;
        diffy = refNodes[j].y-y;
        diffz = refNodes[j].z-z;
        distance = sqrt(diffx*diffx+diffy*diffy+diffz*diffz);

        data.Dist[j-1]=(int)(distance*10)/10.0;
        data.ReceiverId[j-1]=j;
        printf("%d:%.1f ",j,(float)distance);
    }
    printf("\n");

    return true;
}

#define Max_Dist 1000
#define Dist_step 10
#define Max_Dir 90
#define Dir_step 5

static int dis_dir_hist[Max_Dist/Dist_step+1][Max_Dir/Dir_step+1];
static int inited = 0;

//void output2file(void)
static void sigio_handler(int signo) {
    FILE *fp;

    if(inited){
        fp = fopen("./dis_dir.txt","wt");
        if(!fp){
            printf("open dis_dir.txt error!\n");
            return;
        }
        int total = 0;
        for(int i=0;i<=Max_Dist/Dist_step;i++){
            for(int j=0;j<=Max_Dir/Dir_step;j++){
                total += dis_dir_hist[i][j];
                fprintf(fp,"%4d ",dis_dir_hist[i][j]);
            }
            fprintf(fp,"\n");
        }
        //fprintf(fp,"loop is %d,total number is %d\n",ct,total);
        fprintf(fp,"total number is %d\n",total);
        //printf("write dis_dir%d.txt file, and total number is %d\n",((ct%200)!=0? 1:2),total);
        printf("total number is %d\n",total);
        fclose(fp);
    }
}
static void output2file(void)
{
    if(inited){
        sigio_handler(0);
    }
}
int PosInfoQuality(const RefNode refNodes[], PSample data,PositionData Pos)
{
    int i;
    //float std = 0;
    //FILE *fp = stdout;

    static int ct = 1;
    struct sigaction sigact,sigold;
    
    if(!inited){

        if(atexit(output2file)){
            printf("atexit() error!\n");
        }else{
            printf("ateixt() succeed!\n");
        }

        //install signal handler
        sigact.sa_handler = sigio_handler;
        sigemptyset(&sigact.sa_mask);
        //sigact.sa_flags = SA_INTERRUPT;
        if(sigaction(SIGINT,&sigact,&sigold)<0){
            printf("sigaction failed!\n");
            //print("sigaction failed!\n");
        }
        memset(dis_dir_hist,0,sizeof(dis_dir_hist));
        inited = 1;
    }

    //fprintf(fp,"%3d %3d ",ct++,data.SampleCount);
    for(i=0;i<data.SampleCount;i++){
        int rcv_id = data.ReceiverId[i];
        float dx  = Pos.x - refNodes[rcv_id].x;
        float dy  = Pos.y - refNodes[rcv_id].y;
        float dz  = Pos.z - refNodes[rcv_id].z;
        float real_dist = sqrt(dx*dx+dy*dy+dz*dz);
        float offset = real_dist - data.Dist[i];
        float ang = 0;

        ang = fabs(atan2(sqrt(dx*dx+dy*dy),fabs(dz))/3.1415926*180);

        if(offset>-15&&offset<-3){
            int dis = (int)real_dist;
            int dir = (int)ang;

            if(dis<0.0)
                dis=0;
            if(dis>Max_Dist)
                dis = Max_Dist;
            if(dir<0)
                dir = 0;
            if(dir>Max_Dir)
                dir = 90;
            dis_dir_hist[dis/Dist_step][dir/Dir_step]++;
        }
    }
    //fprintf(fp,"std %.1f\n",std);
    //fflush(fp);

    if(!(ct++%5)){
        printf("-----------------------------------loop %d----------------------------\n",ct);
    }
    /*
    if(!(ct%100)){
        FILE *fp;
        int total = 0;
        if(ct%200)
            fp = fopen("./dis_dir1.txt","wt");
        else
        
    }
    */

    return true;
}






