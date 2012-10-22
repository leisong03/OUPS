#include <matrix.h>  
#include <Mymath.h>  
#include <LocationDef.h>  
#include <ReceiverFilter.h>
#include <sys/time.h>
#include "NodePostion.h"

extern PositionData PositionEstimateTriangulation(const RefNode refNodes[], PSample &data,double lastz);
extern PositionData EKF(const RefNode refNodes[], PSample &data,PositionData LsqPos);
extern int calc_offset(const RefNode refNodes[],PSample &saved_data,PositionData pos_result);
extern int PosInfoQuality(const RefNode refNodes[], PSample data,PositionData Pos);

#define SHORTEST_LEN_OF_EDGE 20
#define Loops_of_data 15
typedef struct Shelf_Conf{
    ShelfShape shape;
    Pos3D RefPt[3];
}Shelf_Conf;

typedef struct TagNdDist{
    int TagId;//following distance is about this tagid
    int real_loops;//how many times of data is stored in the following array
    float NdDist[Loops_of_data][max_rx_num];//NdDist[i][j] refers the distance from TagId to Node j at the order of i
}TagNdDist;

//configuration about shelf used to positioning
static Shelf_Conf shelf_conf = {{0.0,0.0},};

//distances from 4 tags to nodes
//TagNodeDist[i] refers the distances about tag i,
//               and 
static TagNdDist TagNodeDist[5] = {{0},{0},{0},{0},{0}};

//configure the shape of shelf which used to construct reference
int shelf_shape_config(ShelfShape shape)
{
    if(shape.BottomEdge<=SHORTEST_LEN_OF_EDGE||shape.WaistEdge<=SHORTEST_LEN_OF_EDGE){
        shelf_conf.shape.BottomEdge = shelf_conf.shape.WaistEdge = 0.0;
    }else{
        Pos3D refpt[3];
        float lb,lw;
        float lh;

        lb = shape.BottomEdge;
        lw = shape.WaistEdge;
        lh = cos(30/180.0*3.1415926)*lb;

        memset(refpt,0,sizeof(refpt));//get the position of tag1 tag2 tag3

        refpt[0].x = refpt[0].y = 0.0;
        refpt[0].z = sqrt(lw*lw-(2.0/3*lh)*(2.0/3*lh));

        refpt[1].x = 2.0/3*lh;
        refpt[1].y = refpt[1].z = 0.0;

        refpt[2].x = -1.0/3*lh;
        refpt[2].y = 1.0/2*lb;
        refpt[2].z = 0.0;

        shelf_conf.shape = shape;
        shelf_conf.Pos3D = refpt;
    }

    memset(TagNodeDist,0,sizeof(TagNodeDist));

    return true;
}


int node_position_calc(PSample RawData,RefNode &RfNds[max_rx_num]);
{
    int ret = true;
    int tagid;
    int i;
    int real_lps,rcv_num=0;

    if(shelf_conf.shape.BottomEdge<=SHORTEST_LEN_OF_EDGE||shelf_conf.shape.WaistEdge<=SHORTEST_LEN_OF_EDGE){
        cout<<"shelf shape is invalid!"<<endl;
        return false;
    }

    tagid= RawData.TargetID;
    if(tagid<=0||tagid>4){
        cout<<"tagId must be in 1-4!\n"<<endl;
        return false;
    }

    TagNdDist *pTND = &TagNodeDist[tagid];
    real_lps = pTND->real_loops;
    if(real_lps<=0){//data is not enough to calculate nodes' position
        real_lps = 0;
        memset(pTND->NdDist[real_lps],0,sizeof(float)*max_rx_num);//init value is 0
        for(i=0;i<RawData.SampleCount;i++){//transform dist in pSample to NdDist array
            int rcv_id = RawData.ReceiverId[i];
            if(rcv_id>=max_rx_num||rcv_id<=0)
                continue;
            pTND->NdDist[real_lps][rcv_id] = RawData.Dist[i];
            rcv_num++;
        }
        if(rcv_num>4){
            pTND->NdDist[real_lps][0] = (float)rcv_num;//update info
            pTND->TagId = tagid;
            pTND->real_loops =++real_lps;
        }
    }else if(real_lps<Loops_of_data){
        int stable_nds = 0;//how many nodes remain stable

        memset(pTND->NdDist[real_lps],0,sizeof(float)*max_rx_num);//init value is 0
        for(i=0;i<RawData.SampleCount;i++){//transform dist in pSample to NdDist array
            int rcv_id = RawData.ReceiverId[i];
            if(rcv_id>=max_rx_num||rcv_id<=0)
                continue;
            pTND->NdDist[real_lps][rcv_id] = RawData.Dist[i];
            rcv_num++;
            if(pTND->NdDist[real_lps-1][rcv_id]>0&&
                    fabs(pTND->NdDist[real_lps][rcv_id]-pTND->NdDist[real_lps-1][rcv_id])<2.0){
                stable_nds++;
            }
        }

        if(rcv_num>4){
            pTND->NdDist[real_lps][0] = (float)rcv_num;//update info
            pTND->TagId = tagid;
            pTND->real_loops = ++real_lps;
        }

        if(real_lps>=3&&stable_nds<4){//shelf's position has been changed or data is not acceptable
            pTND->real_loops = 0;
            real_lps = 0;
            for(int j=1;j<=4;j++){//invalidate other tags' data
                TagNodeDist[j].real_loops = 0;
            }
        }
    }

    if(real_lps>=Loops_of_data){//ha,it's time to get the nodes' position
        RefNode refNodes[max_rx_num];
        int tagDataReady = 0;

        for(i=1;i<=4;i++){
            if(TagNodeDist[i].real_loops>=Loops_of_data){
                tagDataReady++;
            }
        }

        for(i=1;i<=4;i++){
            refNodes[i].x=shelf_conf.Pos3D[i-1].x;
            refNodes[i].y=shelf_conf.Pos3D[i-1].y;
            refNodes[i].z=shelf_conf.Pos3D[i-1].z;
        }

        int cur_nd = 1;//the node which is going to be positioned
        PSample SmpDataArr[Loops_of_data];
        int real_lps = 0;//real length of SmpDataArr

        for(i=5;i<Loops_of_data;i++){//construct data for positioning the node,the data of first 5 is for initial
            int tag;
            tagDataReady = 0;

            PSample *pCurSmpData = &SmpDataArr[real_lps];

            pCurSmpData->SampleCount =0;
            for(tag=1;tag<=4;tag++){
                if(TagNodeDist[tag].real_loops>=i&&
                        TagNodeDist[tag].NdDist[i][0]>1&&TagNodeDist[tag].NdDist[i][cur_nd]>0){

                    pCurSmpData->Dist[pCurSmpData->SampleCount] = TagNodeDist[tag].NdDist[i][cur_nd];
                    pCurSmpData->ReceiverId[pCurSmpData->SampleCount] = tag;
                    pCurSmpData->SampleCount++;
                }
            }
            if(pCurSmpData->SampleCount>=4){//
                pCurSmpData->TargetID = cur_nd;
                pCurSmpData->TimeStamp = -1;
                real_lps++;
            }
        }

        while(--real_lps>=0)//now,calc the position of nodes
        {
            PositionData lsq_pos;
            PositionData ekf_pos;

            PSample saved_data,org_data;
            int calc_again = 0;

            usleep((unsigned int)200*1000);
            //org_data = data;
            org_data = SmpDataArr[real_lps];

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

            ekf_pos=EKF(refNodes,data,lsq_pos);
            printf("%d %lf %lf %lf\n\n",ekf_pos.TargetID,ekf_pos.x,ekf_pos.y,ekf_pos.z);
            fflush(stdout);
        }
    }

    return ret;
}

