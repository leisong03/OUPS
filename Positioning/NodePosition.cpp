#include <matrix.h>  
#include <Mymath.h>  
#include <LocationDef.h>  
#include <ReceiverFilter.h>
#include <sys/time.h>
#include "NodePosition.h"

extern PositionData PositionEstimateTriangulation(const RefNode refNodes[], PSample &data,double lastz);
extern PositionData EKF(const RefNode refNodes[], PSample &data,PositionData LsqPos);
extern int calc_offset(const RefNode refNodes[],PSample &saved_data,PositionData pos_result);
extern int PosInfoQuality(const RefNode refNodes[], PSample data,PositionData Pos);

#define SHORTEST_LEN_OF_EDGE 20
#define Loops_of_data 15
#define DELTA_DIST_FOR_STABLE 4.0 //the threshold of distance which will be assumed acceptable when adjuging the stable dists

typedef struct Shelf_Conf{
    ShelfShape shape;
    Pos3D RefPt[4];
}Shelf_Conf;

typedef struct Tag2NdsDist{
    int TagId;//following distance is about this tagid
    int real_loops;//how many times of data is stored in the following array
    //NdDist[i][j] refers the distance from tag(indicated by TagId) to Node j at the order of i,
    //NdDist[i][0] describe the number of nodes with distances in this loop(NdDist[i])
    float NdDist[Loops_of_data][max_rx_num];
}Tag2NdsDist;

//configuration about shelf used to positioning
static Shelf_Conf shelf_conf = {{0.0,0.0},};

//distances from  tags to nodes
//TagNodeDist[i] refers the distances from tag i to all known nodes,
static Tag2NdsDist TagNodeDist[max_rx_num];

//configure the shape of shelf which used to construct reference
int shelf_shape_config(ShelfShape shape)
{
    if(shape.BottomEdge<=SHORTEST_LEN_OF_EDGE||shape.WaistEdge<=SHORTEST_LEN_OF_EDGE){
        shelf_conf.shape.BottomEdge = shelf_conf.shape.WaistEdge = 0.0;
    }else{
        Pos3D refpt[4];
        float lb,lw;
        float lh;

        lb = shape.BottomEdge;
        lw = shape.WaistEdge;
        lh = cos(30/180.0*3.1415926)*lb;

        memset(refpt,0,sizeof(refpt));//get the position of tag1 tag2 tag3 tag4

        //bottom
        refpt[0].x = 2.0/3*lh;//pointing to positive x
        refpt[0].y = refpt[0].z = 0.0;

        refpt[1].x = -1.0/3*lh;//pointing to positive y
        refpt[1].y = 1.0/2*lb;
        refpt[1].z = 0.0;

        refpt[2].x = -1.0/3*lh;//pointing to negative y
        refpt[2].y = -1.0/2*lb;
        refpt[2].z = 0.0;

        //top
        refpt[3].x = refpt[3].y = 0.0;//pointing to positive z
        refpt[3].z = sqrt(lw*lw-(2.0/3*lh)*(2.0/3*lh));

        shelf_conf.shape = shape;
        memcpy(shelf_conf.RefPt,refpt,sizeof(shelf_conf.RefPt));
    }
    memset(TagNodeDist,0,sizeof(TagNodeDist));

    return true;
}


extern bool getData(PSample &data,FILE *fp);
//int nodes_position_calc(RefNode &RfNds[max_rx_num],bool conf_data_from_stdin)

static FILE *tag1_fp=NULL,*tag2_fp=NULL,*tag3_fp=NULL,*tag4_fp=NULL;
static int loops = 0;

static bool data_input(PSample &RawData,bool conf_data_from_stdin)
{
    bool ret = false;
    static int lp = 1;

    if(!tag1_fp||!tag2_fp||!tag3_fp||!tag4_fp)
        loops = 0;

    if(conf_data_from_stdin==true){
        fprintf(stderr,"loop:%d ",lp++);
        ret = getData(RawData,stdin);
    }else{
        static int loops = 0;

        if(!tag1_fp){
            tag1_fp = fopen("./tags_data/tag1","rt");
            if(!tag1_fp){
                tag1_fp=tag2_fp=tag3_fp=tag4_fp=NULL;
                return false;
            }
        }
        if(!tag2_fp){
            tag2_fp = fopen("./tags_data/tag2","rt");
            if(!tag2_fp){
                fclose(tag1_fp);
                tag1_fp=tag2_fp=tag3_fp=tag4_fp=NULL;
                return false;
            }
        }
        if(!tag3_fp){
            tag3_fp = fopen("./tags_data/tag3","rt");
            if(!tag3_fp){
                fclose(tag1_fp);
                fclose(tag2_fp);
                tag1_fp=tag2_fp=tag3_fp=tag4_fp=NULL;
                return false;
            }
        }
        if(!tag4_fp){
            tag4_fp = fopen("./tags_data/tag4","rt");
            if(!tag4_fp){
                fclose(tag1_fp);
                fclose(tag2_fp);
                fclose(tag3_fp);
                tag1_fp=tag2_fp=tag3_fp=tag4_fp=NULL;
                return false;
            }
        }

        /*
        switch(loops++%4){
            case 0:
                ret = getData(RawData,tag1_fp);
                RawData.TargetID = 1;
                break;
            case 1:
                ret = getData(RawData,tag2_fp);
                RawData.TargetID = 2;
                break;
            case 2:
                ret = getData(RawData,tag3_fp);
                RawData.TargetID = 3;
                break;
            case 3:
                ret = getData(RawData,tag4_fp);
                RawData.TargetID = 4;
                break;
        }
        */
        if(loops<=20){
            //printf("getData from tag1_fp at %d.\n",loops);
            ret = getData(RawData,tag1_fp);
            RawData.TargetID = 1;
        }else if(loops<=40){
            //printf("getData from tag2_fp at %d.\n",loops);
            ret = getData(RawData,tag2_fp);
            RawData.TargetID = 1;
        }else if(loops<=60){
            //printf("getData from tag3_fp at %d.\n",loops);
            ret = getData(RawData,tag3_fp);
            RawData.TargetID = 1;
        }else if(loops<=80){
            //printf("getData from tag4_fp at %d.\n",loops);
            ret = getData(RawData,tag4_fp);
            RawData.TargetID = 1;
        }else{
            fprintf(stderr,"getData from nothing at %d.\n",loops);
            RawData.TargetID = 0;
        }
        loops++;
    }

    if(ret !=true){
        if(tag4_fp){
            fclose(tag4_fp);
            tag1_fp = NULL;
        }
        if(tag3_fp){
            fclose(tag3_fp);
            tag1_fp = NULL;
        }
        if(tag2_fp){
            fclose(tag2_fp);
            tag1_fp = NULL;
        }
        if(tag1_fp){
            fclose(tag1_fp);
            tag1_fp = NULL;
        }
        cout<<"data input error!"<<endl;
        return false;
    }

    return ret;

}
//get the offset of this position result
static float PosQualityCalc(PSample data,PositionData pos_res,RefNode refPts[]){
    float ret = 0;
    int i;

    for(i=0;i<data.SampleCount;i++){
        int rcv_id = data.ReceiverId[i];
        float dx  = pos_res.x - refPts[rcv_id].x;
        float dy  = pos_res.y - refPts[rcv_id].y;
        float dz  = pos_res.z - refPts[rcv_id].z;
        float real_dist = sqrt(dx*dx+dy*dy+dz*dz);
        float offset = real_dist - data.Dist[i];

        ret += offset*offset;
    }
    if(data.SampleCount!=0)
        ret /= data.SampleCount;
    else
        ret = 10000.0;

    ret += 0.001;//avoid 0
    return ret;
}

extern void stopEKF(int tagid);
//function:get the position of a point(maybe node or tag) indicated by CurNodePos.RID
//input:pSmpDataArr contains the data about distances from tags to the same node
//      len refers the real length of pSmpDataArr
//      refTags contains the position of each tag
//output:CurPt output the position of node CurPt.RID
int get_pos_of_point(PSample *pSmpDataArr,int len,RefNode refTags[max_rx_num],RefNode &CurPt)
{
    int ret = true;
    int valid_num = 0;
    int inited =0;
    float res_offset = 0.0;

    memset(&CurPt,0,sizeof(CurPt));
    
    while(--len>=0){//now,calc the position of nodes
        PositionData lsq_pos;
        PositionData ekf_pos;

        PSample saved_data,org_data,data;

        int calc_again = 0;
        int lastz = 70;

        //cout<<"real_loops is "<<len<<endl;
        //usleep((unsigned int)200*1000);
        //org_data = data;
        org_data = pSmpDataArr[len];

        data = org_data;
        //LowThreshhold = 50.0, HighThreshold = 1200.0
        OutlierRejection(50.0, 1200.0,refTags,data);
        saved_data = data;
        lsq_pos = PositionEstimateTriangulation(refTags,data,lastz);

        while(calc_again--){//get a good lsq result 
#ifdef DEBUG
            fprintf(stderr,"lsq_again:%d\n",calc_again);
#endif
            lsq_pos = PositionEstimateTriangulation(refTags,data,lastz);
            if(!calc_offset(refTags,saved_data,lsq_pos))
                break;
            data = saved_data;
        }

        if(!inited){
            inited = 1;
            stopEKF(data.TargetID);
        }
        ekf_pos=EKF(refTags,data,lsq_pos);
        
        if(ekf_pos.TargetID>0){
            float offset =0;

            offset = PosQualityCalc(data,ekf_pos,refTags);
            if(offset>5){//quality is too bad
                continue;
            }
            res_offset += offset;

            if(CurPt.RID==0)
                CurPt.RID = ekf_pos.TargetID;
            CurPt.x += ekf_pos.x;
            CurPt.y += ekf_pos.y;
            CurPt.z += ekf_pos.z;
            valid_num++;
        }
    }

    if(valid_num){
        CurPt.x /=valid_num;
        CurPt.y /=valid_num;
        CurPt.z /=valid_num;
        CurPt.belief = res_offset/valid_num;
    }
    if(CurPt.belief>10){
        memset(&CurPt,0,sizeof(CurPt));
        fprintf(stderr,"the quality of positioning result is bad!\n");
    }

    return ret;
}

//main function to locate all nodes' position
int nodes_position_calc(RefNode *pRfNds,int max_nd_num,bool conf_data_from_stdin)
{
    int ret = true;
    //int tagid = 0;
    int i,real_loops,rcv_num=0;
    ShelfShape shape = {43.3,58.0};//shelf's shape
    bool input_success = true;

    int ready_nds_num = 0;//describes the number of nodes which has already been positioned
    int needed_nds_num = 0;//describes the number of nodes which has already been found by now
    int8_t nd_found_now[max_nd_num+1];//record the nodes' id which has already been found by now
    const int pts_of_shelf = 4;

    if(max_nd_num!=max_rx_num){
        fprintf(stderr,"input parameter error!\n");
        return false;
    }

    memset(pRfNds,0,sizeof(RefNode)*max_nd_num);
    if(true!=shelf_shape_config(shape)){
        cout<<"shelf shape configure failed!"<<endl;
        return false;
    }

    if(shelf_conf.shape.BottomEdge<=SHORTEST_LEN_OF_EDGE||shelf_conf.shape.WaistEdge<=SHORTEST_LEN_OF_EDGE){
        cout<<"shelf shape is invalid!"<<endl;
        return false;
    }

    memset(nd_found_now,0,sizeof(nd_found_now));

    RefNode refTags[max_rx_num];//tags' id and position
    memset(refTags,0,sizeof(refTags));
    for(i=1;i<=pts_of_shelf;i++){//tags from 1 to 4 is the points which can construct a photo shelf
        refTags[i].x=shelf_conf.RefPt[i-1].x;
        refTags[i].y=shelf_conf.RefPt[i-1].y;
        refTags[i].z=shelf_conf.RefPt[i-1].z;
        refTags[i].RID = i;
        //fprintf(stdout,"tag %d:%3.1f %3.1f %3.1f\n",i,refTags[i].x,refTags[i].y,refTags[i].z);
    }

    int pos_tag_num = 0;
    do{
        //first,get stable distances from tag to the nodes,
        //at least Loops_of_data times of data should be collected for each tag
        while(1){ 
            //first,get the data first
            input_success = true;
            Tag2NdsDist proc_dist_buf;
            Tag2NdsDist *pTND = &proc_dist_buf;
            int rcv_tagid;

            float LastAvgNdDist[max_rx_num];

            memset(&proc_dist_buf,0,sizeof(proc_dist_buf));//init
            real_loops = proc_dist_buf.real_loops;//records the value of proc_dist_buf.real_loops
            if(real_loops<0){
                proc_dist_buf.real_loops = real_loops=0;
            }

            while(real_loops<Loops_of_data){//get a buffer of suitable distances about the same tag
                int stable_nds = 0;//how many nodes remain stable
                PSample RawData;//need to process

                input_success = data_input(RawData,conf_data_from_stdin);//get data from file stream
                if(input_success!=true){
                    return false;
                }
                rcv_tagid = RawData.TargetID;
                if(pTND->TagId!=0&&pTND->TagId!=rcv_tagid)//only processing the continuos tagid
                    continue;

                memset(pTND->NdDist[real_loops],0,sizeof(float)*max_rx_num);
                rcv_num = 0;
                for(i=0;i<RawData.SampleCount;i++){//transform dist in pSample to NdDist array
                    int rcv_id = RawData.ReceiverId[i];

                    if(rcv_id>=max_rx_num||rcv_id<=0)//node's id should be reasonable
                        continue;
                    if(!nd_found_now[rcv_id]){//records the node which appears
                        needed_nds_num++;
                        nd_found_now[rcv_id]++;
                    }
                    pTND->NdDist[real_loops][rcv_id] = RawData.Dist[i];
                    rcv_num++;
                    if(real_loops){//this is not the first loop
                        if(pTND->NdDist[real_loops-1][rcv_id]>0&&
                                fabs(pTND->NdDist[real_loops][rcv_id]-pTND->NdDist[real_loops-1][rcv_id])<DELTA_DIST_FOR_STABLE){//
                            stable_nds++;
                        }
                    }
                }
                if(rcv_num>4){
                    if(pos_tag_num>0&&stable_nds>=4){//make sure that the position of this tag is different with the last tag
                        int similar_nds = 0;
                        for(i=1;i<max_rx_num;i++){
                            if(pTND->NdDist[real_loops][i]>0&&LastAvgNdDist[i]>0){
                                if(fabs(pTND->NdDist[real_loops][i]-LastAvgNdDist[i])<5){
                                    similar_nds++;
                                }
                            }
                        }
                        if(similar_nds>stable_nds/2){
#ifdef DEBUG
                            fprintf(stderr,"same with the last tag!\n");
#endif
                            pTND->real_loops = real_loops = 0;
                            continue;
                        }
                    }

                    pTND->NdDist[real_loops][0] = (float)rcv_num;//update info
                    pTND->TagId = rcv_tagid;
                    pTND->real_loops = ++real_loops;

                    if(real_loops>3&&stable_nds<4){
                        fprintf(stderr,"stable distances from tag to node is not enough,reset this pos_tag!\n");
                        pTND->real_loops = real_loops = 0;
                    }else{
                        fprintf(stderr,"position_%d get data at loop %d\n",pos_tag_num+1,real_loops);
                    }
                }else{
                    fprintf(stderr,"only %d nodes received ultra sound,too little!\n",rcv_num);
                }

            }
            pTND->real_loops = real_loops;
            if(pos_tag_num<max_rx_num){
                ++pos_tag_num;
                fprintf(stderr,"Now got data about T%d,You can move this tag to the next position.\n",pos_tag_num);
                TagNodeDist[pos_tag_num] = proc_dist_buf;
                TagNodeDist[pos_tag_num].TagId = pos_tag_num;//change the TagId to position_id

                //now,update the info about LastAvgNdDist
                {
                    Tag2NdsDist *pLastTND = &TagNodeDist[pos_tag_num];

                    int k,nd;
                    int NdDistNum[max_rx_num];

                    memset(LastAvgNdDist,0,sizeof(LastAvgNdDist));
                    memset(NdDistNum,0,sizeof(NdDistNum));
                    for(k=0;k<pLastTND->real_loops;k++){
                        for(nd=1;nd<max_rx_num;nd++){
                            if(pLastTND->NdDist[k][nd]>0){
                                LastAvgNdDist[nd] += pLastTND->NdDist[k][nd];
                                NdDistNum[nd]++;
                            }
                        }
                    }
                    fprintf(stderr,"average NdDist:\n");
                    for(nd=1;nd<max_rx_num;nd++){
                        if(NdDistNum[nd]){
                            LastAvgNdDist[nd] /= NdDistNum[nd];
                            fprintf(stderr,"%d:%.1f ",nd,LastAvgNdDist[nd]);
                        }
                    }
                    fprintf(stderr,"---\n");
                }
            }else{
                fprintf(stderr,"position tags is too many!\n");
            }
            if(pos_tag_num>=pts_of_shelf)//at least 4 tags is ready,so break out
                break;
        }

        //second,locate this new added tag.
        //the tags from 1 to 4 needn't to positioning
        PSample SmpDataArr[Loops_of_data];
        real_loops = 0;//real length of SmpDataArr
        if(pos_tag_num>pts_of_shelf){
            for(i=0;i<=Loops_of_data;i++){
                int nd;
                PSample *pCurSmpData = &SmpDataArr[real_loops];

                memset(pCurSmpData,0,sizeof(PSample));
                //pCurSmpData->SampleCount =0;
                //fprintf(stderr,"%2d ",i);
                for(nd=1;nd<=max_rx_num;nd++){
                    //fprintf(stderr,"T%d:%3.1f ",tag,TagNodeDist[tag].NdDist[i][cur_nd]);
                    float nd_dist = TagNodeDist[pos_tag_num].NdDist[i][nd];
                    if(nd_dist<=0)
                        continue;

                    if(nd_dist<1){
                        fprintf(stderr,"********error with dist is near 0!\n");
                    }
                    pCurSmpData->Dist[pCurSmpData->SampleCount] = nd_dist;
                    pCurSmpData->ReceiverId[pCurSmpData->SampleCount] = nd;
                    pCurSmpData->SampleCount++;
                }
                //fprintf(stderr,"\n");
                if(pCurSmpData->SampleCount>=3){//
                    pCurSmpData->TargetID = pos_tag_num;
                    pCurSmpData->TimeStamp = -1;
                    real_loops++;
                }
            }

            RefNode CurTag={0,0,0};//get the current node's position
            if(real_loops>4){
                CurTag.RID = pos_tag_num;//the node which is going to be positioned
                get_pos_of_point(SmpDataArr,real_loops,pRfNds,CurTag);//get the position of this node
                if(CurTag.RID>0&&CurTag.RID<max_nd_num){
                    refTags[CurTag.RID] = CurTag;
                    fprintf(stderr,"********got new pos_tag %d:%.1lf %.1lf %.1lf,and belief %.1f************\n\n",CurTag.RID,CurTag.x,CurTag.y,CurTag.z,CurTag.belief);
                }
            }
            if(!(CurTag.RID>0&&CurTag.RID<max_nd_num)){//this tag is not correctly positioned 
                fprintf(stderr,"****not got the new pos_tag %d\n",pos_tag_num);
                pos_tag_num--;
                continue;//restart the new pos_tag
            }
        }

        //at last,it's time to locate the nodes' position which still has not been positioned
        int cur_nd;
        for(cur_nd=1;cur_nd<max_rx_num;cur_nd++){
            real_loops = 0;//real length of SmpDataArr
            //PSample SmpDataArr[Loops_of_data];
            RefNode CurNode={0,0,0};//get the current node's position

            CurNode.RID = cur_nd;//the node which is going to be positioned

            //if((!nd_found_now[CurNode.RID])&&pRfNds[CurNode.RID].RID>0)//this node doesn't appear or has already been positioned,so do nothing 
            if(!nd_found_now[CurNode.RID])//this node doesn't appear,so do nothing 
                continue;
            for(i=0;i<Loops_of_data;i++){//construct data for positioning the node,the data of first 5 is for initial
                int tag;
                PSample *pCurSmpData = &SmpDataArr[real_loops];

                pCurSmpData->SampleCount =0;
                fprintf(stderr,"%2d ",i);
                for(tag=1;tag<=pos_tag_num;tag++){
                    //if(TagNodeDist[tag].real_loops>=i&&TagNodeDist[tag].NdDist[i][0]>1&&
                      if(TagNodeDist[tag].NdDist[i][cur_nd]>0){
                        if(TagNodeDist[tag].NdDist[i][cur_nd]<1){
                            fprintf(stderr,"----------------error with dist is near 0!\n");
                        }

                        fprintf(stderr,"T%d:%3.1f ",tag,TagNodeDist[tag].NdDist[i][cur_nd]);

                        pCurSmpData->Dist[pCurSmpData->SampleCount] = TagNodeDist[tag].NdDist[i][cur_nd];
                        pCurSmpData->ReceiverId[pCurSmpData->SampleCount] = tag;
                        pCurSmpData->SampleCount++;
                    }else{
                        ;//fprintf(stderr,"not valid at tag %d!\n",tag);
                    }
                }
                fprintf(stderr,"\n");
                if(pCurSmpData->SampleCount>=3){//
                    pCurSmpData->TargetID = cur_nd;
                    pCurSmpData->TimeStamp = -1;
                    real_loops++;
                }
            }
            if(real_loops<4){
                fprintf(stderr,"not enough data to calculate %d!\n",CurNode.RID);
                continue;
            }

            get_pos_of_point(SmpDataArr,real_loops,refTags,CurNode);//get the position of this node
            //CurNode.belief += max_rx_num-pos_tag_num;
            if(CurNode.RID>0&&CurNode.RID<max_nd_num&&CurNode.belief>0){
                if(pRfNds[CurNode.RID].RID<=0)//this is the first to calculate this node's position
                    ready_nds_num++;
                else{
                    if(pRfNds[CurNode.RID].belief>0&&pRfNds[CurNode.RID].belief<CurNode.belief){//get the better one
                        fprintf(stderr,"last is better,previous belief is %.1f,and now is %.1f\n",pRfNds[CurNode.RID].belief,CurNode.belief);
                        //continue;
                    }
                }
                pRfNds[CurNode.RID] = CurNode;
                fprintf(stderr,"********got node %d:%.1lf %.1lf %.1lf,belief %.1f************\n\n",CurNode.RID,CurNode.x,CurNode.y,CurNode.z,CurNode.belief);
            }else{
                fprintf(stderr,"calculate node %d failed!\n",cur_nd);
            }
        }
        fprintf(stderr,"*******************nodes' position*******************\n");
        fprintf(stderr,"got %d nodes:\n",ready_nds_num);
        for(i=1;i<max_rx_num;i++){
            if(pRfNds[i].RID>0){
                fprintf(stderr,"%3d:%4.1lf %-4.1lf %-4.1lf,belief %-4.1f\n",pRfNds[i].RID,pRfNds[i].x,pRfNds[i].y,pRfNds[i].z,pRfNds[i].belief);
            }
        }
    }while(ready_nds_num<needed_nds_num);

    fprintf(stdout,"every node is positioned now!\n");

    return ret;
}

