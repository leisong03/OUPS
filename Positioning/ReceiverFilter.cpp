#include <math.h>
#include <string.h>

#include <LocationDef.h>

#include <stdio.h>

#include "ReceiverFilter.h"

static void SortSamples(PSample &data)
{
    int m_nLeafNumber = data.SampleCount;

	for(int i=0; i<m_nLeafNumber; i++){
        int min = i;
		for (int j=i+1; j<m_nLeafNumber; j++){
            if(data.Dist[j]<data.Dist[min])
                min = j;
        }
        if(min!=i){
            //SwapSamples(i,j);
            int tempd, tempr;

            tempd = data.Dist[min];
            data.Dist[min] = data.Dist[i];
            data.Dist[i] =  tempd;
            //tempv = data.validity[x];
            //data.validity[x] = data.validity[y];
            //data.validity[y] = tempv;
            tempr = data.ReceiverId[min];
            data.ReceiverId[min] = data.ReceiverId[i];
            data.ReceiverId[i] = tempr;
        }
	}
}

#define INVALIDVALUE 5000
//function:filter the receivers whose distance is not valid
//void OutlierRejection(float LowT, float HighT,const RefNode refNodes[])
void OutlierRejection(float LowT, float HighT,const RefNode refNodes[],PSample &data)
{
	//double Dmin,Dmax;
	//int Emin, Emax;
	//int j,k;
    int ValidSmpCount = 0;
    int min_pos,max_pos;


	//for (int i=0; i<m_nLeafNumber; i++)
	for(int i=0; i<data.SampleCount; i++)
	{
		if(data.Dist[i]>=LowT&&data.Dist[i]<=HighT){
            if(i!=ValidSmpCount){//move this data to correct position
                data.Dist[ValidSmpCount] = data.Dist[i];
                //data.validity[ValidSmpCount] = data.validity[i];
                data.ReceiverId[ValidSmpCount] = data.ReceiverId[i];
                data.Dist[i] = INVALIDVALUE;
                //ValidSmpCount++;
            }
            ValidSmpCount++;
        }
	}
    data.SampleCount = ValidSmpCount;

    //return;
	SortSamples(data);

    /*
    if(data.SampleCount>5){
        //fprintf(stderr,"%d ",(int)data.SampleCount);
        //for(int i=0;i<data.SampleCount;i++)
        //{
            //fprintf(stderr,"%d:%.1f ",(int)data.ReceiverId[i],(float)data.Dist[i]);
            //for(int j=i+1;j<data.SampleCount;j++)
                //if(data.Dist[i]>data.Dist[j])
                    //fprintf(stderr,"error to sort! Dist[i] = %f,Dist[j]= %f\n",data.Dist[i],data.Dist[j]);
        //}
        //fprintf(stderr,"\n");
        
        data.SampleCount = 5;
        return;
    }
    */

    min_pos = 0;
    max_pos = data.SampleCount-1;

    //int validsmp[ValidSmpCount];
    //validsmp = new int;
    //memset(validsmp,0,sizeof(validsmp));
	if(ValidSmpCount > 1)
	{
        //int min = 0,max = 0;
        for(int i=0;i<data.SampleCount;i++){
            double dist_i = data.Dist[i];
            int receiver_i = data.ReceiverId[i];

            if(dist_i==INVALIDVALUE)
                continue;
            for(int j=i+1;j<data.SampleCount;j++){
                double dist_j = data.Dist[j];
                int receiver_j = data.ReceiverId[j];

                double dist2nodes;
                double diffx,diffy,diffz;

                if(dist_j==INVALIDVALUE)
                    continue;

                diffx = refNodes[receiver_i].x - refNodes[receiver_j].x;
                diffy = refNodes[receiver_i].y - refNodes[receiver_j].y;
                diffz = refNodes[receiver_i].z - refNodes[receiver_j].z;

                dist2nodes = sqrt(diffx*diffx+diffy*diffy+diffz*diffz); 
                if(fabs(dist_i-dist_j)>dist2nodes)
                {
                    //PortData.validity[k] = 0;
                    //max_pos--;
                    if(dist_i>dist_j){
                        data.Dist[i] = INVALIDVALUE;
                        break;
                    }else{
                        data.Dist[j] = INVALIDVALUE;
                        continue;
                    }
                    //ValidSmpCount --;
                }
            }
        }
        ValidSmpCount = 0;
        for(int i=0;i<data.SampleCount;i++){
            if(data.Dist[i]<=HighT){
                if(i!=ValidSmpCount){//move this data to correct position
                    data.Dist[ValidSmpCount] = data.Dist[i];
                    data.ReceiverId[ValidSmpCount] = data.ReceiverId[i];
                    //data.Dist[i] = INVALIDVALUE;
                }
                ValidSmpCount++;
            }
        }
	}

    data.SampleCount = ValidSmpCount;
    //delete validsmp;
}
/*
void OutlierRejection(float LowT, float HighT,const RefNode refNodes[],PSample &data)
{
//#define INVALIDVALUE 5000
	double Dmin,Dmax;
	int Emin, Emax;
	//int j,k;
    int ValidSmpCount = 0;
    int min_pos,max_pos;

	//for (int i=0; i<m_nLeafNumber; i++)
	for(int i=0; i<data.SampleCount; i++)
	{
		if(data.Dist[i] > HighT ||data.Dist[i] < LowT)
		{
			//if(data.validity[i] == 1)
			//{
			//	data.validity[i] = 0;
				//SampleCount--;
			//}
			//data.Dist[i] = INVALIDVALUE;
            ;
		}else{
            if(i!=ValidSmpCount){//move this data to correct position
                data.Dist[ValidSmpCount] = data.Dist[i];
                //data.validity[ValidSmpCount] = data.validity[i];
                data.ReceiverId[ValidSmpCount] = data.ReceiverId[i];
                //ValidSmpCount++;
            }
            ValidSmpCount++;
        }
	}
    data.SampleCount = ValidSmpCount;

	SortSamples(data);
    min_pos = 0;
    max_pos = data.SampleCount-1;
	while(ValidSmpCount > 1)
	{
        //int min = 0,max = 0;
        double dist2nodes;
        double diffx,diffy,diffz;

        Dmin = data.Dist[min_pos];
        Emin = data.ReceiverId[min_pos];
        Dmax = data.Dist[max_pos];
        Emax = data.ReceiverId[max_pos];

        diffx = refNodes[Emax].x - refNodes[Emin].x;
        diffy = refNodes[Emax].y - refNodes[Emin].y;
        diffz = refNodes[Emax].z - refNodes[Emin].z;

        dist2nodes = sqrt(diffx*diffx+diffy*diffy+diffz*diffz); 
		if((Dmax - Dmin)>dist2nodes)
		{
			//PortData.validity[k] = 0;
            max_pos--;
			ValidSmpCount --;
		}
		else
		    break;
	}

    data.SampleCount = ValidSmpCount;
}
*/
