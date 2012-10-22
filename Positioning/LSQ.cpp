#include <matrix.h>
#include <Mymath.h>

#ifndef _NO_NAMESPACE
using namespace std;
using namespace math;
#define STD std
#else
#define STD
#endif

#ifndef _NO_TEMPLATE
typedef matrix<double> Matrix;
#else
typedef matrix Matrix;
#endif

#ifndef _NO_EXCEPTION
#  define TRYBEGIN()    try {
#  define CATCHERROR()  } catch (const STD::exception& e) { \
                                                cerr << "Error: " << e.what() << endl; }
#else
#  define TRYBEGIN()
#  define CATCHERROR()
#endif
#include <LocationDef.h>

/*PositionData PositionEstimateTriangulation(*/
	//double x[], 
	//double y[],
	//double z[],
	//double dist[], 
	//double lastz,
	//int size,
	//int TargetID
/*)*/


/*PositionData PositionEstimateTriangulation(   */
	//double x[], 
	//double y[],
	//double z[],
	//double dist[], 
	//double lastz,
	//int size,
	//int TargetID
	/*)*/

PositionData PositionEstimateTriangulation(const RefNode refNodes[], PSample &data,
        double lastz)
{
    //int t;
    int size = data.SampleCount;

    PositionData ret_result;

    double *x = new double[size];
    double *y = new double[size];
    double *z = new double[size];
    double *dist = new double[size];

    ret_result.belief = 0;
    ret_result.TargetID = -1;
    ret_result.x = 0;
    ret_result.y = 0;
    ret_result.z = 0;
    /*
    for(t=0;t<size;t++){
        cout<<"x="<<x[t]<<" y="<<y[t]<<" z="<<z[t]<<" d="<<dist[t]<<endl;
    }
    */

    for(int j=0;j<data.SampleCount;j++){
        int idx = data.ReceiverId[j];
        dist[j]=data.Dist[j];
        x[j]=refNodes[idx].x;
        y[j]=refNodes[idx].y;
        z[j]=refNodes[idx].z;
    }

    if(size >= 3)
    {
        PositionData result;

//#define LOOP_REDUCE
        
#ifndef LOOP_REDUCE
        Matrix A(size-1, 2);
        Matrix AT(2, size-1);
        Matrix ATA(2,2);
        Matrix IATA(2,2);
        Matrix IATAAT(2,size-1);
        Matrix b(size-1,1);
        Matrix X(2,1);

        for(int i=0; i<size-1; i++){
            A(i,0)=2*(x[size-1]-x[i]);
            A(i,1)=2*(y[size-1]-y[i]);
            b(i,0)=(pow(dist[i],2)-pow(dist[size-1],2))-(pow(x[i],2)-pow(x[size-1],2))-(pow(y[i],2)-pow(y[size-1],2));			
        }
#else
        size--;
        Matrix A(size, 2);
        Matrix AT(2, size);
        Matrix ATA(2,2);
        Matrix IATA(2,2);
        Matrix IATAAT(2,size);
        Matrix b(size,1);
        Matrix X(2,1);
        size++;

        for(int i=0; i<size-1; i++){
            A(i,0)=2*(x[i]-x[i+1]);
            A(i,1)=2*(y[i]-y[i+1]);
            b(i,0)=(pow(dist[i+1],2)-pow(dist[i],2))-(pow(x[i+1],2)-pow(x[i],2))-(pow(y[i+1],2)-pow(y[i],2));			
        }

        //A(size-1,0) = 2*(x[1]-x[0]);
        //A(size-1,1) = 2*(y[1]-y[0]);
#endif

        AT=~A;
        ATA=AT*A;

        //cout<<ATA(0,0)<<" "<<ATA(0,1)<<" "<<b(0,0)<<endl<<ATA(1,0)<<" "<<ATA(1,1)<<" "<<b(1,0)<<endl;
        double dt = ATA.Det();
        //cout<<"dt = "<<dt<<endl;
        //if( ATA.Det()==0 )
        if(dt==0 )
        {
            result.belief = 0;
            result.TargetID=-1;

            //return result;
            ret_result = result;
            goto out;
        }
        else
        {

            //cout<<ATA(0,0)<<" "<<ATA(0,1)<<" "<<b(0,0)<<endl<<ATA(1,0)<<" "<<ATA(1,1)<<" "<<b(1,0)<<endl;
            IATA=ATA.Inv();
            //cout<<ATA(0,0)<<" "<<ATA(0,1)<<" "<<b(0,0)<<endl<<ATA(1,0)<<" "<<ATA(1,1)<<" "<<b(1,0)<<endl;

            IATAAT=IATA*AT;
            X=IATAAT*b;

            //cout<<IATA(0,0)<<" "<<IATA(0,1)<<" "<<b(0,0)<<endl<<IATA(1,0)<<" "<<IATA(1,1)<<" "<<b(1,0)<<endl;

            result.x = X(0,0);
            result.y = X(1,0);
            double temp = 0;
            double c = 0;
            for (int i = 0; i< size; i++)
            {
                if((sqr(dist[i])-sqr(result.x-x[i])-sqr(result.y-y[i]))>0)
                {
                    temp += sqrt (sqr(dist[i])-sqr(result.x-x[i])-sqr(result.y-y[i]));	
                    c++;
                }	

            }			
            if (c > 0)
                result.z = temp/c;
            else
                result.z = lastz;

            result.belief=(float)size;
            result.TargetID=data.TargetID;

            //return result;
            ret_result = result;
            goto out;
        }
    }
    /************************************************************************/
    /** */
    /************************************************************************/
    if(size == 2)  
    {
        PositionData result1, result2;

        double a=2*(x[1]-x[0]);
        double b=2*(y[1]-y[0]);
        double c=pow(dist[0],2)-pow(dist[1],2)-(pow((z[0]-lastz),2)-pow((z[1]-lastz),2))-(pow(x[0],2)-pow(x[1],2))-(pow(y[0],2)-pow(y[1],2));

        double A=1+pow(b,2)/pow(a,2);
        double B=2*x[0]*b/a -2*b*c/pow(a,2)-2*y[0];
        double C= pow(x[0],2)+pow(c,2)/pow(a,2)+pow(y[0],2)-2*x[0]*c/a-pow(dist[0],2)+pow((z[1]-lastz),2);

        if((pow(B,2)-4*A*C)<0)
        {
            result1.x=0;
            result1.y=0;
            result1.z=0;
            result1.belief=0.0;
            result1.TargetID=-1;
            //return result1;
            ret_result = result1;
            goto out;
        }
        else
        {
            double y1=(-B+sqrt(pow(B,2)-4*A*C))/2/A;
            double y2=(-B-sqrt(pow(B,2)-4*A*C))/2/A;

            double x1=(c-b*y1)/a;
            double x2=(c-b*y2)/a;

            result1.x=x1;
            result1.y=y1;
            result1.z=lastz;
            result1.belief=2;

            result2.x=x2;
            result2.y=y2;
            result2.z=lastz;
            result2.belief=2;

            //    if(IsOutside(result1,result2,DeviceID,k,refNodes)==1)
            if(1)
            {
                //return result1;
                ret_result = result1;
                goto out;
            }
            else 
            {
                //return result2;
                ret_result = result2;
                goto out;
            }
        }
    }

out:
    delete x;
    delete y;
    delete z;
    delete dist;
    return ret_result;

}

