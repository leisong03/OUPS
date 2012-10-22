//#ifndef _NEWEKFFILTER_
//#define _NEWEKFFILTER_

//#include <time.h>                    
#include <matrix.h>  
#include <LocationDef.h>  

const double R = 80.2218;
const double SQRT3 = sqrt(3.0);

const int KALMAN_OK = 100;
const int REJECT_SAMPLE = 101;
const int BAD_STATE     = 200;

const int MinHeight = 50; //EKFÌõ¼þÏÂ£¬×îÐ¡µÄ¸ß¶ÈÒªÇó£¬µÍÓÚÕâ¸öÏÞÖÆ½«Ê¹ÓÃLSQ

//const double DELAY_THRESH = 1200;
const double DELAY_THRESH = 2000;

const int REJLEN = 20;
const int SUSLEN = 20;

const double rejectThresh = 1000; //1000;
const double suspectThresh = 0xffffffff;

const double rejectRatio = 0.2;


////////////////////////////////
// Note: The following conditional compilation statements are included
//       so that you can (likely to) compile this sample, without any 
//       modification, using a compiler which does not support any or 
//       all of the ANSI C++ features like NAMEPACE, TEMPLATE, and 
//       EXCEPTION, used in this class.
//
//       If you have a compiler, such as C++ Builder, Borland C++ 5.0,
//       MS Visual C++ 5.0 or higher, etc., which supports most of the ANSI 
//       C++ features used in this class, you do not need to include these
//       statements in your program.
//

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
#  define TRYBEGIN()	try {
#  define CATCHERROR()	} catch (const STD::exception& e) { \
						cerr << "Error: " << e.what() << endl; }
#else
#  define TRYBEGIN()
#  define CATCHERROR()
#endif

//class PDevice;
class NEWEKFfilter{
public:
   NEWEKFfilter(bool ThreeStates);
   NEWEKFfilter(){};
   ~NEWEKFfilter();
private:
	Matrix X;
	Matrix Phi;
	Matrix G;
	Matrix P;
	Matrix Qk;
	
	Matrix Xest;
	Matrix Pest;
	
private:
	int DIM;
	double UVAL;
	double qScale;
	double suspectRatio;
	bool ThreeStateEKF;
	
	double fCov;
	int runCounter;
	long lastUpdate;
	bool STARTUP; // start state for EKF
    int rejectBuf[REJLEN];
	int suspectBuf[SUSLEN];

public:
	void resetEKF(PositionData pos);
	PositionData ekfprocess(const RefNode refNodes[], PSample &data );
	double getfCov();

private:
	void InitAllMat();
	void setDiagMatrix1(Matrix &m, int i, int j, int ct, double v);
	void updateG(long dT);;
	void statePrediction(long dT);
	int covarPredictionAndCorrection(const RefNode refNodes[], long dT, PSample &data);
	int covarPredictionAndCorrection_new(const RefNode refNodes[], long dT, PSample &data);
    int get_H_K_X(const RefNode refNodes[],PSample &data,Matrix &H,Matrix &K,Matrix &X);
};

