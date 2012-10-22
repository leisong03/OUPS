#include <newekf.h>
#include <Mymath.h>
//#include <LocationDef.h>

NEWEKFfilter::NEWEKFfilter(bool threeStates)
{
    this->ThreeStateEKF = threeStates;

    if(ThreeStateEKF) {
        suspectRatio = 0.4;
        UVAL = 3.5567;
        qScale = pow(10.0, -4);
        DIM = 3;
    }
	else
	{
		suspectRatio = 0.2;
		UVAL = 6.5567;
		qScale = pow(10.0, -4.4);
		DIM = 6;
	}

	X.SetSize(DIM,1);
	Phi.SetSize(DIM, DIM);// three states
	G.SetSize(DIM,DIM);
	P.SetSize(DIM,DIM);
	Qk.SetSize(DIM,DIM);

	Xest.SetSize(DIM, 1);
	Pest.SetSize(DIM, DIM);

	STARTUP = false;
	lastUpdate = 0;
	// clear chibuffer
	ClearBuff(rejectBuf, REJLEN);
	ClearBuff(suspectBuf, SUSLEN);
}

NEWEKFfilter::~NEWEKFfilter()
{
}

void NEWEKFfilter::resetEKF(PositionData pos)
{
	if(pos.TargetID == -1) // error pos, ignore
		return;

#ifdef DEBUG
	cout<<"reset with"<<pos.x<<" "<<pos.y<<" "<<pos.z<<endl;
#endif
	//add by songlei
	InitAllMat();
	//end 

	Xest(0,0) = pos.x;
	Xest(1,0) = pos.y; 
	Xest(2,0) = pos.z; 
    if(!ThreeStateEKF)
	{

#ifdef DEBUG
	  //  cout<<"six state"<<endl;
#endif
		Xest(3,0) =  0;
		Xest(4,0) =  0;
		Xest(5,0) =  0;
	}
		
	X = Xest; //CopyVector(X, xEst, DIM);
    STARTUP = true;
	P.Unit(); //InitVec(D, DIM, 1.0);
	//newU(U, DIM, UVAL);

	// clear chibuffer
	ClearBuff(rejectBuf, REJLEN);
	ClearBuff(suspectBuf, SUSLEN);
	
	runCounter = 0; // ?
	fCov = 1; // ?
}

void NEWEKFfilter::setDiagMatrix1(Matrix &m, int i, int j, int ct, double v)
{
	for(int c = 0; c < ct; c++) {
		m(i+c, j+c) = v;
	}
}

void NEWEKFfilter::updateG(long dT)
{
	double sqrtdt = sqrt((double)dT);
	double G1 = (SQRT3/3)*sqrtdt*dT;
#ifdef DEBUG
    cout<<"G1="<<G1<<endl;
#endif
	setDiagMatrix1(G, 0, 0, 3, G1);

	if(!ThreeStateEKF)
	{
		double G2 = SQRT3/2*sqrtdt;
		double G3 = (1/2)*sqrtdt;
		setDiagMatrix1(G, 0, 3, 3, G2);
		setDiagMatrix1(G, 3, 0, 3, 0);
		setDiagMatrix1(G, 3, 3, 3, G3);
	}	
}

void NEWEKFfilter::statePrediction(long dT)
{
	if(!STARTUP)
		return;

	if(!ThreeStateEKF)
	{
		setDiagMatrix1(Phi, 0, 3, 3, dT);
	}

	Xest = Phi * X;

    //for debug
    /*
    {
        static int lp =0;
        cout<<"times:"<<lp<<endl;
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                cout<<Phi(i,j)<<" ";
            }
            cout<<endl;
        }
    }
    */
}

void NEWEKFfilter::InitAllMat()
{

#ifdef DEBUG
    cout<<"init all mate"<<endl;
#endif
	//double aa;
	//aa = X(0,0);
	//aa = X(2,0);
	X.Null();
	//aa = X(0,0);
	//aa = X(2,0);

	G.Null();
	P.Null();

    Qk.Null();
	for(int i=0; i<DIM; i++)
		Qk(i,i) = qScale;
	Phi.Unit();
	Xest.Null();
	Pest.Null();
}

PositionData NEWEKFfilter::ekfprocess(const RefNode refNodes[], PSample &data )
{
	long time_now = data.TimeStamp;
	long dT = time_now - lastUpdate;

    //dT=200;
	//lastUpdate = time_now;
    //dT=PK_INTERVAL;
    
    //fprintf(stderr,"----------lastTime %ld,thisTime %ld,dT %ld\n",lastUpdate,time_now,dT);
    //fflush(stderr);

    if(time_now>0&&lastUpdate>0&&dT>0&&dT<60*1000){//get dT
        dT = time_now - lastUpdate;
        lastUpdate = time_now;
    }else{
        dT=PK_INTERVAL;
        lastUpdate = time_now;
    }

    //dT = 500;

	if(!STARTUP)
		InitAllMat();

	updateG(dT);
	statePrediction(dT);
	//int result = covarPredictionAndCorrection(refNodes, dT, data);
	int result = covarPredictionAndCorrection_new(refNodes, dT, data);

	PositionData rst;
	rst.TargetID = -1; 

	if (result == KALMAN_OK ) {
#ifdef DEBUG
	    cout<<"kalman ok"<<endl;
#endif
		runCounter++;
		rst.TargetID = data.TargetID;
		rst.x = X(0,0);
		rst.y = X(1,0);
		rst.z = X(2,0);
		rst.belief = (float)data.SampleCount;
	}

	if (result == REJECT_SAMPLE){

#ifdef DEBUG
	    cout<<"reject sample"<<endl;
#endif
		rst.TargetID = data.TargetID;
		rst.x = X(0,0);
		rst.y = X(1,0);
		rst.z = X(2,0);
		rst.belief = (float)data.SampleCount;		
	}

	if (result == BAD_STATE){
		rst.TargetID = -1;
	    cout<<"bad state"<<endl;
	}
// add by ZJH
  /*	if(rst.z < MinHeight) // if lower than MinHeight, return error and LSQ is used
	{
		rst.TargetID = -1;
	}
	*/
		
	return rst;
}

double NEWEKFfilter::getfCov()
{
	return fCov;
}

int NEWEKFfilter::covarPredictionAndCorrection(const RefNode refNodes[], long dT, PSample &data)
{
 //   cout<<data.SampleCount<<" "<<data.Dist[2]<<" "<<data.ReceiverId[2]<<" "<<endl;
  //  cout<<refNodes[1].z<<endl;
	if(dT > DELAY_THRESH || !STARTUP) // need to bootstrap
	{
#ifdef DEBUG
	    cout<<"big delay"<<endl;
#endif
		return BAD_STATE;
	}
	
	
	//UpdatePPrior(); //Pest = Phi*P*~Phi+G*Qk*~G;
	Matrix PhiT, GT, Pest1, Pest2;
	PhiT.SetSize(DIM,DIM);
	GT.SetSize(DIM, DIM);
	Pest1.SetSize(DIM,DIM);
	Pest2.SetSize(DIM, DIM);
	PhiT = ~Phi;
	GT = ~G;

	Pest1 = Phi*P;
	Pest1 = Pest1*PhiT;
	Pest2 = G*Qk;
	Pest2 = Pest2*GT;
	Pest = Pest1 + Pest2;


	// begion correction step
	int m = data.SampleCount;
	int i;
	Matrix H;
	H.SetSize(m, DIM);
	H.Null();
	Matrix zks;
	//zk.SetSize(m,1);
	//Hx.SetSize(m,1);
	zks.SetSize(m,1);
	zks.Null();
	//Hx(0,0) = htheo;
	//zk(0,0) = data.dist;
	//zks(0,0) = data.dist - htheo;
	double xc, yc, zc;
	int n;

	for(i=0;i<m;i++)
	{
		n = data.ReceiverId[i];
		xc = refNodes[n].x;
		yc = refNodes[n].y;
		zc = refNodes[n].z;
#ifdef DEBUG
		cout<<"no."<<n<<" x"<<xc<<" y"<<yc<<" z"<<zc<<" d"<<data.Dist[i]<<endl;
#endif

		double htheo = sqrt(sqr(Xest(0,0)-xc)+ sqr(Xest(1,0)-yc) + sqr(Xest(2,0)-zc)); // HX

		H(i,0) = (Xest(0,0)-xc)/htheo;
		H(i,1) = (Xest(1,0)-yc)/htheo;
		H(i,2) = (Xest(2,0)-zc)/htheo;

		zks(i,0) = data.Dist[i] - htheo;

	}
#ifdef DEBUG
    cout<<"Xest=("<<Xest(0,0)<<","<<Xest(1,0)<<","<<Xest(2,0)<<")"<<endl;
#endif
     
	// find max value of zks for bad state judgement of kalman filtering
	double zks_max = zks(0,0);
    int zks_max_idx=0;
	for(i=0;i<m;i++)
	{

#ifdef DEBUG
        cout<<"zks="<<zks(i,0)<<"@"<<data.ReceiverId[i]<<endl;
#endif
		if(zks(i,0)>zks_max){
			zks_max = zks(i,0); 
            zks_max_idx=i;
        }
	}

	// do bad state judgement
	double chisqr = sqr(zks_max); 
#ifdef DEBUG
    cout<<"zks_max="<<zks_max<<"@"<<data.ReceiverId[zks_max_idx]<<endl;
#endif
	bool ignoreOutlierRejection = false;
	int badest = 0;
	fCov = 0; // remains 0 if EKF falls into bad state

	// Record chisqr value, must do this before next loop
	if (chisqr > rejectThresh) {
		InsertBuff(rejectBuf, 1, REJLEN); //rejectBuf.insert(1);
		InsertBuff(suspectBuf, 0, SUSLEN); //suspectBuf.insert(0); // We don't want to count an outlier as suspicious
	}
	else {
		InsertBuff(rejectBuf, 0, REJLEN);//	rejectBuf.insert(0);
		if (chisqr > suspectThresh)
			InsertBuff(suspectBuf, 1, SUSLEN);//suspectBuf.insert(1);
		else
			InsertBuff(suspectBuf, 0, SUSLEN);//suspectBuf.insert(0);
	}
	ignoreOutlierRejection = ( Ratio(rejectBuf, REJLEN)>= rejectRatio);

// add by zjh
   /* if (ignoreOutlierRejection)
	{
		return BAD_STATE;
	}*/
	//

	if (chisqr > rejectThresh && (runCounter > 10)) { //throw away sample
		if (!ignoreOutlierRejection) {
			//Util.log("Throwing out sample, ignoring bad state indicator");
			Xest = X;//CopyVector(xEst, X, DIM);//xEst = X.copy();
			return REJECT_SAMPLE;
		}
		else {
			badest = 1;
		}
	}
	else if (Ratio(suspectBuf, SUSLEN) >= suspectRatio) {
		badest = 2;
	}

	if (badest > 0) 
	{
	    return BAD_STATE;
	}

#ifdef DEBUG
	float tempatio=Ratio(suspectBuf, SUSLEN);
	cout<<"Ratio"<<tempatio<<endl;
#endif
	//KGain(); K= Pest*~H*(HPest~H+R).inv();  return inv().det as fcov
	Matrix K, Den, Rm, HT;
	K.SetSize(DIM,m);
	K.Null();
	Den.SetSize(m, m);
	Den.Null();
	Rm.SetSize(m,m);
	Rm.Null();
	for(i=0; i<m; i++)
	   Rm(i,i) = R;

	HT.SetSize(DIM,m);
	HT = ~H;

	Matrix Den1;
	Den1.SetSize(m,DIM);
	Den1 = H*Pest;
	Den = Den1*HT;
	Den = Den+Rm;
	Den = Den.Inv();

	Matrix K1; 
	K1.SetSize(DIM,m);
	K1 = Pest*HT;
	K = K1*Den;
	
	double fcov1;
	fcov1 = Den.Det();
	fCov = exp(-0.5*chisqr*fcov1)/(2*3.1415926/fcov1);
	
	//UpdateX(); X = Xest+K(Zk-Hxest);
	
	Matrix X1;
	X1.SetSize(DIM,1);
	X1 = K*zks;

#ifdef DEBUG
    cout<<"deta:"<<X1(0,0)<<","<<X1(1,0)<<","<<X1(2,0)<<endl;
#endif

	X = Xest + X1;

	//UpdatePPost(); P = (I-KH)*Pest;
	Matrix I;
	I.SetSize(DIM,DIM);
	I.Unit();
	P= K*H;
	P=I-P;
	P = P*Pest;

	return KALMAN_OK;
}

//#define MY_DEBUG

int calc_offset(const RefNode refNodes[],PSample &saved_data,PositionData result);
//----------------added by wl 2010.11.5----------------
int NEWEKFfilter::covarPredictionAndCorrection_new(const RefNode refNodes[], long dT, PSample &data)
{
	if(dT > DELAY_THRESH || !STARTUP) // need to bootstrap
	{
#ifdef DEBUG
	    cout<<"big delay for dT is too large!"<<endl;
#endif
		return BAD_STATE;
	}
	
	//UpdatePPrior(); //Pest = Phi*P*~Phi+G*Qk*~G;
	Matrix PhiT, GT, Pest1, Pest2;
	PhiT.SetSize(DIM,DIM);
	GT.SetSize(DIM, DIM);
	Pest1.SetSize(DIM,DIM);
	Pest2.SetSize(DIM, DIM);
	PhiT = ~Phi;
	GT = ~G;

	Pest1 = Phi*P;
	Pest1 = Pest1*PhiT;
	Pest2 = G*Qk;
	Pest2 = Pest2*GT;
	Pest = Pest1 + Pest2;


	// begin correction step
	Matrix H;
    Matrix K;
	//Matrix zks;

	//zk.SetSize(m,1);
	//Hx.SetSize(m,1);

    int ret;
    int calc_again = 4;

    int saved_fCov;
    int saved_rejectBuf[REJLEN];
    int saved_suspectBuf[SUSLEN];
    Matrix saved_Xest,saved_X;
	saved_Xest.SetSize(DIM, 1);
	saved_X.SetSize(DIM, 1);

    //backup  this state info for the purpose of restoring
    saved_fCov = fCov;
    memcpy(saved_rejectBuf,rejectBuf,sizeof(saved_rejectBuf));
    memcpy(saved_suspectBuf,suspectBuf,sizeof(saved_suspectBuf));
    saved_Xest = Xest;
    saved_X = X;

    while(calc_again--){
#ifdef MY_DEBUG
        fprintf(stderr,"---ekf_again:%d:\n",calc_again);
#endif
        //restore the state info 
        fCov = saved_fCov;
        memcpy(rejectBuf,saved_rejectBuf,sizeof(saved_rejectBuf));
        memcpy(suspectBuf,saved_suspectBuf,sizeof(saved_suspectBuf));
        Xest = saved_Xest;
        X = saved_X;

        ret = get_H_K_X(refNodes,data,H,K,X);

        if(ret!=KALMAN_OK)
            return ret;

        PositionData result_pos;
        result_pos.x = X(0,0);
        result_pos.y = X(1,0);
        result_pos.z = X(2,0);
        if(!calc_offset(refNodes,data,result_pos))
            break;
    }

    //if(ret!=KALMAN_OK)
    //    return ret;

    //UpdatePPost(); P = (I-KH)*Pest;
	Matrix I;
	I.SetSize(DIM,DIM);
	I.Unit();
	P= K*H;
	P=I-P;
	P = P*Pest;

	return KALMAN_OK;
}

//get matrix H,K and X
int NEWEKFfilter::get_H_K_X(const RefNode refNodes[],PSample &data,Matrix &H,Matrix &K,Matrix &X)
//int get_H_K_X(const RefNode refNodes[],PSample &data,Matrix &H,Matrix &K,Matrix &X)
{
    int ret = KALMAN_OK;
    int i,m,n;
	double xc, yc, zc;
	Matrix zks;


//#ifdef DEBUG
//#undef DEBUG
//#endif

	double zks_max = 0;
    int zks_max_idx;
    PSample saved_data = data;

    while(saved_data.SampleCount>=3){
        //init H K zks
        data = saved_data;
        m = data.SampleCount;
        H.SetSize(m, DIM);
        H.Null();

        K.SetSize(DIM,m);
        K.Null();

        zks.SetSize(m,1);
        zks.Null();

	    zks_max = zks(0,0);
        zks_max_idx=0;
        for(i=0;i<m;i++){
            n = data.ReceiverId[i];
            xc = refNodes[n].x;
            yc = refNodes[n].y;
            zc = refNodes[n].z;

            double htheo = sqrt(sqr(Xest(0,0)-xc)+ sqr(Xest(1,0)-yc) + sqr(Xest(2,0)-zc)); // HX

            H(i,0) = (Xest(0,0)-xc)/htheo;
            H(i,1) = (Xest(1,0)-yc)/htheo;
            H(i,2) = (Xest(2,0)-zc)/htheo;

            zks(i,0) = data.Dist[i] - htheo;


            if(zks(i,0)>zks_max){// find max value of zks for bad state judgement of kalman filtering
                zks_max = zks(i,0); 
                zks_max_idx=i;
            }
            if(sqr(zks(i,0))>rejectThresh){//invalidate the unreasonable data first
                saved_data.Dist[i] = INVALIDVALUE;
            }
        }

        int ValidSmpCount = 0;
        for(int k=0;k<saved_data.SampleCount;k++){//remove the unreasonable data
            if(saved_data.Dist[k]<INVALIDVALUE/2){
                if(k!=ValidSmpCount){//move this data to correct position
                    saved_data.Dist[ValidSmpCount] = saved_data.Dist[k];
                    saved_data.ReceiverId[ValidSmpCount] = saved_data.ReceiverId[k];
                    saved_data.Dist[k] = INVALIDVALUE;
                }
                ValidSmpCount++;
            }
        }
        saved_data.SampleCount = ValidSmpCount;
        if(ValidSmpCount==data.SampleCount||ValidSmpCount<4)
            break;
    }

#ifdef DEBUG
    //cout<<"Xest=("<<Xest(0,0)<<","<<Xest(1,0)<<","<<Xest(2,0)<<")"<<endl;
#endif
     
	// find max value of zks for bad state judgement of kalman filtering
    /*
	double zks_max = zks(0,0);
    int zks_max_idx=0;
	for(i=0;i<m;i++)
	{

#ifdef DEBUG
        cout<<"zks="<<zks(i,0)<<"@"<<data.ReceiverId[i]<<endl;
#endif
		if(zks(i,0)>zks_max){
			zks_max = zks(i,0); 
            zks_max_idx=i;
        }
	}
    */

	// do bad state judgement
	double chisqr = sqr(zks_max); 
#ifdef DEBUG
    //cout<<"zks_max="<<zks_max<<"@"<<data.ReceiverId[zks_max_idx]<<endl;
#endif
	bool ignoreOutlierRejection = false;
	int badest = 0;
	fCov = 0; // remains 0 if EKF falls into bad state

	// Record chisqr value, must do this before next loop
	if(chisqr > rejectThresh){
		InsertBuff(rejectBuf, 1, REJLEN); //rejectBuf.insert(1);
		InsertBuff(suspectBuf, 0, SUSLEN); //suspectBuf.insert(0); // We don't want to count an outlier as suspicious
	}else{
		InsertBuff(rejectBuf, 0, REJLEN);//	rejectBuf.insert(0);
		if (chisqr > suspectThresh)
			InsertBuff(suspectBuf, 1, SUSLEN);//suspectBuf.insert(1);
		else
			InsertBuff(suspectBuf, 0, SUSLEN);//suspectBuf.insert(0);
	}
	ignoreOutlierRejection = ( Ratio(rejectBuf, REJLEN)>= rejectRatio);

	if(chisqr > rejectThresh && (runCounter > 10)){ //throw away sample
		if (!ignoreOutlierRejection) {
			//Util.log("Throwing out sample, ignoring bad state indicator");
			Xest = X;//CopyVector(xEst, X, DIM);//xEst = X.copy();
			return REJECT_SAMPLE;
		}else{
			badest = 1;
		}
	}
	else if (Ratio(suspectBuf, SUSLEN) >= suspectRatio) {
		badest = 2;
	}

	if (badest > 0){
	    return BAD_STATE;
        //goto out_ret;
	}

#ifdef DEBUG
	//float tempatio=Ratio(suspectBuf, SUSLEN);
	//cout<<"Ratio"<<tempatio<<endl;
#endif
	//KGain(); K= Pest*~H*(HPest~H+R).inv();  return inv().det as fcov
	Matrix Den, Rm, HT;
	Den.SetSize(m, m);
	Den.Null();
	Rm.SetSize(m,m);
	Rm.Null();
	for(i=0; i<m; i++)
	   Rm(i,i) = R;

	HT.SetSize(DIM,m);
	HT = ~H;

	Matrix Den1;
	Den1.SetSize(m,DIM);
	Den1 = H*Pest;
	Den = Den1*HT;
	Den = Den+Rm;
	Den = Den.Inv();

	Matrix K1; 
	K1.SetSize(DIM,m);
	K1 = Pest*HT;
	K = K1*Den;
	
	double fcov1;
	fcov1 = Den.Det();
	fCov = exp(-0.5*chisqr*fcov1)/(2*3.1415926/fcov1);
	
	//UpdateX(); X = Xest+K(Zk-Hxest);
	
	Matrix X1;
	X1.SetSize(DIM,1);
	X1 = K*zks;

#ifdef DEBUG
    //cout<<"deta:"<<X1(0,0)<<","<<X1(1,0)<<","<<X1(2,0)<<endl;
#endif

	X = Xest + X1;

    return ret;
}

int calc_offset(const RefNode refNodes[],PSample &saved_data,PositionData result)
{
    float max_offset = -10000.0;
    float min_offset = 10000.0;
    float offset_thr = -4.0;//-4.0
    int ValidSmpCount = 0;
    int ret = 1;
    int k;

    //if(repeat_time>=2)
    //    offset_thr = offset_thr - 2*repeat_time;
    
//#define SORT_OFFSET

#ifdef SORT_OFFSET 

    float offset_low = -0.1;
    float offset_high = 1000000.0;
    //get the offset of eacho node
    float offsetArr[saved_data.SampleCount];
    for(k=0;k<saved_data.SampleCount;k++){
        int rec_id = saved_data.ReceiverId[k];
        float dx = result.x-refNodes[rec_id].x;
        float dy = result.y-refNodes[rec_id].y;
        float dz = result.z-refNodes[rec_id].z;

        offsetArr[k] = sqrt(dx*dx+dy*dy+dz*dz) - saved_data.Dist[k];
    }

    int i,j;
    //sort the array from big to small
    for(i=0; i<saved_data.SampleCount; i++){
        int max = i;
		for (j=i+1;j<saved_data.SampleCount;j++){
            if(offsetArr[j]>offsetArr[max])
                max = j;
        }
        if(max!=i){
            //SwapSamples(i,j);
            int tempd, tempr;
            float tempoff;

            tempoff = offsetArr[max];
            offsetArr[max] = offsetArr[i];
            offsetArr[i] = tempoff;

            tempd = saved_data.Dist[max];
            saved_data.Dist[max] = saved_data.Dist[i];
            saved_data.Dist[i] =  tempd;

            tempr = saved_data.ReceiverId[max];
            saved_data.ReceiverId[max] = saved_data.ReceiverId[i];
            saved_data.ReceiverId[i] = tempr;
        }
	}

    int left = 0;
    int right = saved_data.SampleCount-1;
    while(right-left+1>4||offsetArr[right]<-5){
        //printf("while loop!\n");
        if(offsetArr[right]<offset_low){
            right--;
        }else if(offsetArr[left]>offset_high){
            left++;
        }else{
            break;
        }
    }

    max_offset = offsetArr[left];
    min_offset = offsetArr[right];

    //ValidSmpCount = right-left+1;
    ValidSmpCount = 0;
    for(i=left;i<=right;i++){
        if(i!=ValidSmpCount){
            saved_data.Dist[ValidSmpCount] = saved_data.Dist[i];
            saved_data.ReceiverId[ValidSmpCount] = saved_data.ReceiverId[i];
        }

#ifdef DEBUG
        fprintf(stderr,"%d:%.1f ",saved_data.ReceiverId[ValidSmpCount],offsetArr[i]);//for debug
#endif

        ValidSmpCount++;
    }

#ifdef DEBUG
    fprintf(stderr,"\nbef_nodes:%2d,now_nodes:%2d \nmax_offset:%.1f,min_offset:%.1f\n",saved_data.SampleCount,ValidSmpCount,max_offset,min_offset);//for debug
#endif

#else//does not sort the offset
    int max_pos = -1;
    float std = 0.0;
    for(k=0;k<saved_data.SampleCount;k++){
        int rec_id = saved_data.ReceiverId[k];
        float dx = result.x-refNodes[rec_id].x;
        float dy = result.y-refNodes[rec_id].y;
        float dz = result.z-refNodes[rec_id].z;

        float offset = sqrt(dx*dx+dy*dy+dz*dz) - saved_data.Dist[k];

        std += offset*offset;

        if(offset>max_offset){
            max_pos = k;
            max_offset = offset;
        }
        if(offset<min_offset)
            min_offset = offset;
//#define MY_DEBUG

#ifdef MY_DEBUG
        fprintf(stderr,"%d:%.1f ",rec_id,offset);//for debug
#endif

        if(offset>=offset_thr&&saved_data.Dist[k]<1000){
            if(k!=ValidSmpCount){//move this data to correct position
                saved_data.Dist[ValidSmpCount] = saved_data.Dist[k];
                saved_data.ReceiverId[ValidSmpCount] = saved_data.ReceiverId[k];
                saved_data.Dist[k] = INVALIDVALUE;
            }
            ValidSmpCount++;
        }
    }

#ifdef MY_DEBUG
    fprintf(stderr,"\n---std is %.4f\n",std);
#endif

    if(0){//find the best one
        float std_min = std;
        float std_max = std;
        float ax = result.x,ay= result.y,az=result.z;
        int i,j,k;
        fprintf(stderr,"pos_result:%.1f,%.1f,%.1f\n",ax,ay,az);
        for(i=result.x-30;i<=result.x+30;i+=1){
            for(j=result.y-30;j<=result.y+30;j+=1){
                for(k=result.z-30;k<=result.z+30;k+=1){
                    int m;
                    float std_tp = 0;
                    for(m=0;m<saved_data.SampleCount;m++){
                        int rec_id = saved_data.ReceiverId[m];
                        float dx = i-refNodes[rec_id].x;
                        float dy = j-refNodes[rec_id].y;
                        float dz = k-refNodes[rec_id].z;
                        float offset = sqrt(dx*dx+dy*dy+dz*dz) - saved_data.Dist[m];

                        std_tp += offset*offset;
                    }
                    if(std_tp<std_min){
                        std_min = std_tp;
                        ax = i;
                        ay = j;
                        az = k;
                    }
                    if(std_tp>std_max)
                        std_max= std_tp;
                }
            }
        }
        //if(fabs(ax-result.x)+fabs(ay-result.y)+fabs(az-result.z)>1){
        if(fabs(std_min-std)>5){
            fprintf(stderr,"---after adjust:min_std is %.4f,std_max is %.1f,pos is %.1f,%.1f,%.1f\n",
                    std_min,std_max,ax,ay,az);
        }
    }

#ifdef MY_DEBUG
    fprintf(stderr,"\nbef_nodes:%2d,now_nodes:%2d \nmax_offset:%.1f,min_offset:%.1f\n",saved_data.SampleCount,ValidSmpCount,max_offset,min_offset);//for debug
#endif

    if(0)
    {
        int output = 0;
        for(k=1;k<ValidSmpCount;k++){
            if(saved_data.Dist[k]>1000){
                output = 1;
                fprintf(stderr,"---------error with dist is %.2lf--------\n",saved_data.Dist[k]);
            }
        }
        if(output){
            for(k=1;k<ValidSmpCount;k++){
                fprintf(stderr,"%d:%.2lf ",saved_data.ReceiverId[k],saved_data.Dist[k]);
            }
            fprintf(stderr,"\n");
        }
#ifdef DEBUG
    fprintf(stderr,"\n");
#endif
    }
#endif

    ret = (saved_data.SampleCount == ValidSmpCount);
    saved_data.SampleCount = ValidSmpCount;

    /*
    if(max_pos!=-1&&max_pos!=ValidSmpCount){//move the most reliable point to end for lsq
        double tp = saved_data.Dist[max_pos];

        saved_data.Dist[max_pos] = saved_data.Dist[ValidSmpCount-1];
        saved_data.Dist[ValidSmpCount-1] = tp;

        tp = saved_data.ReceiverId[max_pos];
        saved_data.ReceiverId[max_pos] = saved_data.ReceiverId[ValidSmpCount-1];
        saved_data.ReceiverId[ValidSmpCount-1] = tp;
    }
    */

#ifdef MY_DEBUG
    fprintf(stderr,"pos_now:%.3lf %.3lf %.3lf\n",result.x,result.y,result.z);
#endif
    return (!ret&&ValidSmpCount>3);
}

