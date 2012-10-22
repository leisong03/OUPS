#ifndef MYMATH
#define MYMATH

#include <math.h>

//const int DIM = 6;

double diffsqr(double a, double b);
double sqr(double a);
void CopyVector(double *dest, double *src, int length);
void CopyMatrix(double **dest, double **src, int row, int col);
void InitVec(double *src, int length, double val);
void InitMat(double **src, int dim, double val);
void IdentityMat(double **m, int dim);
void newU(double **src, int dim, double val);
void setDiagMatrix(double **m, int i, int j, int ct, double v);

// outlier operation
void ClearBuff(int *buf, int length);
void InsertBuff(int *buf, int val, int length);
double Ratio( int *buf, int length);

// matrix operation
void MatTranspose(double **dest, double **src, int dim);
void MatMulVec(double *dest, double **src1, double *src2, int dim);
void Vec2Diag(double **m, double *v, int dim);
void Diag2Vec(double **m, double *v, int dim);
void PlusEqual(double *a, double *b, double p, int length);
#endif
