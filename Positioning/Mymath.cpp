#include "Mymath.h"	  

double diffsqr(double a, double b)
{
	return (a+b)*(a-b);
}

double sqr(double a)
{
	return a*a;
}

void CopyVector(double *dest, double *src, int length)
{
	for(int i=0; i<length; i++)
		*dest++ = *src++;
}

void CopyMatrix(double **dest, double **src, int row, int col)
{
	int i,j;
	for(i=0; i<row; i++)
		for(j=0; j<col; j++)
			dest[i][j] = src[i][j];

}
void InitVec(double *src, int length, double val)
{
	for(int i=0; i<length; i++)
		*src++ = val;
}

void IdentityMat(double **m, int dim)
{
	newU(m, dim, 1.0);
}

void newU(double **src, int dim, double val)
{
	int i, j;
	for(i=0; i<dim; i++)
	{
		for(j=0; j<dim; j++)
		{
			if(i==j)
				src[i][j] = val;
			else
				src[i][j] = 0;
		}
	}
}
void InitMat(double **src, int dim, double val)
{
	int i, j;
	for(i=0; i<dim; i++)
	{
		for(j=0; j<dim; j++)
		{
			src[i][j] = val;
		}
	}

}

void setDiagMatrix(double **m, int i, int j, int ct, double v)
{
	for(int c = 0; c < ct; c++) {
		m[i+c][j+c] = v;
	}
}
void ClearBuff(int *buf, int length)
{
	for (int i=0; i<length; i++)
		*buf++ = 0;
}

void InsertBuff(int *buf, int val, int length)
{
	for(int i=0; i<length-1;i++)
		buf[i] = buf[i+1]; //remove [0]

	buf[length-1] = val;
}

double Ratio( int *buf, int length)
{
	int sum = 0;
	double rad;

	for(int i=0; i<length; i++)
		sum += buf[i];

	rad = sum*1.0/length;
	return rad;
	
}

void MatTranspose(double **dest, double **src, int dim)
{
	//dest = src'
	for (int i=0; i<dim; i++)
		for(int j=0; j<dim; j++)
			dest[i][j] = src[j][i];
}

void MatMulVec(double *dest, double **src1, double *src2, int dim)
{
	// dest = src1*src2
	int i, j;
	for(i=0; i<dim; i++)
	{
		dest[i] = 0;
		for(j=0; j<dim; j++)
			dest[i] += src1[i][j] * src2[j];
	}
}

void Vec2Diag(double **m, double *v, int dim)
{
	int i,j;
	for(i=0; i<dim; i++)
		for(j=0; j<dim; j++)
		{
			if(i == j)
				m[i][j] = v[i];
			else
				m[i][j] = 0;

		}
}

void Diag2Vec(double **m, double *v, int dim)
{
	int i;
	for(i=0; i<dim; i++)
		v[i] = m[i][i];
		
}

void PlusEqual(double *a, double *b, double p, int length)
{
	for(int k=0; k< length; k++)
	{
		a[k] = a[k] + b[k]*p;
	}
}
