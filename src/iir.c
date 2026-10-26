#include "iir.h"
#define NPOLE 4
#define NZERO 4
REAL acoeff[]={0.4735064529391513,-2.2460436820262153,4.033768392721256,-3.256569310958799,1};
REAL bcoeff[]={1,4,6,4,1};
REAL gain=3432.1118907199498;
REAL xv[]={0,0,0,0,0};
REAL yv[]={0,0,0,0,0};

REAL applyfilter(REAL v)
{
	int i;
	REAL out=0;
	for (i=0; i<NZERO; i++) {xv[i]=xv[i+1];}
	xv[NZERO] = v/gain;
	for (i=0; i<NPOLE; i++) {yv[i]=yv[i+1];}
	for (i=0; i<=NZERO; i++) {out+=xv[i]*bcoeff[i];}
	for (i=0; i<NPOLE; i++) {out-=yv[i]*acoeff[i];}
	yv[NPOLE]=out;
	return out;
}