

#ifndef ELLIPSOID_H
#define ELLIPSOID_H

void ELLIPSOID_Reset(char no_rot);
void ELLIPSOID_AddPoint(float x,float y,float z,int index);
char ELLIPSOID_Fitting(float* Ae,float* x0,float* coef);//return 1 if succeed




#endif



