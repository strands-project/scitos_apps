/*
 * File name: CTransformation.h
 * Date:      2005/11/07 18:10
 * Author:    
 */

#ifndef __CTRANSFORMATION_H__
#define __CTRANSFORMATION_H__

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "scitos_docking/CCircleDetect.h"
#include <semaphore.h> 
#include "ros/ros.h"
#include "mongodb_store/SetParam.h"

#define MAX_STATIONS 4

typedef enum{
	TRANSFORM_NONE,
	TRANSFORM_2D,
	TRANSFORM_3D,
	TRANSFORM_4D,
	TRANSFORM_INV,
	TRANSFORM_NUMBER
}ETransformType;

typedef struct{
	double x,y,z,d;
	float pitch,roll,yaw;
	float roundness;
	float bwratio;
	float error;
	float esterror;
	int id;
	bool valid;
}STrackedObject;

typedef struct{
	float simlar[3][3];
	STrackedObject orig;
}S3DTransform;

class CTransformation
{
	public:
		CTransformation(float diam,ros::NodeHandle *nh);
		~CTransformation();
		void clearOffsets();

		float barrelX(float x,float y);
		float barrelY(float x,float y);
		float unbarrelX(float x,float y);
		float unbarrelY(float x,float y);
		float transformX(float x,float y);
		void updateParams(float a,float b,float c,float d);
		float transformY(float x,float y);
		void transformXY(float *ix,float *iy);
		void transformXYerr(float *ix,float *iy);

		STrackedObject transform(SSegment segment);
		STrackedObject eigen(double data[]);
		int calibrate2D(STrackedObject *o,float gridDimX,float gridDimY);
		int calibrate3D(STrackedObject *o,float gridDimX,float gridDimY);
		int calibrate4D(STrackedObject *o,float gridDimX,float gridDimY);

		S3DTransform calibrate3D(STrackedObject o0,STrackedObject o1,STrackedObject o2,float gridDimX,float gridDimY);
		STrackedObject crossPrd(STrackedObject o0,STrackedObject o1,STrackedObject o2,float gridDimX,float gridDimY);

		ETransformType transformType;
		bool saveCalibration();
		bool loadCalibration();
		bool updateStationParams(STrackedObject *own,STrackedObject *station);
		float distance(STrackedObject o1,STrackedObject o2);

		STrackedObject getDock(STrackedObject o[]);
		STrackedObject getDockID(STrackedObject o[],SSegment s[],CRawImage *image,CCircleDetect* detector);
		STrackedObject getOwnPosition(STrackedObject o[]);
		STrackedObject ownOffset,dockOffset;
		void updateCalibration(STrackedObject own,STrackedObject station);
		bool saveParamInDB(const char *param);
		float fc[2];
		float cc[2];

	private:
		STrackedObject  normalize(STrackedObject o);
		float establishError(STrackedObject o);
		STrackedObject transform2D(STrackedObject o);
		STrackedObject transform3D(STrackedObject o,int num = 4);
		STrackedObject transform4D(STrackedObject o);
		float *xArray;
		float *yArray;
		float *gArrayX;
		float *gArrayY;
		int *pArray;
		
		float hom[9];
		float trf4D[16];
		float gDimX,gDimY;
		S3DTransform D3transform[4];
		int width,height;
		float trackedObjectDiameter;
		float kc[6];
		float kcerr[6];
		float fcerr[2];
		float error2D;
		STrackedObject c2D[4];
		sem_t trfparamsem;
		ros::NodeHandle *nh;
};

#endif
/* end of CTransformation.h */
