#include "scitos_docking/CTransformation.h"
#include <stdio.h>
#include <stdint.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_complex_math.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_linalg.h>


int sortByDistance(const void* m1,const void* m2)
{
        if (((STrackedObject*)m1)->d > ((STrackedObject*)m2)->d) return -1;
        if (((STrackedObject*)m1)->d < ((STrackedObject*)m2)->d) return 1;
        return 0;
}

void CTransformation::clearOffsets()
{
	ownOffset.x = ownOffset.y = ownOffset.z = 0;
	dockOffset.x = dockOffset.y = dockOffset.z = 0;
}

CTransformation::CTransformation(float diam, ros::NodeHandle *n)
{
	nh = n;
	clearOffsets();
	char dummy[1000];
	trackedObjectDiameter = diam;
	cc[0] = 320;
	cc[1] = 240; 
	fc[0] = fc[1] = 570;
	memset(kc,0,sizeof(float)*6);
	kc[0] = 1.0;
	sem_init(&trfparamsem,0,1);
}

void CTransformation::updateParams(float a,float b, float c, float d)
{
	if (a != cc[0] || cc[1] != b || kc[0] != c ||kc[1] != d){
		sem_wait(&trfparamsem);
		cc[0] = a;
		cc[1] = b;
		fc[0] = c;
		fc[1] = d;
		sem_post(&trfparamsem);
	}
}

CTransformation::~CTransformation()
{
}

float CTransformation::barrelX(float x,float y)
{
	x = (x-cc[0])/fc[0];
	y = (y-cc[1])/fc[1];
	float cx,dx;
	float r = x*x+y*y;
	dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
	cx = (1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r)*x+dx;
	cx = cx*fc[0]+cc[0];
	return cx;
}

float CTransformation::barrelY(float x,float y)
{
	x = (x-cc[0])/fc[0];
	y = (y-cc[1])/fc[1];
	float cy,dy;
	float r = x*x+y*y;
	dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
	cy = (1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r)*y+dy;
	cy = cy*fc[1]+cc[1];
	return cy;
}

float CTransformation::unbarrelX(float x,float y)
{
	float ix,iy,dx,dy,r,rad;
	ix = x = (x-cc[0])/fc[0];
	iy = y = (y-cc[1])/fc[1];
	for (int i= 0;i<5;i++){
		r = x*x+y*y;
		dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
		dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
		rad = 1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r;
		x = (ix-dx)/rad;
		y = (iy-dy)/rad;
	}
	return (x*fc[0]+cc[0]);
}

float CTransformation::unbarrelY(float x,float y)
{
	float ix,iy,dx,dy,r,rad;
	ix = x = (x-cc[0])/fc[0];
	iy = y = (y-cc[1])/fc[1];
	for (int i= 0;i<5;i++){
		r = x*x+y*y;
		dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
		dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
		rad = 1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r;
		x = (ix-dx)/rad;
		y = (iy-dy)/rad;
	}
	return (y*fc[1]+cc[1]);
}

void CTransformation::transformXYerr(float *ax,float *ay)
{
	float x,y,dx,dy,r,rad;
	//*ax = x = (*ax-cc[0])/fc[0];
	//*ay = y = (*ay-cc[1])/fc[1];
	x = *ax;
	y = *ay;
	r = x*x+y*y;
	dx = 2*kcerr[3]*x*y + kcerr[4]*(r + 2*x*x);
	dy = 2*kcerr[4]*x*y + kcerr[3]*(r + 2*y*y);
	rad = kcerr[1]*r+kcerr[2]*r*r+kcerr[5]*r*r*r;
	*ax=rad*x+dx;
	*ay=rad*y+dy;
}

void CTransformation::transformXY(float *ax,float *ay)
{
	float x,y,ix,iy,dx,dy,r,rad;
	*ax = ix = x = (*ax-cc[0])/fc[0];
	*ay = iy = y = (*ay-cc[1])/fc[1];
	for (int i= 0;i<5;i++){
		r = x*x+y*y;
		dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
		dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
		rad = 1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r;
		x = (ix-dx)/rad;
		y = (iy-dy)/rad;
	}
	*ax=x;
	*ay=y;
}

float CTransformation::transformX(float xc,float yc)
{
	return (unbarrelX(xc,yc)-cc[0])/fc[0];
}

float CTransformation::transformY(float xc,float yc)
{
	return (unbarrelY(xc,yc)-cc[1])/fc[1];
}

float CTransformation::distance(STrackedObject o1,STrackedObject o2)
{
	return sqrt((o1.x-o2.x)*(o1.x-o2.x)+(o1.y-o2.y)*(o1.y-o2.y)+(o1.z-o2.z)*(o1.z-o2.z));
}

STrackedObject CTransformation::getDock(STrackedObject o[])
{
	STrackedObject result;
	result.valid = false;
	if (o[0].valid && o[1].valid && o[2].valid){ 
		STrackedObject trk[4];
		for (int i=0;i<3;i++) trk[i] = o[i];
		for (int i=0;i<3;i++) trk[i].d = distance(trk[(i+1)%3],trk[(i+2)%3]);
		qsort(trk,3,sizeof(STrackedObject),sortByDistance);
		float an = atan2(trk[0].x-trk[2].x,trk[0].y-trk[2].y);
		trk[0].roll = 180*an/M_PI;
		trk[0].x-=dockOffset.x;
		trk[0].y-=dockOffset.y;
		trk[0].z-=dockOffset.z;
		result=trk[0];
		result.valid = ((trk[0].d+trk[1].d+trk[2].d) < 1.0); //only for small station
		//fprintf(stdout,"Dockbase: %.3f \n",trk[0].d+trk[1].d+trk[2].d);
		//for (int i = 0;i<3;i++) fprintf(stdout,"Dock: %i %.3f %.3f %.3f\n",i,trk[i].x,trk[i].y,trk[i].z);
	}
	return result;
}

STrackedObject CTransformation::getOwnPosition(STrackedObject o[])
{
	STrackedObject trk[4];
	for (int i=0;i<3;i++) trk[i] = o[i];
	for (int i=0;i<3;i++) trk[i].d = distance(trk[(i+1)%3],trk[(i+2)%3]);
	qsort(trk,3,sizeof(STrackedObject),sortByDistance);
//	for (int i = 0;i<3;i++) fprintf(stdout,"Dock position %i: %.3f %.3f %.3f %.3f\n",i,trk[i].x,trk[i].y,trk[i].z,trk[i].d);
//	for (int i=0;i<3;i++) fprintf(stdout,"%.3f ",trk[i].d);
	//fprintf(stdout,"%.3f \n",sqrt(trk[1].d*trk[1].d+trk[2].d*trk[2].d));
	D3transform[0] =  calibrate3D(trk[0],trk[2],trk[1],0.12,0.09);
	trk[3].x = trk[3].y = trk[3].z =  0;
	trk[3] = transform3D(trk[3],1);
	trk[3].x -= ownOffset.x;
	trk[3].y -= ownOffset.y;
	trk[3].z -= ownOffset.z;
	trk[3].yaw = atan2(trk[2].x-trk[0].x,trk[2].y-trk[0].y);
//	fprintf(stdout,"Robot: %.3f %.3f %.3f %.3f\n",trk[3].x,trk[3].y,trk[3].z,trk[3].yaw*180.0/M_PI);
	return trk[3];
}

STrackedObject CTransformation::transform3D(STrackedObject o,int num)
{
	STrackedObject result[4];
	STrackedObject final;
	STrackedObject a;
	final.x = final.y = final.z = 0;
	float str = 0;
	float strAll = 0;
	for (int k = 0;k<1;k++){
		a.x = o.x-D3transform[k].orig.x;
		a.y = o.y-D3transform[k].orig.y;
		a.z = o.z-D3transform[k].orig.z;
		result[k].x = D3transform[k].simlar[0][0]*a.x+D3transform[k].simlar[0][1]*a.y+D3transform[k].simlar[0][2]*a.z;
		result[k].y = D3transform[k].simlar[1][0]*a.x+D3transform[k].simlar[1][1]*a.y+D3transform[k].simlar[1][2]*a.z;
		result[k].z = D3transform[k].simlar[2][0]*a.x+D3transform[k].simlar[2][1]*a.y+D3transform[k].simlar[2][2]*a.z;
		//result[k].x = (k%2)*gDimX+(1-(k%2)*2)*result[k].x;
		//result[k].y = (k/2)*gDimY+(1-(k/2)*2)*result[k].y;
		//if (k ==0 || k == 3) result[k].z = -result[k].z;
		//result.y = +result.y+(D3transform[k].orig.y-D3transform[0].orig.y);
		//result.z = -result.z+(D3transform[k].orig.z-D3transform[0].orig.z);
		str=1.0;

		final.x += str*result[k].x;
		final.y += str*result[k].y;
		final.z += str*result[k].z;
		strAll +=str;
		//printf("UUU: %f %f %f %f %f\n",result[k].x,result[k].y,result[k].z,str,establishError(result[k]));
	}
	final.x=final.x/strAll;
	final.y=final.y/strAll;	
	final.z=final.z/strAll;	
 	
	float x,y,z;
	/*final.esterror = 0;
	for (int k = 0;k<num;k++){
		x = final.x-result[k].x;
		y = final.y-result[k].y;
		z = final.z-result[k].z,
		final.esterror+=sqrt(x*x+y*y+z*z);
	}
	float xerr0 = -o.z/o.x;
	float yerr0 = -o.y/o.x;
	transformXYerr(&xerr0,&yerr0);
	final.esterror= sqrt(xerr0*xerr0+yerr0*yerr0)+fcerr[0]/fc[0]+fcerr[1]/fc[1];
	final.esterror= (fcerr[0]/fc[0]+fcerr[1]/fc[1])/2;
	final.esterror = 100*final.esterror;
	
	//final.esterror = final.esterror/num;
	final.error = establishError(final);*/
	return final;
}

void CTransformation::updateCalibration(STrackedObject own,STrackedObject station)
{
	ownOffset = own;
	dockOffset = station;
	nh->setParam("/charging/ownOffsetX", ownOffset.x);
	nh->setParam("/charging/ownOffsetY", ownOffset.y);
	nh->setParam("/charging/ownOffsetZ", ownOffset.z);
	nh->setParam("/charging/dockOffsetX", dockOffset.x);
	nh->setParam("/charging/dockOffsetY", dockOffset.y);
	nh->setParam("/charging/dockOffsetZ", dockOffset.z);

}

bool CTransformation::saveParamInDB(const char *param)
{
	ros::ServiceClient client = nh->serviceClient<mongodb_store::SetParam>("/config_manager/save_param");
	mongodb_store::SetParam srv;
	ROS_INFO("Requesting update for %s", param);
	srv.request.param = param;
	if (client.call(srv))
	{
		//ROS_INFO("%s Param updated on MongoDB", param);
		return true;
	}
	else
	{
		//ROS_ERROR("Failed to call service save_param");
		return false;
	}
}


bool CTransformation::loadCalibration()
{
	bool calibrated = true;
	calibrated = calibrated && nh->getParam("/charging/ownOffsetX", ownOffset.x);
	calibrated = calibrated && nh->getParam("/charging/ownOffsetY", ownOffset.y);
	calibrated = calibrated && nh->getParam("/charging/ownOffsetZ", ownOffset.z);
	calibrated = calibrated && nh->getParam("/charging/dockOffsetX", dockOffset.x);
	calibrated = calibrated && nh->getParam("/charging/dockOffsetY", dockOffset.y);
	calibrated = calibrated && nh->getParam("/charging/dockOffsetZ", dockOffset.z);
	if (calibrated){
		 ROS_INFO("Calibration parameters updated from rosParam");
		 return true;
	}

	std::string configFilename = "";
	if (nh->getParam("configFile", configFilename) == false){
		ROS_WARN("Config file not set, calibration parameters will not be loaded.");
		return false;
	}else{
		FILE* file = fopen(configFilename.c_str(),"r");
		if (file == NULL){
			ROS_WARN("Calibration file %s could not be loaded and the calibration parameters are not set.",configFilename.c_str());
			ROS_WARN("Place the robot on the charging station and run calibration");
			return false;
		}
		fscanf(file,"charging:\n");
		fscanf(file," dockOffsetX: %lf\n",&dockOffset.x);
		fscanf(file," dockOffsetY: %lf\n",&dockOffset.y);
		fscanf(file," dockOffsetZ: %lf\n",&dockOffset.z);
		fscanf(file," ownOffsetX: %lf\n",&ownOffset.x);
		fscanf(file," ownOffsetY: %lf\n",&ownOffset.y);
		fscanf(file," ownOffsetZ: %lf\n",&ownOffset.z);
		fclose(file);
		ROS_WARN("Calibration parameters loaded from: %s",configFilename.c_str());
		updateCalibration(ownOffset,dockOffset);
	}
	return true;
}

bool CTransformation::saveCalibration()
{
	std::string configFilename = "";
	bool savedOK = true;

	savedOK = savedOK && saveParamInDB("/charging/ownOffsetX");
	savedOK = savedOK && saveParamInDB("/charging/ownOffsetY");
	savedOK = savedOK && saveParamInDB("/charging/ownOffsetZ");
	savedOK = savedOK && saveParamInDB("/charging/dockOffsetX");
	savedOK = savedOK && saveParamInDB("/charging/dockOffsetY");
	savedOK = savedOK && saveParamInDB("/charging/dockOffsetZ");
	if (savedOK == false) ROS_WARN("Calibration parameters could not be saved in the datacentre.");
	if (nh->getParam("configFile", configFilename) == false){
		ROS_WARN("Config file not set, calibration parameters will not be saved.");
		return false;
	}else{
		FILE* file = fopen(configFilename.c_str(),"w");
		if (file == NULL){
			ROS_WARN("Calibration file %s could not be saved.",configFilename.c_str());
			return false;
		}
		fprintf(file,"charging:\n");
		fprintf(file," dockOffsetX: %lf\n",dockOffset.x);
		fprintf(file," dockOffsetY: %lf\n",dockOffset.y);
		fprintf(file," dockOffsetZ: %lf\n",dockOffset.z);
		fprintf(file," ownOffsetX: %lf\n",ownOffset.x);
		fprintf(file," ownOffsetY: %lf\n",ownOffset.y);
		fprintf(file," ownOffsetZ: %lf\n",ownOffset.z);
		fclose(file);
		ROS_INFO("Calibration parameters saved to: %s",configFilename.c_str());
		return true;
	}
	return savedOK;
}

STrackedObject CTransformation::normalize(STrackedObject o)
{
	float scale = sqrt(o.x*o.x+o.y*o.y+o.z*o.z);
	STrackedObject r;
	r.x = o.x/scale;
	r.y = o.y/scale;
	r.z = o.z/scale;
	return r;
}

int CTransformation::calibrate2D(STrackedObject *inp,float dimX,float dimY)
{
	STrackedObject r[4];
	STrackedObject o[4];

	for (int i = 0;i<4;i++){
		r[i].x = (i%2)*dimX;
		r[i].y = (i/2)*dimY;
		o[i].x = -inp[i].y/inp[i].x;
		o[i].y = -inp[i].z/inp[i].x;
	}

	float xerr,yerr;
	float err = 0;
	for (int i = 0;i<4;i++){
		xerr = o[i].x;
		yerr = o[i].y;
		transformXYerr(&xerr,&yerr);
		err += sqrt(xerr*xerr+yerr*yerr);
	}
	error2D = err/4;

	int sign =0;
	gsl_matrix *est = gsl_matrix_alloc(8,8);
	gsl_vector *v = gsl_vector_alloc(8);
	gsl_vector *a = gsl_vector_alloc(8);
	gsl_permutation *dmy = gsl_permutation_alloc(8);

	for (int i = 0;i<4;i++){
		gsl_matrix_set(est,2*i,0,-o[i].x);
		gsl_matrix_set(est,2*i,1,-o[i].y);
		gsl_matrix_set(est,2*i,2,-1);
		gsl_matrix_set(est,2*i,3,0);
		gsl_matrix_set(est,2*i,4,0);
		gsl_matrix_set(est,2*i,5,0);
		gsl_matrix_set(est,2*i,6,r[i].x*o[i].x);
		gsl_matrix_set(est,2*i,7,r[i].x*o[i].y);
		gsl_matrix_set(est,2*i+1,0,0);
		gsl_matrix_set(est,2*i+1,1,0);
		gsl_matrix_set(est,2*i+1,2,0);
		gsl_matrix_set(est,2*i+1,3,-o[i].x);
		gsl_matrix_set(est,2*i+1,4,-o[i].y);
		gsl_matrix_set(est,2*i+1,5,-1);
		gsl_matrix_set(est,2*i+1,6,r[i].y*o[i].x);
		gsl_matrix_set(est,2*i+1,7,r[i].y*o[i].y);
		gsl_vector_set(v,2*i,-r[i].x);
		gsl_vector_set(v,2*i+1,-r[i].y);
	}
	gsl_linalg_LU_decomp(est,dmy,&sign);
	gsl_linalg_LU_solve (est, dmy, v, a);
	for (int i = 0;i<8;i++)  hom[i] = gsl_vector_get(a,i);
	hom[8] = 1;
	gsl_permutation_free (dmy);
	gsl_matrix_free(est);
	gsl_vector_free(a);
	gsl_vector_free(v);
	transformType = TRANSFORM_2D;

	return 0;
}

STrackedObject CTransformation::crossPrd(STrackedObject o0,STrackedObject o1,STrackedObject o2,float gridDimX,float gridDimY)
{
	STrackedObject v[3];
	v[0].x = o1.x-o0.x;
	v[0].y = o1.y-o0.y;
	v[0].z = o1.z-o0.z;
	v[0] = normalize(v[0]);

	v[1].x = o2.x-o0.x;
	v[1].y = o2.y-o0.y;
	v[1].z = o2.z-o0.z;
	v[1] = normalize(v[1]);

	v[2].x = +v[0].y*v[1].z-v[1].y*v[0].z+o0.x;
	v[2].y = +v[0].z*v[1].x-v[1].z*v[0].x+o0.y;
	v[2].z = +v[0].x*v[1].y-v[1].x*v[0].y+o0.z;
	return v[2];
}

int CTransformation::calibrate4D(STrackedObject o[],float gridDimX,float gridDimY)
{
	STrackedObject v[8];
	STrackedObject r[8];
	for ( int i =0;i<4;i++)v[i] = o[i];
	v[4] = crossPrd(o[0],o[1],o[2],gridDimX,gridDimY);
	v[5] = crossPrd(o[1],o[3],o[0],gridDimX,gridDimY);
	v[6] = crossPrd(o[2],o[0],o[3],gridDimX,gridDimY);
	v[7] = crossPrd(o[3],o[2],o[1],gridDimX,gridDimY);
	//for ( int i =0;i<8;i++) printf("%f %f %f\n",v[i].x,v[i].y,v[i].z);
	
	for (int i = 0;i<2;i++){
		r[4*i+0].x = 0;
		r[4*i+0].y = 0;
		r[4*i+0].z = i;
		r[4*i+1].x = gridDimX;
		r[4*i+1].y = 0;
		r[4*i+1].z = i;
		r[4*i+2].x = 0;
		r[4*i+2].y = gridDimY;
		r[4*i+2].z = i;
		r[4*i+3].x = gridDimX;
		r[4*i+3].y = gridDimY;
		r[4*i+3].z = i;
	}

        gsl_matrix* est = gsl_matrix_alloc(24, 16);
	gsl_matrix* V = gsl_matrix_alloc(16,16);
        gsl_vector* S = gsl_vector_alloc(16);
	gsl_vector* work=gsl_vector_alloc(16);

	for (int i = 0;i<24;i++){
		for (int j = 0;j<16;j++){
			gsl_matrix_set(est,i,j,0);
		}
	}

	for (int i = 0;i<8;i++){
		gsl_matrix_set(est,3*i,0,-v[i].x);
		gsl_matrix_set(est,3*i,1,-v[i].y);
		gsl_matrix_set(est,3*i,2,-v[i].z);
		gsl_matrix_set(est,3*i,3,-1);
		gsl_matrix_set(est,3*i,12,r[i].x*v[i].x);
		gsl_matrix_set(est,3*i,13,r[i].x*v[i].y);
		gsl_matrix_set(est,3*i,14,r[i].x*v[i].z);
		gsl_matrix_set(est,3*i,15,r[i].x);

		gsl_matrix_set(est,3*i+1,4,-v[i].x);
		gsl_matrix_set(est,3*i+1,5,-v[i].y);
		gsl_matrix_set(est,3*i+1,6,-v[i].z);
		gsl_matrix_set(est,3*i+1,7,-1);
		gsl_matrix_set(est,3*i+1,12,r[i].y*v[i].x);
		gsl_matrix_set(est,3*i+1,13,r[i].y*v[i].y);
		gsl_matrix_set(est,3*i+1,14,r[i].y*v[i].z);
		gsl_matrix_set(est,3*i+1,15,r[i].y);

		gsl_matrix_set(est,3*i+2,8,-v[i].x);
		gsl_matrix_set(est,3*i+2,9,-v[i].y);
		gsl_matrix_set(est,3*i+2,10,-v[i].z);
		gsl_matrix_set(est,3*i+2,11,-1);
		gsl_matrix_set(est,3*i+2,12,r[i].z*v[i].x);
		gsl_matrix_set(est,3*i+2,13,r[i].z*v[i].y);
		gsl_matrix_set(est,3*i+2,14,r[i].z*v[i].z);
		gsl_matrix_set(est,3*i+2,15,r[i].z);
	}
	gsl_linalg_SV_decomp(est, V, S, work);
	for (int i = 0;i<16;i++){
		  trf4D[i] = gsl_matrix_get(V,i,15);
	}

        gsl_matrix_free(est);
        gsl_matrix_free(V);
        gsl_vector_free(S);
        gsl_vector_free(work);
	return 0;
}

S3DTransform CTransformation::calibrate3D(STrackedObject o0,STrackedObject o1,STrackedObject o2,float gridDimX,float gridDimY)
{
	S3DTransform result;
	STrackedObject v[3];
	int sign = 0;
	gsl_matrix *m23D = gsl_matrix_alloc(3,3);
	gsl_matrix *inv = gsl_matrix_alloc(3,3);
	gsl_permutation *dmy = gsl_permutation_alloc(3);
	result.orig = o0;

	v[0].x = o1.x-o0.x;
	v[0].y = o1.y-o0.y;
	v[0].z = o1.z-o0.z;
	//v[0] = normalize(v[0]);

	v[1].x = o2.x-o0.x;
	v[1].y = o2.y-o0.y;
	v[1].z = o2.z-o0.z;
	//v[1] = normalize(v[1]);

	v[2].x = +v[0].y*v[1].z-v[1].y*v[0].z;
	v[2].y = +v[0].z*v[1].x-v[1].z*v[0].x;
	v[2].z = +v[0].x*v[1].y-v[1].x*v[0].y;
	//v[2] = normalize(v[2]);

	for (int i = 0;i<3;i++){
		gsl_matrix_set(m23D,0,i,v[i].x);
		gsl_matrix_set(m23D,1,i,v[i].y);
		gsl_matrix_set(m23D,2,i,v[i].z);
	}
	gsl_linalg_LU_decomp(m23D,dmy,&sign);
	gsl_linalg_LU_invert(m23D,dmy,inv);
	for (int i = 0;i<3;i++){
		result.simlar[0][i] = gsl_matrix_get(inv,0,i)*gridDimX;
		result.simlar[1][i] = gsl_matrix_get(inv,1,i)*gridDimY;
		result.simlar[2][i] = gsl_matrix_get(inv,2,i)*gridDimX*gridDimY;
	}
	gsl_permutation_free (dmy);
	gsl_matrix_free(m23D);
	gsl_matrix_free(inv);
	return result;
}

//implemented according to   
STrackedObject CTransformation::eigen(double data[])
{
	STrackedObject result;
	result.error = 0;
	gsl_matrix_view m = gsl_matrix_view_array (data, 3, 3);
	gsl_vector *eval = gsl_vector_alloc (3);
	gsl_matrix *evec = gsl_matrix_alloc (3, 3);
	//
	gsl_eigen_symmv_workspace * w = gsl_eigen_symmv_alloc (3);
	gsl_eigen_symmv (&m.matrix, eval, evec, w);
	gsl_eigen_symmv_free (w);
	gsl_eigen_symmv_sort (eval, evec,GSL_EIGEN_SORT_ABS_ASC);

	//eigenvalues
	float L1 = gsl_vector_get(eval,1); 
	float L2 = gsl_vector_get(eval,2);
	float L3 = gsl_vector_get(eval,0);
	//eigenvectors
	int V2=2;
	int V3=0;

	//detected pattern position
	float z = trackedObjectDiameter/sqrt(-L2*L3)/2.0;
	float c0 =  sqrt((L2-L1)/(L2-L3));
	float c0x = c0*gsl_matrix_get(evec,0,V2);
	float c0y = c0*gsl_matrix_get(evec,1,V2);
	float c0z = c0*gsl_matrix_get(evec,2,V2);
	float c1 =  sqrt((L1-L3)/(L2-L3));
	float c1x = c1*gsl_matrix_get(evec,0,V3);
	float c1y = c1*gsl_matrix_get(evec,1,V3);
	float c1z = c1*gsl_matrix_get(evec,2,V3);

	float z0 = -L3*c0x+L2*c1x;
	float z1 = -L3*c0y+L2*c1y;
	float z2 = -L3*c0z+L2*c1z;
	float s1,s2;
	s1=s2=1;
	float n0 = +s1*c0x+s2*c1x;
	float n1 = +s1*c0y+s2*c1y;
	float n2 = +s1*c0z+s2*c1z;

	//n0 = -L3*c0x-L2*c1x;
	//n1 = -L3*c0y-L2*c1y;
	//n2 = -L3*c0z-L2*c1z;
	
	//rotate the vector accordingly
	if (z2*z < 0){
		 z2 = -z2;
		 z1 = -z1;
		 z0 = -z0;
	//	 n0 = -n0;
	//	 n1 = -n1;
	//	 n2 = -n2;
	}
	result.x = z2*z;	
	result.y = -z0*z;	
	result.z = -z1*z;
	result.pitch = n0;//cos(segment.m1/segment.m0)/M_PI*180.0;
	result.roll = n1;//atan2(segment.v1,segment.v0)/M_PI*180.0;
	result.yaw = n2;//segment.v1/segment.v0;
	//result.roll = n2*z;	
	//result.pitch = -n0*z;	
	//result.yaw = -n1*z;

	gsl_vector_free (eval);
	gsl_matrix_free (evec);

	return result;
}

STrackedObject CTransformation::transform(SSegment segment)
{
	float x,y,x1,x2,y1,y2,major,minor,v0,v1;
	STrackedObject result;
	//Transform to the Canonical camera coordinates
	x = segment.x;
	y = segment.y;
	sem_wait(&trfparamsem);
	transformXY(&x,&y);
	//major axis
	//vertices in image coords
	x1 = segment.x+segment.v0*segment.m0*2;
	x2 = segment.x-segment.v0*segment.m0*2;
	y1 = segment.y+segment.v1*segment.m0*2;
	y2 = segment.y-segment.v1*segment.m0*2;
	//vertices in canonical camera coords 
	transformXY(&x1,&y1);
	transformXY(&x2,&y2);
	//semiaxes length 
	major = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0;
	v0 = (x2-x1)/major/2.0;
	v1 = (y2-y1)/major/2.0;
	//printf("AAA: %f %f\n",sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1))-sqrt((x-x2)*(x-x2)+(y-y2)*(y-y2)),major);

	//the minor axis 
	//vertices in image coords
	x1 = segment.x+segment.v1*segment.m1*2;
	x2 = segment.x-segment.v1*segment.m1*2;
	y1 = segment.y-segment.v0*segment.m1*2;
	y2 = segment.y+segment.v0*segment.m1*2;
	//vertices in canonical camera coords 
	transformXY(&x1,&y1);
	transformXY(&x2,&y2);
	sem_post(&trfparamsem);
	//minor axis length 
	minor = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0;
	//printf("BBB: %f %f\n",sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1))-sqrt((x-x2)*(x-x2)+(y-y2)*(y-y2)),minor);

	//Construct the ellipse characteristic equation
	float a,b,c,d,e,f;
	a = v0*v0/(major*major)+v1*v1/(minor*minor);
	b = v0*v1*(1.0/(major*major)-1.0/(minor*minor));
	c = v0*v0/(minor*minor)+v1*v1/(major*major);
	d = (-x*a-b*y);
	e = (-y*c-b*x);
	f = (a*x*x+c*y*y+2*b*x*y-1.0);
	
	double data[] ={a,b,d,b,c,e,d,e,f};
	result = eigen(data);
	result.valid=boost::math::isnormal(result.x) && boost::math::isnormal(result.y) && boost::math::isnormal(result.z);
	return result;
}

