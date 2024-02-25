#pragma once
#ifndef _BLENDER_H
#define _BLENDER_H


#include "motion.h"
#include "quaternion.h"

class slot
{
public:
	double weight;
	Motion* m;
	double startFrames=0, endFrames=0; 
	double startJoint=0, endJoint = MAX_BONES_IN_ASF_FILE;
	slot() {};
	slot(Motion* m, double weight,int startJoint,int endJoint) 
	{ 
		this->m = m;
		this->weight = weight; 
		this->endFrames = m->GetNumFrames()-1; 
		this->startJoint = startJoint;
		this->endJoint = endJoint;
	};

};



class blender
{

public:
	slot animationSlots[4];
	int animationNum;
	blender();
	void setAnimation(Motion* m, double weight, int startJoint, int endJoint);
	void setPartialAnimation(Motion* m, double weight,int startJoint,int endJoint);
	void calcaluteResult(Motion* pOutputMotion);

	Quaternion<double> Slerp(double t, Quaternion<double>& qStart, Quaternion<double>& qEnd);
	void Euler2Quaternion(double angles[3], Quaternion<double>& q);
	void Quaternion2Euler(Quaternion<double>& q, double angles[3]);

};







#endif