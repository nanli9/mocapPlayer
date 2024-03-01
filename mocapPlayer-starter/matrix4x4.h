/*
skeleton.h

Definition of the skeleton. 

Written by Jehee Lee

Revision 1 - Steve Lin, Jan. 14, 2002
Revision 2 - Alla and Kiran, Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao, Feb, 2012
*/

#ifndef _MATRIX4X4_H
#define _MATRIX4X4_H

#include "vector.h"
class vector4
{
public:
	double p[4];
};



class Matrix4x4
{
  // negation
  friend Matrix4x4    operator-(Matrix4x4 const& );

  // addtion
  friend Matrix4x4    operator+(Matrix4x4 const&, Matrix4x4 const& );

  // subtraction
  friend Matrix4x4    operator-(Matrix4x4 const&, Matrix4x4 const& );

  // scalar Multiplication
  friend Matrix4x4    operator*(Matrix4x4 const&, double );

  // scalar Division
  friend Matrix4x4    operator/(Matrix4x4 const&, double );

  // member functions
public:
  // constructors
	Matrix4x4() 
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
				p[i][j] = 0;
		}
	}
	Matrix4x4(float x[16]) 
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
				p[i][j] = x[i*4+j];
		}
	}
	~Matrix4x4() {};
	float* flat()
	{
		float array[16];
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
				array[4*i+j] = p[i][j];
		}
		return array;
	}
  // inquiry functions
 // double& operator[][](int i,int j) { return p[i][j];}
  //data members
  float p[4][4]; //X, Y, Z components of the vector

};

#endif

