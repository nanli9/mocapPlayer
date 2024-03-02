/*

Revision 1 - Steve Lin, Jan. 14, 2002
Revision 2 - Alla and Kiran, Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao, Feb, 2012

*/
#include <cmath>
#include <cstdio>
#include "transform.h"
#include "types.h"
#include "Matrix4x4.h"

Matrix4x4 operator-(Matrix4x4 const& a, Matrix4x4 const& b )
{
	Matrix4x4 c;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
			c.p[i][j] = a.p[i][j] - b.p[i][j];
	}
	return c;
}
Matrix4x4 operator-(Matrix4x4 const& a)
{
	Matrix4x4 c;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
			c.p[i][j] = -c.p[i][j];
	}
	return c;
}
Matrix4x4 operator+(Matrix4x4 const& a, Matrix4x4 const& b )
{
	Matrix4x4 c;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
			c.p[i][j] = a.p[i][j] + b.p[i][j];
	}
	return c;
}

Matrix4x4 operator/(Matrix4x4 const& a, double b )
{
	Matrix4x4 c;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
			c.p[i][j] = a.p[i][j] / b ;
	}
	return c;
}

//multiply
Matrix4x4 operator*(Matrix4x4 const& a, double b )
{
	Matrix4x4 c;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
			c.p[i][j] = a.p[i][j] * b;
	}
	return c;
}
Matrix4x4 operator*(Matrix4x4 const& a, Matrix4x4 const& b)
{
	Matrix4x4 c;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
			c.p[i][j] = a.p[i][j] * b.p[i][j];
	}
	return c;
}
void Matrix4x4::rotationX(float a)
{
	a = a * M_PI / 180.;
	p[0][0] = 1;       p[0][1] = 0;       p[0][2] = 0;       p[0][3] = 0;
	p[1][0] = 0;       p[1][1] = cos(a);  p[1][2] = -sin(a); p[1][3] = 0;
	p[2][0] = 0;       p[2][1] = sin(a);  p[2][2] = cos(a);  p[2][3] = 0;
	p[3][0] = 0;       p[3][1] = 0;       p[3][2] = 0;       p[3][3] = 1;
}
void Matrix4x4::rotationY(float a)
{
	a = a * M_PI / 180.;
	p[0][0] = cos(a);  p[0][1] = 0;       p[0][2] = sin(a); p[0][3] = 0;
	p[1][0] = 0;       p[1][1] = 1;       p[1][2] = 0;      p[1][3] = 0;
	p[2][0] = -sin(a); p[2][1] = 0;       p[2][2] = cos(a); p[2][3] = 0;
	p[3][0] = 0;       p[3][1] = 0;       p[3][2] = 0;      p[3][3] = 1;
}
void Matrix4x4::rotationZ(float a)
{
	a = a * M_PI / 180.;
	p[0][0] = cos(a); p[0][1] = -sin(a); p[0][2] = 0; p[0][3] = 0;
	p[1][0] = sin(a); p[1][1] = cos(a);  p[1][2] = 0; p[1][3] = 0;
	p[2][0] = 0;      p[2][1] = 0;       p[2][2] = 1; p[2][3] = 0;
	p[3][0] = 0;      p[3][1] = 0;       p[3][2] = 0; p[3][3] = 1;
}





