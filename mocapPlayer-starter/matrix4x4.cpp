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
	int i, j, k;
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			c.p[i][j] = 0;
			for (k = 0; k < 4; k++)
				c.p[i][j] += a.p[i][k] * b.p[k][j];
		}
	}
	return c;
}
Matrix4x4 Matrix4x4::rotationX(float a)
{
	Matrix4x4 c;
	a = a * M_PI / 180.;
	c.p[0][0] = 1;       c.p[0][1] = 0;       c.p[0][2] = 0;       c.p[0][3] = 0;
	c.p[1][0] = 0;       c.p[1][1] = cos(a);  c.p[1][2] = -sin(a); c.p[1][3] = 0;
	c.p[2][0] = 0;       c.p[2][1] = sin(a);  c.p[2][2] = cos(a);  c.p[2][3] = 0;
	c.p[3][0] = 0;       c.p[3][1] = 0;       c.p[3][2] = 0;       c.p[3][3] = 1;
	return c;
}
Matrix4x4 Matrix4x4::rotationY(float a)
{
	Matrix4x4 c;
	a = a * M_PI / 180.;
	c.p[0][0] = cos(a);  c.p[0][1] = 0;       c.p[0][2] = sin(a); c.p[0][3] = 0;
	c.p[1][0] = 0;       c.p[1][1] = 1;       c.p[1][2] = 0;      c.p[1][3] = 0;
	c.p[2][0] = -sin(a); c.p[2][1] = 0;       c.p[2][2] = cos(a); c.p[2][3] = 0;
	c.p[3][0] = 0;       c.p[3][1] = 0;       c.p[3][2] = 0;      c.p[3][3] = 1;
	return c;
}
Matrix4x4 Matrix4x4::rotationZ(float a)
{
	Matrix4x4 c;
	a = a * M_PI / 180.;
	c.p[0][0] = cos(a); c.p[0][1] = -sin(a); c.p[0][2] = 0; c.p[0][3] = 0;
	c.p[1][0] = sin(a); c.p[1][1] = cos(a);  c.p[1][2] = 0; c.p[1][3] = 0;
	c.p[2][0] = 0;      c.p[2][1] = 0;       c.p[2][2] = 1; c.p[2][3] = 0;
	c.p[3][0] = 0;      c.p[3][1] = 0;       c.p[3][2] = 0; c.p[3][3] = 1;
	return c;
}





