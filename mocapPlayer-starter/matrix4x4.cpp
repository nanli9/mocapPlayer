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






