#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      //test
      double Euler[3],euler[3];
      Euler[0] = interpolatedPosture.bone_rotation[0].x();
      Euler[1] = interpolatedPosture.bone_rotation[0].y();
      Euler[2] = interpolatedPosture.bone_rotation[0].z();
      Quaternion < double> q;
      Euler2Quaternion(Euler,q);
      Quaternion2Euler(q, euler);

      //test

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}
double* MatrixMultiplication(double a[9],double b[9])
{
    double result[9];
    result[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
    result[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
    result[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

    result[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
    result[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
    result[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

    result[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
    result[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
    result[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];

    return result;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
  // 
    //convert degree into radius
    for (int i = 0; i < 3; i++)
        angles[i] *= M_PI / 180;
    //euler rotation in XYZ order
    double Rx[9];
    double Ry[9];
    double Rz[9];
    for (int i = 0; i < 9; i++)
    {
        Rx[i] = 0;
        Ry[i] = 0;
        Rz[i] = 0;
    }
    //rotate matrix about X
    {
        Rx[0] = 1;
        Rx[4] = cos(angles[0]);
        Rx[5] = -sin(angles[0]);
        Rx[7] = sin(angles[0]);
        Rx[8] = cos(angles[0]);
    }
    //rotate matrix about Y
    {
        Rx[0] = cos(angles[1]);
        Rx[2] = sin(angles[1]);
        Rx[4] = 1;
        Rx[6] = -sin(angles[1]);
        Rx[8] = cos(angles[1]);
    }
    //rotate matrix about Z
    {
        Rx[0] = cos(angles[2]);
        Rx[1] = -sin(angles[2]);
        Rx[3] = sin(angles[2]);
        Rx[4] = cos(angles[2]);
        Rx[8] = 1;
    }
    R = MatrixMultiplication(MatrixMultiplication(Rz, Ry),Rx);

    //convert it back to degree
    for (int i = 0; i < 3; i++)
        angles[i] *= 180 / M_PI;
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
    //convert degree into radius
    for (int i = 0; i < 3; i++)
        angles[i] *= M_PI / 180;
    Quaternion<double> x = Quaternion<double>(cos(angles[0] / 2),sin(angles[0] / 2),0,0 );
    Quaternion<double> y = Quaternion<double>(cos(angles[1] / 2),0, sin(angles[1] / 2),0 );
    Quaternion<double> z = Quaternion<double>(cos(angles[2] / 2),0,0, sin(angles[2] / 2));
    q = z * y * x;
    //convert it back to degree
    for (int i = 0; i < 3; i++)
        angles[i] *= 180 / M_PI;

}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
    double t0, t1, t2, t3, t4;
    t0 = 2 * (q.Gets()*q.Getx()+q.Gety()* q.Getz());
    t1 = 1 - 2 * (q.Getx()* q.Getx()+ q.Gety()* q.Gety());
    angles[0] = atan2(t0,t1)*180/ M_PI;
    t2 = 2 * (q.Gets()* q.Gety()- q.Getz()* q.Getx());
    if (t2 > 1)
        t2 = 1;
    else if (t2 < -1)
        t2 = -1;
    angles[1] = asin(t2) * 180 / M_PI;
    t3 = 2 * (q.Gets()* q.Getz()+ q.Getx()* q.Gety());
    t4 = 1 - 2 * (q.Gety()* q.Gety()+ q.Getz()* q.Getz());
    angles[2] = atan2(t3, t4) * 180 / M_PI;
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
  Quaternion<double> result;
  Quaternion<double> qStartAlternative = -1 * qStart;
  double delta = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety()+ qStart.Getz() * qEnd_.Getz();
  double beta = qStartAlternative.Gets() * qEnd_.Gets() + qStartAlternative.Getx() * qEnd_.Getx() + qStartAlternative.Gety() * qEnd_.Gety()+ qStartAlternative.Getz() * qEnd_.Getz();
  double theta = acos(delta);
  double alpha = acos(beta);
  if (theta > alpha)
      qStart = qStartAlternative;
  if (theta < 0.01 || alpha<0.01)
  {
      //theta is small then we do LERP
      result = (1 - t) * qStart + t * qEnd_;
      result.Normalize();
  }
  else
  {
      result = (sin((1 - t) * theta) / sin(theta)) * qStart + (sin(theta*t)/sin(theta))* qEnd_;
  }
  return result;
}
//generate four points for DeCasteljau Quaternion
Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
  vector q0 = LERPEuler(t,p0,p1);
  vector q1 = LERPEuler(t,p1,p2);
  vector q2 = LERPEuler(t,p2,p3);
  vector r0 = LERPEuler(t,q0,q1);
  vector r1 = LERPEuler(t,q1,q2);
  result = LERPEuler(t, r0, r1);
  return result;
}
vector LERPEuler(double t, vector p0, vector p1)
{
    vector result;
    result = p0 * (1 - t) + p1 * t;
    return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
  Quaternion<double> q0 = Slerp(t, p0, p1);
  Quaternion<double> q1 = Slerp(t, p1, p2);
  Quaternion<double> q2 = Slerp(t, p2, p3);
  Quaternion<double> r0 = Slerp(t, q0, q1);
  Quaternion<double> r1 = Slerp(t, q1, q2);
  result = Slerp(t, r0, r1);
  return result;
}

