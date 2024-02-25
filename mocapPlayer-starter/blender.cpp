

#include "blender.h"
#include <math.h>
#include <algorithm> 

blender::blender()
{
	animationNum = 0;
	for (int i = 0; i < 4; i++)
		animationSlots[i] = slot();

}

void blender::setAnimation(Motion* m, double weight,int startJoint, int endJoint)
{
	if (animationNum >= 4)
		return;

	animationSlots[animationNum] = slot(m, weight, startJoint, endJoint);

	animationNum++;
}

void blender::setPartialAnimation(Motion* m, double weight, int startJoint, int endJoint)
{
	if (animationNum >= 4)
		return;

	animationSlots[animationNum] = slot(m, weight, startJoint, endJoint);

	animationNum++;
}
void blender::Euler2Quaternion(double angles[3], Quaternion<double>& q)
{
	// students should implement this
	  //convert degree into radius
	for (int i = 0; i < 3; i++)
		angles[i] *= M_PI / 180;
	Quaternion<double> x = Quaternion<double>(cos(angles[0] / 2), sin(angles[0] / 2), 0, 0);
	Quaternion<double> y = Quaternion<double>(cos(angles[1] / 2), 0, sin(angles[1] / 2), 0);
	Quaternion<double> z = Quaternion<double>(cos(angles[2] / 2), 0, 0, sin(angles[2] / 2));
	q = z * y * x;
	q.Normalize();
	//convert it back to degree
	for (int i = 0; i < 3; i++)
		angles[i] *= 180 / M_PI;

}

void blender::Quaternion2Euler(Quaternion<double>& q, double angles[3])
{
	// students should implement this
	double t0, t1, t2, t3, t4;
	t0 = 2 * (q.Gets() * q.Getx() + q.Gety() * q.Getz());
	t1 = 1 - 2 * (q.Getx() * q.Getx() + q.Gety() * q.Gety());
	angles[0] = atan2(t0, t1) * 180 / M_PI;
	t2 = 2 * (q.Gets() * q.Gety() - q.Getz() * q.Getx());
	if (t2 > 1)
		t2 = 1;
	else if (t2 < -1)
		t2 = -1;
	angles[1] = asin(t2) * 180 / M_PI;
	t3 = 2 * (q.Gets() * q.Getz() + q.Getx() * q.Gety());
	t4 = 1 - 2 * (q.Gety() * q.Gety() + q.Getz() * q.Getz());
	angles[2] = atan2(t3, t4) * 180 / M_PI;
}


Quaternion<double> blender::Slerp(double t, Quaternion<double>& qStart, Quaternion<double>& qEnd_)
{
	if (qStart == qEnd_)
		return qStart;
	// students should implement this
	Quaternion<double> result;
	Quaternion<double> qStartAlternative = -1 * qStart;
	double delta = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();
	double beta = qStartAlternative.Gets() * qEnd_.Gets() + qStartAlternative.Getx() * qEnd_.Getx() + qStartAlternative.Gety() * qEnd_.Gety() + qStartAlternative.Getz() * qEnd_.Getz();
	double theta = acos(delta);
	double alpha = acos(beta);
	
	if (theta > alpha)
	{
		qStart = qStartAlternative;
		theta = alpha;
	}
	if (theta < 0.01)
	{
		//theta is small then we do LERP
		result = (1 - t) * qStart + t * qEnd_;
		result.Normalize();
	}
	else
	{
		result = (sin((1 - t) * theta) / sin(theta)) * qStart + (sin(theta * t) / sin(theta)) * qEnd_;
	}
	return result;
}

void blender::calcaluteResult(Motion* pOutputMotion)
{
	if (animationNum > 1)
	{
		int inputLength = std::min(animationSlots[0].m->GetNumFrames(), animationSlots[1].m->GetNumFrames());
		int startFrame = 0;
		while (startFrame < inputLength)
		{
			//do blend for two animation
			Posture* firstStartPosture = animationSlots[0].m->GetPosture(startFrame);

			Posture* secondStartPosture = animationSlots[1].m->GetPosture(startFrame);

			double weight = animationSlots[0].weight;
			Quaternion<double> result;

			Posture blendPosture;

			blendPosture.root_pos = firstStartPosture->root_pos * (1 - weight) + secondStartPosture->root_pos * weight;

			//go through all the bones
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				//if the blend is partial, assume first slot is always full body
				if (bone< animationSlots[1].startJoint || bone > animationSlots[1].endJoint)
					blendPosture.bone_rotation[bone] = firstStartPosture->bone_rotation[bone];
				else
				{
					Quaternion<double> q_1, q_2;
					double firstAngles[3], secondAngles[3], resultEuler[3];

					firstStartPosture->bone_rotation[bone].getValue(firstAngles);
					secondStartPosture->bone_rotation[bone].getValue(secondAngles);

					Euler2Quaternion(firstAngles, q_1);
					Euler2Quaternion(secondAngles, q_2);

					result = Slerp(weight, q_1, q_2);
					Quaternion2Euler(result, resultEuler);
					blendPosture.bone_rotation[bone] = resultEuler;
				}
				
			}


			//more one animation to blend
			if (animationNum > 2)
			{
				Posture* thirdStartPosture = animationSlots[2].m->GetPosture(startFrame);
				for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
				{
					if (bone< animationSlots[2].startJoint || bone > animationSlots[2].endJoint)
						continue;
					else
					{
						Quaternion<double> q_1, q_2;
						double firstAngles[3], secondAngles[3], resultEuler[3];

						blendPosture.bone_rotation[bone].getValue(firstAngles);
						thirdStartPosture->bone_rotation[bone].getValue(secondAngles);

						Euler2Quaternion(firstAngles, q_1);
						Euler2Quaternion(secondAngles, q_2);

						result = Slerp(weight, q_1, q_2);
						Quaternion2Euler(result, resultEuler);
						blendPosture.bone_rotation[bone] = resultEuler;
					}
					
				}

				if (animationNum > 3)
				{
					Posture* fourthStartPosture = animationSlots[3].m->GetPosture(startFrame);
					for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
					{
						if (bone< animationSlots[3].startJoint || bone > animationSlots[3].endJoint)
							continue;
						else
						{
							Quaternion<double> q_1, q_2;
							double firstAngles[3], secondAngles[3], resultEuler[3];

							blendPosture.bone_rotation[bone].getValue(firstAngles);
							fourthStartPosture->bone_rotation[bone].getValue(secondAngles);

							Euler2Quaternion(firstAngles, q_1);
							Euler2Quaternion(secondAngles, q_2);

							result = Slerp(weight, q_1, q_2);
							Quaternion2Euler(result, resultEuler);
							blendPosture.bone_rotation[bone] = resultEuler;
						}
					}
				}

			}

			pOutputMotion->SetPosture(startFrame, blendPosture);
			startFrame++;
		}
			

		



	}
	

}



