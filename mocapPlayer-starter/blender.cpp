

#include "blender.h"


blender::blender()
{
	animationNum = 0;
	for (int i = 0; i < 4; i++)
		animationSlots[i] = slot();

}

void blender::setAnimation(Motion* m, double weight)
{
	if (animationNum >= 4)
		return;
	animationNum++;




}

void blender::setPartialAnimation(Motion* m, double weight, int startJoint, int endJoint)
{
	if (animationNum >= 4)
		return;
	animationNum++;




}

void blender::calcaluteResult()
{
	if (animationNum > 1)
	{
		//do blend for two animation

		if (animationNum > 2)
		{



			if (animationNum > 3)
			{


			}

		}
	}
	

}