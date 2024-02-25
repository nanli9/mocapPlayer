#ifdef WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include "motion.h"
#include "blender.h"
#include <string>

int main(int argc, char** argv)
{
    if (argc != 7)
    {
        printf("Animation blend motion capture data.");
        printf("Usage: %s <input skeleton file> <input motion capture file> <interpolation type> <angle representation for interpolation> <N> <output motion capture file>\n", argv[0]);
        printf("  interpolation method:\n");
        printf("    l: linear\n");
        printf("    b: Bezier\n");
        printf("  angle representation for interpolation:\n");
        printf("    e: Euler angles\n");
        printf("    q: quaternions\n");
        printf("  N: number of skipped frames\n");
        printf("Example: %s skeleton.asf motion.amc l e 5 outputMotion.amc\n", argv[0]);
        return -1;
    }

    char* inputSkeletonFile = argv[1];
    char* firstInputMotionCaptureFile = argv[2];
    char* secondInputMotionCaptureFile = argv[3];
    char* TString = argv[4];
    char* blendFlag = argv[5];
    char* outputMotionCaptureFile = argv[6];


    Skeleton* pSkeleton = NULL;	// skeleton as read from an ASF file (input)
    Motion* firstMotion = NULL; // motion as read from an AMC file (input)
    Motion* secondMotion = NULL; // motion as read from an AMC file (input)

    double t = std::stod(TString);
    if (t < 0)
    {
        printf("Error: invalid t value (%.1f).\n", t);
        exit(1);
    }
    printf("N=%.1f\n", t);

    printf("Loading skeleton from %s...\n", inputSkeletonFile);
    try
    {
        pSkeleton = new Skeleton(inputSkeletonFile, MOCAP_SCALE);
    }
    catch (int exceptionCode)
    {
        printf("Error: failed to load skeleton from %s. Code: %d\n", inputSkeletonFile, exceptionCode);
        exit(1);
    }

    printf("Loading input motion from %s...\n", firstInputMotionCaptureFile);
    try
    {
        firstMotion = new Motion(firstInputMotionCaptureFile, MOCAP_SCALE, pSkeleton);
    }
    catch (int exceptionCode)
    {
        printf("Error: failed to load motion from %s. Code: %d\n", firstInputMotionCaptureFile, exceptionCode);
        exit(1);
    }

    printf("Loading input motion from %s...\n", secondInputMotionCaptureFile);
    try
    {
        secondMotion = new Motion(secondInputMotionCaptureFile, MOCAP_SCALE, pSkeleton);
    }
    catch (int exceptionCode)
    {
        printf("Error: failed to load motion from %s. Code: %d\n", secondInputMotionCaptureFile, exceptionCode);
        exit(1);
    }

    pSkeleton->enableAllRotationalDOFs();


    int frameNum = std::min(firstMotion->GetNumFrames(), secondMotion->GetNumFrames());

    Motion* pOutputMotion = new Motion(frameNum, pSkeleton);

    blender* b = new blender();
    b->setAnimation(firstMotion,0.5, 0, MAX_BONES_IN_ASF_FILE);
    b->setAnimation(secondMotion,0.5, 0, MAX_BONES_IN_ASF_FILE);
    b->calcaluteResult(pOutputMotion);

    int forceAllJointsBe3DOF = 1;
    pOutputMotion->writeAMCfile(outputMotionCaptureFile, 0.06, forceAllJointsBe3DOF);

    return 0;
}

