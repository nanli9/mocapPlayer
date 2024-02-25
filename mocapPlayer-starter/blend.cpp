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


    return 0;
}

