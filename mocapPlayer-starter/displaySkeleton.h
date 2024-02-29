/*
display.h

Display the skeleton, ground plane and other objects.			

Revision 1 - Steve Lin, Jan. 14, 2002
Revision 2 - Alla and Kiran, Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao, Feb, 2012
*/

#ifndef _DISPLAY_SKELETON_H_
#define _DISPLAY_SKELETON_H_

#include <FL/glu.h>
#include "skeleton.h"
#include "motion.h"
#include "Mesh.h"


#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>
#include <assimp/matrix4x4.h>

class DisplaySkeleton 
{

  //member functions
public: 
  enum RenderMode
  {
    BONES_ONLY, BONES_AND_LOCAL_FRAMES
  };
  enum JointColor
  {
    GREEN, RED, BLUE, NUMBER_JOINT_COLORS
  };

  DisplaySkeleton();
  ~DisplaySkeleton();

  //set skeleton for display
  void LoadSkeleton(Skeleton * pSkeleton);
  //set motion for display
  void LoadMotion(Motion * pMotion);

  void LoadMesh(Mesh* pMesh);


  //display the scene (skeleton, ground plane ....)
  void Render(RenderMode renderMode);
  void RenderShadow(double ground[4], double light[4]);

  void SetDisplayedSpotJoint(int jointID) {m_SpotJoint = jointID;}
  int GetDisplayedSpotJoint(void) {return m_SpotJoint;}
  int GetNumSkeletons(void) {return numSkeletons;}
  Skeleton * GetSkeleton(int skeletonIndex);
  Motion * GetSkeletonMotion(int skeletonIndex);

  void Reset(void);
  void DrawMesh(int skelNum);
  int GetNumMesh() { return numMeshes;};
  void GenShader();
protected:
  RenderMode renderMode;
  // Draw a particular bone
  void DrawBone(Bone *ptr, int skelNum);
  //draw mesh

  // Draw the skeleton hierarchy
  void Traverse(Bone *ptr, int skelNum);
  // Model matrix for the shadow
  void SetShadowingModelviewMatrix(double ground[4], double light[4]);
  void DrawSpotJointAxis(void);
  void SetDisplayList(int skeletonID, Bone *bone, GLuint *pBoneList);

  int m_SpotJoint;		//joint whose local coordinate framework is drawn
  int numSkeletons;
  int numMeshes;
  Skeleton *m_pSkeleton[MAX_SKELS];		//pointer to current skeleton
  Motion *m_pMotion[MAX_SKELS];		//pointer to current motion	
  GLuint m_BoneList[MAX_SKELS];		//display list with bones
  Mesh *m_MeshList[MAX_SKELS];		//display list with bones
  //std::map<int, float*> vertices_modelview_map;  //store each modelview matrix and vertices

  //vertex shader variable here
  GLuint vertex_shader;
  GLuint fragment_shader;
  GLuint mesh_shader_program;
  GLuint VAO, VBO;


  static float jointColors[NUMBER_JOINT_COLORS][3];
};

#endif
