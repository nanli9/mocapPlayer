/*
Revision 1 - Steve Lin, Jan. 14, 2002
Revision 2 - Alla and Kiran, Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao, Feb, 2012
*/
#include <cstdio>
#include <cstring>
#include <cmath>
#include "types.h"

#if defined(__APPLE__)
#  include <OpenGL/gl3.h> // defines OpenGL 3.0+ functions
#else
#  if defined(_WIN32)
#    define GLEW_STATIC 1
#  endif
#  include <GL/glew.h>
#endif

#include <FL/gl.h>
#include <FL/glut.H>

#include "skeleton.h"
#include "motion.h"
#include "displaySkeleton.h"
#include "transform.h"
#include "Matrix4x4.h"

float DisplaySkeleton::jointColors[NUMBER_JOINT_COLORS][3] =
{
  {0.0f, 1.0f, 0.0f},  // GREEN
  {1.0f, 0.0f, 0.0f},  // RED
  {0.0f, 0.0f, 1.0f}   // BLUE
};

DisplaySkeleton::DisplaySkeleton(void)
{
  m_SpotJoint = -1;
  numSkeletons = 0;
  numMeshes = 0;
  for(int skeletonIndex = 0; skeletonIndex < MAX_SKELS; skeletonIndex++)
  {
    m_pSkeleton[skeletonIndex] = NULL;
    m_pMotion[skeletonIndex] = NULL;
  }
}

DisplaySkeleton::~DisplaySkeleton(void)
{
  Reset();
}


//Draws the world coordinate axis
void DisplaySkeleton::DrawSpotJointAxis(void) 
{
  GLfloat axisLength = 0.5f;
  glBegin(GL_LINES);
  // draw x axis in red, y axis in green, z axis in blue 
  glColor3f(1.0f, 0.2f, 0.2f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(axisLength, 0.0f, 0.0f);

  glColor3f(0.2f, 1.0f, 0.2f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, axisLength, 0.0f);

  glColor3f(0.2f, 0.2f, 1.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, axisLength);

  glEnd();
}

//Build display lists for bones
void DisplaySkeleton::SetDisplayList(int skeletonID, Bone *bone, GLuint *pBoneList)
{
  GLUquadricObj *qobj;
  int numbones = m_pSkeleton[skeletonID]->numBonesInSkel(bone[0]);
  *pBoneList = glGenLists(numbones);
  qobj=gluNewQuadric();

  gluQuadricDrawStyle(qobj, (GLenum) GLU_FILL);
  gluQuadricNormals(qobj, (GLenum) GLU_SMOOTH);

  float ambientFskeleton = 0.1f;
  float diffuseFskeleton = 0.9f;
  float specularFskeleton = 0.1f;

  int colorIndex = numSkeletons % NUMBER_JOINT_COLORS;
  float jointShininess = 120.0f;
  float jointAmbient[4] = {ambientFskeleton * jointColors[colorIndex][0], ambientFskeleton * jointColors[colorIndex][1], ambientFskeleton * jointColors[colorIndex][2], 1.0};
  float jointDiffuse[4] = {diffuseFskeleton * jointColors[colorIndex][0], diffuseFskeleton * jointColors[colorIndex][1], diffuseFskeleton * jointColors[colorIndex][2], 1.0};
  float jointSpecular[4] = {specularFskeleton * jointColors[colorIndex][0], specularFskeleton * jointColors[colorIndex][1], specularFskeleton * jointColors[colorIndex][2], 1.0};

  float boneColor[3] = {1.0f, 1.0f, 1.0f};
  float boneShininess = 120.0f; 
  float boneAmbient[4] = {ambientFskeleton * boneColor[0], ambientFskeleton * boneColor[1], ambientFskeleton * boneColor[2], 1.0};
  float boneDiffuse[4] = {diffuseFskeleton * boneColor[0], diffuseFskeleton * boneColor[1], diffuseFskeleton * boneColor[2], 1.0};
  float boneSpecular[4] = {specularFskeleton * boneColor[0], specularFskeleton * boneColor[1], specularFskeleton * boneColor[2], 1.0};

  double jointRadius = 0.10;
  double boneRadius = 0.10;
  double sizeDifferenceJointAndBone = 0.05;

  for(int j=0;j<numbones;j++)
  {
    glNewList(*pBoneList + j, GL_COMPILE);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, jointAmbient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, jointDiffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, jointSpecular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, jointShininess);
    glPushMatrix();
    glScalef(float(bone[j].aspy + sizeDifferenceJointAndBone), float(bone[j].aspy + sizeDifferenceJointAndBone), float(bone[j].aspy + sizeDifferenceJointAndBone));
    gluSphere(qobj, jointRadius, 20, 20);
    glPopMatrix();

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, boneAmbient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, boneDiffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, boneSpecular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, boneShininess);
    glPushMatrix();
    glScalef(float(bone[j].aspx), float(bone[j].aspy), 1.0f);
    gluCylinder(qobj, boneRadius, boneRadius, bone[j].length, 20, 20);

    // Two disks to close the cylinder at the bottom and the top
    gluDisk(qobj, 0.0, boneRadius, 20, 20);
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, float(bone[j].length));
    gluDisk(qobj, 0.0, boneRadius, 20, 20);
    glPopMatrix();
    
    glPopMatrix();    
    glEndList();
  }
}
void DisplaySkeleton::GenShader()
{
    GLint vertex_shader_compiled, fragment_shader_compiled;
    GLint linked;
    mesh_shader_program = glCreateProgram();

    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    //fs can be skipped
    const char* fs_source[] =
    {
        "uniform vec3 uColor;"
        "void main(void) {        "
        "  gl_FragColor = vec4(uColor, 1.0);"
        "}"
    };
    const char* vs_source[] = {
        "uniform mat4 modelview;"
        "uniform mat4 projection;"
        
        "uniform mat4 boneMatrix[31];"
        "uniform int boneIndex[4];"
        "uniform float weight[4];"
        "in vec3 pos;"

        "void main()"
        "{"
        "  mat4 m = mat4(0.0);"
        "   for(int i=0;i<4;i++)"
        "   {"
        "   if(boneIndex[i] != -1)"
        "       m += weight[i]*boneMatrix[boneIndex[i]];"  
        "   }"
        "   gl_Position =  projection * m * modelview * vec4(pos,1.0);"
        "}"
    };
   
    glShaderSource(vertex_shader, 1, vs_source, NULL);
    glShaderSource(fragment_shader, 1, fs_source, NULL);
    glCompileShader(vertex_shader);
    glCompileShader(fragment_shader);
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &vertex_shader_compiled);
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &fragment_shader_compiled);
    if (!vertex_shader_compiled)
    {
        GLint length;
        GLchar* log;
        glGetShaderiv(vertex_shader,GL_INFO_LOG_LENGTH,&length);

        log = (GLchar*)malloc(length);
        glGetShaderInfoLog(vertex_shader,length,&length,log);
        fprintf(stderr, "compile log = '%s'\n", log);
        free(log);

    }
    if (!fragment_shader_compiled)
    {
        fprintf(stderr, "Error in fragment shader\n");
    }
    //link the vertex shader
    glAttachShader(mesh_shader_program,vertex_shader);

    glLinkProgram(mesh_shader_program);

    glGetProgramiv(mesh_shader_program,GL_LINK_STATUS,&linked);

    if (!linked)
    {
        GLint length;
        GLchar* log;
        glGetProgramiv(mesh_shader_program, GL_INFO_LOG_LENGTH, &length);

        log = (GLchar*)malloc(length);
        glGetProgramInfoLog(mesh_shader_program, length, &length, log);
        fprintf(stderr, "link log = '%s'\n", log);
        free(log);
    }
    else
    {
        printf("link success");
    }

}

void DisplaySkeleton::DrawMesh(int skelNum)
{
    //put mesh vertices into the buffer
    for (int i = 0; i < m_MeshList[0]->verticesList.size(); i++)
    {
        verticesBuffer.push_back(m_MeshList[0]->verticesList[i].x);
        verticesBuffer.push_back(m_MeshList[0]->verticesList[i].y);
        verticesBuffer.push_back(m_MeshList[0]->verticesList[i].z);
    }

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, verticesBuffer.size() * sizeof(double), &verticesBuffer[0], GL_STATIC_DRAW);

    glBindVertexArray(VAO);
    // position attribute
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(double), (void*)0);
    glEnableVertexAttribArray(0);


    glUseProgram(mesh_shader_program);

    //get uniform index projection * modelview
    GLint modelview_index, projection_index;
    modelview_index = glGetUniformLocation(mesh_shader_program,"modelview");
    projection_index = glGetUniformLocation(mesh_shader_program,"projection");

    //glUniformMatrix4fv(modelview_index,1,GL_FALSE,);
    //get project and view matrix here

    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScalef(0.3,0.3,0.3);
    //glTranslatef(0,-3.4,0);

    GLfloat modelview[16],projection[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
    glGetFloatv(GL_PROJECTION_MATRIX, projection);
    /*for (int i = 0; i < 16; i++)
    {
        modelview[i] = 0;
    }*/
    Matrix4x4 modelview_matrix(modelview);
    Matrix4x4 projection_matrix(projection);
    //set uniform
    glUniformMatrix4fv(modelview_index, 1, GL_FALSE, &modelview_matrix.p[0][0]);
    glUniformMatrix4fv(projection_index, 1, GL_FALSE, &projection_matrix.p[0][0]);
    glBindVertexArray(VAO);
    glPointSize(2.5);

    //set uniform for bones matrices
    GLint boneMatricesIndex = glGetUniformLocation(mesh_shader_program, "boneMatrix");
    GLfloat matrix[16 * 31];
    for (int i = 0; i < 31; i++)
    {
        float* tmp= boneMatrices[i].flat();
        for (int j = 0; j < 16; j++)
        {
            matrix[16*i+j] = tmp[j];
        }
    }
    glUniformMatrix4fv(boneMatricesIndex, 31, GL_FALSE, matrix);

    GLint boneBinedIndex = glGetUniformLocation(mesh_shader_program, "boneIndex");
    GLint weightIndex = glGetUniformLocation(mesh_shader_program, "weight");
    

    for (int i = 0; i < m_MeshList[0]->verticesList.size(); i++)
    {
        /*for (auto j = m_MeshList[0]->vertices_bone_map.begin(); j != m_MeshList[0]->vertices_bone_map.end();j++)
        {
            if (j->second.boneCount == 4)
                printf("****************\n");
        }*/
        int* boneIndex = m_MeshList[0]->vertices_bone_map.at(i).boneID;
        float* weight = m_MeshList[0]->vertices_bone_map.at(i).boneWeights;
        /*for (int j = 0; j < 4; j++)
        {
            if (weight[j] < 0)
                printf("****************\n");
        }*/
        glUniform1iv(boneBinedIndex, 4 , boneIndex);
        
        glUniform1fv(weightIndex, 4 , weight);

        glDrawArrays(GL_POINTS, i, 1);
    }
    verticesBuffer.clear();
    glUseProgram(0);

    glPopMatrix();
}
/*
  Define M_k = Modelview matrix at the kth node (bone) in the heirarchy
  M_k stores the transformation matrix of the kth bone in world coordinates
  Our goal is to draw the (k+1)th bone, using its local information and M_k

  In the k+1th node, compute the following matrices:
  rot_parent_current: this is the rotation matrix that 
  takes us from k+1 to the kth local coordinate system 
  R_k+1 : Rotation matrix for the k+1 th node (bone)
  using angles specified by the AMC file in local coordinates
  T_k+1 : Translation matrix for the k+1th node
  
  The update relation is given by:
  M_k+1 = M_k * (rot_parent_current) * R_k+1 + T_k+1
*/

void DisplaySkeleton::DrawBone(Bone *pBone,int skelNum)
{
  static double z_dir[3] = {0.0, 0.0, 1.0};
  double r_axis[3], theta;

  //Transform (rotate) from the local coordinate system of this bone to it's parent
  //This step corresponds to doing: ModelviewMatrix = M_k * (rot_parent_current)
  glMultMatrixd((double*)&pBone->rot_parent_current);     


/*
  // The following code is for creating Figure 2 on the webpage of HW2  
  const int jointsDisplayNum = 5;
  int jointsDisplay[jointsDisplayNum] = {2,3,4,18,19};
  //Draw the local coordinate system for the selected bone.
  if(renderMode == BONES_AND_LOCAL_FRAMES)
  {
    int i;
    for(i = 0; i < jointsDisplayNum; i++)
      if (pBone->idx == jointsDisplay[i])
	break;
    if (i < jointsDisplayNum)
    {
      GLint lightingStatus;
      glGetIntegerv(GL_LIGHTING, &lightingStatus);
      glDisable(GL_LIGHTING);
      DrawSpotJointAxis();
      if (lightingStatus)
        glEnable(GL_LIGHTING);
    }
  }
*/

  //Draw the local coordinate system for the selected bone.
  if((renderMode == BONES_AND_LOCAL_FRAMES) && (pBone->idx == m_SpotJoint))
  {
    GLint lightingStatus;
    glGetIntegerv(GL_LIGHTING, &lightingStatus);
    glDisable(GL_LIGHTING);
    DrawSpotJointAxis();
    if (lightingStatus)
      glEnable(GL_LIGHTING);
  }

  //translate AMC (rarely used)
  if(pBone->doftz) 
    glTranslatef(0.0f, 0.0f, float(pBone->tz));
  if(pBone->dofty) 
    glTranslatef(0.0f, float(pBone->ty), 0.0f);
  if(pBone->doftx) 
    glTranslatef(float(pBone->tx), 0.0f, 0.0f);

  //rotate AMC 
  if(pBone->dofrz) 
    glRotatef(float(pBone->rz), 0.0f, 0.0f, 1.0f);
  if(pBone->dofry) 
    glRotatef(float(pBone->ry), 0.0f, 1.0f, 0.0f);
  if(pBone->dofrx) 
    glRotatef(float(pBone->rx), 1.0f, 0.0f, 0.0f);

  //Store the current ModelviewMatrix (before adding the translation part)
  glPushMatrix();

  //Compute tx, ty, tz : translation from pBone to its child (in local coordinate system of pBone)
  double tx = pBone->dir[0] * pBone->length;
  double ty = pBone->dir[1] * pBone->length;
  double tz = pBone->dir[2] * pBone->length;


  // Use the current ModelviewMatrix to display the current bone
  // Rotate the bone from its canonical position (elongated sphere 
  // with its major axis parallel to X axis) to its correct orientation
  if(pBone->idx == Skeleton::getRootIndex())
  {
    // glCallList(m_BoneList[skelNum] + pBone->idx);  // no need to draw the root here any more (it is not a bone) 
  }
  else
  { 
    //Compute the angle between the canonical pose and the correct orientation 
    //(specified in pBone->dir) using cross product.
    //Using the formula: r_axis = z_dir x pBone->dir
    v3_cross(z_dir, pBone->dir, r_axis);

    theta =  GetAngle(z_dir, pBone->dir, r_axis);

    glRotatef(float(theta*180./M_PI), float(r_axis[0]), float(r_axis[1]), float(r_axis[2]));
    glCallList(m_BoneList[skelNum] + pBone->idx);
  }
  

  glPopMatrix(); 

  // Finally, translate the bone, depending on its length and direction
  // This step corresponds to doing: M_k+1 = ModelviewMatrix += T_k+1
  glTranslatef(float(tx), float(ty), float(tz));


  //take bone matrices into list
  GLfloat boneMatrix[16];
  glGetFloatv(GL_MODELVIEW_MATRIX, boneMatrix);
  Matrix4x4 matrix(boneMatrix);
  /*boneMatrix[12] += float(tx);
  boneMatrix[13] += float(ty);
  boneMatrix[14] += float(tz);*/

  boneMatrices[pBone->idx] = matrix;

}

void DisplaySkeleton::SetShadowingModelviewMatrix(double ground[4], double light[4])
{
  double dot;
  double shadowMat[4][4];

  dot = ground[0] * light[0] + ground[1] * light[1] + ground[2] * light[2] + ground[3] * light[3];

  shadowMat[0][0] = dot - light[0] * ground[0];
  shadowMat[1][0] = 0.0 - light[0] * ground[1];
  shadowMat[2][0] = 0.0 - light[0] * ground[2];
  shadowMat[3][0] = 0.0 - light[0] * ground[3];

  shadowMat[0][1] = 0.0 - light[1] * ground[0];
  shadowMat[1][1] = dot - light[1] * ground[1];
  shadowMat[2][1] = 0.0 - light[1] * ground[2];
  shadowMat[3][1] = 0.0 - light[1] * ground[3];

  shadowMat[0][2] = 0.0 - light[2] * ground[0];
  shadowMat[1][2] = 0.0 - light[2] * ground[1];
  shadowMat[2][2] = dot - light[2] * ground[2];
  shadowMat[3][2] = 0.0 - light[2] * ground[3];

  shadowMat[0][3] = 0.0 - light[3] * ground[0];
  shadowMat[1][3] = 0.0 - light[3] * ground[1];
  shadowMat[2][3] = 0.0 - light[3] * ground[2];
  shadowMat[3][3] = dot - light[3] * ground[3];

  glMultMatrixd((const GLdouble*)shadowMat);
}

//Traverse the hierarchy starting from the root 
//Every node in the data structure has just one child pointer. 
//If there are more than one children for any node, they are stored as sibling pointers
//The algorithm draws the current node (bone), visits its child and then visits siblings
void DisplaySkeleton::Traverse(Bone *ptr,int skelNum)
{
  if(ptr != NULL)
  {
    glPushMatrix();
    DrawBone(ptr,skelNum);

    Traverse(ptr->child,skelNum);
    glPopMatrix();
    Traverse(ptr->sibling,skelNum);
  }
}

//Draw the skeleton
void DisplaySkeleton::Render(RenderMode renderMode_)
{
  // Set render mode
  renderMode = renderMode_;
 
  glPushMatrix();

  //Translate the root to the correct position (it is (0,0,0) if no motion is loaded)
  //   glTranslatef(m_pSkeleton->m_RootPos[0], m_pSkeleton->m_RootPos[1], m_pSkeleton->m_RootPos[2]);
  double translation[3];
  double rotationAngle[3];
  //draw the skeleton starting from the root
  for (int i = 0; i < numSkeletons; i++)
  {
    glPushMatrix();
    //double translation[3];
    m_pSkeleton[i]->GetTranslation(translation);
    //double rotationAngle[3];
    m_pSkeleton[i]->GetRotationAngle(rotationAngle);

    glTranslatef(float(MOCAP_SCALE * translation[0]), float(MOCAP_SCALE * translation[1]), float(MOCAP_SCALE * translation[2]));
    glRotatef(float(rotationAngle[0]), 1.0f, 0.0f, 0.0f);
    glRotatef(float(rotationAngle[1]), 0.0f, 1.0f, 0.0f);
    glRotatef(float(rotationAngle[2]), 0.0f, 0.0f, 1.0f);
    Traverse(m_pSkeleton[i]->getRoot(),i);
    glPopMatrix();

  }
  glPopMatrix(); 
  /*glPushMatrix();
  for (int i = 0; i < numMeshes; i++)
  {
      glPointSize(5);
      glScalef(0.3, 0.3, 0.3);
      glTranslatef(translation[0], translation[1], translation[2]);
      glBegin(GL_POINTS);
      for (int j = 0; j < m_MeshList[numMeshes - 1]->verticesNum; j++)
      {
          aiVector3D pos = (m_MeshList[numMeshes - 1]->verticesList)[j];
          double x = pos.x;
          double y = pos.y;
          double z = pos.z;
          glVertex3f(x, y , z );
      }
      glEnd();

  }
  glPopMatrix();*/

}

void DisplaySkeleton::LoadMotion(Motion * pMotion)
{
  // always load the motion for the latest skeleton
  if(m_pMotion[numSkeletons - 1] != NULL) 
    delete m_pMotion[numSkeletons - 1];
  m_pMotion[numSkeletons - 1] = pMotion;
}

void DisplaySkeleton::LoadMesh(Mesh* pMesh)
{
    printf("Parsing %d meshes\n\n", pMesh->verticesNum);
    m_MeshList[numMeshes] = pMesh;
    //bind the vertices to the skeleton
    //m_pSkeleton[numSkeletons-1]->bindVertices(pMesh);
    numMeshes++;
}


//Set skeleton for display
void DisplaySkeleton::LoadSkeleton(Skeleton *pSkeleton)
{
  if (numSkeletons >= MAX_SKELS) 
    return;

  m_pSkeleton[numSkeletons] = pSkeleton;

  //Create the display list for the skeleton
  //All the bones are the elongated spheres centered at (0,0,0).
  //The axis of elongation is the X axis.
  SetDisplayList(numSkeletons, m_pSkeleton[numSkeletons]->getRoot(), &m_BoneList[numSkeletons]);
  numSkeletons++;
}

void DisplaySkeleton::RenderShadow(double ground[4], double light[4])
{
  GLint lightingStatus;
  glGetIntegerv(GL_LIGHTING, &lightingStatus);
  glDisable(GL_LIGHTING);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  SetShadowingModelviewMatrix(ground, light);
  Render(DisplaySkeleton::BONES_ONLY);
  glPopMatrix();

  if (lightingStatus)
    glEnable(GL_LIGHTING);
}

Motion * DisplaySkeleton::GetSkeletonMotion(int skeletonIndex)
{
  if (skeletonIndex < 0 || skeletonIndex >= MAX_SKELS)
  {
    printf("Error in DisplaySkeleton::GetSkeletonMotion: index %d is illegal.\n", skeletonIndex);
    exit(0);
  }
  return m_pMotion[skeletonIndex];
}

Skeleton * DisplaySkeleton::GetSkeleton(int skeletonIndex)
{
  if (skeletonIndex < 0 || skeletonIndex >= numSkeletons)
  {
    printf("Error in DisplaySkeleton::GetSkeleton: skeleton index %d is illegal.\n", skeletonIndex);
    exit(0);
  }
  return m_pSkeleton[skeletonIndex];
}

void DisplaySkeleton::Reset(void)
{
  for(int skeletonIndex = 0; skeletonIndex < MAX_SKELS; skeletonIndex++)
  {
    if (m_pSkeleton[skeletonIndex] != NULL)
    {
      delete (m_pSkeleton[skeletonIndex]);
      glDeleteLists(m_BoneList[skeletonIndex], 1);
      m_pSkeleton[skeletonIndex] = NULL;
    }
    if (m_pMotion[skeletonIndex] != NULL)
    {
      delete (m_pMotion[skeletonIndex]);
      m_pMotion[skeletonIndex] = NULL;
    }
  }
  numSkeletons = 0;
}



