#pragma once


#ifndef MESH_H
#define MESH_H


#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>
#include <assimp/matrix4x4.h>
#include <vector>
#include <map>

#define MAX_BONE_PER_VERTEX 4

struct vertices
{
	int boneID[MAX_BONE_PER_VERTEX];
	float boneWeights[MAX_BONE_PER_VERTEX];
	int boneCount;
};

class Mesh
{
public:
	std::vector<aiVector3D> verticesList;//store vertices pos
	std::vector<vertices> verticesInfo;//store vertices pos and bone ID
	std::map<int, vertices> vertices_bone_map; //store vertice index and bone info
	//std::map<std::string,std::vector<aiVertexWeight>> vertices_bone_map;
	int verticesNum;
	int BoneNum;
	Mesh(char* fbx_filename);
	int BoneStringMappingInt(char* boneName);
};

#endif // !MESH_H



