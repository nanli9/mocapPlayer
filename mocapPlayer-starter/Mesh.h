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

class Mesh
{
public:
	std::vector<aiVector3D> verticesList;//store vertices pos
	std::map<std::string,std::vector<aiVertexWeight>> vertices_bone_map;
	int verticesNum;
	Mesh(char* fbx_filename);
};

#endif // !MESH_H



