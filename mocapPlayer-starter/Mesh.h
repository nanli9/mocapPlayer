#pragma once


#ifndef MESH_H
#define MESH_H


#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>
#include <assimp/matrix4x4.h>
#include <vector>

class Mesh
{
public:
	std::vector<aiVector3D> verticesList;
	int verticesNum;
	Mesh(char* fbx_filename);
};







#endif // !MESH_H



