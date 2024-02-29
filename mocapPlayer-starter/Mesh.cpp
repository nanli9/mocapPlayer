#include "Mesh.h"


Mesh::Mesh(char* fbx_filename)
{
    // Create an instance of the Importer class
    Assimp::Importer importer;
    // And have it read the given file with some example postprocessing
    // Usually - if speed is not the most important aspect for you - you'll 
    // propably to request more postprocessing than we do in this example.
    const aiScene* scene = importer.ReadFile(fbx_filename,
        aiProcess_CalcTangentSpace |
        aiProcess_Triangulate |
        aiProcess_JoinIdenticalVertices |
        aiProcess_SortByPType);
    if (!scene)
    {
        printf(importer.GetErrorString());
        exit(1);
    }
    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
        const aiMesh* pMesh = scene->mMeshes[i];
        verticesNum = pMesh->mNumVertices;
        for (unsigned int j = 0; j < verticesNum; j++)
        {
            verticesList.push_back(pMesh->mVertices[j]);
        }
        if(pMesh->HasBones()) 
        {
            for (unsigned int j = 0; j < pMesh->mNumBones; j++) 
            {
                aiBone* bone = (pMesh->mBones)[j];
                std::vector<aiVertexWeight> list;
                for (int k = 0; k < bone->mNumWeights; k++)
                {
                    list.push_back((bone->mWeights)[k]);
                    //store each bone in vertices
                    if (vertices_bone_map.find((bone->mWeights)[k].mVertexId)!= vertices_bone_map.end())
                    {
                        //add one more bone effect vertices
                        int& boneIndex = vertices_bone_map.at((bone->mWeights)[k].mVertexId).boneCount;
                        vertices_bone_map.at((bone->mWeights)[k].mVertexId).boneID[boneIndex] = bone->mName.data;
                        boneIndex++;
                    }
                    else
                    {
                        struct vertices v = { {bone->mName.data,"","","" }, 1};
                        vertices_bone_map.insert({ (bone->mWeights)[k].mVertexId,v});
                    }

                }
                //vertices_bone_map.insert({bone->mName.data,list});

            }


        }
    }
    
    

}
