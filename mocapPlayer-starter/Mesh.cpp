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
            BoneNum = pMesh->mNumBones;
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
                        if (BoneStringMappingInt(bone->mName.data) != -2)
                            vertices_bone_map.at((bone->mWeights)[k].mVertexId).boneID[boneIndex] = BoneStringMappingInt(bone->mName.data);
                            //printf("%s", bone->mName.data);
                        vertices_bone_map.at((bone->mWeights)[k].mVertexId).boneWeights[boneIndex] = (bone->mWeights)[k].mWeight;
                        boneIndex++;
                    }
                    else
                    {
                        if (BoneStringMappingInt(bone->mName.data) != -2)
                        {
                            struct vertices v = { {BoneStringMappingInt(bone->mName.data),-1,-1,-1 },{},1 };
                            vertices_bone_map.insert({ (bone->mWeights)[k].mVertexId,v });
                        }
                    }

                }
                //vertices_bone_map.insert({bone->mName.data,list});

            }


        }
    }
}
int Mesh::BoneStringMappingInt(char* boneName)
{
    int BoneId = -2;
    if (!strcmp(boneName, "root"))
        BoneId = 0;
    else if (!strcmp(boneName, "lhipjoint"))
        BoneId = 1;
    else if(!strcmp(boneName, "lfemur"))
        BoneId = 2;
    else if (!strcmp(boneName, "ltibia"))
        BoneId = 3;
    else if (!strcmp(boneName, "lfoot"))
        BoneId = 4;
    else if (!strcmp(boneName, "ltoes"))
        BoneId = 5;
    else if (!strcmp(boneName, "rhipjoint"))
        BoneId = 6;
    else if (!strcmp(boneName, "rfemur"))
        BoneId = 7;
    else if (!strcmp(boneName, "rtibia"))
        BoneId = 8;
    else if (!strcmp(boneName, "rfoot"))
        BoneId = 9;
    else if (!strcmp(boneName, "rtoes"))
        BoneId = 10;
    else if (!strcmp(boneName, "lowerback"))
        BoneId = 11;
    else if (!strcmp(boneName, "upperback"))
        BoneId = 12;
    else if (!strcmp(boneName, "thorax"))
        BoneId = 13;
    else if (!strcmp(boneName, "lowerneck"))
        BoneId = 14;
    else if (!strcmp(boneName, "upperneck"))
        BoneId = 15;
    else if (!strcmp(boneName, "head"))
        BoneId = 16;
    else if (!strcmp(boneName, "lclavicle"))
        BoneId = 17;
    else if (!strcmp(boneName, "lhumerus"))
        BoneId = 18;
    else if (!strcmp(boneName, "lradius"))
        BoneId = 19;
    else if (!strcmp(boneName, "lwrist"))
        BoneId = 20;
    else if (!strcmp(boneName, "lhand"))
        BoneId = 21;
    else if (!strcmp(boneName, "lfingers"))
        BoneId = 22;
    else if (!strcmp(boneName, "lthumb"))
        BoneId = 23;
    else if (!strcmp(boneName, "rclavicle"))
        BoneId = 24;
    else if (!strcmp(boneName, "rhumerus"))
        BoneId = 25;
    else if (!strcmp(boneName, "rradius"))
        BoneId = 26;
    else if (!strcmp(boneName, "rwrist"))
        BoneId = 27;
    else if (!strcmp(boneName, "rhand"))
        BoneId = 28;
    else if (!strcmp(boneName, "rfingers"))
        BoneId = 29;
    else if (!strcmp(boneName, "rthumb"))
        BoneId = 30;

    return BoneId;
}
