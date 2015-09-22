#include "stdafx.h"
#include "ModelLoader.h"
#include <map>
#include <assert.h>
#include <limits>
#include "assimp/DefaultLogger.hpp"
#include "Mesh.h"
#include "Skeleton.h"
#include "AnimationController.h"

using namespace DirectX;

ModelLoader::ModelLoader( ID3D11Device* device, std::string modelDir, std::string textureDir ) :
device( device ),
modelDir( modelDir ),
textureDir( textureDir ),
scene( nullptr ) {
	Assimp::DefaultLogger::create( "assimp.log", Assimp::Logger::VERBOSE );
}

ModelLoader::~ModelLoader() {
	Assimp::DefaultLogger::kill();
}

bool ModelLoader::Load( std::string filename, Vertex::VERTEX_TYPE type ) {
	// reset extents
	minX = minY = minZ = FLT_MAX;
	maxX = maxY = maxZ = FLT_MIN;

	Assimp::Importer importer;
	std::string file = modelDir+filename;
	Assimp::DefaultLogger::get()->info( "Importing: "+file );
	scene = importer.ReadFile( file,
		//aiProcess_CalcTangentSpace|
		aiProcess_ImproveCacheLocality|
		//aiProcess_MakeLeftHanded|
		aiProcess_FlipWindingOrder|
		aiProcess_Triangulate|
		//aiProcess_JoinIdenticalVertices|
		aiProcess_SortByPType|
		aiProcess_FlipUVs
		);

	if( !scene ) {
		Assimp::DefaultLogger::get()->error( importer.GetErrorString() );
		return false;
	}
	if( !scene->HasMeshes() ) {
		Assimp::DefaultLogger::get()->error( "File contains no mesh" );
		return false;
	}
	aiMesh* sceneMesh = scene->mMeshes[0];
	CreateVertexBuffer( sceneMesh, type );
	CreateIndexBuffer( sceneMesh->mFaces, sceneMesh->mNumFaces );
	if( scene->HasAnimations() ) {
		CreateSkeleton( sceneMesh->mBones, sceneMesh->mNumBones );
		CreateBoneHierarchy();
		CreateAnimations();
	}
	return true;
}

Mesh* ModelLoader::GetMesh() {
	Mesh* mesh = new Mesh();
	mesh->SetVB( vb );
	mesh->SetIB( ib, indexCount );
	return mesh;
}

Skeleton* ModelLoader::GetSkeleton() {
	return skeleton;
}

std::vector<PointLight> ModelLoader::GetPointLights() {
	return pointLights;
}

void ModelLoader::CreateIndexBuffer( const aiFace* indices, UINT count ) {
	indexCount = count*3;
	std::vector<USHORT> indexData( count*3 );
	for( UINT faceIndex = 0, dataIndex = 0; faceIndex<count; ++faceIndex, dataIndex += 3 ) {
		assert( indices[faceIndex].mNumIndices==3 ); //mesh should be triangulated
		indexData[dataIndex] = (USHORT)indices[faceIndex].mIndices[0];
		indexData[dataIndex+1] = (USHORT)indices[faceIndex].mIndices[1];
		indexData[dataIndex+2] = (USHORT)indices[faceIndex].mIndices[2];
	}

	D3D11_BUFFER_DESC ibd;
	ibd.Usage = D3D11_USAGE_IMMUTABLE;
	ibd.ByteWidth = sizeof( USHORT ) * indexData.size();
	ibd.BindFlags = D3D11_BIND_INDEX_BUFFER;
	ibd.CPUAccessFlags = 0;
	ibd.MiscFlags = 0;
	ibd.StructureByteStride = 0;

	D3D11_SUBRESOURCE_DATA iinitData;
	iinitData.pSysMem = indexData.data();

	HR( device->CreateBuffer( &ibd, &iinitData, &ib ) );
}

void ModelLoader::CreateVertexBuffer( aiMesh* mesh, Vertex::VERTEX_TYPE type ) {
	UINT count = mesh->mNumVertices;
	aiVector3D* vertices = mesh->mVertices;
	/*  The switch case looks like duplicated code.  It is not.
		vertData is a different type in each and SetVertices() is
		a template. */
	switch( type ) {
	case Vertex::SIMPLE:
	{
		std::vector<Vertex::SimpleVertex> vertData( count );
		for( UINT i = 0; i<count; ++i ) {
			UpdateExtents( vertices[i].x, vertices[i].y, vertices[i].z );
			vertData[i].Pos = XMFLOAT3( vertices[i].x, vertices[i].y, vertices[i].z );
		}
		SetVertices( device, count, vertData.data() );
		break;
	}
	case Vertex::CHARACTER:
	case Vertex::TERRAIN:
	{
		aiVector3D* normals = mesh->mNormals;
		aiVector3D* texCoords = mesh->mTextureCoords[0];
		std::vector<Vertex::TerrainVertex> vertData( count );
		for( UINT i = 0; i<count; ++i ) {
			UpdateExtents( vertices[i].x, vertices[i].y, vertices[i].z );
			vertData[i].Pos = XMFLOAT3( vertices[i].x, vertices[i].y, vertices[i].z );
			vertData[i].Normal = XMFLOAT3( normals[i].x, normals[i].y, normals[i].z );
			vertData[i].Tex = XMFLOAT2( texCoords[i].x, texCoords[i].y );
		}
		SetVertices( device, count, vertData.data() );
		break;
	}
	case Vertex::CHARACTER_SKINNED:
	{
		std::multimap<int, BoneWeight> vertexBoneWeight;
		for( unsigned int boneIndex = 0; boneIndex<mesh->mNumBones; ++boneIndex ) {
			auto bone = mesh->mBones[boneIndex];
			for( int i = 0; i<bone->mNumWeights; ++i ) {
				auto boneWeight = BoneWeight( boneIndex, bone->mWeights[i].mWeight );
				vertexBoneWeight.insert( std::pair<int, BoneWeight>( bone->mWeights[i].mVertexId, boneWeight ) );
			}
		}
		aiVector3D* normals = mesh->mNormals;
		aiVector3D* texCoords = mesh->mTextureCoords[0];
		std::vector<Vertex::CharacterSkinnedVertex> vertData( count );
		for( UINT i = 0; i<count; ++i ) {
			UpdateExtents( vertices[i].x, vertices[i].y, vertices[i].z );
			vertData[i].Pos = XMFLOAT3( vertices[i].x, vertices[i].y, vertices[i].z );
			vertData[i].Normal = XMFLOAT3( normals[i].x, normals[i].y, normals[i].z );
			vertData[i].Tex = XMFLOAT2( texCoords[i].x, texCoords[i].y );

			BYTE boneIndices[4] = { 0, 0, 0, 0 };
			float weights[4] = { 0, 0, 0, 0 };
			int j = 0;
			auto itlow = vertexBoneWeight.lower_bound( i );
			auto itup = vertexBoneWeight.upper_bound( i );
			assert( itlow!=itup ); // every vertex should have some influence
			for( auto it = itlow; it!=itup; ++it ) {
				if( j>=4 ) {
					break;
				}
				assert( j<4 ); // each vertex should not be influenced by more than 4 bones
				boneIndices[j] = it->second.boneIndex;
				weights[j] = it->second.weight;
				++j;
			}
			vertData[i].BoneIndicies[0] = boneIndices[0];
			vertData[i].BoneIndicies[1] = boneIndices[1];
			vertData[i].BoneIndicies[2] = boneIndices[2];
			vertData[i].BoneIndicies[3] = boneIndices[3];
			vertData[i].Weights = XMFLOAT4( weights );
			assert( weights[0]+weights[1]+weights[2]+weights[3]<1.0001f );
			assert( weights[0]+weights[1]+weights[2]+weights[3]>0.9999f );
		}
		SetVertices( device, count, vertData.data() );
		break;
	}
	}
}

void ModelLoader::CreateSkeleton( aiBone** bones, int numBones ) {
	skeleton = new Skeleton();
	for( int i = 0; i<numBones; ++i ) {
		Bone* newBone = new Bone();
		aiBone* bone = bones[i];
		newBone->idx = i;
		newBone->name = bone->mName.data;
		auto boneOffset = bone->mOffsetMatrix;
		XMMATRIX convertedOffset = ConvertMatrix( boneOffset );
		newBone->offset = convertedOffset;
		skeleton->AddBone( newBone );
	}
}

void ModelLoader::CreateBoneHierarchy() {
	aiNode* root = scene->mRootNode->FindNode( "Skeleton_Root" );
	FindBoneChildren( root, -1 );
}

void ModelLoader::FindBoneChildren( aiNode* node, int parentIdx ) {
	Bone* bone = skeleton->GetBoneByName( node->mName.data );
	bone->parentIdx = parentIdx;

	DirectX::XMMATRIX transformMat = ConvertMatrix( node->mTransformation );
	bone->localTransform = transformMat;

	if( node->mNumChildren==0 ) { return; }
	for( int i = 0; i<node->mNumChildren; ++i ) {
		aiNode* childNode = node->mChildren[i];
		std::string childName = childNode->mName.data;
		Bone* childBone = skeleton->GetBoneByName( childName );
		if( childBone==nullptr ) {
			// Bones with no skin influence will be missing from the previous list
			childBone = new Bone();
			childBone->idx = skeleton->BoneCount();
			childBone->name = childName;
			XMMATRIX transform = ConvertMatrix( childNode->mTransformation );
			XMMATRIX parentOffset = bone->offset;
			childBone->offset = DirectX::XMMatrixMultiply( transform, parentOffset );
			skeleton->AddBone( childBone );
		}
		bone->children.push_back( childBone );
		FindBoneChildren( childNode, bone->idx );
	}
}

DirectX::XMMATRIX XM_CALLCONV ModelLoader::ConvertMatrix( aiMatrix4x4 inMat ) {
	DirectX::XMMATRIX transposed = XMMATRIX(
		inMat.a1, inMat.b1, inMat.c1, inMat.d1,
		inMat.a2, inMat.b2, inMat.c2, inMat.d2,
		inMat.a3, inMat.b3, inMat.c3, inMat.d3,
		inMat.a4, inMat.b4, inMat.c4, inMat.d4 );
	/*DirectX::XMMATRIX rotX = XMMatrixRotationX( XM_PIDIV2 );
	DirectX::XMMATRIX rotZ = XMMatrixRotationZ( XM_PIDIV2 );
	DirectX::XMMATRIX flipY = XMMATRIX(
		1.f, 0.f, 0.f, 0.f,
		0.f, -1.f, 0.f, 0.f,
		0.f, 0.f, 1.f, 0.f,
		0.f, 0.f, 0.f, 1.f );
	DirectX::XMMATRIX converted = transposed*rotX*rotZ*flipY;*/
	return transposed;
}

void ModelLoader::GetMeshExtents( float &outMinX, float &outMaxX, float &outMinY, float &outMaxY, float &outMinZ, float &outMaxZ ) {
	outMinX = minX;
	outMaxX = maxX;
	outMinY = minY;
	outMaxY = maxY;
	outMinZ = minZ;
	outMaxZ = maxZ;
}

inline void ModelLoader::UpdateExtents( float x, float y, float z ) {
	if( x<minX ) {
		minX = x;
	} else if( x>maxX ) {
		maxX = x;
	}
	if( y<minY ) {
		minY = y;
	} else if( y>maxY ) {
		maxY = y;
	}
	if( z<minZ ) {
		minZ = z;
	} else if( z>maxZ ) {
		maxZ = z;
	}
}

std::vector<Anim*> ModelLoader::GetAnimations() {
	return anims;
}

void ModelLoader::CreateAnimations() {
	for( int i = 0; i<scene->mNumAnimations; ++i ) {
		aiAnimation* aiAnim = scene->mAnimations[i];
		Anim* anim = new Anim();
		anim->name = aiAnim->mName.C_Str();
		float framesPerSec = (float)aiAnim->mTicksPerSecond;
		anim->totalTime = (float)(aiAnim->mDuration/aiAnim->mTicksPerSecond);
		float timePerFrame = 1/framesPerSec;
		for( int j = 0; j<aiAnim->mNumChannels; ++j ) {
			aiNodeAnim* aiNodeAnim = aiAnim->mChannels[j];
			Bone* bone = skeleton->GetBoneByName( aiNodeAnim->mNodeName.C_Str() );
			anim->boneSet.insert( bone );

			keySet_t rotKeySet;
			for( int k = 0; k<aiNodeAnim->mNumRotationKeys; ++k ) {
				aiQuatKey quatKey = aiNodeAnim->mRotationKeys[k];
				rotKeySet[(float)quatKey.mTime * timePerFrame] = XMFLOAT4( quatKey.mValue.x, quatKey.mValue.y, quatKey.mValue.z, quatKey.mValue.w );
			}
			anim->rotChannels[bone] = rotKeySet;

			keySet_t posKeySet;
			for( int m = 0; m<aiNodeAnim->mNumPositionKeys; ++m ) {
				aiVectorKey posKey = aiNodeAnim->mPositionKeys[m];
				posKeySet[(float)posKey.mTime * timePerFrame] = XMFLOAT4( posKey.mValue.x, posKey.mValue.y, posKey.mValue.z, 1.0f );
			}
			anim->posChannels[bone] = posKeySet;

			keySet_t scaleKeySet;
			for( int n = 0; n<aiNodeAnim->mNumScalingKeys; ++n ) {
				aiVectorKey scaleKey = aiNodeAnim->mScalingKeys[n];
				scaleKeySet[(float)scaleKey.mTime* timePerFrame] = XMFLOAT4( scaleKey.mValue.x, scaleKey.mValue.y, scaleKey.mValue.z, 1.0f );
			}
			anim->scaleChannels[bone] = scaleKeySet;
		}
		anims.push_back( anim );
	}
}