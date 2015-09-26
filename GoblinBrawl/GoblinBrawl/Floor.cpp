#include "stdafx.h"
#include "Floor.h"
#include "ModelLoader.h"
#include "Vertex.h"
#include "Mesh.h"
#include "MyEffects.h"
#include "MathUtils.h"
#include "WICTextureLoader.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "PhysicsWorld.h"
#include "SharedResources.h"

using namespace DirectX;

Floor::Floor() :
mesh( nullptr ),
diffuseView( nullptr ),
physicsWorld( nullptr ) {}

Floor::~Floor() {}

bool Floor::Init( ModelLoader* modelLoader, ID3D11Device* device, PhysicsWorld* physicsWorld ) {
	this->device = device;
	modelLoader->Load( "floor.lxo", Vertex::STATIC_GEOMETRY );
	mesh = modelLoader->GetMesh();
	if( !mesh ) {
		return false;
	}
	this->physicsWorld = physicsWorld;
	float minX, minY, minZ, maxX, maxY, maxZ;
	modelLoader->GetMeshExtents( minX, maxX, minY, maxY, minZ, maxZ );
	CreateHeightfield( minX, maxX, minY, maxY, minZ, maxZ );
	HR( CreateWICTextureFromFile( device, L"./art/textures/floor_color.tif", NULL, &diffuseView, NULL ) );
	mat.Ambient = XMFLOAT4( 0.02f, 0.3f, 0.5f, 1.0f );
	mat.Diffuse = XMFLOAT4( 0.8f, 0.8f, 0.8f, 1.0f );
	mat.Specular = XMFLOAT4( 0.08f, 0.05f, 0.05f, 4.0f );
	return true;
}

void XM_CALLCONV Floor::Draw( FXMMATRIX viewProj, FXMVECTOR cameraPos, std::vector<PointLight> pointLights, ID3D11DeviceContext* context ) {
	context->IASetInputLayout( InputLayouts::StaticGeom );
	context->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
	UINT stride = sizeof( Vertex::StaticGeomVertex );
	UINT offset = 0;
	ID3D11Buffer* buffers[1] = { mesh->VB() };
	context->IASetVertexBuffers( 0, 1, &buffers[0], &stride, &offset );
	context->IASetIndexBuffer( mesh->IB(), mesh->IndexFormat(), 0 );

	XMMATRIX world = XMMatrixIdentity();
	XMMATRIX worldInvTranspose = MathUtils::InverseTranspose( world );
	XMMATRIX worldViewProj = world*viewProj;

	MyEffects::StaticGeomFX->SetWorld( world );
	MyEffects::StaticGeomFX->SetWorldInvTranspose( worldInvTranspose );
	MyEffects::StaticGeomFX->SetWorldViewProj( worldViewProj );
	MyEffects::StaticGeomFX->SetDiffuseMap( diffuseView );
	MyEffects::StaticGeomFX->SetAmbientMap( SharedResources::directionalAmbientView );
	MyEffects::StaticGeomFX->SetEyePosW( cameraPos );
	MyEffects::StaticGeomFX->SetPointLights( pointLights.data() );
	MyEffects::StaticGeomFX->SetMaterial( mat );

	ID3DX11EffectTechnique* tech = MyEffects::StaticGeomFX->staticGeomLight5Tech;
	D3DX11_TECHNIQUE_DESC td;
	tech->GetDesc( &td );
	for( UINT p = 0; p<td.Passes; ++p ) {
		tech->GetPassByIndex( p )->Apply( 0, context );
		context->DrawIndexed( mesh->IndexCount(), 0, 0 );
	}
}

void Floor::CreateHeightfield( float minX, float maxX, float minY, float maxY, float minZ, float maxZ ) {
	const int gridSize = 64+1;  // must be (2^N) + 1
	float deltaX = maxX-minX;
	float deltaZ = maxZ-minZ;
	float maxDimension = deltaX>deltaZ ? deltaX : deltaZ;
	const btScalar gridSpacing( maxDimension/(float)gridSize );
	float heightScale = maxY-minY;
	const btScalar minHeight( -heightScale );
	const btScalar maxHeight( heightScale );
	int upAxis = 1;		// start with Y-axis as "up"
	PHY_ScalarType type = PHY_FLOAT;
	bool flipQuadEdges = false;
	BYTE* heightfieldData = GetRawHeightData( gridSize, heightScale, gridSpacing, type );
	btHeightfieldTerrainShape* heightfieldShape = new btHeightfieldTerrainShape(
		gridSize,
		gridSize,
		heightfieldData,
		NULL, // heightscale is ignored for floats
		minHeight,
		maxHeight,
		upAxis,
		type,
		flipQuadEdges
		);

	// scale the shape

	btVector3 v( gridSpacing, gridSpacing, gridSpacing );
	v[upAxis] = 1.0;
	heightfieldShape->setLocalScaling( v );

	// stash this shape away
	physicsWorld->AddCollisionShape( heightfieldShape );

	// set origin to middle of heightfield
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin( btVector3( 0, -heightScale/2.f, 0 ) );

	// create ground object
	btScalar mass( 0.0 );
	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, NULL, heightfieldShape );
	rbInfo.m_startWorldTransform = tr;
	btRigidBody* body = new btRigidBody( rbInfo );

	short group = COLLIDE_MASK::GROUND;
	short mask = COLLIDE_MASK::PLAYER_CONTROLLER|COLLIDE_MASK::PLAYER_BODY|COLLIDE_MASK::FIRE_PLINTH;
	physicsWorld->World()->addRigidBody( body, group, mask );
}

BYTE* Floor::GetRawHeightData( int gridSize, float heightScale, btScalar gridSpacing, PHY_ScalarType type ) {
	long nElements = ((long)gridSize) * gridSize;
	int bytesPerElement = 0;
	switch( type ) {
	case PHY_FLOAT:
		bytesPerElement = sizeof( float );
		break;
	case PHY_UCHAR:
		bytesPerElement = sizeof( unsigned char );
		break;
	case PHY_SHORT:
		bytesPerElement = sizeof( short );
		break;
	default:
		btAssert( !"Bad heightfield data type" );
	}
	btAssert( bytesPerElement>0&&"bad bytes per element" );

	ID3D11Resource* heightfieldResource;
	HR(CreateWICTextureFromFileEx(
		device,
		L"./art/textures/floor_heightfield.tif",
		NULL,
		D3D11_USAGE_STAGING,
		NULL,
		D3D11_CPU_ACCESS_READ,
		NULL,
		true, //should be true maybe
		&heightfieldResource,
		NULL ));
	ID3D11DeviceContext* deviceContext;
	device->GetImmediateContext( &deviceContext );
	D3D11_MAPPED_SUBRESOURCE heightfieldMap;
	
	HR(deviceContext->Map( heightfieldResource, 0, D3D11_MAP_READ, NULL, &heightfieldMap ));
	Pixel* pixelData = reinterpret_cast<Pixel*>(heightfieldMap.pData);
	Pixel test0 = pixelData[0];
	Pixel test1 = pixelData[1];
	Pixel test2 = pixelData[2];
	float texWidth = heightfieldMap.RowPitch/8; // 8 bytes per pixel
	float gridToTexScale = texWidth/gridSize;
	long nBytes = nElements * bytesPerElement;
	BYTE * raw = new BYTE[nBytes];
	btAssert( raw && "out of memory" );
	BYTE* p = raw;
	for( int i = 0; i<gridSize; ++i ) {
		for( int j = 0; j<gridSize; ++j ) {
			//float value = 4.75;
			int xTexCoord = j * gridToTexScale;
			int yTexCoord = i * gridToTexScale;
			int pixel = yTexCoord*texWidth+xTexCoord;
			float value = (float)(pixelData[pixel].r/65536.f)*heightScale;
			switch( type ) {
			case PHY_FLOAT:
			{
				float * pf = (float *)p;
				*pf = value;
			}
			break;
			case PHY_UCHAR:
			{
				unsigned char * pu = (unsigned char *)p;
				*pu = (unsigned char)value;
			}
			break;
			case PHY_SHORT:
			{
				short * ps = (short *)p;
				*ps = (short)value;
			}
			break;
			default:
				btAssert( !"bad type" );
			}
			p += bytesPerElement;
		}
	}
	return raw;
}

