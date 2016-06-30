/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2011 Robert
 *
 * This library is open source and may be redistributed and/or modified under  
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or 
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * OpenSceneGraph Public License for more details.
*/
// Written by Wang Rui & Sukender

#include <osg/Notify>
#include <osg/ComputeBoundsVisitor>
#include <sstream>
#include <iostream>
#include "PhysXInterface"

using namespace osgPhysics;

static NxPhysicsSDK* g_physicsSDK = NULL;
static NxCookingInterface* g_cooking = NULL;

void osgPhysics::setPhysxInstance( NxPhysicsSDK* sdk )
{
    if ( g_physicsSDK )
    {
        NxReleasePhysicsSDK( g_physicsSDK );
    }
    g_physicsSDK = sdk;
}

NxPhysicsSDK* osgPhysics::physxInstance() { return g_physicsSDK; }
NxCookingInterface* osgPhysics::cookingInstance() { return g_cooking; }

class UserMemoryStream : public NxStream
{
public:
    UserMemoryStream( const NxU8* input=0 )
    : _currentSize(0), _maxSize(0), _buffer(0) { _input = input; }
    
    virtual ~UserMemoryStream()
    { NX_DELETE_ARRAY(_buffer); }
    
    #define READ_TYPE(TYPE, FUNC) \
    virtual TYPE FUNC() const { TYPE v; memcpy(&v,_input,sizeof(TYPE)); _input+=sizeof(TYPE); return v; }
    
    READ_TYPE( NxU8, readByte )
    READ_TYPE( NxU16, readWord )
    READ_TYPE( NxU32, readDword )
    READ_TYPE( float, readFloat )
    READ_TYPE( double, readDouble )
    
    virtual void readBuffer( void* dest, NxU32 size ) const
    { memcpy(dest, _input, size); _input += size; }
    
    #define WRITE_TYPE(TYPE, FUNC) \
    virtual NxStream& FUNC(TYPE v) { storeBuffer(&v, sizeof(TYPE)); return *this; }
    
    WRITE_TYPE( NxU8, storeByte )
    WRITE_TYPE( NxU16, storeWord )
    WRITE_TYPE( NxU32, storeDword )
    WRITE_TYPE( NxReal, storeFloat )
    WRITE_TYPE( NxF64, storeDouble )
    
    virtual NxStream& storeBuffer( const void* data, NxU32 size )
    {
        NxU32 expectedSize = _currentSize + size;
        if ( expectedSize>_maxSize )
        {
            _maxSize = expectedSize + 4096;
            
            NxU8* newData = new NxU8[_maxSize];
            NX_ASSERT( newData!=NULL );
            
            if ( _buffer )
            {
                memcpy( newData, _buffer, _currentSize );
                delete[] _buffer;
            }
            _buffer = newData;
        }
        memcpy( _buffer+_currentSize, data, size );
        _currentSize += size;
        return *this;
    }
    
    NxU32 _currentSize;
    NxU32 _maxSize;
    NxU8* _buffer;
    mutable const NxU8* _input;
};

PhysXInterface::PhysXInterface()
:   AbstractInterface()
{
    if ( !g_cooking )
    {
        g_cooking = NxGetCookingLib( NX_PHYSICS_SDK_VERSION );
    }
}

PhysXInterface::~PhysXInterface()
{
#if 0
    // FIXME: this will cause R6025 error under Windows?
    if ( g_physicsSDK )
    {
        NxReleasePhysicsSDK( g_physicsSDK );
    }
#endif
}

PhysXInterface::PhysXInterface( const PhysXInterface& copy, const osg::CopyOp& copyop )
:   AbstractInterface(copy, copyop)
{
}

bool PhysXInterface::createTriangleMesh( NxActorDesc* actor, osg::Vec3Array* vertices, osg::UIntArray* indices )
{
    unsigned int sizeV = vertices->size(), sizeI = indices->size();
    NxVec3* meshVertices = new NxVec3[sizeV];
    for ( unsigned int i=0; i<sizeV; ++i )
    {
        const osg::Vec3& vertex = (*vertices)[i];
        meshVertices[i].set( vertex.x(), vertex.y(), vertex.z() );
    }
    
    NxU32* meshTriangles = new NxU32[sizeI];
    for ( unsigned int i=0; i<sizeI; ++i )
    {
        meshTriangles[i] = (*indices)[i];
    }
    
    NxTriangleMeshDesc meshDesc;
    meshDesc.flags = 0;
    meshDesc.numVertices = sizeV;
    meshDesc.numTriangles = sizeI / 3;
    meshDesc.pointStrideBytes = sizeof(NxVec3);
    meshDesc.triangleStrideBytes = 3 * sizeof(NxU32);
    meshDesc.points = meshVertices;
    meshDesc.triangles = meshTriangles;
    
    if ( g_cooking )
    {
        UserMemoryStream writeStream;
        g_cooking->NxInitCooking();
        g_cooking->NxCookTriangleMesh( meshDesc, writeStream );
        
        UserMemoryStream readStream(writeStream._buffer);
        NxTriangleMesh* meshData = g_physicsSDK->createTriangleMesh(readStream);;
        
        NxTriangleMeshShapeDesc* meshShapeDesc = new NxTriangleMeshShapeDesc;
        meshShapeDesc->meshData = meshData;
        meshShapeDesc->shapeFlags = NX_SF_FEATURE_INDICES;
        actor->shapes.pushBack( meshShapeDesc );
    }
    
    delete meshVertices;
    delete meshTriangles;
    return g_cooking!=NULL;
}

bool PhysXInterface::createSoftBodyMesh( const VerticesData& data, NxSoftBodyDesc* body )
{
    NxSoftBodyMeshDesc meshDesc;
    meshDesc.numVertices = data.maxVertices;
    meshDesc.numTetrahedra = data.maxIndices / 4;
    meshDesc.vertexStrideBytes = sizeof(NxVec3);
    meshDesc.tetrahedronStrideBytes = 4*sizeof(NxU32);
    meshDesc.vertexMassStrideBytes = sizeof(NxReal);
    meshDesc.vertexFlagStrideBytes = sizeof(NxU32);
    meshDesc.vertices = data.position;
    meshDesc.tetrahedra = data.indicesData;
    meshDesc.vertexMasses = 0;
    meshDesc.vertexFlags = 0;
    meshDesc.flags = 0;
    
    if ( g_cooking )
    {
        UserMemoryStream writeStream;
        g_cooking->NxInitCooking();
        g_cooking->NxCookSoftBodyMesh( meshDesc, writeStream );
        
        UserMemoryStream readStream(writeStream._buffer);
        body->softBodyMesh = g_physicsSDK->createSoftBodyMesh(readStream);
    }
    return g_cooking!=NULL;
}

bool PhysXInterface::createVerticesData( NxMeshData& meshData, VerticesData* verticesData,
                                         osg::Vec3Array* vertices, osg::Vec3Array* normals, osg::UIntArray* indices )
{
    if ( !vertices || !normals || !indices )
        return false;
    
    verticesData->maxVertices = vertices->size();
    verticesData->maxIndices  = indices->size();
    verticesData->position = (NxVec3*)malloc( sizeof(NxVec3) * verticesData->maxVertices );
    memset( verticesData->position, 0, sizeof(NxVec3) * verticesData->maxVertices );
    
    verticesData->normal = (NxVec3*)malloc( sizeof(NxVec3) * verticesData->maxVertices );
    memset( verticesData->normal, 0, sizeof(NxVec3) * verticesData->maxVertices );
    
    for ( unsigned int i=0; i<verticesData->maxVertices; ++i )
    {
        const osg::Vec3& vertex = (*vertices)[i];
        verticesData->position[i].set( vertex.x(), vertex.y(), vertex.z() );
        if ( i<normals->size() )
        {
            const osg::Vec3& normalv = (*normals)[i];
            verticesData->normal[i].set( normalv.x(), normalv.y(), normalv.z() );
        }
    }
    
    verticesData->indicesData = (NxU32*)malloc( sizeof(NxU32) * verticesData->maxIndices );
    memset( verticesData->indicesData, 0, sizeof(NxU32) * verticesData->maxIndices );
    
    for ( unsigned int i=0; i<verticesData->maxIndices; ++i )
    {
        verticesData->indicesData[i] = (*indices)[i];
    }
    
    meshData.verticesPosBegin = &(verticesData->position[0].x);
    meshData.verticesNormalBegin = &(verticesData->normal[0].x);
    meshData.verticesPosByteStride = sizeof(NxVec3);
    meshData.verticesNormalByteStride = sizeof(NxVec3);
    meshData.maxVertices = verticesData->maxVertices;
    meshData.numVerticesPtr = &verticesData->numIndices;
    
    // The number of triangles is constant, even if the cloth is torn
    meshData.indicesBegin = verticesData->indicesData;
    meshData.indicesByteStride = sizeof(NxU32);
    meshData.maxIndices = verticesData->maxIndices;
    meshData.numIndicesPtr = &verticesData->numIndices;
    
    // The parent index information would be needed if we used textured cloth
    meshData.parentIndicesBegin = (NxU32*)malloc(sizeof(NxU32) * verticesData->maxVertices);
    meshData.parentIndicesByteStride = sizeof(NxU32);
    meshData.maxParentIndices = verticesData->maxVertices;
    meshData.numParentIndicesPtr = &verticesData->numParentIndices;
    meshData.dirtyBufferFlagsPtr = &verticesData->meshDirtyFlags;
    return true;
}

NxScene* PhysXInterface::findSceneElement( BaseElement* element )
{
    NxScene* scene = NULL;
    if ( element->getParentElement() )
    {
        scene = element->getParentElement()->getPhysicsData<NxScene>();
    }
    if ( !scene ) OSG_WARN << "[PhysXInterface] The element doesn't belong to a world" << std::endl;
    return scene;
}

void PhysXInterface::convertMatrix( const osg::Matrix& mat, NxMat34& nxMat )
{
    NxF32 d[16];
    for ( int i=0; i<16; ++i )
        d[i] = *(mat.ptr() + i);
    nxMat.setColumnMajor44( &d[0] );
}

bool PhysXInterface::createWorldImplementation( BaseElement* element, osg::Node* node )
{
    if ( !element || !node )
        return false;
    
    PhysicsAttribute* attr = element->getAttributeMap();
    NxSceneDesc sceneDesc;
    
    if ( attr )
    {
        osg::Vec3 vec;
        if ( attr->getAttribute("gravity", vec) )
            sceneDesc.gravity = NxVec3(vec.x(), vec.y(), vec.z());
    }
    
    NxScene* scene = g_physicsSDK->createScene(sceneDesc);
    if ( !scene )
    {
        OSG_WARN << "[PhysXInterface] Unable to initialize the PhysX scene" << std::endl;
        return false;
    }
    
    // FIXME: default material settings. Make it user-changable?
    NxMaterial* defaultMaterial = scene->getMaterialFromIndex(0);
    defaultMaterial->setRestitution(0.5f);
    defaultMaterial->setStaticFriction(0.5f);
    defaultMaterial->setDynamicFriction(0.5f);
    
    element->setPhysicsData( scene );
    return true;
}

bool PhysXInterface::createJointImplementation( Joint* joint, BaseElement* world )
{
    if ( !joint || !world )
        return false;
    
    NxScene* scene = world->getPhysicsData<NxScene>();
    if ( !scene || !joint->valid() )
        return false;
    
    NxActor* actor1 = joint->getElement1()->getPhysicsData<NxActor>();
    NxActor* actor2 = joint->getElement2()->getPhysicsData<NxActor>();
    if ( !actor1 || !actor2 )
        return false;
    
    NxJoint* internalJoint = createSpecifiedJoint( scene, joint->getAttributeMap(), actor1, actor2 );
    joint->setPhysicsData( internalJoint );
    return true;
}

bool PhysXInterface::resetElementImplementation( BaseElement* element, osg::Node* node )
{
    if ( !element || !node )
        return false;
    
    if ( element->asWorld() )
    {
        NxScene* scene = element->getPhysicsData<NxScene>();
        if ( !scene ) return false;
        
        // Set parameters
        PhysicsAttribute* attr = element->getAttributeMap();
        if ( attr )
        {
            osg::Vec3 vec;
            
            if ( attr->getAttribute("gravity", vec) )
                scene->setGravity( NxVec3(vec.x(), vec.y(), vec.z()) );
        }
    }
    else if ( element->asRigidElement() )
    {
        return resetRigidElement( element, node );
    }
    else
    {
        // TODO: handle softbody and cloth here
    }
    return true;
}

bool PhysXInterface::resetJointImplementation( Joint* joint, BaseElement* world )
{
    // FIXME: could it be done in such a cheap way?
    releaseJoint( joint, world );
    createJoint( joint, world );
    return false;
}

void PhysXInterface::getBodyResultImplementation( BaseElement* element, PhysicsResult& result )
{
    if ( !element )
        return;
    else if ( element->asRigidElement() )
    {
        NxActor* actor = element->getPhysicsData<NxActor>();
        if ( actor )
        {
            float mat[16];
            actor->getGlobalPose().getColumnMajor44(mat);
            result.matrix.set( &mat[0] );
        }
    }
    else if ( element->asSoftBodyElement() || element->asClothElement() )
    {
        if ( !result.vertices ) result.vertices = new osg::Vec3Array;
        if ( !result.normals ) result.normals = new osg::Vec3Array;
        
        VerticesData* verticesData = element->getPhysicsData<VerticesData>();
        if ( verticesData && verticesData->meshDirtyFlags )
        {
            result.vertices->resize( verticesData->maxVertices );
            result.normals->resize( verticesData->maxVertices );
            for ( unsigned int i=0; i<verticesData->maxVertices; ++i )
            {
                NxVec3* position = &(verticesData->position[i]);
                (*result.vertices)[i].set( position->x, position->y, position->z );
                
                NxVec3* normal = &(verticesData->normal[i]);
                (*result.normals)[i].set( normal->x, normal->y, normal->z );
            }
        }
    }
}

bool PhysXInterface::releaseElementImplementation( BaseElement* element )
{
    if ( !element )
        return false;
    else if ( element->asWorld() )
    {
        NxScene* scene = element->getPhysicsData<NxScene>();
        if ( scene )
        {
            g_physicsSDK->releaseScene( *scene );
            element->clearPhysicsData();
            return true;
        }
    }
    else if ( element->asRigidElement() )
    {
        NxActor* actor = element->getPhysicsData<NxActor>();
        if ( actor )
        {
            NxScene* parentScene = findSceneElement(element);
            if ( parentScene )
            {
                parentScene->releaseActor( *actor );
                element->clearPhysicsData();
            }
            return parentScene!=NULL;
        }
    }
    else
    {
        // TODO: handle softbody and cloth here
    }
    return false;
}

bool PhysXInterface::releaseJointImplementation( Joint* joint, BaseElement* world )
{
    if ( !joint || !world )
        return false;
    else if ( world->asWorld() )
    {
        NxJoint* internalJoint = joint->getPhysicsData<NxJoint>();
        NxScene* parentScene = world->getPhysicsData<NxScene>();
        if ( internalJoint && parentScene )
        {
            parentScene->releaseJoint( *internalJoint );
            joint->clearPhysicsData();
            return true;
        }
    }
    return false;
}

void PhysXInterface::simulateImplementation( double step, BaseElement* world )
{
    NxScene* scene = NULL;
    if ( world )
    {
        scene = world->getPhysicsData<NxScene>();
        if ( !scene ) return;
    }
    
    if ( scene )
    {
        scene->simulate(step);
        scene->flushStream();
        scene->fetchResults(NX_RIGID_BODY_FINISHED, true);
    }
}

NxJoint* PhysXInterface::createSpecifiedJoint( NxScene* scene, JointAttribute* attr,
                                               NxActor* actor1, NxActor* actor2 )
{
    NxJoint* internalJoint = NULL;
    if ( attr )
    {
        osg::Vec3 vec;
        switch ( attr->getType() )
        {
        case JointAttribute::FIXED_JOINT:
            {
                NxFixedJointDesc desc;
                desc.actor[0] = actor1; desc.actor[1] = actor2;
                internalJoint = scene->createJoint( desc );
            }
            break;
        case JointAttribute::BALL_JOINT:
            {
                NxSphericalJointDesc desc;
                desc.actor[0] = actor1; desc.actor[1] = actor2;
                if ( attr->getAttribute("anchor", vec) )
                    desc.setGlobalAnchor( NxVec3(vec[0], vec[1], vec[2]) );
                internalJoint = scene->createJoint( desc );
            }
            break;
        case JointAttribute::HINGE_JOINT:
            {
                NxRevoluteJointDesc desc;
                desc.actor[0] = actor1; desc.actor[1] = actor2;
                if ( attr->getAttribute("anchor", vec) )
                    desc.setGlobalAnchor( NxVec3(vec[0], vec[1], vec[2]) );
                if ( attr->getAttribute("axis", vec) )
                    desc.setGlobalAxis( NxVec3(vec[0], vec[1], vec[2]) );
                internalJoint = scene->createJoint( desc );
            }
            break;
        case JointAttribute::SLIDER_JOINT:
            {
                NxPrismaticJointDesc desc;
                desc.actor[0] = actor1; desc.actor[1] = actor2;
                if ( attr->getAttribute("axis", vec) )
                    desc.setGlobalAxis( NxVec3(vec[0], vec[1], vec[2]) );
                internalJoint = scene->createJoint( desc );
            }
            break;
        case JointAttribute::PISTON_JOINT:
            {
                NxCylindricalJointDesc desc;
                desc.actor[0] = actor1; desc.actor[1] = actor2;
                if ( attr->getAttribute("anchor", vec) )
                    desc.setGlobalAnchor( NxVec3(vec[0], vec[1], vec[2]) );
                if ( attr->getAttribute("axis", vec) )
                    desc.setGlobalAxis( NxVec3(vec[0], vec[1], vec[2]) );
                internalJoint = scene->createJoint( desc );
            }
            break;
        case JointAttribute::UNIVERSAL_JOINT:
            // FIXME: how to define universal joint here?
            OSG_WARN << "[PhysXInterface] Universal joint not implemented at present" << std::endl;
            break;
        default:
            OSG_WARN << "[PhysXInterface] Unknown joint type applied" << std::endl;
            break;
        }
        
        if ( internalJoint )
        {
            double maxForce = NX_MAX_REAL, maxTorque = NX_MAX_REAL;
            attr->getAttribute( "max_force", maxForce );
            attr->getAttribute( "max_torque", maxTorque );
            internalJoint->setBreakable( maxForce, maxTorque );
        }
    }
    return internalJoint;
}
