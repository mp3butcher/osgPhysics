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
#include "ODEInterface"

using namespace osgPhysics;

static dJointGroupID g_contactGroup;

static void nearCallback( void* data, dGeomID o1, dGeomID o2 )
{
    if ( dGeomIsSpace(o1) || dGeomIsSpace(o2) )
    {
        dSpaceCollide2( o1, o2, data, &nearCallback );
        return;
    }

    const int N = 32;
    dContact contact[N];
    int n = dCollide( o1, o2, N, &(contact[0].geom), sizeof(dContact) );
    if ( n > 0 )
    {
        InternalODEWorld* world = (InternalODEWorld*)data;
        for ( int i=0; i<n; ++i )
        {
            contact[i].surface.slip1 = 0.7;
            contact[i].surface.slip2 = 0.7;
            contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 |
                                      dContactSlip1 | dContactSlip2;
            contact[i].surface.mu = 50.0;  // was: dInfinity
            contact[i].surface.soft_erp = 0.96;
            contact[i].surface.soft_cfm = 0.04;
            
            dJointID c = dJointCreateContact( world->worldID, g_contactGroup, &contact[i] );
            dJointAttach( c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2) );
        }
    }
}

static void convertMatrix( const osg::Matrix& mat, dMatrix3& odeMat )
{
    osg::Quat q; mat.get(q);
    osg::Matrix m(q.inverse());

    odeMat[0] = m(0,0); odeMat[1] = m(0,1); odeMat[2] = m(0,2); odeMat[3] = m(0,3);
    odeMat[4] = m(1,0); odeMat[5] = m(1,1); odeMat[6] = m(1,2); odeMat[7] = m(1,3);
    odeMat[8] = m(2,0); odeMat[9] = m(2,1); odeMat[10] = m(2,2); odeMat[11] = m(2,3);
}

static InternalODEWorld* findSceneElement( BaseElement* element )
{
    InternalODEWorld* world = NULL;
    if ( element->getParentElement() )
    {
        world = element->getParentElement()->getPhysicsData<InternalODEWorld>();
    }
    if ( !world ) OSG_WARN << "[ODEInterface] The element doesn't belong to a world" << std::endl;
    return world;
}

class GeomGenerator : public osg::ShapeVisitor
{
public:
    GeomGenerator( InternalODEWorld* world, InternalODEGeom* geom )
    : _world(world), _geom(geom) {}

    virtual void apply( osg::Sphere& sphere )
    {
        if ( _geom->mass>0.0 )
        {
            dMassSetSphereTotal( &_massObj, _geom->mass, sphere.getRadius() );
            applyMassToBody();
        }
        _geom->geomID = dCreateSphere( _world->spaceID, sphere.getRadius() );
    }

    virtual void apply( osg::Box& box )
    {
        const osg::Vec3& lengths = box.getHalfLengths() * 2.0f;
        if ( _geom->mass>0.0 )
        {
            dMassSetBoxTotal( &_massObj, _geom->mass, lengths[0], lengths[1], lengths[2] );
            applyMassToBody();
        }
        _geom->geomID = dCreateBox( _world->spaceID, lengths[0], lengths[1], lengths[2] );
    }

    virtual void apply( osg::Cone& cone )
    { OSG_NOTICE << "[ODEInterface] Cone shape not implemented" << std::endl; }

    virtual void apply( osg::Cylinder& cylinder )
    {
        if ( _geom->mass>0.0 )
        {
            dMassSetCylinderTotal( &_massObj, _geom->mass, 3, cylinder.getRadius(), cylinder.getHeight() );
            applyMassToBody();
        }
        _geom->geomID = dCreateCylinder( _world->spaceID, cylinder.getRadius(), cylinder.getHeight() );
    }

    virtual void apply( osg::Capsule& capsule )
    {
        if ( _geom->mass>0.0 )
        {
            dMassSetCapsuleTotal( &_massObj, _geom->mass, 3, capsule.getRadius(), capsule.getHeight() );
            applyMassToBody();
        }
        _geom->geomID = dCreateCapsule( _world->spaceID, capsule.getRadius(), capsule.getHeight() );
    }

    virtual void apply( osg::InfinitePlane& plane )
    {
        _geom->geomID = dCreatePlane( _world->spaceID, plane[0], plane[1], plane[2], plane[3] );
    }

    virtual void apply( osg::TriangleMesh& mesh )
    {
        osg::UIntArray* indices = dynamic_cast<osg::UIntArray*>( mesh.getIndices() );
        if ( indices )
        {
            _geom->geomID = ODEInterface::createTriangleMesh( _world, _geom,
                                                              mesh.getVertices(), indices );
        }
        else
            OSG_NOTICE << "[ODEInterface] TriangleMesh not implemented" << std::endl;
    }

    virtual void apply( osg::ConvexHull& convexHull )
    {
        // TODO
    }

    virtual void apply( osg::HeightField& heightField )
    {
        // TODO
    }

    virtual void apply( osg::CompositeShape& group )
    { OSG_NOTICE << "[ODEInterface] CompositeShape not implemented" << std::endl; }

protected:
    void applyMassToBody()
    {
        dMassRotate( &_massObj, _geom->massRotation );
        dBodySetMass( _geom->bodyID, &_massObj );
    }

    InternalODEWorld* _world;
    InternalODEGeom* _geom;

    dMass _massObj;
};

ODEInterface::ODEInterface()
:   AbstractInterface()
{
    dInitODE2(0);
    g_contactGroup = dJointGroupCreate(0);
}

ODEInterface::~ODEInterface()
{
    dJointGroupEmpty( g_contactGroup );
    dJointGroupDestroy( g_contactGroup );
    dCloseODE();
}

ODEInterface::ODEInterface( const ODEInterface& copy, const osg::CopyOp& copyop )
:   AbstractInterface(copy, copyop)
{
}

dGeomID ODEInterface::createTriangleMesh( InternalODEWorld* world, InternalODEGeom* geom,
                                          osg::Vec3Array* vertices, osg::UIntArray* indices )
{
    unsigned int sizeV = vertices->size(), sizeI = indices->size();
    geom->triMeshVertices = new dReal[sizeV * 3];
    for ( unsigned int i=0; i<sizeV; ++i )
    {
        const osg::Vec3& vertex = (*vertices)[i];
        geom->triMeshVertices[i*3 + 0] = vertex.x();
        geom->triMeshVertices[i*3 + 1] = vertex.y();
        geom->triMeshVertices[i*3 + 2] = vertex.z();
    }
    
    geom->triMeshIndices = new dTriIndex[sizeI];
    for ( unsigned int i=0; i<sizeI; ++i )
    {
        geom->triMeshIndices[i] = (*indices)[i];
    }
    
    geom->triMeshDataID = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSimple( geom->triMeshDataID, geom->triMeshVertices, sizeV,
                                 geom->triMeshIndices, sizeI );
    return dCreateTriMesh(world->spaceID, geom->triMeshDataID, NULL, NULL, NULL);
}

bool ODEInterface::createWorldImplementation( BaseElement* element, osg::Node* node )
{
    if ( !element || !node )
        return false;

    InternalODEWorld* world = new InternalODEWorld;
    world->worldID = dWorldCreate();
    world->spaceID = dHashSpaceCreate(0);

    PhysicsAttribute* attr = element->getAttributeMap();
    applyWorldParameters( world, attr );
    element->setPhysicsData( world );
    return true;
}

bool ODEInterface::createRigidBodyImplementation( BaseElement* element, osg::Node* node )
{
    if ( !element || !node ) return false;

    InternalODEWorld* parentWorld = findSceneElement(element);
    if ( !parentWorld ) return false;

    InternalODEGeom* geom = new InternalODEGeom;
    geom->bodyID = dBodyCreate(parentWorld->worldID);
    dRSetIdentity( geom->massRotation );

    // Set mass attributes
    PhysicsAttribute* attr = element->getAttributeMap();
    applyRigidParameters( parentWorld, geom, attr );
    applyRigidShape( element->getComputeShapeType(), element->getShape(), node, parentWorld, geom );
    
    if ( element->getBodyType()==BaseElement::DYNAMIC_BODY )
        dGeomSetBody( geom->geomID, geom->bodyID );
    else
    {
        dBodyDestroy( geom->bodyID );
        geom->bodyID = 0;
    }
    element->setPhysicsData( geom );
    return true;
}

bool ODEInterface::createJointImplementation( Joint* joint, BaseElement* world )
{
    if ( !joint || !world )
        return false;
    
    InternalODEWorld* internalWorld = world->getPhysicsData<InternalODEWorld>();
    if ( !internalWorld || !joint->valid() )
        return false;
    
    InternalODEGeom* geom1 = joint->getElement1()->getPhysicsData<InternalODEGeom>();
    InternalODEGeom* geom2 = joint->getElement2()->getPhysicsData<InternalODEGeom>();
    if ( !geom1 || !geom2 )
        return false;
    
    InternalODEJoint* internalJoint = new InternalODEJoint;
    internalJoint->worldID = internalWorld->worldID;
    applyJointParameters( internalJoint, joint->getAttributeMap(), geom1, geom2 );
    joint->setPhysicsData( internalJoint );
    return true;
}

bool ODEInterface::resetElementImplementation( BaseElement* element, osg::Node* node )
{
    if ( !element || !node )
        return false;
    
    PhysicsAttribute* attr = element->getAttributeMap();
    if ( element->asRigidElement() )
    {
        InternalODEGeom* geom = element->getPhysicsData<InternalODEGeom>();
        if ( !geom ) return false;
        
        InternalODEWorld* parentWorld = findSceneElement(element);
        if ( !parentWorld ) return false;
        
        if ( element->getDirtyParts()&BaseElement::DIRTY_SHAPE )
        {
            // TODO: change geometry shape at runtime
        }
        
        if ( element->getDirtyParts()&BaseElement::DIRTY_PARAMETERS )
        {
            applyRigidParameters( parentWorld, geom, attr );
        }
    }
    else if ( element->asWorld() )
    {
        InternalODEWorld* world = element->getPhysicsData<InternalODEWorld>();
        if ( !world ) return false;
        
        applyWorldParameters( world, attr );
    }
    return true;
}

bool ODEInterface::resetJointImplementation( Joint* joint, BaseElement* world )
{
    if ( !joint || !world )
        return false;
    
    InternalODEWorld* internalWorld = world->getPhysicsData<InternalODEWorld>();
    InternalODEJoint* internalJoint = joint->getPhysicsData<InternalODEJoint>();
    if ( !internalWorld || !internalJoint || !joint->valid() )
        return false;
    
    InternalODEGeom* geom1 = joint->getElement1()->getPhysicsData<InternalODEGeom>();
    InternalODEGeom* geom2 = joint->getElement2()->getPhysicsData<InternalODEGeom>();
    if ( !geom1 || !geom2 )
        return false;
    
    internalJoint->worldID = internalWorld->worldID;
    applyJointParameters( internalJoint, joint->getAttributeMap(), geom1, geom2 );
    return true;
}

void ODEInterface::getBodyResultImplementation( BaseElement* element, PhysicsResult& result )
{
    if ( !element )
        return;
    else if ( element->asRigidElement() )
    {
        InternalODEGeom* geom = element->getPhysicsData<InternalODEGeom>();
        if ( geom && geom->bodyID )
        {
            const dReal* pos = dBodyGetPosition(geom->bodyID);
            const dReal* q = dBodyGetQuaternion(geom->bodyID);

            result.matrix.postMultRotate( osg::Quat(q[1], q[2], q[3], q[0]) );
            result.matrix.postMultTranslate( osg::Vec3(pos[0], pos[1], pos[2]) );
        }
    }
}

bool ODEInterface::releaseElementImplementation( BaseElement* element )
{
    if ( !element )
        return false;
    else if ( element->asWorld() )
    {
        InternalODEWorld* world = element->getPhysicsData<InternalODEWorld>();
        if ( world )
        {
            dSpaceDestroy( world->spaceID );
            dWorldDestroy( world->worldID );
            delete world;

            element->clearPhysicsData();
            return true;
        }
    }
    else if ( element->asRigidElement() )
    {
        InternalODEGeom* geom = element->getPhysicsData<InternalODEGeom>();
        if ( geom )
        {
            dGeomDestroy( geom->geomID );
            delete geom;
            
            if ( geom->triMeshDataID )
            {
                dGeomTriMeshDataDestroy( geom->triMeshDataID );
                delete geom->triMeshVertices;
                delete geom->triMeshIndices;
            }
            
            element->clearPhysicsData();
            return true;
        }
    }
    return false;
}

bool ODEInterface::releaseJointImplementation( Joint* joint, BaseElement* world )
{
    if ( !joint )
        return false;
    
    InternalODEJoint* internalJoint = joint->getPhysicsData<InternalODEJoint>();
    if ( !internalJoint )
        return false;
    
    dJointDestroy( internalJoint->jointID );
    delete internalJoint;
    joint->clearPhysicsData();
    return true;
}

void ODEInterface::simulateImplementation( double step, BaseElement* world )
{
    InternalODEWorld* internalWorld = NULL;
    if ( world )
    {
        internalWorld = world->getPhysicsData<InternalODEWorld>();
        if ( !internalWorld ) return;
    }

    if ( internalWorld )
    {
        dSpaceCollide( internalWorld->spaceID, internalWorld, &nearCallback );
        dWorldQuickStep( internalWorld->worldID, step );
        dJointGroupEmpty( g_contactGroup );
    }
}

bool ODEInterface::applyWorldParameters( InternalODEWorld* world, PhysicsAttribute* attr )
{
    if ( attr )
    {
        osg::Vec3 vec;
        if ( attr->getAttribute("gravity", vec) )
            dWorldSetGravity( world->worldID, vec[0], vec[1], vec[2] );
    }
    return true;
}

bool ODEInterface::applyRigidParameters( InternalODEWorld* world, InternalODEGeom* geom, PhysicsAttribute* attr )
{
    if ( attr )
    {
        double value = 0.0;
        if ( attr->getAttribute("mass", value) )
            geom->mass = value;

        if ( attr->getAttribute("damping", value) )
            dBodySetLinearDamping( geom->bodyID, value );

        if ( attr->getAttribute("angular_damping", value) )
            dBodySetAngularDamping( geom->bodyID, value );

        osg::Matrix mat;
        if ( attr->getAttribute("mass_matrix", mat) )
            convertMatrix( mat, geom->massRotation );
        
        if ( attr->getAttribute("matrix", mat) )
        {
            dMatrix3 rotation;
            convertMatrix( mat, rotation );
            osg::Vec3 trans = mat.getTrans();

            dBodySetPosition( geom->bodyID, trans[0], trans[1], trans[2] );
            dBodySetRotation( geom->bodyID, rotation );
        }

        osg::Vec3 vec;
        if ( attr->getAttribute("velocity", vec) )
            dBodySetLinearVel( geom->bodyID, vec[0], vec[1], vec[2] );

        if ( attr->getAttribute("angular_velocity", vec) )
            dBodySetAngularVel( geom->bodyID, vec[0], vec[1], vec[2] );
    }
    return true;
}

bool ODEInterface::applyRigidShape( BaseElement::ComputeShapeType type, osg::Shape* shape, osg::Node* node, 
                                    InternalODEWorld* world, InternalODEGeom* geom )
{
    if ( type==BaseElement::USE_SHAPE_OBJECT && shape )
    {
        GeomGenerator generator( world, geom );
        shape->accept( generator );
    }
    else if ( type==BaseElement::USE_CHILD_DRAWABLES )
    {
        ComputeTrianglesVisitor ctv;
        node->accept( ctv );
        geom->geomID = createTriangleMesh( world, geom, ctv.getVertexArray(), ctv.getIndexArray() );
        
        dMass massObj;
        if ( geom->mass>0.0 )
        {
            // FIXME: crash when using setTrimesh()
            //dMassSetTrimeshTotal( &massObj, geom->mass, geom->geomID );
            osg::ComputeBoundsVisitor cbv; node->accept( cbv );
            osg::BoundingBox& bb = cbv.getBoundingBox();
            osg::Vec3 lengths = (bb._max - bb._min);
            
            dMassSetBoxTotal( &massObj, geom->mass, lengths[0], lengths[1], lengths[2] );
            dMassRotate( &massObj, geom->massRotation );
            dBodySetMass( geom->bodyID, &massObj );
        }
    }
    else if ( type==BaseElement::USE_BOUNDING_BOX )
    {
        osg::ComputeBoundsVisitor cbv; node->accept( cbv );
        osg::BoundingBox& bb = cbv.getBoundingBox();
        osg::Vec3 lengths = (bb._max - bb._min);
        
        dMass massObj;
        if ( geom->mass>0.0 )
        {
            dMassSetBoxTotal( &massObj, geom->mass, lengths[0], lengths[1], lengths[2] );
            dMassRotate( &massObj, geom->massRotation );
            dBodySetMass( geom->bodyID, &massObj );
        }
        geom->geomID = dCreateBox( world->spaceID, lengths[0], lengths[1], lengths[2] );
    }
    else
    {
        dMass massObj;
        if ( geom->mass>0.0 )
        {
            dMassSetSphereTotal( &massObj, geom->mass, node->getBound().radius() );
            dMassRotate( &massObj, geom->massRotation );
            dBodySetMass( geom->bodyID, &massObj );
        }
        geom->geomID = dCreateSphere( world->spaceID, node->getBound().radius() );
    }
    if ( !geom->geomID ) return false;
    return true;
}

bool ODEInterface::applyJointParameters( InternalODEJoint* joint, JointAttribute* attr,
                                         InternalODEGeom* geom1, InternalODEGeom* geom2 )
{
    if ( attr )
    {
        osg::Vec3 vec;
        if ( joint->jointID )
            dJointDestroy( joint->jointID );
        
        switch ( attr->getType() )
        {
        case JointAttribute::FIXED_JOINT:
            joint->jointID = dJointCreateFixed(joint->worldID, 0);
            dJointAttach( joint->jointID, geom1->bodyID, geom2->bodyID );
            dJointSetFixed( joint->jointID );
            return true;
        case JointAttribute::BALL_JOINT:
            joint->jointID = dJointCreateBall(joint->worldID, 0);
            dJointAttach( joint->jointID, geom1->bodyID, geom2->bodyID );
            if ( attr->getAttribute("anchor", vec) )
                dJointSetBallAnchor( joint->jointID, vec.x(), vec.y(), vec.z() );
            return true;
        case JointAttribute::HINGE_JOINT:
            joint->jointID = dJointCreateHinge(joint->worldID, 0);
            dJointAttach( joint->jointID, geom1->bodyID, geom2->bodyID );
            if ( attr->getAttribute("anchor", vec) )
                dJointSetHingeAnchor( joint->jointID, vec.x(), vec.y(), vec.z() );
            if ( attr->getAttribute("axis", vec) )
                dJointSetHingeAxis( joint->jointID, vec.x(), vec.y(), vec.z() );
            return true;
        case JointAttribute::SLIDER_JOINT:
            joint->jointID = dJointCreateSlider(joint->worldID, 0);
            dJointAttach( joint->jointID, geom1->bodyID, geom2->bodyID );
            if ( attr->getAttribute("axis", vec) )
                dJointSetSliderAxis( joint->jointID, vec.x(), vec.y(), vec.z() );
            return true;
        case JointAttribute::PISTON_JOINT:
            joint->jointID = dJointCreatePiston(joint->worldID, 0);
            dJointAttach( joint->jointID, geom1->bodyID, geom2->bodyID );
            if ( attr->getAttribute("anchor", vec) )
                dJointSetPistonAnchor( joint->jointID, vec.x(), vec.y(), vec.z() );
            if ( attr->getAttribute("axis", vec) )
                dJointSetPistonAxis( joint->jointID, vec.x(), vec.y(), vec.z() );
            return true;
        case JointAttribute::UNIVERSAL_JOINT:
            joint->jointID = dJointCreateUniversal(joint->worldID, 0);
            dJointAttach( joint->jointID, geom1->bodyID, geom2->bodyID );
            if ( attr->getAttribute("anchor", vec) )
                dJointSetUniversalAnchor( joint->jointID, vec.x(), vec.y(), vec.z() );
            if ( attr->getAttribute("axis1", vec) )
                dJointSetUniversalAxis1( joint->jointID, vec.x(), vec.y(), vec.z() );
            if ( attr->getAttribute("axis2", vec) )
                dJointSetUniversalAxis2( joint->jointID, vec.x(), vec.y(), vec.z() );
            return true;
        default:
            OSG_WARN << "[ODEInterface] Unknown joint type applied" << std::endl;
            return false;
        }
        
        // TODO: max_force / max_torque?
    }
    return false;
}
