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
#include <osgPhysics/RigidElement>
#include <sstream>
#include <iostream>
#include "PhysXInterface"

using namespace osgPhysics;

class BodyGenerator : public osg::ShapeVisitor
{
public:
    BodyGenerator( NxActor* actor )
    : _actor(actor), _actorDesc(NULL), useCooking(false) {}
    BodyGenerator( NxActorDesc* actorDesc )
    : _actor(NULL), _actorDesc(actorDesc), useCooking(false) {}
    
    bool useCooking;
    
    virtual void apply( osg::Sphere& sphere )
    {
        NxSphereShapeDesc* sphereDesc = new NxSphereShapeDesc;
        sphereDesc->radius = sphere.getRadius();
        applyShapeDesc( sphereDesc );
    }
    
    virtual void apply( osg::Box& box )
    {
        const osg::Vec3& lengths = box.getHalfLengths();
        NxBoxShapeDesc* boxDesc = new NxBoxShapeDesc;
        boxDesc->dimensions = NxVec3(lengths.x(), lengths.y(), lengths.z());
        applyShapeDesc( boxDesc );
    }
    
    virtual void apply( osg::Cone& )
    { OSG_NOTICE << "[PhysXInterface] Cone shape not implemented" << std::endl; }
    
    virtual void apply( osg::Cylinder& )
    { OSG_NOTICE << "[PhysXInterface] Cylinder shape not implemented" << std::endl; }
    
    virtual void apply( osg::Capsule& capsule )
    {
        NxCapsuleShapeDesc* capsuleDesc = new NxCapsuleShapeDesc;
        capsuleDesc->radius = capsule.getRadius();
        capsuleDesc->height = capsule.getHeight();
        applyShapeDesc( capsuleDesc );
    }
    
    virtual void apply( osg::InfinitePlane& plane )
    {
        NxPlaneShapeDesc* planeDesc = new NxPlaneShapeDesc;
        planeDesc->normal = NxVec3(plane[0], plane[1], plane[2]);
        planeDesc->d = plane[3];
        applyShapeDesc( planeDesc );
    }
    
    virtual void apply( osg::TriangleMesh& mesh )
    {
        osg::UIntArray* indices = dynamic_cast<osg::UIntArray*>( mesh.getIndices() );
        if ( _actor || !indices )
            OSG_NOTICE << "[PhysXInterface] Dynamic triangleMesh shape not implemented" << std::endl;
        else if ( _actorDesc )
            useCooking = PhysXInterface::createTriangleMesh( _actorDesc, mesh.getVertices(), indices );
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
    {
        for ( unsigned int i=0; i<group.getNumChildren(); ++i )
        {
            group.getChild(i)->accept( *this );
        }
    }
    
protected:
    void applyShapeDesc( NxShapeDesc* shape )
    {
        if ( _actor )
        {
            _actor->createShape( *shape );
            delete shape;
        }
        else if ( _actorDesc ) _actorDesc->shapes.pushBack( shape );
    }
    
    NxActor* _actor;
    NxActorDesc* _actorDesc;
};

bool PhysXInterface::createRigidBodyImplementation( BaseElement* element, osg::Node* node )
{
    bool actorCreated = false, useCooking = false;
    if ( !element || !node ) return false;
    
    PhysicsAttribute* attr = element->getAttributeMap();
    NxActorDesc actorDesc;
    NxBodyDesc bodyDesc;
    
    NxScene* parentScene = findSceneElement(element);
    if ( !parentScene ) return false;
    
    // Set shape
    if ( element->getComputeShapeType()==BaseElement::USE_SHAPE_OBJECT && element->getShape() )
    {
        BodyGenerator generator( &actorDesc );
        element->getShape()->accept( generator );
        useCooking = generator.useCooking;
    }
    else if ( element->getComputeShapeType()==BaseElement::USE_CHILD_DRAWABLES )
    {
        // Dynamic triangle mesh is never as cool as we want
        // Be careful to use it in PhysX
        ComputeTrianglesVisitor ctv;
        node->accept( ctv );
        useCooking = createTriangleMesh( &actorDesc, ctv.getVertexArray(), ctv.getIndexArray() );
    }
    else if ( element->getComputeShapeType()==BaseElement::USE_BOUNDING_BOX )
    {
        osg::ComputeBoundsVisitor cbv; node->accept( cbv );
        osg::BoundingBox& bb = cbv.getBoundingBox();
        osg::Vec3 lengths = (bb._max - bb._min) * 0.5f;
        
        NxBoxShapeDesc* boxDesc = new NxBoxShapeDesc;
        boxDesc->dimensions = NxVec3(lengths.x(), lengths.y(), lengths.z());
        actorDesc.shapes.pushBack( boxDesc );
    }
    else
    {
        // FIXME: not handle scaling of parent nodes?
        NxSphereShapeDesc* sphereDesc = new NxSphereShapeDesc;
        sphereDesc->radius = node->getBound().radius();
        actorDesc.shapes.pushBack( sphereDesc );
    }
    
    // Set attributes
    if ( attr )
    {
        double value = 0.0;
        if ( attr->getAttribute("mass", value) )
            bodyDesc.mass = value;
        
        if ( attr->getAttribute("damping", value) )
            bodyDesc.linearDamping = value;
        
        if ( attr->getAttribute("angular_damping", value) )
            bodyDesc.angularDamping = value;
        
        osg::Matrix mat;
        if ( attr->getAttribute("mass_matrix", mat) )
            convertMatrix( mat, bodyDesc.massLocalPose );
        
        if ( attr->getAttribute("matrix", mat) )
            convertMatrix( mat, actorDesc.globalPose );
        
        osg::Vec3 vec;
        if ( attr->getAttribute("velocity", vec) )
            bodyDesc.linearVelocity = NxVec3(vec.x(), vec.y(), vec.z());
        
        if ( attr->getAttribute("angular_velocity", vec) )
            bodyDesc.angularVelocity = NxVec3(vec.x(), vec.y(), vec.z());
        
        NxMaterialDesc material;
        bool hasMaterialDesc = false;
        if ( attr->getAttribute("dynamic_friction", value) )
        {
            material.dynamicFriction = value;
            hasMaterialDesc = true;
        }
        
        if ( attr->getAttribute("static_friction", value) )
        {
            material.staticFriction = value;
            hasMaterialDesc = true;
        }
        
        if ( hasMaterialDesc )
        {
            NxMaterial* actorMaterial = parentScene->createMaterial(material);
            for ( unsigned int i=0; i<actorDesc.shapes.size(); ++i )
                actorDesc.shapes[i]->materialIndex = actorMaterial->getMaterialIndex();
        }
    }
    
    // Create the actor
    if ( element->getBodyType()==BaseElement::DYNAMIC_BODY )
        actorDesc.body = &bodyDesc;
    
    NxActor* actor = parentScene->createActor(actorDesc);
    if ( actor )
    {
        element->setPhysicsData( actor );
        actorCreated = true;
    }
    
    if ( cookingInstance() && useCooking )
    {
        cookingInstance()->NxCloseCooking();
    }
    
    // Delete shape descriptions created before
    for ( unsigned int i=0; i<actorDesc.shapes.size(); ++i )
        delete actorDesc.shapes[i];
    return actorCreated;
}

bool PhysXInterface::resetRigidElement( BaseElement* element, osg::Node* node )
{
    NxMaterial* actorMaterial = NULL;
    PhysicsAttribute* attr = element->getAttributeMap();
    
    NxActor* actor = element->getPhysicsData<NxActor>();
    if ( !actor ) return false;
    
    NxScene* parentScene = findSceneElement(element);
    if ( !parentScene ) return false;
    
    if ( element->getDirtyParts()&BaseElement::DIRTY_SHAPE )
    {
        // Set shape
        std::vector<NxShape*> existingShapes( actor->getNbShapes() );
        for ( unsigned int i=0; i<existingShapes.size(); ++i )
            existingShapes[i] = actor->getShapes()[i];
        
        if ( element->getComputeShapeType()==BaseElement::USE_SHAPE_OBJECT && element->getShape() )
        {
            BodyGenerator generator( actor );
            element->getShape()->accept( generator );
        }
        else if ( element->getComputeShapeType()==BaseElement::USE_CHILD_DRAWABLES )
        {
            OSG_WARN << "[PhysXInterface] Dynamic modifying of triangle mesh shape is not implemented "
                     << "and not recommended in PhysX" << std::endl;
        }
        else if ( element->getComputeShapeType()==BaseElement::USE_BOUNDING_BOX )
        {
            osg::ComputeBoundsVisitor cbv; node->accept( cbv );
            osg::BoundingBox& bb = cbv.getBoundingBox();
            osg::Vec3 lengths = (bb._max - bb._min) * 0.5f;
            
            NxBoxShapeDesc boxDesc;
            boxDesc.dimensions = NxVec3(lengths.x(), lengths.y(), lengths.z());
            actor->createShape( boxDesc );
        }
        else
        {
            // FIXME: not handle scaling of parent nodes?
            NxSphereShapeDesc sphereDesc;
            sphereDesc.radius = node->getBound().radius();
            actor->createShape( sphereDesc );
        }
        
        // Remove previous shapes
        for ( unsigned int i=0; i<existingShapes.size(); ++i )
            actor->releaseShape( *(existingShapes[i]) );
    }
    
    if ( element->getDirtyParts()&BaseElement::DIRTY_PARAMETERS )
    {
        // Set parameters
        if ( attr )
        {
            double value = 0.0;
            if ( attr->getAttribute("mass", value) )
                actor->setMass( value );
            
            if ( attr->getAttribute("damping", value) )
                actor->setLinearDamping( value );
            
            if ( attr->getAttribute("angular_damping", value) )
                actor->setAngularDamping( value );
            
            osg::Matrix mat; NxMat34 nxMat;
            if ( attr->getAttribute("mass_matrix", mat) )
            {
                convertMatrix( mat, nxMat );
                actor->setCMassGlobalPose( nxMat );
            }
            
            if ( attr->getAttribute("matrix", mat) )
            {
                convertMatrix( mat, nxMat );
                actor->setGlobalPose( nxMat );
            }
            
            osg::Vec3 vec;
            if ( attr->getAttribute("velocity", vec) )
                actor->setLinearVelocity( NxVec3(vec.x(), vec.y(), vec.z()) );
            
            if ( attr->getAttribute("angular_velocity", vec) )
                actor->setAngularVelocity( NxVec3(vec.x(), vec.y(), vec.z()) );
            
            if ( actor->getShapes() )
            {
                // FIXME: we suppose that all shapes use the same material at present
                NxMaterialIndex matIndex = actor->getShapes()[0]->getMaterial();
                if ( matIndex>0 ) actorMaterial = parentScene->getMaterialFromIndex(matIndex);
                
                if ( actorMaterial )
                {
                    if ( attr->getAttribute("dynamic_friction", value) )
                        actorMaterial->setDynamicFriction( value );
                    
                    if ( attr->getAttribute("static_friction", value) )
                        actorMaterial->setStaticFriction( value );
                }
            }
        }  // if (attr)
    }
    return true;
}
