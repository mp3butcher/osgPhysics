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
#include <osgPhysics/SoftBodyElement>
#include <sstream>
#include <iostream>
#include "PhysXInterface"

using namespace osgPhysics;

bool PhysXInterface::createSoftBodyImplementation( BaseElement* element, osg::Node* node )
{
    if ( !element || !node )
        return false;
    else if ( !cookingInstance() )
    {
        OSG_WARN << "[PhysXInterface] Soft body must make use of the cooking interface" << std::endl;
        return false;
    }
    
    PhysicsAttribute* attr = element->getAttributeMap();
    NxSoftBodyDesc bodyDesc;
    
    NxScene* parentScene = findSceneElement(element);
    if ( !parentScene ) return false;
    
    if ( element->getComputeShapeType()!=BaseElement::USE_CHILD_DRAWABLES )
    {
        OSG_NOTICE << "[PhysXInterface] Soft body must compute the initial shape using drawables" << std::endl;
    }
    
    ComputeTrianglesVisitor ctv;
    node->accept( ctv );
    
    VerticesData* softBodyData = new VerticesData;
    softBodyData->maxIndices = ctv.getIndexArray()->size();
    
    NxMeshData meshData;
    createVerticesData( meshData, softBodyData, ctv.getVertexArray(), ctv.getNormalArray(), ctv.getIndexArray() );
    bodyDesc.meshData = meshData;
    
    bool useCooking = createSoftBodyMesh( *softBodyData, &bodyDesc );
    NxSoftBody* softBody = parentScene->createSoftBody( bodyDesc );
    if ( softBody )
    {
        softBodyData->actor = softBody;
        element->setPhysicsData<VerticesData>( softBodyData );
        
        SoftBodyElement* softBodyElement = element->asSoftBodyElement();
        if ( softBodyElement )
        {
            BaseElement* attached = softBodyElement->getAttachedElement();
            if ( attached && attached->asRigidElement() )
            {
                NxActor* attachedActor = attached->getPhysicsData<NxActor>();
                if ( attachedActor )
                {
                    NxShape*const* shape = attachedActor->getShapes();
                    softBody->attachToShape( *shape, NX_SOFTBODY_ATTACHMENT_TWOWAY );
                }
                else
                    softBody->attachToCollidingShapes( NX_SOFTBODY_ATTACHMENT_TWOWAY );

            }
            else
                softBody->attachToCollidingShapes( NX_SOFTBODY_ATTACHMENT_TWOWAY );
        }
        else
            softBody->attachToCollidingShapes( NX_SOFTBODY_ATTACHMENT_TWOWAY );
    }
    
    if ( attr )
    {
        osg::Matrix mat;
        if ( attr->getAttribute("matrix", mat) )
            convertMatrix( mat, bodyDesc.globalPose );
        
        osg::Vec3 vec;
        if ( attr->getAttribute("external_acceleration", vec) )
             bodyDesc.externalAcceleration = NxVec3(vec.x(), vec.y(), vec.z());
        if ( attr->getAttribute("valid_bounds_min", vec) )
             bodyDesc.validBounds.min = NxVec3(vec.x(), vec.y(), vec.z());
        if ( attr->getAttribute("valid_bounds_max", vec) )
             bodyDesc.validBounds.max = NxVec3(vec.x(), vec.y(), vec.z());
        
        double value = 0.0;
        if ( attr->getAttribute("particle_radius", value) )
            bodyDesc.particleRadius = value;
        if ( attr->getAttribute("density", value) )
            bodyDesc.density = value;
        if ( attr->getAttribute("volume_stiffness", value) )
            bodyDesc.volumeStiffness = value;
        if ( attr->getAttribute("stretching_stiffness", value) )
            bodyDesc.stretchingStiffness = value;
        if ( attr->getAttribute("damping_coefficient", value) )
            bodyDesc.dampingCoefficient = value;
        if ( attr->getAttribute("friction", value) )
            bodyDesc.friction = value;
        if ( attr->getAttribute("tear_factor", value) )
            bodyDesc.tearFactor = value;
        if ( attr->getAttribute("collision_response_coefficient", value) )
            bodyDesc.collisionResponseCoefficient = value;
        if ( attr->getAttribute("attachment_response_coefficient", value) )
            bodyDesc.attachmentResponseCoefficient = value;
        if ( attr->getAttribute("attachment_tear_factor", value) )
            bodyDesc.attachmentTearFactor = value;
        if ( attr->getAttribute("to_fluid_response_coefficient", value) )
            bodyDesc.toFluidResponseCoefficient = value;
        if ( attr->getAttribute("from_fluid_response_coefficient", value) )
            bodyDesc.fromFluidResponseCoefficient = value;
        if ( attr->getAttribute("min_adhere_velocity", value) )
            bodyDesc.minAdhereVelocity = value;
        if ( attr->getAttribute("wake_up_counter", value) )
            bodyDesc.wakeUpCounter = value;
        if ( attr->getAttribute("sleep_linear_velocity", value) )
            bodyDesc.sleepLinearVelocity = value;
        if ( attr->getAttribute("relative_grid_spacing", value) )
            bodyDesc.relativeGridSpacing = value;
        
        unsigned int uValue = 0;
        if ( attr->getAttribute("solver_iterations", uValue) )
            bodyDesc.solverIterations = uValue;
        if ( attr->getAttribute("force_field_material", uValue) )
            bodyDesc.forceFieldMaterial = uValue;
        if ( attr->getAttribute("flags", uValue) )
            bodyDesc.flags = uValue;
    }
    
    if ( cookingInstance() && useCooking )
    {
        cookingInstance()->NxCloseCooking();
    }
    return true;
}
