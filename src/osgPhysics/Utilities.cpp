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

#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osgPhysics/Utilities>
#include <osgPhysics/RigidElement>
#include <osgPhysics/World>

namespace osgPhysics {

osg::Node* createStaticGround( const osg::Plane& plane, const osg::Vec3& pos, RigidElement** ptr )
{
    osg::ref_ptr<osg::InfinitePlane> planeShape = new osg::InfinitePlane;
    planeShape->set( plane );
    
    osg::ref_ptr<RigidElement> rigidPlane = new RigidElement;
    rigidPlane->setShape( planeShape.get() );
    rigidPlane->setBodyType( BaseElement::STATIC_BODY );
    rigidPlane->getOrCreateAttributeMap()->setMatrix( osg::Matrix::translate(pos) );
    if ( ptr ) *ptr = rigidPlane.get();
    
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( new osg::ShapeDrawable(new osg::Box(pos, 100.0f, 100.0f, 0.001f)) );
    
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    mt->addChild( geode.get() );
    mt->setUpdateCallback( rigidPlane.get() );
    return mt.release();
}

osg::Node* createDynamicShape( osg::Shape* shape, const osg::Vec3& pos, const osg::Vec3& vel,
                               double mass, RigidElement** ptr )
{
    if ( !shape )
        return NULL;
    
    osg::ref_ptr<RigidElement> rigidBody = new RigidElement;
    rigidBody->setShape( shape );
    rigidBody->setBodyType( BaseElement::DYNAMIC_BODY );
    if ( ptr ) *ptr = rigidBody.get();
    
    BodyAttribute* attr = rigidBody->getOrCreateAttributeMap();
    attr->setMatrix( osg::Matrix::translate(pos) );
    attr->setVelocity( vel );
    attr->setMass( mass );
    
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( new osg::ShapeDrawable(shape) );
    
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    mt->addChild( geode.get() );
    mt->setUpdateCallback( rigidBody.get() );
    return mt.release();
}

osg::Group* createWorld( const osg::Vec3& gravity, World** ptr )
{
    osg::ref_ptr<osgPhysics::World> world = new osgPhysics::World;
    world->getOrCreateAttributeMap()->setGravity( gravity );
    if ( ptr ) *ptr = world.get();
    
    osg::ref_ptr<osg::Group> worldNode = new osg::Group;
    worldNode->setUpdateCallback( world.get() );
    return worldNode.release();
}

osg::Node* getOrCreateDynamicBody( osg::Node* node, const osg::Vec3& pos, const osg::Vec3& vel,
                                   double mass, RigidElement** ptr )
{
    osg::ref_ptr<osg::MatrixTransform> mt;
    if ( !node )
        return NULL;
    
    osg::ref_ptr<RigidElement> rigidBody = new RigidElement;
    rigidBody->setComputeShapeType( BaseElement::USE_BOUNDING_BOX );
    rigidBody->setBodyType( BaseElement::DYNAMIC_BODY );
    if ( ptr ) *ptr = rigidBody.get();
    
    BodyAttribute* attr = rigidBody->getOrCreateAttributeMap();
    attr->setMatrix( osg::Matrix::translate(pos) );
    attr->setVelocity( vel );
    attr->setMass( mass );
    
    osg::Transform* transform = node->asTransform();
    if ( transform )
        mt = transform->asMatrixTransform();
    
    if ( !mt )
    {
        mt = new osg::MatrixTransform;
        mt->addChild( node );
        mt->setUpdateCallback( rigidBody.get() );
        return mt.release();
    }
    else
    {
        mt->setUpdateCallback( rigidBody.get() );
        return mt.get();
    }
}

BaseElement* getPhysicalCallback( osg::Node* node )
{
    BaseElement* element = NULL;
    if ( node && node->getUpdateCallback() )
    {
        osg::NodeCallback* cb = node->getUpdateCallback();
        while ( cb )
        {
            element = dynamic_cast<BaseElement*>(cb);
            if ( element ) break;
            else cb = cb->getNestedCallback();
        }
    }
    return element;
}

}
