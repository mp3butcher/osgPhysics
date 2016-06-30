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

#include <osgDB/ReadFile>
#include <osgPhysics/AbstractInterface>
#include <osgPhysics/BaseElement>

using namespace osgPhysics;

osg::ref_ptr<AbstractInterface> g_engineInstance;

AbstractInterface* osgPhysics::loadPhysicsEngine( const std::string& name, const osgDB::Options* options )
{
    if ( g_engineInstance.valid() )
    {
        OSG_NOTICE << "[osgPhysics] Destroy the engine instance before loading a new one" << std::endl;
        g_engineInstance = NULL;
    }
    
    std::string loaderName("loader."); loaderName += name;
    g_engineInstance = dynamic_cast<AbstractInterface*>( osgDB::readObjectFile(loaderName, options) );
    return g_engineInstance.get();
}

AbstractInterface* osgPhysics::engineInstance()
{
    return g_engineInstance.get();
}

AbstractInterface::AbstractInterface()
:   _privateData(NULL)
{
}

AbstractInterface::~AbstractInterface()
{
}

AbstractInterface::AbstractInterface( const AbstractInterface& copy, const osg::CopyOp& copyop )
:   osg::Object(copy, copyop), _privateData(copy._privateData)
{
}

class FindParentVisitor : public osg::NodeVisitor
{
public:
    FindParentVisitor()
    : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_PARENTS), foundElement(NULL), foundNode(NULL) {}
    
    void apply( osg::Node& node )
    {
        osg::NodeCallback* cb = node.getUpdateCallback();
        while ( cb && !foundElement )
        {
            foundElement = dynamic_cast<BaseElement*>(cb);
            foundNode = &node;
            cb = cb->getNestedCallback();
        }
        traverse( node );
    }
    
    BaseElement* foundElement;
    osg::Node* foundNode;
};

AbstractInterface::ElementAndNode AbstractInterface::findParentElement( osg::Node* node )
{
    if ( node && node->getNumParents() )
    {
        FindParentVisitor fpv;
        for ( unsigned int i=0; i<node->getNumParents(); ++i )
        {
            node->getParent(i)->accept( fpv );
            if ( fpv.foundElement )
                return ElementAndNode(fpv.foundElement, fpv.foundNode);
        }
    }
    return ElementAndNode();
}

bool AbstractInterface::createWorld( BaseElement* element, osg::Node* node )
{
    return createWorldImplementation(element, node);
}

bool AbstractInterface::createRigidBody( BaseElement* element, osg::Node* node )
{
    return createRigidBodyImplementation(element, node);
}

bool AbstractInterface::createSoftBody( BaseElement* element, osg::Node* node )
{
    return createSoftBodyImplementation(element, node);
}

bool AbstractInterface::createCloth( BaseElement* element, osg::Node* node )
{
    return createClothImplementation(element, node);
}

bool AbstractInterface::createJoint( Joint* joint, BaseElement* world )
{
    return createJointImplementation(joint, world);
}

bool AbstractInterface::resetElement( BaseElement* element, osg::Node* node )
{
    return resetElementImplementation(element, node);
}

bool AbstractInterface::resetJoint( Joint* joint, BaseElement* world )
{
    return resetJointImplementation(joint, world);
}

PhysicsResult AbstractInterface::getBodyResult( BaseElement* element )
{
    PhysicsResult result;
    getBodyResultImplementation(element, result);
    return result;
}

bool AbstractInterface::releaseElement( BaseElement* element )
{
    return releaseElementImplementation(element);
}

bool AbstractInterface::releaseJoint( Joint* joint, BaseElement* world )
{
    return releaseJointImplementation(joint, world);
}

void AbstractInterface::simulate( double step, BaseElement* world )
{
    return simulateImplementation(step, world);
}
