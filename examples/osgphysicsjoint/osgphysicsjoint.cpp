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

#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgPhysics/Utilities>
#include <osgPhysics/AbstractInterface>
#include <osgPhysics/World>
#include <osgPhysics/RigidElement>
#include <osgPhysics/Joint>

class ShootManipulator : public osgGA::GUIEventHandler
{
public:
    ShootManipulator( osg::Group* root ) : _root(root) {}
    
    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        osgViewer::View* view = dynamic_cast<osgViewer::View*>( &aa );
        if ( !view || !_root ) return false;
        
        if ( ea.getEventType()==osgGA::GUIEventAdapter::KEYUP )
        {
            if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Return )
            {
                osg::Vec3 eye, center, up, dir;
                view->getCamera()->getViewMatrixAsLookAt( eye, center, up );
                dir = center - eye; dir.normalize();
                
                osg::Node* sphere = osgPhysics::createDynamicShape(
                    new osg::Sphere(osg::Vec3(), 0.5f), eye, dir * 80.0f, 2.0 );
                _root->addChild( sphere );
            }
        }
        return false;
    }
    
protected:
    osg::observer_ptr<osg::Group> _root;
};

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments( &argc, argv );
    
    std::string engineName("ode");
    while ( arguments.read("--physx") ) { engineName.assign("physx"); }
    while ( arguments.read("--ode") ) { engineName.assign("ode"); }
    
    if ( !osgPhysics::loadPhysicsEngine(engineName) )
    {
        OSG_WARN << "No actual physics engine loaded." << std::endl;
        return 1;
    }
    
    osgPhysics::World* world = NULL;
    osg::ref_ptr<osg::Group> root = osgPhysics::createWorld( osg::Vec3(0.0f, 0.0f,-9.81f), &world );
    for ( int i=0; i<10; ++i )
    {
        osg::Node* box1 = osgPhysics::createDynamicShape(
            new osg::Box(osg::Vec3(), 0.99f), osg::Vec3((float)i, 0.0f, 1.0f) );
        root->addChild( box1 );
        
        osg::Node* box2 = osgPhysics::createDynamicShape(
            new osg::Box(osg::Vec3(), 0.8f, 0.8f, 2.0f), osg::Vec3((float)i, 0.0f, 2.5f) );
        root->addChild( box2 );
        
        osg::ref_ptr<osgPhysics::SliderJointAttribute> attr = new osgPhysics::SliderJointAttribute;
        attr->setAxis( osg::Vec3(0.0f, 0.0f, 1.0f) );
        
        osg::ref_ptr<osgPhysics::Joint> joint = new osgPhysics::Joint;
        joint->setAttributeMap( attr.get() );
        joint->attachNodes( box1, box2 );
        world->addJoint( joint.get() );
    }
    
    osg::Node* ground = osgPhysics::createStaticGround(
        osg::Plane(osg::Vec3(0.0f, 0.0f, 1.0f), 0.0f), osg::Vec3() );
    root->addChild( ground );
    
    osgViewer::Viewer viewer;
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
    viewer.addEventHandler( new osgViewer::StatsHandler );
    viewer.addEventHandler( new osgViewer::WindowSizeHandler );
    
    viewer.addEventHandler( new ShootManipulator(root.get()) );
    viewer.setSceneData( root.get() );
    viewer.run();
}
