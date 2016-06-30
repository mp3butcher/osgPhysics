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

#include <osgPhysics/AbstractInterface>
#include <osgPhysics/World>
#include <osg/MatrixTransform>
#include <algorithm>

using namespace osgPhysics;

World::World()
:   BaseElement(),
    _deltaTime(USE_REFERENCE_TIME), _prevousReferenceTime(0.0)
{
}

World::~World()
{
    AbstractInterface* interface = engineInstance();
    if ( interface ) interface->releaseElement(this);
}

World::World( const World& copy, const osg::CopyOp& copyop )
:   BaseElement(copy, copyop), _joints(copy._joints),
    _deltaTime(copy._deltaTime), _prevousReferenceTime(copy._prevousReferenceTime)
{
}

WorldAttribute* World::getOrCreateAttributeMap()
{
    if ( _attribute.valid() )
    {
        return dynamic_cast<WorldAttribute*>( _attribute.get() );
    }
    else
    {
        WorldAttribute* wa = new WorldAttribute;
        setAttributeMap( wa );
        return wa;
    }
}

bool World::removeJoint( Joint* joint )
{
    JointList::iterator itr = std::find(_joints.begin(), _joints.end(), joint);
    if ( itr==_joints.end() ) return false;
    
    AbstractInterface* interface = engineInstance();
    if ( interface )
    {
        interface->releaseJoint( joint, this );
        _joints.erase( itr );
        return true;
    }
    return false;
}

void World::reallocate( osg::Node* node, osg::NodeVisitor* nv, bool recreated )
{
    AbstractInterface* interface = engineInstance();
    if ( interface )
    {
        if ( getPhysicsData<void>() )
        {
            if ( recreated )
            {
                interface->releaseElement( this );
                interface->createWorld( this, node );
            }
            else
                interface->resetElement( this, node );
        }
        else
            interface->createWorld( this, node );
    }
}

void World::update( osg::Node* node, osg::NodeVisitor* nv )
{
    AbstractInterface* interface = engineInstance();
    if ( interface )
    {
        const osg::FrameStamp* fs = nv->getFrameStamp();
        if ( _deltaTime==USE_REFERENCE_TIME )
            interface->simulate( fs->getReferenceTime() - _prevousReferenceTime, this );
        else
            interface->simulate( _deltaTime, this );
        _prevousReferenceTime = fs->getReferenceTime();
    }
}

void World::postevent( osg::Node* node, osg::NodeVisitor* nv )
{
    AbstractInterface* interface = engineInstance();
    if ( interface )
    {
        // Dirty joints after all bodies and the world updated
        for ( JointList::iterator itr=_joints.begin(); itr!=_joints.end(); ++itr )
        {
            Joint* joint = (*itr).get();
            if ( !joint->isInitialized() )
            {
                interface->createJoint( joint, this );
                joint->setInitialized( true );
            }
            else if ( joint->isDirty() )
            {
                interface->resetJoint( joint, this );
                joint->dirty( false );
            }
        }
    }
}
