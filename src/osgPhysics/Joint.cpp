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

#include <osgPhysics/Utilities>
#include <osgPhysics/AbstractInterface>
#include <osgPhysics/Joint>

using namespace osgPhysics;

Joint::Joint()
:   _initialized(false), _dirty(false), _privateData(NULL)
{
}

Joint::Joint( JointType type, osg::Node* node1, osg::Node* node2 )
:   _initialized(false), _dirty(false), _privateData(NULL)
{
    switch ( type )
    {
    case JointAttribute::FIXED_JOINT:
        setAttributeMap( new FixedJointAttribute );
        break;
    case JointAttribute::BALL_JOINT:
        setAttributeMap( new BallJointAttribute );
        break;
    case JointAttribute::HINGE_JOINT:
        setAttributeMap( new HingeJointAttribute );
        break;
    case JointAttribute::SLIDER_JOINT:
        setAttributeMap( new SliderJointAttribute );
        break;
    case JointAttribute::PISTON_JOINT:
        setAttributeMap( new PistonJointAttribute );
        break;
    case JointAttribute::UNIVERSAL_JOINT:
        setAttributeMap( new UniversalJointAttribute );
        break;
    default:
        setAttributeMap( new JointAttribute );
        break;
    }
    attachNodes( node1, node2 );
}

Joint::~Joint()
{
}

Joint::Joint( const Joint& copy, const osg::CopyOp& copyop )
:   osg::Object(copy, copyop),
    _element1(copy._element1), _element2(copy._element2), _attribute(copy._attribute),
    _initialized(copy._initialized), _dirty(copy._dirty), _privateData(copy._privateData)
{
}

bool Joint::attachNodes( osg::Node* node1, osg::Node* node2 )
{
    BaseElement* element1 = getPhysicalCallback(node1);
    BaseElement* element2 = getPhysicalCallback(node2);
    if ( !element1 || !element2 )
        return false;
    
    setElement1( element1 );
    setElement2( element2 );
    return true;
}
