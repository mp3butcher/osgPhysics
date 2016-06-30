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

PhysicsAttribute::PhysicsAttribute()
{
}

PhysicsAttribute::~PhysicsAttribute()
{
}

PhysicsAttribute::PhysicsAttribute( const PhysicsAttribute& copy, const osg::CopyOp& copyop )
:   osg::Object(copy, copyop), _attributeMap(copy._attributeMap)
{
}

void PhysicsAttribute::setAttribute( const std::string& name, const osg::Matrixd& value )
{
    std::stringstream ss;
    for ( int i=0; i<16; ++i ) ss << *(value.ptr() + i) << " ";
    _attributeMap[name] = ss.str();
}

void PhysicsAttribute::setAttribute( const std::string& name, const osg::Matrixf& value )
{
    std::stringstream ss;
    for ( int i=0; i<16; ++i ) ss << *(value.ptr() + i) << " ";
    _attributeMap[name] = ss.str();
}

bool PhysicsAttribute::getAttribute( const std::string& name, osg::Matrixd& value ) const
{
    AttributeMap::const_iterator itr = _attributeMap.find(name);
    if ( itr==_attributeMap.end() ) return false;
    
    std::stringstream ss(itr->second);
    for ( int i=0; i<16; ++i ) ss >> *(value.ptr() + i);
    return true;
}

bool PhysicsAttribute::getAttribute( const std::string& name, osg::Matrixf& value ) const
{
    AttributeMap::const_iterator itr = _attributeMap.find(name);
    if ( itr==_attributeMap.end() ) return false;
    
    std::stringstream ss(itr->second);
    for ( int i=0; i<16; ++i ) ss >> *(value.ptr() + i);
    return true;
}
