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
// Written by Geng BeiLei

#include <osg/NodeVisitor>
#include <osg/Drawable>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osgPhysics/ApplyVerticesVisitor>
#include <osgPhysics/AbstractInterface>
#include <osgPhysics/ClothElement>

using namespace osgPhysics;

ClothElement::ClothElement()
:   BaseElement()
{
}

ClothElement::ClothElement( const ClothElement& copy, const osg::CopyOp& copyop )
:   BaseElement(copy, copyop), _attachedElement(copy._attachedElement)
{

}

ClothElement::~ClothElement()
{
    AbstractInterface* interface = engineInstance();
    if ( interface ) interface->releaseElement(this);
}

ClothAttribute* ClothElement::getOrCreateAttributeMap()
{
    if ( _attribute.valid() )
    {
        return dynamic_cast<ClothAttribute*>( _attribute.get() );
    }
    else
    {
        ClothAttribute* ba = new ClothAttribute;
        setAttributeMap( ba );
        return ba;
    }
}

void ClothElement::reallocate( osg::Node* node, osg::NodeVisitor* nv, bool recreated )
{
    AbstractInterface* interface = engineInstance();
    if ( interface )
    {
        if ( getPhysicsData<void>() )
        {
            if ( recreated )
            {
                interface->releaseElement( this );
                interface->createCloth( this, node );
            }
            else
                interface->resetElement( this, node );
        }
        else
            interface->createCloth( this, node );
    }
}

void ClothElement::update( osg::Node* node, osg::NodeVisitor* nv )
{
    PhysicsResult result;
    if ( engineInstance() )
        result = engineInstance()->getBodyResult(this);
    
    ApplyVerticesVisitor visitor;
    visitor.result = result;
    node->accept( visitor );
}
