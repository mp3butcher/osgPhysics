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

#include <osgPhysics/ApplyVerticesVisitor>
#include <osgPhysics/AbstractInterface>
#include <osgPhysics/SoftBodyElement>

using namespace osgPhysics;

SoftBodyElement::SoftBodyElement()
:   BaseElement()
{
}

SoftBodyElement::SoftBodyElement( const SoftBodyElement& copy, const osg::CopyOp& copyop )
:   BaseElement(copy, copyop), _attachedElement(copy._attachedElement)
{
}

SoftBodyElement::~SoftBodyElement()
{
    AbstractInterface* interface = engineInstance();
    if ( interface ) interface->releaseElement(this);
}

SoftBodyAttribute* SoftBodyElement::getOrCreateAttributeMap()
{
    if ( _attribute.valid() )
    {
        return dynamic_cast<SoftBodyAttribute*>( _attribute.get() );
    }
    else
    {
        SoftBodyAttribute* ba = new SoftBodyAttribute;
        setAttributeMap( ba );
        return ba;
    }
}

void SoftBodyElement::reallocate( osg::Node* node, osg::NodeVisitor* nv, bool recreated )
{
    AbstractInterface* interface = engineInstance();
    if ( interface )
    {
        if ( getPhysicsData<void>() )
        {
            if ( recreated )
            {
                interface->releaseElement( this );
                interface->createSoftBody( this, node );
            }
            else
                interface->resetElement( this, node );
        }
        else
            interface->createSoftBody( this, node );
    }

}

void SoftBodyElement::update( osg::Node* node, osg::NodeVisitor* nv )
{
    PhysicsResult result;
    if ( engineInstance() )
        result = engineInstance()->getBodyResult(this);
    
    ApplyVerticesVisitor visitor;
    visitor.result = result;
    node->accept( visitor );
}
