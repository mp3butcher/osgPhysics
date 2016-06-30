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
#include <osgPhysics/BaseElement>

using namespace osgPhysics;

BaseElement::BaseElement()
:   _parent(NULL), _parentNode(NULL),
    _dirtyParts(DIRTY_ALL), _bodyType(STATIC_BODY), _computeShapeType(USE_BOUNDING_SPHERE),
    _initParentElement(false), _active(true), _privateData(NULL)
{
}

BaseElement::~BaseElement()
{
}

BaseElement::BaseElement( const BaseElement& copy, const osg::CopyOp& copyop )
:   osg::NodeCallback(copy, copyop),
    _parent(copy._parent), _parentNode(copy._parentNode),
    _shape(copy._shape), _attribute(copy._attribute),
    _bodyType(copy._bodyType), _computeShapeType(copy._computeShapeType),
    _dirtyParts(copy._dirtyParts), _initParentElement(copy._initParentElement),
    _active(copy._active), _privateData(copy._privateData)
{
}

void BaseElement::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    if ( _active )
    {
        if ( !_parent && !_initParentElement )
        {
            AbstractInterface* interface = engineInstance();
            if ( interface )
            {
                AbstractInterface::ElementAndNode ean = interface->findParentElement(node);
                _parent = ean.first; _parentNode = ean.second;
                _initParentElement = true;
            }
        }
        
        if ( _dirtyParts>0 )
        {
            reallocate( node, nv, (_dirtyParts&RECREATE_ELEMENT)>0 );
            _dirtyParts = 0;
        }
        else
            update( node, nv );
    }
    traverse( node, nv );
    
    if ( _active )
    {
        postevent( node, nv );
    }
}
