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
#include <osgPhysics/RigidElement>
#include <osg/MatrixTransform>

using namespace osgPhysics;

RigidElement::RigidElement()
:   BaseElement()
{
}

RigidElement::~RigidElement()
{
    AbstractInterface* interface = engineInstance();
    if ( interface ) interface->releaseElement(this);
}

RigidElement::RigidElement( const RigidElement& copy, const osg::CopyOp& copyop )
:   BaseElement(copy, copyop)
{
}

BodyAttribute* RigidElement::getOrCreateAttributeMap()
{
    if ( _attribute.valid() )
    {
        return dynamic_cast<BodyAttribute*>( _attribute.get() );
    }
    else
    {
        BodyAttribute* ba = new BodyAttribute;
        setAttributeMap( ba );
        return ba;
    }
}

void RigidElement::reallocate( osg::Node* node, osg::NodeVisitor* nv, bool recreated )
{
    AbstractInterface* interface = engineInstance();
    if ( interface )
    {
        if ( getPhysicsData<void>() )
        {
            if ( recreated )
            {
                interface->releaseElement( this );
                interface->createRigidBody( this, node );
            }
            else
                interface->resetElement( this, node );
        }
        else
            interface->createRigidBody( this, node );
    }
}

void RigidElement::update( osg::Node* node, osg::NodeVisitor* nv )
{
    PhysicsResult result;
    if ( engineInstance() )
        result = engineInstance()->getBodyResult(this);
    
    osg::Matrix subMatrix;
    if ( node->getNumParents()>0 )
    {
        // The matrix from physics engine is always computed in the physics world's reference frame
        // Here we must remove the parent matrix of the node here before applying the result
        subMatrix = computeLocalToWorld( node->getParent(0)->getParentalNodePaths(_parentNode.get())[0] );
        subMatrix = osg::Matrix::inverse(subMatrix) * result.matrix;
    }
    
    osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>(node);
    if ( mt )
    {
        mt->setMatrix( subMatrix );
    }
}
