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

using namespace osgPhysics;

ApplyVerticesVisitor::ApplyVerticesVisitor( TraversalMode mode )
:   osg::NodeVisitor(mode)
{
    reset();
}

void ApplyVerticesVisitor::reset()
{
    _matrixStack.clear();
    _matrixStack.push_back( osg::Matrix::identity() );
}

void ApplyVerticesVisitor::apply( osg::Transform& node )
{
    osg::Matrix matrix = _matrixStack.back();
    node.computeLocalToWorldMatrix( matrix, this );

    _matrixStack.push_back( matrix );
    traverse( node );
    _matrixStack.pop_back();
}

void ApplyVerticesVisitor::apply( osg::Geode& node )
{
    for ( unsigned int i=0; i<node.getNumDrawables(); ++i )
    {
        applyDrawable( node.getDrawable(i) );
    }
    traverse( node );
}

void ApplyVerticesVisitor::applyDrawable( osg::Drawable* drawable )
{
    osg::Geometry* geometry = drawable->asGeometry();
    if ( geometry )
    {
        osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
        if ( !va ) return;
        
        unsigned int minIndex = 0;
        if ( va->size()>result.vertices->size() )
            minIndex = result.vertices->size();
        else
            minIndex = va->size();
        
        const osg::Matrix& matrix = _matrixStack.back();
        for ( unsigned int i = 0; i < minIndex; ++i )
        {
            (*va)[i] = (*result.vertices.get())[i] * matrix;
        }
        
        osg::Vec3Array* na = dynamic_cast<osg::Vec3Array*>(geometry->getNormalArray());
        if ( !na ) return;
        
        minIndex = 0;
        if ( na->size()>result.normals->size() )
            minIndex = result.normals->size();
        else
            minIndex = na->size();
        
        for ( unsigned int i = 0; i < minIndex; i ++ )
        {
            (*na)[i] = (*result.normals.get())[i];
        }
        
        if ( drawable->getUseVertexBufferObjects() )
        {
            va->dirty();
            na->dirty();
        }
    }
    
    if ( drawable->getUseDisplayList() )
    {
        drawable->dirtyDisplayList();
    }
    drawable->dirtyBound();
}
