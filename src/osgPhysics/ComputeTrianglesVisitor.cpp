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

#include <osg/Geometry>
#include <osg/TriangleFunctor>
#include <osg/TriangleIndexFunctor>
#include <osgPhysics/ComputeTrianglesVisitor>

using namespace osgPhysics;

/* CollectTriangleIndicesFunctor */

struct CollectTriangleIndicesFunctor
{
    osg::UIntArray* indices;
    unsigned int base;
    
    void operator()( unsigned int p1, unsigned int p2, unsigned int p3 )
    {
        if ( p1==p2 || p1==p3 || p2==p3 )
            return;
        
        indices->push_back( base + p1 );
        indices->push_back( base + p2 );
        indices->push_back( base + p3 );
    }
};

/* CollectTrianglesFunctor */

struct CollectTrianglesFunctor
{
    osg::Vec3Array* vertices;
    osg::UIntArray* indices;
    unsigned int base;
    
    void operator()( const osg::Vec3 v1, const osg::Vec3 v2, const osg::Vec3 v3, bool temp )
    {
        if ( v1==v2 || v1==v3 || v2==v3 )
            return;
        
        vertices->push_back( v1 ); indices->push_back( base++ );
        vertices->push_back( v2 ); indices->push_back( base++ );
        vertices->push_back( v3 ); indices->push_back( base++ );
    }
};

/* ComputeTrianglesVisitor */

ComputeTrianglesVisitor::ComputeTrianglesVisitor( TraversalMode mode )
:   osg::NodeVisitor(mode)
{
    reset();
}

void ComputeTrianglesVisitor::reset()
{
    _matrixStack.clear();
    _matrixStack.push_back( osg::Matrix::identity() );
    _vertices = new osg::Vec3Array;
    _normal = new osg::Vec3Array;
    _indices = new osg::UIntArray;
}

void ComputeTrianglesVisitor::apply( osg::Transform& node )
{
    osg::Matrix matrix = _matrixStack.back();
    node.computeLocalToWorldMatrix( matrix, this );
    
    _matrixStack.push_back( matrix );
    traverse( node );
    _matrixStack.pop_back();
}

void ComputeTrianglesVisitor::apply( osg::Geode& node )
{
    unsigned int numVertices = _vertices->size();
    for ( unsigned int i=0; i<node.getNumDrawables(); ++i )
    {
        applyDrawable( node.getDrawable(i) );
    }
    
    const osg::Matrix& matrix = _matrixStack.back();
    for ( unsigned int i=numVertices; i<_vertices->size(); ++i )
    {
        osg::Vec3& vertex = (*_vertices)[i];
        vertex = vertex * matrix;
    }
    traverse( node );
}

void ComputeTrianglesVisitor::applyDrawable( osg::Drawable* drawable )
{
    unsigned int numVertices = _vertices->size();
    osg::Geometry* geom = drawable->asGeometry();
    if ( geom )
    {
        osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>( geom->getVertexArray() );
        if ( va )
        {
            osg::TriangleIndexFunctor<CollectTriangleIndicesFunctor> functor;
            functor.indices = _indices.get();
            functor.base = numVertices;
            drawable->accept( functor );
            
            osg::Vec3Array *na = dynamic_cast<osg::Vec3Array*>( geom->getNormalArray() );
            if ( na ) _normal->insert( _normal->end(), na->begin(), na->end() );
            else if ( va->size()>0 ) _normal->insert( _normal->end(), va->size(), osg::Vec3() );
            _vertices->insert( _vertices->end(), va->begin(), va->end() );
            return;
        }
    }
    
    osg::TriangleFunctor<CollectTrianglesFunctor> functor;
    functor.vertices = _vertices.get();
    functor.indices = _indices.get();
    functor.base = numVertices;
    drawable->accept( functor );
}
