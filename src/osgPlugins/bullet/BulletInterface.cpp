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
// Written by Yun Shuai

#include "BulletInterface"

#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>

using namespace osgPhysics;

static btDynamicsWorld* findSceneElement( BaseElement* element )
{
    btDynamicsWorld* world = NULL;
    if ( element->getParentElement() )
    {
        world = element->getParentElement()->getPhysicsData<btDynamicsWorld>();
    }
    if ( !world )
        OSG_WARN << "[BulletInterface] The element doesn't belong to a world" << std::endl;
    return world;
}

static btTransform asBtTransform( const osg::Matrix& m )
{
    const osg::Matrixd::value_type* oPtr = m.ptr();
    btScalar bPtr[ 16 ];
    int idx;
    for (idx=0; idx<16; idx++)
        bPtr[ idx ] = oPtr[ idx ];
    btTransform t;
    t.setFromOpenGLMatrix( bPtr );
    return t;
}
static btVector3
asBtVector3( const osg::Vec3& v )
{
    return btVector3( v.x(), v.y(), v.z() );
}

static osg::Matrix asOsgMatrix( const btTransform& t )
{
    btScalar ogl[ 16 ];
    t.getOpenGLMatrix( ogl );
    osg::Matrix m( ogl );
    return m;
}

static btBoxShape* btBoxCollisionShapeFromOSG( osg::Node* node, const osg::BoundingBox* bb=NULL )  //
{
    osg::BoundingBox bbox;
    if (bb)
        bbox = *bb;
    else
    {
        osg::ComputeBoundsVisitor visitor;
        node->accept( visitor );
        bbox = visitor.getBoundingBox();
    }

    btBoxShape* shape = new btBoxShape( btVector3( ( bbox.xMax() - bbox.xMin() ) / 2.0,
        ( bbox.yMax() - bbox.yMin() ) / 2.0,
        ( bbox.zMax() - bbox.zMin() ) / 2.0 ) );
    return( shape );
}

MotionState::MotionState( osg::MatrixTransform *parent )
: mObject(parent)
{
}

MotionState::~MotionState()
{
}

void MotionState::getWorldTransform(btTransform& worldTrans) const
{
    assert (mObject);
    worldTrans.setOrigin( asBtTransform(mObject->getMatrix()).getOrigin() );
    worldTrans.setRotation( asBtTransform(mObject->getMatrix()).getRotation() );
}

void MotionState::setWorldTransform(const btTransform& worldTrans)
{
    assert (mObject);
    mObject->setMatrix( asOsgMatrix(worldTrans) );
}

BulletInterface::BulletInterface()
:   AbstractInterface()
{
}

BulletInterface::~BulletInterface()
{
}

BulletInterface::BulletInterface( const BulletInterface& copy, const osg::CopyOp& copyop )
:   AbstractInterface(copy, copyop)
{
}

bool BulletInterface::createWorldImplementation( BaseElement* element, osg::Node* node )
{
    if ( !element || !node )
        return false;

    ///
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );  //TODO: replace by BoudingBox
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    //dynamicsWorld->setGravity( btVector3( 0, 0, -10 ) );

    PhysicsAttribute* attr = element->getAttributeMap();
    applyWorldParameters( dynamicsWorld, attr );

    element->setPhysicsData( dynamicsWorld );
    return true;
}

bool BulletInterface::applyWorldParameters( btDynamicsWorld* world, PhysicsAttribute* attr )
{
    if ( attr )
    {
        osg::Vec3 vec;
        if ( attr->getAttribute("gravity", vec) )
            world->setGravity( btVector3(vec[0], vec[1], vec[2]) );
    }
    return true;
}

class   ComputeTriMeshVisitor : public osg::NodeVisitor
{
public:
    ComputeTriMeshVisitor( osg::NodeVisitor::TraversalMode traversalMode = TRAVERSE_ALL_CHILDREN );
#if( OSGWORKS_OSG_VERSION >= 20800 )
    META_NodeVisitor(osgbCollision,ComputeTriMeshVisitor);
#endif

    virtual void reset();

    osg::Vec3Array* getTriMesh()
    {
        return( mesh.get() );
    }

    void apply( osg::Geode & geode );

protected:
    void applyDrawable( osg::Drawable * drawable );

    osg::ref_ptr< osg::Vec3Array > mesh;
};

/* \cond */
struct ComputeTriMeshFunc
{
    ComputeTriMeshFunc()
    {
        vertices = new osg::Vec3Array;

        vertices->clear();
    }

    void inline operator()( const osg::Vec3 v1, const osg::Vec3 v2, const osg::Vec3 v3, bool _temp )
    {
        vertices->push_back( v1 );
        vertices->push_back( v2 );
        vertices->push_back( v3 );
    }

    osg::ref_ptr< osg::Vec3Array > vertices;
};
/* \endcond */
#include <osg/Transform>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/PrimitiveSet>
#include <osg/TriangleFunctor>
ComputeTriMeshVisitor::ComputeTriMeshVisitor( osg::NodeVisitor::TraversalMode traversalMode )
    : osg::NodeVisitor( traversalMode )
{
    mesh = new osg::Vec3Array;
}

void ComputeTriMeshVisitor::reset()
{
    mesh->clear();
}

void ComputeTriMeshVisitor::apply( osg::Geode & geode )
{
    unsigned int idx;
    for( idx = 0; idx < geode.getNumDrawables(); idx++ )
        applyDrawable( geode.getDrawable( idx ) );
}

void ComputeTriMeshVisitor::applyDrawable( osg::Drawable * drawable )
{
    osg::TriangleFunctor< ComputeTriMeshFunc > functor;
    drawable->accept( functor );

    osg::Matrix m = osg::computeLocalToWorld( getNodePath() );
    osg::Vec3Array::iterator iter;
    for( iter = functor.vertices->begin(); iter != functor.vertices->end(); ++iter )
    {
        mesh->push_back( *iter * m );
    }
}
bool BulletInterface::createRigidBodyImplementation( BaseElement* element, osg::Node* node )
{
    if ( !element || !node )
        return false;

    btDynamicsWorld* parentWorld = findSceneElement(element);
    if ( !parentWorld )
        return false;

    osg::MatrixTransform *matTrans;
    if( ( matTrans = dynamic_cast< osg::MatrixTransform * >( node ) ) == NULL )
    {
        matTrans = new osg::MatrixTransform;
        matTrans->addChild( node );
    }

    MotionState * motion = new MotionState( matTrans );  //
    btCollisionShape * collision = btBoxCollisionShapeFromOSG( matTrans );  //
    ComputeTriMeshVisitor visitor;
    matTrans->accept( visitor );

    osg::Vec3Array* vertices = visitor.getTriMesh();
       btCollisionShape* meshShape;
    if( vertices->size() < 3 )
    {
        OSG_WARN<< "osgbCollision::btTriMeshCollisionShapeFromOSG, no triangles found" << std::endl;
       // return( NULL );
        meshShape = collision;
    }else{

    btTriangleMesh* mesh = new btTriangleMesh;
    for( size_t i = 0; i + 2 < vertices->size(); i += 3 )
    {
        osg::Vec3& p1 = ( *vertices )[ i ];
        osg::Vec3& p2 = ( *vertices )[ i + 1 ];
        osg::Vec3& p3 = ( *vertices )[ i + 2 ];
        OSG_WARN<<p1<<" "<<p2<<" "<<p3<<std::endl;
        mesh->addTriangle( asBtVector3( p1 ),
            asBtVector3( p2 ), asBtVector3( p3 ) );
    }
if(element->getBodyType()== BaseElement::STATIC_BODY )

     meshShape = new btBvhTriangleMeshShape ( mesh, true );
     else
     meshShape = new btConvexTriangleMeshShape ( mesh, true );

    //osg::Node* debugNode = osgbBullet::osgNodeFromBtCollisionShape( collision );
    //mat->addChild( debugNode );
}
    btRigidBody::btRigidBodyConstructionInfo * rbinfo;
    btRigidBody * body=0;
    btScalar mass(0.0);  //set a default value to AVOID as initialization error.
    btVector3 inertia;
    osg::Matrix mat;
    osg::Vec3 velocity;
    osg::Vec3 angular_velocity;
    PhysicsAttribute* attr = element->getAttributeMap();
    if ( attr )
    {
        double value = 0.0;
        if ( attr->getAttribute("mass", value) )
        {
            mass =btScalar(value);
            collision->calculateLocalInertia( mass, inertia );
        }

        if ( attr->getAttribute("matrix", mat) )
        {
            motion->setWorldTransform( asBtTransform( mat ) );
        }

        rbinfo = new btRigidBody::btRigidBodyConstructionInfo( mass, motion, meshShape, inertia );
        body = new btRigidBody( *rbinfo );

        if ( attr->getAttribute("velocity", velocity) )
        {
            body->setLinearVelocity( btVector3( velocity[0], velocity[1], velocity[2] ) );
        }

        if ( attr->getAttribute("angular_velocity", angular_velocity) )
        {
            body->setAngularVelocity( btVector3( angular_velocity[0], angular_velocity[1], angular_velocity[2] ) );
        }


if(body){
    parentWorld->addRigidBody( body );  //

    element->setPhysicsData( body );
}}
    return true;
}

bool BulletInterface::applyRigidParameters( btDynamicsWorld* world, btRigidBody* body, PhysicsAttribute* attr )
{
    return true;
}

bool BulletInterface::applyRigidShape( BaseElement::ComputeShapeType type, osg::Shape* shape, osg::Node* node,
                                       btDynamicsWorld* world, btRigidBody* body )
{
    return true;
}

bool BulletInterface::resetElementImplementation( BaseElement* element, osg::Node* node )
{
    if ( !element || !node )
        return false;

    PhysicsAttribute* attr = element->getAttributeMap();
    if ( element->asRigidElement() )
    {
        btRigidBody* body = element->getPhysicsData<btRigidBody>();
        if ( !body ) return false;

        btDynamicsWorld* parentWorld = findSceneElement(element);
        if ( !parentWorld ) return false;

        if ( element->getDirtyParts()&BaseElement::DIRTY_SHAPE )
        {
            // TODO: change geometry shape at runtime
        }

        if ( element->getDirtyParts()&BaseElement::DIRTY_PARAMETERS )
        {
            //applyRigidParameters( parentWorld, body, attr );  //TODO
        }
    }
    else if ( element->asWorld() )
    {
        btDynamicsWorld* world = element->getPhysicsData<btDynamicsWorld>();
        if ( !world ) return false;

        applyWorldParameters( world, attr );  //
    }
    return true;
}

void BulletInterface::getBodyResultImplementation( BaseElement* element, PhysicsResult& result )
{
    if ( !element )
        return;
    else if ( element->asRigidElement() )
    {
        btRigidBody* body = element->getPhysicsData<btRigidBody>();
        if ( body )
        {
            btTransform trans;
            body->getMotionState()->getWorldTransform( trans );
            result.matrix.set( asOsgMatrix(trans) );
        }
    }
}

bool BulletInterface::releaseElementImplementation( BaseElement* element )
{
    if ( !element )
        return false;
    else if ( element->asWorld() )
    {
        btDynamicsWorld* world = element->getPhysicsData<btDynamicsWorld>();
        if ( world )
        {
            delete world->getBroadphase();  //
            delete world->getDispatcher();  //
            delete world;

            element->clearPhysicsData();
            return true;
        }
    }
    else if ( element->asRigidElement() )
    {
        btRigidBody* body = element->getPhysicsData<btRigidBody>();
        if ( body )
        {
            delete body;

            element->clearPhysicsData();
            return true;
        }
    }
    return false;
}

void BulletInterface::simulateImplementation( double step, BaseElement* world )
{
    btDynamicsWorld * btWorld = NULL;
    if (world)
    {
        btWorld = world->getPhysicsData<btDynamicsWorld>();
        if ( !btWorld ) return;
    }
    if ( btWorld )
    {
        btWorld->stepSimulation( step );
        OSG_WARN<<"endframe"<<std::endl;
    }
}

bool BulletInterface::createJointImplementation( Joint* joint, BaseElement* world )
{ return true; }

bool BulletInterface::resetJointImplementation( Joint* joint, BaseElement* world )
{ return true; }

bool BulletInterface::releaseJointImplementation( Joint* joint, BaseElement* world )
{ return true; }
