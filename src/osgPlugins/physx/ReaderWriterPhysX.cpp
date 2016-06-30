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

#include <osg/Notify>
#include <osg/Group>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>

#include "PhysXInterface"

class ErrorStream : public NxUserOutputStream
{
public:
    virtual void reportError( NxErrorCode e, const char* message, const char* file, int line );
    virtual NxAssertResponse reportAssertViolation( const char* message, const char* file, int line );
    
    virtual void print( const char* message )
    { OSG_NOTICE << "[PhysXInterface] " << message << std::endl; }
};

void ErrorStream::reportError( NxErrorCode e, const char* message, const char* file, int line )
{
    switch (e)
    {
    case NXE_INVALID_PARAMETER:
        OSG_WARN << "[PhysXInterface] Invalid parameter at ";
        break;
    case NXE_INVALID_OPERATION:
        OSG_WARN << "[PhysXInterface] Invalid operation at ";
        break;
    case NXE_OUT_OF_MEMORY:
        OSG_WARN << "[PhysXInterface] Out of memory at ";
        break;
    case NXE_DB_INFO:
        OSG_WARN << "[PhysXInterface] Debug information at ";
        break;
    case NXE_DB_WARNING:
        OSG_WARN << "[PhysXInterface] Warning message at ";
        break;
    default:
        OSG_WARN << "[PhysXInterface] Unknown error at ";
        break;
    }
    
    std::stringstream ss; ss << line;
    OSG_WARN << file << " (" << ss.str() << "): " << message << std::endl;
}

NxAssertResponse ErrorStream::reportAssertViolation( const char* message, const char* file, int line )
{
    std::stringstream ss; ss << line;
    OSG_FATAL << "[PhysXInterface] Access violation at " << file
              << " (" << ss.str() << "): " << message << std::endl;
    return NX_AR_BREAKPOINT;
}

class ReaderWriterPhysX : public osgDB::ReaderWriter
{
public:
    ReaderWriterPhysX()
    {
        supportsExtension( "physx", "PhysX physics engine pseudo-loader" );
    }
    
    virtual ~ReaderWriterPhysX()
    {
    }
    
    virtual const char* className() const
    { return "PhysX physics engine implemewntation"; }
    
    virtual ReadResult readObject(const std::string& file, const osgDB::ReaderWriter::Options* options) const
    {
        std::string ext = osgDB::getLowerCaseFileExtension( file );
        if ( !acceptsExtension(ext) ) return ReadResult::FILE_NOT_HANDLED;
        
        NxPhysicsSDKDesc desc;
        NxSDKCreateError errorCode = NXCE_NO_ERROR;
        NxPhysicsSDK* physicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION, NULL, new ErrorStream, desc, &errorCode);
        if ( !physicsSDK ) 
        {
            OSG_WARN << "[PhysXInterface] Unable to initialize the PhysX SDK, error code: " << errorCode << std::endl;
            return NULL;
        }
        osgPhysics::setPhysxInstance( physicsSDK );
        
        osg::ref_ptr<osgPhysics::PhysXInterface> interface = new osgPhysics::PhysXInterface;
        interface->setPhysicsData( physicsSDK );
        return interface.get();
    }
};

// Now register with Registry to instantiate the above reader/writer.
REGISTER_OSGPLUGIN( physx, ReaderWriterPhysX )
