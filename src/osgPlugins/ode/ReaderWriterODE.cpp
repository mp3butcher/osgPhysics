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

#include "ODEInterface"

class ReaderWriterODE : public osgDB::ReaderWriter
{
public:
    ReaderWriterODE()
    {
        supportsExtension( "ode", "ODE physics engine pseudo-loader" );
    }
    
    virtual ~ReaderWriterODE()
    {
    }
    
    virtual const char* className() const
    { return "ODE physics engine implemewntation"; }
    
    virtual ReadResult readObject(const std::string& file, const osgDB::ReaderWriter::Options* options) const
    {
        std::string ext = osgDB::getLowerCaseFileExtension( file );
        if ( !acceptsExtension(ext) ) return ReadResult::FILE_NOT_HANDLED;
        
        osg::ref_ptr<osgPhysics::ODEInterface> interface = new osgPhysics::ODEInterface;
        //interface->setPhysicsData( physicsSDK );
        return interface.get();
    }
};

// Now register with Registry to instantiate the above reader/writer.
REGISTER_OSGPLUGIN( ode, ReaderWriterODE )
