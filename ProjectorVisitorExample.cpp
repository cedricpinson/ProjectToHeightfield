/* -*- c++ -*-
 *
 * Copyright (C) 2007 Cedric Pinson
 *
 *		   GNU LESSER GENERAL PUBLIC LICENSE
 *                      Version 3, 29 June 2007
 *
 * Copyright (C) 2007 Free Software Foundation, Inc. <http://fsf.org/>
 * Everyone is permitted to copy and distribute verbatim copies
 * of this license document, but changing it is not allowed.
 *
 *
 *  This version of the GNU Lesser General Public License incorporates
 * the terms and conditions of version 3 of the GNU General Public
 * License, supplemented by the additional permissions listed below.
 *
 * Authors:
 *  Cedric Pinson <cedric.pinson@plopbyte.com>
 */

#include <osg/ArgumentParser>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/ShapeDrawable>
#include <cstdlib>
#include "ProjectorVisitor"

osg::HeightField* createWorldHeightField(int sizex, int sizey)
{
    osg::HeightField* hf = new osg::HeightField;
    hf->allocate(sizex, sizey);
    hf->setOrigin(osg::Vec3(-180,-90,0));

    double dx = 360.0/(sizex-1);
    double dy = 180.0/(sizey-1);
    hf->setXInterval(dx);
    hf->setYInterval(dy);

    for (int x = 0; x < sizex; x++) {
        for (int y = 0; y < sizey; y++) {
            hf->setHeight(x,y,0);
        }
    }

    return hf;
}

static void usage( const char *prog, const char *msg )
{
    if (msg)
    {
        osg::notify(osg::NOTICE)<< std::endl;
        osg::notify(osg::NOTICE) << msg << std::endl;
    }

    // basic usage
    osg::notify(osg::NOTICE)<< std::endl;
    osg::notify(osg::NOTICE)<<"usage:"<< std::endl;
    osg::notify(osg::NOTICE)<<"    " << prog << " [options] projectfile [outfile]"<< std::endl;
    osg::notify(osg::NOTICE)<< std::endl;

    // print tool options
    osg::notify(osg::NOTICE)<<"options:"<< std::endl;
    osg::notify(osg::NOTICE)<<"    --world w h         - use a plane world as heightfield with width and height as resolution"<< std::endl;
    osg::notify(osg::NOTICE)<<"    --tif filename      - use tif heightfield"<< std::endl;
    osg::notify(osg::NOTICE)<<"    --mergegeometry     - merge geometry projected on heightfield"<< std::endl;
    osg::notify(osg::NOTICE)<<"    --project           - project on WSG84 after process"<< std::endl;
}


int main(int argc, char** argv)
{

    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a utility for projecting polygon onto heightfield");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filetoproject [result]");
    arguments.getApplicationUsage()->addCommandLineOption("-h or --help","Display command line parameters");

    // if user request help write it out to cout.
    if (arguments.read("-h") || arguments.read("--help"))
    { 
        usage( arguments.getApplicationName().c_str(), 0 );
        return 1;
    }

    if (arguments.argc()<=1)
    {
        arguments.getApplicationUsage()->write(std::cout,osg::ApplicationUsage::COMMAND_LINE_OPTION);
        return 1;
    }

    unsigned int width, height;
    bool world = false;
    if (arguments.read("--world", width, height)) {
        osg::notify(osg::NOTICE) << "will project on default world " << width << " x " << height << std::endl;
        world = true;
    }

    std::string tif;
    if (arguments.read("--tif", tif)) {
        osg::notify(osg::NOTICE) << "will project on world " << tif << std::endl;
    }

    bool mergeGeometry = false;
    if (arguments.read("--mergegeometry")) {
        mergeGeometry = true;
    }

    bool project = false;
    if (arguments.read("--project")) {
        project = true;
    }

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }


    typedef std::vector<std::string> FileNameList;
    FileNameList fileNames;

    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            fileNames.push_back(arguments[pos]);
        }
    }

    std::string fileNameOut("converted.osg");
    if (fileNames.size()>1)
    {
        fileNameOut = fileNames.back();
        fileNames.pop_back();
    }


    osg::ref_ptr<osg::HeightField> hf;
    if (world) {
        hf = createWorldHeightField(height, width);
        osg::ShapeDrawable* shape = new osg::ShapeDrawable(hf);
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(shape);
        osgDB::writeNodeFile(*geode, "world_grid.osg");
    } else {

        osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension("gdal");
        if (!rw) {
            std::cout << "Reader gdal not found" << std::endl;
            return 0;
        }
        osgDB::ReaderWriter::ReadResult result = rw->readHeightField(tif);
        hf = result.getHeightField();
    }

    ProjectVisitor prj(hf);
    prj.mergeGeometry(mergeGeometry);
    prj.projectToXYZ(project);

    osg::Node* node = osgDB::readNodeFile(fileNames[0]);
    node->accept(prj);

    osg::Vec4 color;
    for (int i = 0; i < 3; i++)
        color[i] = (1.0 * (rand() / (1.0*RAND_MAX)));
    color[3] = 1;
    osg::Material* mat = new osg::Material;
    mat->setDiffuse(osg::Material::FRONT_AND_BACK, color);
    node->getOrCreateStateSet()->setAttributeAndModes(mat,true);

    node->getOrCreateStateSet()->setBinNumber(10);
    node->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,false);
    osgDB::writeNodeFile(*node, fileNameOut);
    return 0;
}
