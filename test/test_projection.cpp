/* -*- c++ -*-
 *
 * Copyright (C) 2007 ArchiVideo
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
 *  Cedric Pinson <mornifle@plopbyte.net>
 */

#include <UnitTest++.h>
#include <osg/Vec3>
#include <osg/Array>
#include <osg/Geometry>
#include <osg/Plane>
#include <osgUtil/Tessellator>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include "ProjectorVisitor"

#ifndef TEST_DIR
#define TEST_DIR "./"
#endif

#define TOKENIFY(x) #x

static std::string getTestDir()
{
  std::string str(TEST_DIR);
  std::cout << str << std::endl;
  return str;
}

struct MyFixture : public ProjectVisitor
{
  MyFixture(): ProjectVisitor(initFakeData1()) {}

  static osg::HeightField* initFakeData1() {
    osg::HeightField* hf = new osg::HeightField;
    int size = 5;
    hf->allocate(size,size);
    hf->setOrigin(osg::Vec3(1,10,0));

    hf->setXInterval(2);
    hf->setYInterval(-2);
    for (int i = 0; i < size; i++)
      for (int j = 0; j < size; j++)
        hf->setHeight(i,j,0);
    return hf;
  }

  static osg::HeightField* initFakeData2() {
    osg::HeightField* hf = new osg::HeightField;
    int size = 5;
    hf->allocate(size,size);
    hf->setOrigin(osg::Vec3(1,1,0));

    hf->setXInterval(2);
    hf->setYInterval(2);
    for (int i = 0; i < size; i++)
      for (int j = 0; j < size; j++)
        hf->setHeight(i,j,0);
    return hf;
  }

  static osg::HeightField* initFakeData3() {
    osg::HeightField* hf = new osg::HeightField;
    int size = 10;
    hf->allocate(size,size);
    hf->setOrigin(osg::Vec3(1,1,0));

    hf->setXInterval(1);
    hf->setYInterval(1);
    for (int i = 0; i < size; i++)
      for (int j = 0; j < size; j++)
        hf->setHeight(i,j,sin(i+j));
    return hf;
  }

};

static osg::ref_ptr<osg::HeightField> readHeightFieldTest1() {
  osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension("gdal");
  if (!rw) {
    std::cout << "Reader gdal not found" << std::endl;
    return 0;
  }
  osgDB::ReaderWriter::ReadResult result = rw->readHeightField(getTestDir() + "test.tif");
  return result.getHeightField();
}

static osg::ref_ptr<osg::HeightField> readHeightFieldTest2() {
  osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension("gdal");
  if (!rw) {
    std::cout << "Reader gdal not found" << std::endl;
    return 0;
  }
  osgDB::ReaderWriter::ReadResult result = rw->readHeightField(getTestDir() + "test2.tif");
  return result.getHeightField();
}

static osg::ref_ptr<osg::HeightField> readHeightFieldTest3() {
  osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension("gdal");
  if (!rw) {
    std::cout << "Reader gdal not found" << std::endl;
    return 0;
  }
  osgDB::ReaderWriter::ReadResult result = rw->readHeightField(getTestDir() + "test3.tif");
  return result.getHeightField();
}


struct MyTriangleFixture : public Triangle2HeightField
{
  MyTriangleFixture():Triangle2HeightField(MyFixture::initFakeData3()) {}

};


TEST_FIXTURE(MyTriangleFixture, case2d)
{
  _heightField = readHeightFieldTest1();

  (*this)(osg::Vec3(201986, 2.36809e+06, 0),
          osg::Vec3(202222, 2.36808e+06, 0),
          osg::Vec3(202242, 2.36792e+06, 0));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(24, 198));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(22, 199));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(23, 199));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(24, 199));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(22, 200));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(23, 200));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(24, 200));
}

TEST_FIXTURE(MyTriangleFixture, case2c)
{
  _heightField = readHeightFieldTest1();

  (*this)(osg::Vec3(199875, 2.36931e+06, 0),
          osg::Vec3(200118, 2.36804e+06, 0),
          osg::Vec3(200184, 2.36914e+06, 0));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 199));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 200));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 201));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 201));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 202));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 202));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 203));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 203));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 204));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 204));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 205));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 205));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 206));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 206));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 207));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 207));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 208));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 208));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 209));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 209));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 210));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 210));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 211));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 211));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(2, 211));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 212));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 212));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(2, 212));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 213));
}

TEST_FIXTURE(MyTriangleFixture, case2b)
{
  _heightField = readHeightFieldTest1();

  (*this)(osg::Vec3(200118, 2368040, 0),
          osg::Vec3(199875, 2369310, 0),
          osg::Vec3(199850, 2368390, 0));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 199));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 200));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 200));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 201));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(1, 201));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 202));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 203));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 204));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 205));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(0, 206));
}

TEST_FIXTURE(MyTriangleFixture, case6)
{
  _heightField = readHeightFieldTest1();

  (*this)(osg::Vec3(257955.422, 2405009, 0),
          osg::Vec3(257611.5, 2398775.75, 0),
          osg::Vec3(257450.641, 2404640.75, 0));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 541));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 542));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 543));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 544));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 545));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(639, 546));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 546));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(639, 547));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 547));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(639, 548));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 548));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(639, 549));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 549));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(639, 550));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 550));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(639, 551));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 551));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(639, 552));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 552));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(639, 553));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 553));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(639, 554));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(640, 554));
}

TEST_FIXTURE(MyTriangleFixture, case1)
{
  _heightField = readHeightFieldTest1();

  (*this)(osg::Vec3(235160, 2.39399e+06, 0),
          osg::Vec3(231201, 2.38817e+06, 0),
          osg::Vec3(235160, 2.37399e+06, 0));
#include "case1_result.cpp"
  write("case1_result.osg");

}

TEST_FIXTURE(MyTriangleFixture, case2e)
{
  _heightField = readHeightFieldTest1();
  // 14 25 26
  (*this)(osg::Vec3(201047, 2.36843e+06, 0),
          osg::Vec3(201521, 2.36736e+06, 0),
          osg::Vec3(201009, 2.36746e+06, 0));
  //  write("./case2e.osg");
  CHECK_EQUAL(1, isBlockAlreadyProcessed(13, 192));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(14, 192));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(15, 192));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(16, 192));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 193));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(12, 193));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(13, 193));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(14, 193));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(15, 193));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(16, 193));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 194));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(12, 194));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(13, 194));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(14, 194));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(15, 194));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(16, 194));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 195));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(12, 195));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(13, 195));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(14, 195));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(15, 195));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 196));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(12, 196));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(13, 196));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(14, 196));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(15, 196));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 197));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(12, 197));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(13, 197));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(14, 197));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 198));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(12, 198));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(13, 198));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(14, 198));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 199));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(12, 199));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(13, 199));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 200));
  CHECK_EQUAL(2, isBlockAlreadyProcessed(12, 200));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(13, 200));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 201));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(12, 201));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(13, 201));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 202));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(12, 202));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 203));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(12, 203));
  CHECK_EQUAL(1, isBlockAlreadyProcessed(11, 204));
}

TEST_FIXTURE(MyTriangleFixture, y0_is_negative)
{
  _heightField = readHeightFieldTest2();

  GetBlocksTouchedByLine* blockLeft = new GetBlocksTouchedByLine(_heightField.get());
  GetBlocksTouchedByLine* blockRight = new GetBlocksTouchedByLine(_heightField.get());
  double miny,maxy;
  setup(osg::Vec3(227658.172, 2351000.25, 0),
        osg::Vec3(227664.25, 2350893.25, 0),
        osg::Vec3(227679.141, 2350337.25, 0),
        miny, maxy,
        blockLeft,
        blockRight);
  CHECK_EQUAL(true, miny >= 0);
}


TEST_FIXTURE(MyTriangleFixture, clientTriangle)
{
  std::vector<osg::Vec2d> result;
  clipTriangleAgainstHeightField(osg::Vec2d(-1000,-1000),
                                 osg::Vec2d(1000,-1000),
                                 osg::Vec2d(0,1000),
                                 result);
  CHECK_EQUAL(4, result.size());
  CHECK_CLOSE(0, result[0][0], 1e-5 );
  CHECK_CLOSE(9, result[0][1], 1e-5 );

  CHECK_CLOSE(0, result[1][0], 1e-5 );
  CHECK_CLOSE(0, result[1][1], 1e-5 );

  CHECK_CLOSE(9, result[2][0], 1e-5 );
  CHECK_CLOSE(0, result[2][1], 1e-5 );

  CHECK_CLOSE(9, result[3][0], 1e-5 );
  CHECK_CLOSE(9, result[3][1], 1e-5 );

//   for (int i = 0; i < result.size(); i++)
//     std::cout << result[i] << std::endl;
}



TEST_FIXTURE(MyFixture, findGridFromPoint)
{
  _projector.init(initFakeData2());

  osg::Vec3d v1(6.1,2.1,0);
  osg::Vec2d res;

  _projector.findGridFromPoint(v1, res);
  CHECK_CLOSE(2.55, res[0], 1e-4);
  CHECK_CLOSE(0.55, res[1], 1e-4);
}

TEST_FIXTURE(MyFixture, lineIntersect)
{
  
  osg::Vec2d result;

  bool res = lineIntersect(osg::Vec2(0, 4),osg::Vec2(0, 10),
                           osg::Vec2(-4, 5), osg::Vec2(4, 5),
                           result);
  CHECK_EQUAL(true, res);
  CHECK_EQUAL(0, result[0]);
  CHECK_EQUAL(5, result[1]);
}

TEST_FIXTURE(MyFixture, intersectGridLinesFromBox)
{
  osg::Vec2d s0(2.1,2.1);
  osg::Vec2d s1(4.3,3.3);
  std::vector<osg::Vec2d> result;
  intersectGridLinesFromSegment(s0, s1, result);
  CHECK_EQUAL(3, result.size());
  CHECK_EQUAL(3.0,result[0][0]);
  CHECK_EQUAL(true,result[0][1] > 2 && result[0][1] < 3);
  CHECK_EQUAL(3.0,result[1][1]);
  CHECK_EQUAL(true,result[1][0] > 3 && result[1][0] < 4);
  CHECK_EQUAL(4.0,result[2][0]);
  CHECK_EQUAL(true,result[2][1] > 3 && result[2][1] < 4);
}

TEST_FIXTURE(MyFixture, clipLine)
{
  _projector.init(initFakeData2());

  osg::Vec3d start(6.1, 2.1, 0);
  osg::Vec3d end(8, 8, 0);

  osg::Vec2d startInGrid;
  osg::Vec2d endInGrid;
  osg::Vec2d startInGridOrg;
  osg::Vec2d endInGridOrg;

  {
  _projector.findGridFromPoint(start, startInGrid);
  _projector.findGridFromPoint(end, endInGrid);
  startInGridOrg = startInGrid;
  endInGridOrg = endInGrid;

  bool result;
  result = _projector.clipLine(startInGrid, endInGrid);
  CHECK_EQUAL(true, result);
  CHECK_EQUAL(startInGridOrg[0], startInGrid[0]);
  CHECK_EQUAL(startInGridOrg[1], startInGrid[1]);
  CHECK_EQUAL(endInGridOrg[0], endInGrid[0]);
  CHECK_EQUAL(endInGridOrg[1], endInGrid[1]);
  }

  {
  _projector.findGridFromPoint(osg::Vec3(-5,8,0) , startInGrid);
  _projector.findGridFromPoint(osg::Vec3(2,5,0), endInGrid);
  startInGridOrg = startInGrid;
  endInGridOrg = endInGrid;

  bool result;
  result = _projector.clipLine(startInGrid, endInGrid);
  CHECK_EQUAL(true, result);
  CHECK_EQUAL(0, endInGrid[0]);
  CHECK_CLOSE(2.214286, endInGrid[1], 1e-4);
  CHECK_EQUAL(endInGridOrg[0], startInGrid[0]);
  CHECK_EQUAL(endInGridOrg[1], startInGrid[1]);
  }

  {
  _projector.findGridFromPoint(osg::Vec3(-5,8,0) , startInGrid);
  _projector.findGridFromPoint(osg::Vec3(-7,5,0), endInGrid);
  startInGridOrg = startInGrid;
  endInGridOrg = endInGrid;

  bool result;
  result = _projector.clipLine(startInGrid, endInGrid);
  CHECK_EQUAL(false, result);
  }

  {
  _projector.findGridFromPoint(osg::Vec3(15,8,0) , startInGrid);
  _projector.findGridFromPoint(osg::Vec3(8,5,0), endInGrid);
  startInGridOrg = startInGrid;
  endInGridOrg = endInGrid;

  bool result;
  result = _projector.clipLine(startInGrid, endInGrid);
  CHECK_EQUAL(true, result);
  CHECK_EQUAL(5, endInGrid[0]);
  CHECK_CLOSE(2.64285707, endInGrid[1], 1e-4);
  CHECK_EQUAL(endInGridOrg[0], startInGrid[0]);
  CHECK_EQUAL(endInGridOrg[1], startInGrid[1]);
  }

  {
  _projector.findGridFromPoint(osg::Vec3(15,8,0) , startInGrid);
  _projector.findGridFromPoint(osg::Vec3(12,5,0), endInGrid);
  startInGridOrg = startInGrid;
  endInGridOrg = endInGrid;

  bool result;
  result = _projector.clipLine(startInGrid, endInGrid);
  CHECK_EQUAL(false, result);
  }


  {
  _projector.findGridFromPoint(osg::Vec3(15,-8,0) , startInGrid);
  _projector.findGridFromPoint(osg::Vec3(8,5,0), endInGrid);
  startInGridOrg = startInGrid;
  endInGridOrg = endInGrid;

  bool result;
  result = _projector.clipLine(startInGrid, endInGrid);
  CHECK_EQUAL(true, result);
  CHECK_CLOSE(4.57692289, startInGrid[0], 1e-4 );
  CHECK_EQUAL(0, startInGrid[1]);
  CHECK_EQUAL(endInGridOrg[0], endInGrid[0]);
  CHECK_EQUAL(endInGridOrg[1], endInGrid[1]);
  }

  {
  _projector.findGridFromPoint(osg::Vec3(10,-8,0) , startInGrid);
  _projector.findGridFromPoint(osg::Vec3(12,5,0), endInGrid);
  startInGridOrg = startInGrid;
  endInGridOrg = endInGrid;

  bool result;
  result = _projector.clipLine(startInGrid, endInGrid);
  CHECK_EQUAL(false, result);
  }


  {
  _projector.findGridFromPoint(osg::Vec3(4,5,0), startInGrid);
  _projector.findGridFromPoint(osg::Vec3(8,15,0), endInGrid);
  startInGridOrg = startInGrid;
  endInGridOrg = endInGrid;

  bool result;
  result = _projector.clipLine(startInGrid, endInGrid);
  CHECK_EQUAL(true, result);
  CHECK_EQUAL(startInGridOrg[0], startInGrid[0]);
  CHECK_EQUAL(startInGridOrg[1], startInGrid[1]);
  CHECK_CLOSE(2.7, endInGrid[0], 1e-4);
  CHECK_EQUAL(5, endInGrid[1]);
  }

  {
  _projector.findGridFromPoint(osg::Vec3(8,15,0) , startInGrid);
  _projector.findGridFromPoint(osg::Vec3(8,12,0), endInGrid);
  startInGridOrg = startInGrid;
  endInGridOrg = endInGrid;

  bool result;
  result = _projector.clipLine(startInGrid, endInGrid);
  CHECK_EQUAL(false, result);
  }

}


TEST_FIXTURE(MyFixture, followLine)
{
  _projector.init(initFakeData2());

  osg::Vec2d p1InGrind;
  osg::Vec2d p2InGrind;
  _projector.findGridFromPoint(osg::Vec3(6.1, 2.1, 0), p1InGrind);
  _projector.findGridFromPoint(osg::Vec3(7.9, 7.9, 0), p2InGrind);

  osg::ref_ptr<osg::Vec3Array> array = _projector.followLine(p1InGrind,
                                                             p2InGrind);
  CHECK_EQUAL(6, array->size());

  CHECK_CLOSE(6.1, (*array)[0][0], 1e-4);
  CHECK_CLOSE(2.1, (*array)[0][1], 1e-4);

  CHECK_CLOSE(6.379310, (*array)[1][0], 1e-4);
  CHECK_EQUAL(3, (*array)[1][1]);

  CHECK_CLOSE(7.0, (*array)[2][0], 1e-4);
  CHECK_CLOSE(5.0, (*array)[2][1], 1e-4);

  CHECK_EQUAL(7, (*array)[3][0]);
  CHECK_CLOSE(5.0, (*array)[3][1], 1e-4);

  CHECK_CLOSE(7.620690, (*array)[4][0], 1e-4);
  CHECK_CLOSE(7.0, (*array)[4][1], 1e-4);

  CHECK_CLOSE(7.900000, (*array)[5][0], 1e-4);
  CHECK_CLOSE(7.900000, (*array)[5][1], 1e-4);


  _projector.findGridFromPoint(osg::Vec3(9, 2, 0), p1InGrind);
  _projector.findGridFromPoint(osg::Vec3(9, 4, 0), p2InGrind);

  array = _projector.followLine(p1InGrind, p2InGrind);

  CHECK_EQUAL(3, array->size());

  CHECK_EQUAL(9, (*array)[0][0]);
  CHECK_EQUAL(2, (*array)[0][1]);

  CHECK_CLOSE(9.0, (*array)[1][0], 1e-4);
  CHECK_EQUAL(3, (*array)[1][1]);

  CHECK_CLOSE(9., (*array)[2][0], 1e-4);
  CHECK_EQUAL(4, (*array)[2][1]);


  _projector.findGridFromPoint(osg::Vec3(5, 3, 0), p1InGrind);
  _projector.findGridFromPoint(osg::Vec3(9, 3, 0), p2InGrind);

  array = _projector.followLine(p1InGrind, p2InGrind);

  CHECK_EQUAL(5, array->size());

  CHECK_EQUAL(5, (*array)[0][0]);
  CHECK_EQUAL(3, (*array)[0][1]);

  CHECK_CLOSE(5.0, (*array)[1][0], 1e-4);
  CHECK_EQUAL(3, (*array)[1][1]);

  CHECK_CLOSE(7., (*array)[2][0], 1e-4);
  CHECK_EQUAL(3, (*array)[2][1]);

  CHECK_CLOSE(9., (*array)[3][0], 1e-4);
  CHECK_EQUAL(3, (*array)[3][1]);

  CHECK_CLOSE(9., (*array)[4][0], 1e-4);
  CHECK_EQUAL(3, (*array)[4][1]);

  _projector.findGridFromPoint(osg::Vec3(9, 3, 0), p1InGrind);
  _projector.findGridFromPoint(osg::Vec3(5, 3, 0), p2InGrind);

  array = _projector.followLine(p1InGrind, p2InGrind);

  CHECK_EQUAL(5, array->size());

  CHECK_EQUAL(5, (*array)[3][0]);
  CHECK_EQUAL(3, (*array)[3][1]);

  CHECK_EQUAL(7, (*array)[2][0]);
  CHECK_EQUAL(3, (*array)[2][1]);

  CHECK_CLOSE(9.0, (*array)[1][0], 1e-4);
  CHECK_EQUAL(3, (*array)[1][1]);

  CHECK_CLOSE(9., (*array)[0][0], 1e-4);
  CHECK_EQUAL(3, (*array)[0][1]);


  _projector.findGridFromPoint(osg::Vec3(8, 8, 0), p1InGrind);
  _projector.findGridFromPoint(osg::Vec3(8, 15, 0), p2InGrind);

  array = _projector.followLine(p1InGrind, p2InGrind);
  CHECK_EQUAL(6, array->size());

  CHECK_EQUAL(8, (*array)[0][0]);
  CHECK_EQUAL(8, (*array)[0][1]);

  CHECK_CLOSE(8, (*array)[1][0], 1e-4);
  CHECK_EQUAL(9, (*array)[1][1]);

  CHECK_CLOSE(8, (*array)[2][0], 1e-4);
  CHECK_EQUAL(11, (*array)[2][1]);

  CHECK_CLOSE(8, (*array)[3][0], 1e-4);
  CHECK_EQUAL(13, (*array)[3][1]);


  _projector.findGridFromPoint(osg::Vec3(8, -8, 0), p1InGrind);
  _projector.findGridFromPoint(osg::Vec3(8, 15, 0), p2InGrind);
  bool valid = _projector.clipLine(p1InGrind, p2InGrind);
  CHECK_EQUAL(true, valid);
  array = _projector.followLine(p1InGrind, p2InGrind);
  CHECK_EQUAL(8, array->size());

  CHECK_EQUAL(8, (*array)[0][0]);
  CHECK_EQUAL(1, (*array)[0][1]);

  CHECK_CLOSE(8, (*array)[1][0], 1e-4);
  CHECK_EQUAL(1., (*array)[1][1]);

  CHECK_CLOSE(8, (*array)[2][0], 1e-4);
  CHECK_EQUAL(3, (*array)[2][1]);

  CHECK_CLOSE(8, (*array)[3][0], 1e-4);
  CHECK_EQUAL(5, (*array)[3][1]);

  CHECK_CLOSE(8, (*array)[4][0], 1e-4);
  CHECK_EQUAL(7, (*array)[4][1]);

  CHECK_CLOSE(8, (*array)[5][0], 1e-4);
  CHECK_EQUAL(9, (*array)[5][1]);

  CHECK_CLOSE(8, (*array)[6][0], 1e-4);
  CHECK_EQUAL(11, (*array)[6][1]);

  CHECK_CLOSE(8, (*array)[7][0], 1e-4);
  CHECK_EQUAL(11, (*array)[7][1]);
}


TEST(GetBlocksTouchedByLine)
{
  osg::HeightField* hf = MyFixture::initFakeData3();
  GetBlocksTouchedByLine ok(hf);
  ok(osg::Vec2d(1,1), osg::Vec2d(9,3));
  CHECK_EQUAL(3, ok._result.size());
  CHECK_EQUAL(1, ok._result[1]._min);
  CHECK_EQUAL(4, ok._result[1]._max);
  CHECK_EQUAL(5, ok._result[2]._min);
  CHECK_EQUAL(8, ok._result[2]._max);
  CHECK_EQUAL(8, ok._result[3]._min);
  CHECK_EQUAL(8, ok._result[3]._max);


  GetBlocksTouchedByLine ok2(hf);
  ok2(osg::Vec2d(9,1), osg::Vec2d(3,3));
  CHECK_EQUAL(3, ok2._result.size());
  CHECK_EQUAL(6, ok2._result[1]._min);
  CHECK_EQUAL(8, ok2._result[1]._max);
  CHECK_EQUAL(3, ok2._result[2]._min);
  CHECK_EQUAL(5, ok2._result[2]._max);
  CHECK_EQUAL(2, ok2._result[3]._min);
  CHECK_EQUAL(2, ok2._result[3]._max);

  GetBlocksTouchedByLine ok3(hf);
  ok3(osg::Vec2d(2.5499999523162842, 0.54999995231628418), osg::Vec2d(2.5499999523162842, 2.0499999523162842));
  CHECK_EQUAL(3, ok3._result.size());
  CHECK_EQUAL(2, ok3._result[0]._min);
  CHECK_EQUAL(2, ok3._result[0]._max);
  CHECK_EQUAL(2, ok3._result[1]._min);
  CHECK_EQUAL(2, ok3._result[1]._max);
  CHECK_EQUAL(2, ok3._result[2]._min);
  CHECK_EQUAL(2, ok3._result[2]._max);

  GetBlocksTouchedByLine ok4(hf);
  ok4(osg::Vec2d(-2.0, 0.54999995231628418), osg::Vec2d(13.5499999523162842, 2.0499999523162842));
  CHECK_EQUAL(3, ok4._result.size());
  CHECK_EQUAL(0, ok4._result[0]._min);
  CHECK_EQUAL(2, ok4._result[0]._max);
  CHECK_EQUAL(2, ok4._result[1]._min);
  CHECK_EQUAL(8, ok4._result[1]._max);
  CHECK_EQUAL(8, ok4._result[2]._min);
  CHECK_EQUAL(8, ok4._result[2]._max);
}



TEST_FIXTURE(MyFixture, insertTouchedTriangles)
{
  _projector.init(initFakeData2());
  osg::Vec3 v1(6.1,2.1,0);
  osg::Vec3 v2(9.1,5.1,0);
  osg::Vec3 v3(6.1,6.1,0);
  _projector.insertTouchedTriangles(v1,v2,v3);
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
  _projector.write();
}


TEST_FIXTURE(MyFixture, insertTouchedPoint)
{
  _projector.init(initFakeData2());
  osg::Vec3 v1(6.1,2.1,10);
  osg::Vec3 v2(9.1,5.1,-10);
  osg::Vec3 v3(6.1,6.1,20);
  _projector.insertTouchedPoint(v1);
  _projector.insertTouchedPoint(v2);
  _projector.insertTouchedPoint(v3);
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
  _projector.write("./result_test_casePoint1.osg");
}


TEST_FIXTURE(MyFixture, testCase1)
{
  _projector.init(readHeightFieldTest1().get());
  osg::Vec3 v1(235160, 2.39399e+06, 0);
  osg::Vec3 v2(231201, 2.38817e+06, 0);
  osg::Vec3 v3(235160, 2.37399e+06, 0);
  _projector.insertTouchedTriangles(v1,v2,v3);
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
  _projector.write("./result_test_case1.osg");
}

TEST_FIXTURE(MyFixture, testCase2)
{
  _projector.init(readHeightFieldTest1().get());
  osg::Node* node = osgDB::readNodeFile(getTestDir() + "test_case2.osg");
  
  node->accept(*this);
  _projector.write("./result_test_case2.osg");
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
}

TEST_FIXTURE(MyFixture, testCase2c)
{
  _projector.init(readHeightFieldTest1().get());
  _projector.insertTouchedTriangles( osg::Vec3(199875, 2.36931e+06, 0),
                                     osg::Vec3(200118, 2.36804e+06, 0),
                                     osg::Vec3(200184, 2.36914e+06, 0));
  _projector.write("./result_test_case2c.osg");
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
}

TEST_FIXTURE(MyFixture, testCase2b)
{
  _projector.init(readHeightFieldTest1().get());
  _projector.insertTouchedTriangles( osg::Vec3(200118, 2368040, 0),
                                     osg::Vec3(199875, 2369310, 0),
                                     osg::Vec3(199850, 2368390, 0));

  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  osg::Geometry* geom = new osg::Geometry;
  osg::Vec3Array* array = new osg::Vec3Array;
  array->push_back(osg::Vec3(200118, 2368040, 0));
  array->push_back(osg::Vec3(199875, 2369310, 0));
  array->push_back(osg::Vec3(199850, 2368390, 0));
  geom->setVertexArray(array);
  geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,3));
  geode->addDrawable(geom);
  
  osgDB::writeNodeFile(*geode, "./result_test_case2b_source.osg");
  _projector.write("./result_test_case2b.osg");
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
}

TEST_FIXTURE(MyFixture, testCase2d)
{
  _projector.init(readHeightFieldTest1().get());
  _projector.insertTouchedTriangles( osg::Vec3(201986, 2.36809e+06, 0),
                                     osg::Vec3(202222, 2.36808e+06, 0),
                                     osg::Vec3(202242, 2.36792e+06, 0));
  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  osg::Geometry* geom = new osg::Geometry;
  osg::Vec3Array* array = new osg::Vec3Array;
  array->push_back(osg::Vec3(201986, 2.36809e+06, 0));
  array->push_back(osg::Vec3(202222, 2.36808e+06, 0));
  array->push_back(osg::Vec3(202242, 2.36792e+06, 0));
  geom->setVertexArray(array);
  geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,3));
  geode->addDrawable(geom);

  osgDB::writeNodeFile(*geode, "./result_test_case2d_source.osg");
   _projector.write("./result_test_case2d.osg");
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
}



TEST_FIXTURE(MyFixture, testCase3)
{
  _projector.init(readHeightFieldTest1().get());
  osg::Node* node = osgDB::readNodeFile(getTestDir() + "test_case3.osg");
  
  node->accept(*this);
  _projector.write("./result_test_case3.osg");
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
}

TEST_FIXTURE(MyFixture, testCase4)
{
  _projector.init(readHeightFieldTest1().get());
  osg::Vec3 v1(235160, 2.39399e+06, 0);
  osg::Vec3 v2(235140, 2.38817e+06, 0);
  osg::Vec3 v3(235160, 2.37399e+06, 0);
  _projector.insertTouchedTriangles(v1,v2,v3);
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
  _projector.write("./result_test_case4.osg");
}


TEST_FIXTURE(MyFixture, testCaseLine1)
{
  _projector.init(readHeightFieldTest1().get());
  osg::Vec3 v1(235160, 2.39399e+06, 0);
  osg::Vec3 v2(235110, 2.38817e+06, 0);
  _projector.insertTouchedLines(v1,v2);
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
  _projector.write("./result_test_caseLine1.osg");
}

TEST_FIXTURE(MyFixture, testCaseLine2)
{
  _projector.init(readHeightFieldTest3().get());


//       -267732 4.91822e+06 996.788
//       -267663 4.91825e+06 1002.94
//       -267656 4.91825e+06 1003.06

  osg::Vec3 v1(-267732, 4.91822e+06, 996.788);
  osg::Vec3 v2(-267656, 4.91825e+06, 1003.06);
  _projector.insertTouchedLines(v1,v2);
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
  _projector.write("./result_test_caseLine2.osg");
}

TEST_FIXTURE(MyFixture, testCaseLine3)
{
  _projector.init(readHeightFieldTest3().get());


//       -267656 4.91825e+06 1003.06
//       -267646 4.91825e+06 1003.21
//       -267641 4.91826e+06 992.743
//       -267536 4.91829e+06 997.445

  osg::Vec3 v1(-267656, 4.91825e+06, 1003.06);
  osg::Vec3 v2(-267536, 4.91829e+06, 997.445);
  _projector.insertTouchedLines(v1,v2);
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
  _projector.write("./result_test_caseLine3.osg");
}

TEST_FIXTURE(MyFixture, testCaseLine4)
{
  _projector.init(initFakeData3());

  osg::Vec3 v1(6.1, 2.1, 100);
  osg::Vec3 v2(7.9, 7.9, 100);
  _projector.insertTouchedLines(v1,v2);
  CHECK_EQUAL(true, _projector._result->getNumPrimitiveSets() != 0);
  _projector.write("./result_test_caseLine4.osg");
}


TEST(pointInsideTriangle2d)
{
  osg::Vec3d tri[3];
  tri[2] = osg::Vec3d( 4, 4 ,4);
  tri[1] = osg::Vec3d( 4, 2 ,4);
  tri[0] = osg::Vec3d( 2, 2 ,4);
  osg::Vec3d p(2.5, 2.5, 0);
  CHECK_EQUAL(true, pointInsideTriangle2d(tri[0], tri[1], tri[2], p));
  p = osg::Vec3d(5, 2.5, 0);
  CHECK_EQUAL(false, pointInsideTriangle2d(tri[0], tri[1], tri[2], p));

  tri[2] = osg::Vec3d( 6, 4, 0.98935824632644653);
  tri[1] = osg::Vec3d( 6, 3, 0.65698659420013428);
  tri[0] = osg::Vec3d( 7, 4, 0.41211849451065063);
  p = osg::Vec3d(6.379310299468524, 3, 0);
  CHECK_EQUAL(false, pointInsideTriangle2d(tri[0], tri[1], tri[2], p));
}


TEST(distanceLineOnPlane)
{
  osg::Vec3 tri[3];
  tri[2] = osg::Vec3( 4, 4 ,4);
  tri[1] = osg::Vec3( 4, 2 ,4);
  tri[0] = osg::Vec3( 2, 2 ,4);

  osg::Vec3d src(10, 10, 10);
  osg::Vec3d dst(10, 10, -10);
  osg::Plane plane(tri[0], tri[1], tri[2]);
  CHECK_EQUAL(-6.0 , (dst[2]-src[2]) * distanceLineOnPlane(src, dst, plane));
}

TEST(projectTriangleToPlane)
{
  osg::Vec3 triDst[3];
  triDst[2] = osg::Vec3( 4, 4 ,4);
  triDst[1] = osg::Vec3( 4, 2 ,4);
  triDst[0] = osg::Vec3( 2, 2 ,4);

  osg::Vec3 triToProject[3];
  triToProject[2] = osg::Vec3( 40, 40 ,10);
  triToProject[1] = osg::Vec3( 40, 20 ,10);
  triToProject[0] = osg::Vec3( 20, 20 ,10);

  osg::Vec3d result[3];
  osg::Vec3 normal;
  projectTriangleToPlane(triToProject[0], triToProject[1], triToProject[2],
                         triDst[0], triDst[1], triDst[2],
                         result, normal);
  CHECK_EQUAL(0, normal[0]);
  CHECK_EQUAL(0, normal[1]);
  CHECK_EQUAL(1, normal[2]);

  CHECK_EQUAL(20, result[0][0]);
  CHECK_EQUAL(20, result[0][1]);
  CHECK_EQUAL(4, result[0][2]);

  CHECK_EQUAL(40, result[1][0]);
  CHECK_EQUAL(20, result[1][1]);
  CHECK_EQUAL(4, result[1][2]);

  CHECK_EQUAL(40, result[2][0]);
  CHECK_EQUAL(40, result[2][1]);
  CHECK_EQUAL(4, result[2][2]);
}


TEST(projectTriangleToTriangle)
{
  osg::Vec3 triDst[3];
  triDst[2] = osg::Vec3( 4, 4 ,4);
  triDst[1] = osg::Vec3( 4, 2 ,4);
  triDst[0] = osg::Vec3( 2, 2 ,4);

  osg::Vec3 triToProject[3];
  triToProject[2] = osg::Vec3( 3, 6 ,10);
  triToProject[1] = osg::Vec3( 6, 3 ,10);
  triToProject[0] = osg::Vec3( 2, 3 ,10);

  osg::Vec3d result[3];
  osg::Vec3 normal;
  osg::ref_ptr<osg::Geometry> geom;
  geom = projectTriangleToTriangle(triToProject[0], triToProject[1], triToProject[2],
                                   triDst[0], triDst[1], triDst[2]);

  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  geode->addDrawable(geom.get());
  osgDB::writeNodeFile(*geode,"./test.osg");
  //  std::cout << geom->getVertexArray()->getNumElements() << std::endl;
  
  CHECK_EQUAL(true, geom.valid());
}

TEST(gridTest)
{
  osg::Vec3Array* array = new osg::Vec3Array;
  osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
  geom->setVertexArray(array);
  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  geode->addDrawable(geom.get());

  int size = 50;
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      array->push_back( osg::Vec3(i, j+1 ,0));
      array->push_back( osg::Vec3(i, j ,0));
      array->push_back( osg::Vec3(i+1, j ,0));
      array->push_back( osg::Vec3(i+1, j+1 ,0));
    }
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,i*size*4,size*4));
  }
  osgDB::writeNodeFile(*geode,"./test.osg");

  int start = array->size();
  array->push_back( osg::Vec3(1.5, 1.5, 0));
  array->push_back( osg::Vec3(4.2, 0.5, 0));
  array->push_back( osg::Vec3(4.2, 2.5, 0));

  array->push_back( osg::Vec3(5.5, 1.5, 0));
  array->push_back( osg::Vec3(6.2, 0.5, 0));
  array->push_back( osg::Vec3(6.2, 2.5, 0));
  geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,start,6));

  osgUtil::Tessellator* tsl = new osgUtil::Tessellator;
  tsl->setTessellationNormal(osg::Vec3(0,0,1));
  tsl->setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
  tsl->setWindingType(osgUtil::Tessellator::TESS_WINDING_ABS_GEQ_TWO);
  tsl->retessellatePolygons(*geom);
  osgDB::writeNodeFile(*geode,"./tesselate.osg");
}


int main(int argc, char** argv)
{
  return UnitTest::RunAllTests();
}
