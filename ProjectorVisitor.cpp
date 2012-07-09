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
 *  Cedric Pinson <cedric.pinson@plopbyte.com>
 */

#include <ProjectorVisitor>
#include <osg/CoordinateSystemNode>
#include <osgUtil/Tessellator>
#include <cassert>

struct LatLongHeight2xyz : public osg::NodeVisitor {
    osg::ref_ptr<osg::EllipsoidModel> _coordinates;

    LatLongHeight2xyz() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {
        _coordinates = new osg::EllipsoidModel();
    }
    
    void apply(osg::Geometry& geom) {
        osg::Vec3Array* array = dynamic_cast<osg::Vec3Array*>(geom.getVertexArray());
        if (array) {
            for (unsigned int i = 0; i < array->size(); i++) {
                double x,y,z;
                double lng = (*array)[i][0];
                double lat = (*array)[i][1];
                _coordinates->convertLatLongHeightToXYZ(osg::DegreesToRadians(lat), osg::DegreesToRadians(lng), 0, x, y, z);
                (*array)[i] = osg::Vec3(x,y,z);
            }
        }
    }

    void apply(osg::Geode& node) {
        for (int i = 0; i < node.getNumDrawables(); i++) {
            if (node.getDrawable(i) && node.getDrawable(i)->asGeometry())
                apply(*(node.getDrawable(i)->asGeometry()));
        }
    }
};

void intersectGridLinesFromSegment(const osg::Vec2d& s0, const osg::Vec2d& s1, std::vector<osg::Vec2d>& list)
{
  struct Entry { 
    float dist;
    osg::Vec2d value;
  };


  osg::Vec2d min(FLT_MAX,FLT_MAX);
  osg::Vec2d max(-FLT_MAX,-FLT_MAX);

  if(s0.x()<min.x()) min.x() = s0.x();
  if(s0.x()>max.x()) max.x() = s0.x();

  if(s0.y()<min.y()) min.y() = s0.y();
  if(s0.y()>max.y()) max.y() = s0.y();

  if(s1.x()<min.x()) min.x() = s1.x();
  if(s1.x()>max.x()) max.x() = s1.x();

  if(s1.y()<min.y()) min.y() = s1.y();
  if(s1.y()>max.y()) max.y() = s1.y();

  list.clear();

  osg::Vec2d result;

  {
    int start = static_cast<int>(floor(min[0]));
    int stop = static_cast<int>(ceil(max[0]));
    double myMin = floor(min[1]);
    double myMax = ceil(max[1]);
    for (int i = start; i <= stop; i++) {
      if (lineIntersect(s0,s1,osg::Vec2d(i,myMin - 1), osg::Vec2d(i,myMax + 1), result)) {
        list.push_back(osg::Vec2d(i, result[1]));
      }
    }
  }

  {
    int start = static_cast<int>(floor(min[1]));
    int stop = static_cast<int>(ceil(max[1]));
    double myMin = floor(min[0]);
    double myMax = ceil(max[0]);
    for (int j = start; j <= stop; j++) {
      if (lineIntersect(s0,s1,osg::Vec2d(myMin - 1, j), osg::Vec2d(myMax + 1,j ), result)) {
        list.push_back(osg::Vec2d(result[0], j));
      }
    }
  }
      
  std::sort(list.begin(), list.end(), less_mag(s0));
}


// from http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
bool lineIntersect(const osg::Vec2d& v1, 
                   const osg::Vec2d& v2, 
                   const osg::Vec2d& v3, 
                   const osg::Vec2d& v4, 
                   osg::Vec2d& result,
                   bool excludeBorder)
{
  // x1 + ua (x2 - x1) = x3 + ub (x4 - x3)
  // y1 + ua (y2 - y1) = y3 + ub (y4 - y3)
  osg::Vec2d dl1 = v2-v1;
  osg::Vec2d dl2 = v4-v3;

  double ua1 = ( dl2[0] * (v1[1] - v3[1]) ) - ( dl2[1] * (v1[0] - v3[0]));
  double ua2 = ( dl2[1] * dl1[0]) - ( dl2[0] * dl1[1]);
  if ( fabs(ua2) < 1e-5 )
    return false;

  double denom = 1.0/ua2;
  double ua = ua1 * denom;

  double ub1 = ( dl1[0] * (v1[1] - v3[1]) ) - ( dl1[1] * (v1[0] - v3[0]));
  double ub = ub1 * denom;

  if (excludeBorder) {
    if (!( ua > 0.0 && ua <1 && ub > 0.0 && ub <1))
      return false;
  } else if (!( ua >= 0.0 && ua <=1 && ub >= 0.0 && ub <=1))
    return false;

  double x = v1[0] + ua * dl1[0];
  double y = v1[1] + ua * dl1[1];
  result[0] = x;
  result[1] = y;
  return true;
}

int pointInsideTriangle(const osg::Vec3& t1, const osg::Vec3& t2, const osg::Vec3& t3, osg::Vec3* vertexes, int nbVertexes)
{
  osg::Vec3 v0 = t2 - t1;
  osg::Vec3 v1 = t3 - t1;

  // Compute dot products
  float dot00 = v0*v0;
  float dot01 = v0*v1;
  float dot11 = v1*v1;

  // Compute barycentric coordinates
  double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);

  int inside = 0;
  for (int i = 0; i < nbVertexes; i++) {
    osg::Vec3 v2 = vertexes[i] - t1;
    float dot02 = v0*v2;
    float dot12 = v1*v2;

    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    if ((u >= 0) && (v >= 0) && (u + v <= 1)) {
      inside++;
    }
  }
  return inside;
}

bool pointInsideTriangle2d(const osg::Vec3d& t1, const osg::Vec3d& t2, const osg::Vec3d& t3,const osg::Vec3d& vertex)
{
  osg::Vec2d v1(t1[0], t1[1]);
  osg::Vec2d v2(t2[0], t2[1]);
  osg::Vec2d v3(t3[0], t3[1]);

  osg::Vec2d d0 = v1 - v3;
  osg::Vec2d p(vertex[0], vertex[1]);

  osg::Vec2d dp = p - v1;
  double c0 = d0[0] * dp[1] - d0[1] * dp[0];

  osg::Vec2d d1 = v2 - v1;
  dp = p - v2;
  double c1 = d1[0] * dp[1] - d1[1] *dp[0];

  if ( (c0 > 0.0 && c1 < 0.0) ||
       (c0 < 0 && c1 > 0) )
    return false;
  
  osg::Vec2d d2 = v3 - v2;
  dp = p - v2;
  double c2 = d2[0] * dp[1] - d2[1] *dp[0];

  if ( (c0 > 0.0 && c2 < 0.0) ||
       (c0 < 0 && c2 > 0) )
    return false;

  return true;
}

struct ConvertToDrawArray 
{
  osg::ref_ptr<osg::Geometry> _geom;
  osg::ref_ptr<osg::Vec3Array> _array;
  osg::ref_ptr<osg::DrawArrays> _dw;
    
  void init(osg::Geometry* geom) {
    _geom = geom;
    _array = static_cast<osg::Vec3Array*>(geom->getVertexArray());
    if (!_geom->getNumPrimitiveSets()) {
      _dw = new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,0);
      geom->addPrimitiveSet(_dw.get());
    } else {
      _dw = dynamic_cast<osg::DrawArrays*>(_geom->getPrimitiveSet(_geom->getNumPrimitiveSets()-1));
    }
  }

  void operator ()(const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3, bool treatVertexDataAsTemporary) {
    _array->push_back(v1);
    _array->push_back(v2);
    _array->push_back(v3);
    _dw->setCount(_dw->getCount() + 3);
  }
};

struct ConvertToDrawArrayAndAppend
{
  osg::ref_ptr<osg::Geometry> _geom;
  osg::ref_ptr<osg::Vec3Array> _array;
  int _count;

  void init(osg::Geometry* geom) {
    assert(geom);
    _geom = geom;
    _array = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
    assert(_array.get());
    _count = 0;
  }

  void operator ()(const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3, bool treatVertexDataAsTemporary) {
    _count+=3;
    _array->push_back(v1);
    _array->push_back(v2);
    _array->push_back(v3);
  }
  void postProcess() {
    _geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, _array->size()-_count, _count));
    _count = 0;
  }
};


struct InsertGeometryHelper 
{
  osg::ref_ptr<osg::Geometry> _geom;
  osg::ref_ptr<osg::Vec3Array> _array;

  void init(osg::Geometry* geom) {
    _geom = geom;
    _array = static_cast<osg::Vec3Array*>(geom->getVertexArray());
  }

  void appendLineStrip(osg::Vec3Array* src) {
    int size = _array->size();
    _array->insert(_array->end(), src->begin(), src->end());
    _geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, size, src->size()));
  }
  void appendPoint(const osg::Vec3& p) {
    int size = _array->size();
    _array->push_back(p);
    _geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, size, 1));
  }

};


void ProjectVisitor::apply(osg::Geode& node) 
{
    if (_clearEveryDrawable)
        _projector.clear();

    std::vector<osg::Drawable*> toremove;
    std::vector<osg::ref_ptr<osg::Geometry> > toadd;
    for (int i = 0; i < node.getNumDrawables(); i++) {

        osg::Timer_t t0 = osg::Timer::instance()->tick();
        node.getDrawable(i)->accept(_projector);
        double duration = osg::Timer::instance()->delta_s(t0, osg::Timer::instance()->tick());

        if (_projector.hasGeometry()) {
            toremove.push_back(node.getDrawable(i));
            std::stringstream ss;
            ss << "projected_" << _index << ".osg";
            osg::notify(osg::INFO) <<  "duration " << duration << " " << ss.str() << std::endl;
            if (_writeResultOnDisk)
                _projector.write(ss.str());
            
            if (!_mergeGeometry) {
                osg::ref_ptr<osg::Geometry> result = _projector._result.get();
                result->setUserData(node.getDrawable(i)->getUserData());
                toadd.push_back(result);
                _projector.clear();
            }
        }
        _index++;
        _cummulatedTime += duration;
    }

    if (_mergeGeometry) {
        if (_projector.hasGeometry()) {
            toadd.push_back(_projector._result.get());
        }
    }

    for (int i = 0; i < toremove.size(); i++) // remove only where success to project data
        node.removeDrawable(toremove[i]);

    for (int i = 0; i < toadd.size(); i++) {// remove only where success to project data
        node.addDrawable(toadd[i]);
    }

    if (_projectToXYZ) {
        osg::ref_ptr<LatLongHeight2xyz> visitor = new LatLongHeight2xyz();
        node.accept(*visitor);
    }

    // smooth after project
    if (_smooth) {
        osgUtil::SmoothingVisitor smooth;
        for (int i = 0; i < toadd.size(); i++) {// remove only where success to project data
            smooth.smooth(*toadd[i]);
        }
    }

}


void ProjectVisitor::ProjectTriangleToHeightField::insertGeometryTo(osg::Geometry* src, osg::Geometry* dst) 
{
  osg::Vec3Array* datadst = dynamic_cast<osg::Vec3Array*>(dst->getVertexArray());
  osg::Vec3Array* datasrc = dynamic_cast<osg::Vec3Array*>(src->getVertexArray());

  if (!datadst || !datasrc)
    return;

  _insert++;
  //      std::cout << "current insertion " << _insert << std::endl;
  osg::TriangleFunctor<ConvertToDrawArray> convert;
  convert.init(dst);
  src->accept(convert);
  return;
  for (int i = 0 ; i < src->getNumPrimitiveSets(); i++) {

    if (src->getPrimitiveSet(i)->getType() == osg::PrimitiveSet::DrawArraysPrimitiveType) {
      osg::DrawArrays* dw = static_cast<osg::DrawArrays*>(src->getPrimitiveSet(i));
      datadst->insert(datadst->end(), datasrc->begin()+dw->getFirst(), datasrc->begin()+(dw->getFirst()+dw->getNumIndices()));
      osg::DrawArrays* ndw = new osg::DrawArrays(dw->getMode(),datadst->size()-dw->getCount(), dw->getCount());
      dst->addPrimitiveSet(ndw);
    } else if (src->getPrimitiveSet(i)->getType() == osg::PrimitiveSet::DrawElementsUBytePrimitiveType ||
               src->getPrimitiveSet(i)->getType() == osg::PrimitiveSet::DrawElementsUShortPrimitiveType ||
               src->getPrimitiveSet(i)->getType() == osg::PrimitiveSet::DrawElementsUIntPrimitiveType) {
      osg::TriangleFunctor<ConvertToDrawArray> convert;
      convert.init(dst);
      src->accept(convert);
    } else {
      std::cout << "draw type (" << src->getPrimitiveSet(i)->getType() << ") not supported" << std::endl;
    }
  }
}

bool ProjectVisitor::ProjectTriangleToHeightField::clipLine(osg::Vec2d& startInGrid, osg::Vec2d& endInGrid)
{
  if (startInGrid[1] > endInGrid[1]) {
    osg::Vec2d tmp = endInGrid;
    endInGrid = startInGrid;
    startInGrid = tmp;
  }

  // clip against heighfield
  if (startInGrid[1] > _heightField->getNumRows()-1 || endInGrid[1] < 0 || 
      (startInGrid[0] > _heightField->getNumColumns()-1 && endInGrid[0] > _heightField->getNumColumns()-1) ||
      (startInGrid[0] < 0 && endInGrid[0] < 0))
    return false;

  // not sure yet we intersect but we exclude a lot of case before
  // clip against heighfield
  double coefX = (endInGrid[1] - startInGrid[1]) / (endInGrid[0] - startInGrid[0]);
  double coefY = (endInGrid[0] - startInGrid[0]) / (endInGrid[1] - startInGrid[1]);

  if (startInGrid[0] < 0)
    if ( coefX > 0 ) {
      startInGrid[1] = startInGrid[1] + -startInGrid[0] * coefX ;
      startInGrid[0] = 0;
    } else 
      return false;

  if (startInGrid[0] > _heightField->getNumColumns())
    if ( coefX < 0 ) {
      startInGrid[1] = startInGrid[1] - (startInGrid[0] - _heightField->getNumColumns()) * coefX ;
      startInGrid[0] = _heightField->getNumColumns();
    } else 
      return false;

  if (endInGrid[0] < 0)
    if ( coefX < 0 ) {
      endInGrid[1] = endInGrid[1] - endInGrid[0] * coefX;
      endInGrid[0] = 0;
    } else 
      return false;

  if (endInGrid[0] > _heightField->getNumColumns())
    if ( coefX > 0 ) {
      endInGrid[1] = endInGrid[1] - (endInGrid[0] - _heightField->getNumColumns()) * coefX ;
      endInGrid[0] = _heightField->getNumColumns();
    } else 
      return false;


  if (startInGrid[1] < 0) {
    startInGrid[0] = startInGrid[0] + -startInGrid[1] * coefY ;
    startInGrid[1] = 0;
  }

  if (endInGrid[1] > _heightField->getNumRows()) {
    endInGrid[0] = endInGrid[0] - (endInGrid[1] - _heightField->getNumRows()) * coefY ;
    endInGrid[1] = _heightField->getNumRows();
  }
  return true;
}



osg::Vec3Array* ProjectVisitor::ProjectTriangleToHeightField::followLine(const osg::Vec2d& startInGridOrg, const osg::Vec2d& endInGridOrg)
{
  // every segment must be oriented with p1.y <= p2.y !!!!!
  assert(startInGridOrg[1] <= endInGridOrg[1]);

  osg::Vec3Array* array = new osg::Vec3Array;

  osg::Vec2d startInGrid = startInGridOrg;
  osg::Vec2d endInGrid = endInGridOrg;

  std::vector<osg::Vec2d> result;
  intersectGridLinesFromSegment(startInGridOrg, endInGridOrg, result);

  array->push_back(convertGridPointInWorldSpaceAndProject(startInGrid));

  bool right = (endInGridOrg[0]-startInGridOrg[0]) > 0;
  for (int i = 0; i < result.size(); i++) {
//     if (right) {
      osg::Vec3d projectedPoint = convertGridPointInWorldSpaceAndProject(result[i]); // optimize
      array->push_back(projectedPoint);
//     }
  }
  osg::Vec3d projectedPoint = convertGridPointInWorldSpaceAndProject(endInGrid);
  array->push_back(projectedPoint);
  return array;
}


void ProjectVisitor::ProjectTriangleToHeightField::insertTouchedLines(const osg::Vec3d& v1, const osg::Vec3d& v2)
{
  osg::Vec2d p1InGrind, p2InGrind;
  findGridFromPoint(v1, p1InGrind);
  findGridFromPoint(v2, p2InGrind);

  bool valid = clipLine(p1InGrind, p2InGrind);
  if (!valid)
    return;

  osg::ref_ptr<osg::Vec3Array> array = followLine(p1InGrind, p2InGrind);
  InsertGeometryHelper inserter;
  inserter.init(_result.get());
  inserter.appendLineStrip(array.get());
}

void ProjectVisitor::ProjectTriangleToHeightField::insertTouchedPoint(const osg::Vec3d& v1)
{
  osg::Vec2d p1InGrind;
  findGridFromPoint(v1, p1InGrind);
  int x,y;
  
  x = static_cast<int>(floor(p1InGrind[0]));
  y = static_cast<int>(floor(p1InGrind[1]));

  if (x > _heightField->getNumColumns()-1 || x < 0 ||
      y > _heightField->getNumRows()-1 || y < 0)
    return;
  
  osg::Vec3 result = convertGridPointInWorldSpaceAndProject(p1InGrind);
  InsertGeometryHelper inserter;
  inserter.init(_result.get());
  inserter.appendPoint(result);
}

void ProjectVisitor::ProjectTriangleToHeightField::insertTouchedTriangles(const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3) 
{
  Triangle2HeightField functor(_heightField.get());
  if (_result.get())
    functor.setDestinationGeometry(_result.get());

  functor(v1, v2, v3);
  _result = functor.getDestinationGeometry();
  return;
}


bool Triangle2HeightField::clipXPart(double& ystart, double& ystop,
                                     double& xLeft, double& xRight,
                                     const double& coefLeftY, const double& coefRightY)
{
  if (xRight < 0) { // find y to have xright inside 
    double yFound = ystart + -xRight / coefRightY;
    if (yFound > ystop) // not in this part
      return true;
    xRight = 0;
    double delta = yFound - ystart;
    ystart = yFound;
    xLeft += delta * coefLeftY;

  } else if (xLeft > _heightField->getNumColumns()-2) { // find y to have xleft inside

    double offset = xLeft - _heightField->getNumColumns()-2;
    double yFound = ystart - offset / coefLeftY;
    if (yFound > ystop) // we are on the second part of triangle or outside the square
      return true;

    xLeft = _heightField->getNumColumns();
    double delta = yFound - ystart;
    ystart = yFound;
    xRight += delta * coefRightY;
  }
  return false;
}


void Triangle2HeightField::clipLeft(const std::vector<osg::Vec2d>& in, std::vector<osg::Vec2d>& out, double limit)
{
  osg::Vec2d result;
  for (int i = 0; i < in.size(); i++) {
    const osg::Vec2d& prev = in[(in.size()+i-1)%in.size()];
    const osg::Vec2d& cur = in[i];
    bool previousInside = prev[0] >= limit;
    bool currentInside = cur[0] >= limit;
    if ( (previousInside && !currentInside) || (!previousInside && currentInside) ) {
      double min(FLT_MAX);
      double max(-FLT_MAX);
      if(prev.y()<min) min = prev.y();
      if(prev.y()>max) max = prev.y();
      if(cur.y()<min) min = cur.y();
      if(cur.y()>max) max = cur.y();
      bool intersect = lineIntersect(prev, 
                                     cur, 
                                     osg::Vec2d(limit, min - 1),
                                     osg::Vec2d(limit, max + 1),
                                     result,
                                     false);
      result[0] = limit;
      out.push_back(result);
      if (!previousInside && currentInside) {
        out.push_back(cur);
      }
    } else if (previousInside && currentInside) {
      out.push_back(cur);
    }
  }
}

void Triangle2HeightField::clipRight(const std::vector<osg::Vec2d>& in, std::vector<osg::Vec2d>& out, double limit)
{
  
  osg::Vec2d result;
  for (int i = 0; i < in.size(); i++) {
    const osg::Vec2d& prev = in[(in.size()+i-1)%in.size()];
    const osg::Vec2d& cur = in[i];
    bool previousInside = prev[0] <= limit;
    bool currentInside = cur[0] <= limit;
    if ( (previousInside && !currentInside) || (!previousInside && currentInside) ) {
      double min(FLT_MAX);
      double max(-FLT_MAX);
      if(prev.y()<min) min = prev.y();
      if(prev.y()>max) max = prev.y();
      if(cur.y()<min) min = cur.y();
      if(cur.y()>max) max = cur.y();
      bool intersect = lineIntersect(prev, 
                                     cur, 
                                     osg::Vec2d(limit, min - 1),
                                     osg::Vec2d(limit, max + 1),
                                     result,
                                     false);
      result[0] = limit;
      out.push_back(result);
      if (!previousInside && currentInside) {
        out.push_back(cur);
      }
    } else if (previousInside && currentInside) {
      out.push_back(cur);
    }
  }
}


void Triangle2HeightField::clipBottom(const std::vector<osg::Vec2d>& in, std::vector<osg::Vec2d>& out, double limit)
{
  osg::Vec2d result;
  for (int i = 0; i < in.size(); i++) {
    const osg::Vec2d& prev = in[(in.size()+i-1)%in.size()];
    const osg::Vec2d& cur = in[i];
    bool previousInside = prev[1] >= limit;
    bool currentInside = cur[1] >= limit;
    if ( (previousInside && !currentInside) || (!previousInside && currentInside) ) {
      double min(FLT_MAX);
      double max(-FLT_MAX);
      if(prev.x()<min) min = prev.x();
      if(prev.x()>max) max = prev.x();
      if(cur.x()<min) min = cur.x();
      if(cur.x()>max) max = cur.x();
      bool intersect = lineIntersect(prev, 
                                     cur, 
                                     osg::Vec2d(min - 1, limit),
                                     osg::Vec2d(max + 1, limit),
                                     result,
                                     false);
      result[1] = limit;
      out.push_back(result);
      if (!previousInside && currentInside) {
        out.push_back(cur);
      }
    } else if (previousInside && currentInside) {
      out.push_back(cur);
    }
  }
}

void Triangle2HeightField::clipTop(const std::vector<osg::Vec2d>& in, std::vector<osg::Vec2d>& out, double limit)
{
  osg::Vec2d result;
  for (int i = 0; i < in.size(); i++) {
    const osg::Vec2d& prev = in[(in.size()+i-1)%in.size()];
    const osg::Vec2d& cur = in[i];
    bool previousInside = prev[1] <= limit;
    bool currentInside = cur[1] <= limit;
    if ( (previousInside && !currentInside) || (!previousInside && currentInside) ) {
      double min(FLT_MAX);
      double max(-FLT_MAX);
      if(prev.x()<min) min = prev.x();
      if(prev.x()>max) max = prev.x();
      if(cur.x()<min) min = cur.x();
      if(cur.x()>max) max = cur.x();
      bool intersect = lineIntersect(prev, 
                                     cur, 
                                     osg::Vec2d(min - 1, limit),
                                     osg::Vec2d(max + 1, limit),
                                     result,
                                     false);
      result[1] = limit;
      out.push_back(result);
      if (!previousInside && currentInside) {
        out.push_back(cur);
      }
    } else if (previousInside && currentInside) {
      out.push_back(cur);
    }
  }
}

void Triangle2HeightField::clipTriangleAgainstHeightField(const osg::Vec2d& v1, const osg::Vec2d& v2, const osg::Vec2d& v3,
                                                          std::vector<osg::Vec2d>& result)
{
  std::vector<osg::Vec2d>& in = result;
  in.clear();
  in.push_back(v1);
  in.push_back(v2);
  in.push_back(v3);
  std::vector<osg::Vec2d> out;
  clipRight(in, out, _heightField->getNumColumns()-1);
  in.clear();
  clipLeft(out, in, 0);
  out.clear();
  clipTop(in, out, _heightField->getNumRows()-1);
  in.clear();
  clipBottom(out, in, 0);
}

void Triangle2HeightField::insertGeometryTo(osg::Geometry* src, osg::Geometry* dst) 
{
  osg::Vec3Array* datadst = dynamic_cast<osg::Vec3Array*>(dst->getVertexArray());
  osg::Vec3Array* datasrc = dynamic_cast<osg::Vec3Array*>(src->getVertexArray());

  assert(datadst && datasrc);

  _insert++;
  osg::TriangleFunctor<ConvertToDrawArrayAndAppend> convert;
  convert.init(dst);
  src->accept(convert);
  convert.postProcess();
}


void Triangle2HeightField::loopOnY(double ystart,
                                   double ystop,
                                   GetBlocksTouchedByLine* blockLeft,
                                   GetBlocksTouchedByLine* blockRight)
{
  int y0, y1;
  y0 = static_cast<int>(floor(ystart));
  y1 = static_cast<int>(floor(ystop));
  if (y0 > _heightField->getNumRows()-2) {
    y0 = _heightField->getNumRows()-2;
  }
  if (y1 > _heightField->getNumRows()-2) {
    y1 = _heightField->getNumRows()-2;
  }
  if (y0 > y1) {
    *((int*)0) = 0;
    assert(0 && "y1 < y0");
  }

  int x0,x1;
  for (int y = y0; y <= y1; y++) {
    if (blockLeft->_result.find(y) != blockLeft->_result.end()) {
      GetBlocksTouchedByLine::Entry entry = blockLeft->_result[y];
      osg::Vec3 vtx[4];
      for (int i = entry._min; i <= entry._max; i++) {
        if (isBlockAlreadyProcessed(i,y))
          continue;
        setBlockAsProcessed(i,y, 1);
        vtx[0] = _heightField->getVertex(i,y+1);
        vtx[1] = _heightField->getVertex(i,y);
        vtx[2] = _heightField->getVertex(i+1,y+1);
        vtx[3] = _heightField->getVertex(i+1,y);

        osg::ref_ptr<osg::Geometry> geom;
        geom = projectTriangleToTriangle(_v1, _v2, _v3,
                                         vtx[0], vtx[1], vtx[2]);
        if (geom->getNumPrimitiveSets()) {
          insertGeometryTo(geom.get(), _result.get());
          _found++;
        }
        geom = projectTriangleToTriangle(_v1, _v2, _v3,
                                         vtx[1], vtx[3], vtx[2]);
        if (geom->getNumPrimitiveSets()) {
          insertGeometryTo(geom.get(), _result.get());
          _found++;
        }
      }
      x0 = entry._max + 1;
    } else {
      *((int*)0) = 0;
      assert(0 && " Left Something wrong");
    }

    if (blockRight->_result.find(y) != blockRight->_result.end()) {
      GetBlocksTouchedByLine::Entry entry = blockRight->_result[y];
      osg::Vec3 vtx[4];
      for (int i = entry._min; i <= entry._max; i++) {
        if (isBlockAlreadyProcessed(i,y))
          continue;
        setBlockAsProcessed(i,y, 1);
        vtx[0] = _heightField->getVertex(i,y+1);
        vtx[1] = _heightField->getVertex(i,y);
        vtx[2] = _heightField->getVertex(i+1,y+1);
        vtx[3] = _heightField->getVertex(i+1,y);

        osg::ref_ptr<osg::Geometry> geom;
        geom = projectTriangleToTriangle(_v1, _v2, _v3,
                                         vtx[0], vtx[1], vtx[2]);
        if (geom->getNumPrimitiveSets()) {
          insertGeometryTo(geom.get(), _result.get());
          _found++;
        }
        geom = projectTriangleToTriangle(_v1, _v2, _v3,
                                         vtx[1], vtx[3], vtx[2]);
        if (geom->getNumPrimitiveSets()) {
          insertGeometryTo(geom.get(), _result.get());
          _found++;
        }
      }
      x1 = entry._min - 1;
    } else {
      *((int*)0) = 0;
      assert(0 && "Right Something wrong");
    }

    if (x1 - x0 >= 0) {
      int startBlock = -1;
      int stopBlock = x1;
      for (int i = x0; i <= x1; i++) {
        if (isBlockAlreadyProcessed(i,y))
          continue;
        startBlock = i;
        break;
      }
      if (startBlock != -1) {
        _vertexArray->push_back(_heightField->getVertex(startBlock,y+1));
        _vertexArray->push_back(_heightField->getVertex(startBlock,y));
        for (int i = x0; i <= x1; i++) {
          setBlockAsProcessed(i,y, 2);
          _vertexArray->push_back(_heightField->getVertex(i+1,y+1));
          _vertexArray->push_back(_heightField->getVertex(i+1,y));
          if (isBlockAlreadyProcessed(i+1,y)) {
            stopBlock = i;
            break;
          }
        }
        int size = 2 + ((stopBlock - startBlock) + 1) *2;
        _result->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP,_vertexArray->size() - size, size));
      }
    }
  }
}

bool Triangle2HeightField::setup(const osg::Vec3& v1org, const osg::Vec3& v2org, const osg::Vec3& v3org,
                                 double& miny, double& maxy,
                                 GetBlocksTouchedByLine*& blockLeft,
                                 GetBlocksTouchedByLine*& blockRight)
{
  _v1 = v1org;
  _v2 = v2org;
  _v3 = v3org;

  osg::Vec2d v1,v2,v3;
  findGridFromPoint(v1org, v1);
  findGridFromPoint(v2org, v2);
  findGridFromPoint(v3org, v3);


  if (clipBoundingBox(v1, v2, v3))
    return false;

  std::vector<osg::Vec2d> result;
  clipTriangleAgainstHeightField(v1, v2, v3, result);

  if (result.size() < 3)
    return false;

  int indexMinY = -1;
  int indexMaxY = -1;
  miny = FLT_MAX;
  maxy = -FLT_MAX;
  for (int i = 0; i < result.size(); i++) {
    if (result[i][1] < miny) {
      miny = result[i][1];
      indexMinY = i;
    }
    if (result[i][1] > maxy) {
      maxy = result[i][1];
      indexMaxY = i;
    }
  }

  if (!_result.valid()) {
    _result = new osg::Geometry;
    _vertexArray = new osg::Vec3Array;
    _result->setVertexArray(_vertexArray.get());
  }

  for (int i = indexMinY; i != indexMaxY; i = (i + 1) % result.size()) {
    int idx1 = (i+1+result.size())%result.size();
    (*blockRight)(result[i], result[idx1]);
  }
  for (int i = indexMinY; i != indexMaxY; i = (i - 1 + result.size()) % result.size()) {
    int idx1 = (i-1+result.size())%result.size();
    (*blockLeft)(result[i], result[idx1]);
  }

  osg::Vec2d vright = result[(indexMinY+1)%result.size()] - result[indexMinY] ;
  osg::Vec2d vleft = result[(indexMinY-1+result.size())%result.size()] - result[indexMinY] ;
  double coefRight = vright[0]/vright[1];
  double coefLeft = vleft[0]/vleft[1];
  if (coefLeft > coefRight) {
    GetBlocksTouchedByLine* tmp = blockLeft;
    blockLeft = blockRight;
    blockRight = tmp;
  }
    
  return true;
}

void Triangle2HeightField::operator()(const osg::Vec3& v1org, const osg::Vec3& v2org, const osg::Vec3& v3org)
{
  GetBlocksTouchedByLine* blockLeft = new GetBlocksTouchedByLine(_heightField.get());
  GetBlocksTouchedByLine* blockRight = new GetBlocksTouchedByLine(_heightField.get());
  double miny,maxy;
  if (setup(v1org, v2org, v3org, miny, maxy,
            blockLeft,
            blockRight))
    loopOnY(miny, maxy, blockLeft, blockRight);
}


bool Triangle2HeightField::clipBoundingBox(const osg::Vec2d& v1, const osg::Vec2d& v2, const osg::Vec2d& v3)
{
  osg::Vec2d min(FLT_MAX,FLT_MAX);
  osg::Vec2d max(-FLT_MAX,-FLT_MAX);

  if(v1.x()<min.x()) min.x() = v1.x();
  if(v1.x()>max.x()) max.x() = v1.x();

  if(v1.y()<min.y()) min.y() = v1.y();
  if(v1.y()>max.y()) max.y() = v1.y();

  if(v2.x()<min.x()) min.x() = v2.x();
  if(v2.x()>max.x()) max.x() = v2.x();

  if(v2.y()<min.y()) min.y() = v2.y();
  if(v2.y()>max.y()) max.y() = v2.y();

  if(v3.x()<min.x()) min.x() = v3.x();
  if(v3.x()>max.x()) max.x() = v3.x();

  if(v3.y()<min.y()) min.y() = v3.y();
  if(v3.y()>max.y()) max.y() = v3.y();
  // clip boundingbox  up down left right
  if (min[1] > _heightField->getNumRows()-1 || 
      max[1] < 0 ||
      max[0] < 0 ||
      min[0] > _heightField->getNumColumns()-1 )
    return true;
  return false;
}


osg::Geometry* projectTriangleToTriangle(const osg::Vec3& sv1, const osg::Vec3& sv2, const osg::Vec3& sv3,
                                         const osg::Vec3& dv1, const osg::Vec3& dv2, const osg::Vec3& dv3)
{
  osg::Vec3d newtriangle[3];
  osg::Vec3 normal;
  osg::Plane plane(sv1, sv2, sv3);

  projectTriangleToPlane(sv1, sv2, sv3,
                         dv1, dv2, dv3,
                         newtriangle,
                         normal);


  osg::Vec3Array* array = new osg::Vec3Array;
  osg::Geometry* geom = new osg::Geometry;
  if (plane.getNormal() * normal < 0) {
    // wrong triangle orientation
    // we revert the source
    //    std::cout << "revert triangle " << std::endl;
    array->push_back(osg::Vec3(newtriangle[2][0], newtriangle[2][1], newtriangle[2][2]));
    array->push_back(osg::Vec3(newtriangle[1][0], newtriangle[1][1], newtriangle[1][2]));
    array->push_back(osg::Vec3(newtriangle[0][0], newtriangle[0][1], newtriangle[0][2]));
  } else {
    array->push_back(osg::Vec3(newtriangle[0][0], newtriangle[0][1], newtriangle[0][2]));
    array->push_back(osg::Vec3(newtriangle[1][0], newtriangle[1][1], newtriangle[1][2]));
    array->push_back(osg::Vec3(newtriangle[2][0], newtriangle[2][1], newtriangle[2][2]));
  }
  geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, 3) );
  array->push_back(dv1);
  array->push_back(dv2);
  array->push_back(dv3);
  geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 3, 3));
  geom->setVertexArray(array);
  osgUtil::Tessellator* tsl = new osgUtil::Tessellator;
  tsl->setTessellationNormal(normal);
  tsl->setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
  tsl->setWindingType(osgUtil::Tessellator::TESS_WINDING_ABS_GEQ_TWO);
  tsl->retessellatePolygons(*geom);
  delete tsl;
  return geom;
}


double distanceLineOnPlane(const osg::Vec3d& src, const osg::Vec3d& dst, const osg::Plane& plane)
{
  // can be optimized -> dst-src is a axis aligned vector
  return (-plane[3] - plane.dotProductNormal(src))/plane.dotProductNormal(dst - src);
}

void projectTriangleToPlane(const osg::Vec3& sv1, const osg::Vec3& sv2, const osg::Vec3& sv3,
                            const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3,
                            osg::Vec3d* newTriangleOnPlane,
                            osg::Vec3& normal)
{
  osg::Plane plane(v1, v2, v3);
  int axis = 2;
  osg::Vec3d ProjectionDirection(0,0,0);
  ProjectionDirection[axis] = -1;

  osg::BoundingBox bb;
  bb.expandBy(v1);
  bb.expandBy(v2);
  bb.expandBy(v3);
  
  float min, max;
  min = bb._min[axis];
  max = bb._max[axis];

  {
  osg::Vec3d src = sv1;
  osg::Vec3d dst = src;
  dst[axis] = min - 10; // add a threshold
  src[axis] = max + 10; // add a threshold
  double t = distanceLineOnPlane(src, dst, plane);
  newTriangleOnPlane[0] = src + (dst-src) * t; // can be optimized too
  }


  {
  osg::Vec3d src = sv2;
  osg::Vec3d dst = src;
  dst[axis] = min - 10; // add a threshold
  src[axis] = max + 10; // add a threshold
  double t = distanceLineOnPlane(src, dst, plane);
  newTriangleOnPlane[1] = src + (dst-src) * t; // can be optimized too
  }


  {
  osg::Vec3d src = sv3;
  osg::Vec3d dst = src;
  dst[axis] = min - 10; // add a threshold
  src[axis] = max + 10; // add a threshold
  double t = distanceLineOnPlane(src, dst, plane);
  newTriangleOnPlane[2] = src + (dst-src) * t; // can be optimized too
  }
  
  normal = plane.getNormal();
}





