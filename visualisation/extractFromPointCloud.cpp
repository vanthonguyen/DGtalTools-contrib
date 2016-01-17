
/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/
/**
 * @file 
 * @author Van-Tho Nguyen (vantho.nguyen@nancy.inra.fr)
 *
 * @date 2016/01/16
 *
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>


#ifndef Q_MOC_RUN
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/io/readers/MeshReader.h"

#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/readers/GenericReader.h"

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#endif


using namespace std;
using namespace DGtal;
using namespace Z3i;

static const double approxSamePlane = 0.1;
///////////////////////////////////////////////////////////////////////////////
namespace po = boost::program_options;

bool sameSide(const RealPoint &p1,const RealPoint &p2, const RealPoint &a,const RealPoint &b)
{
  RealPoint cp1 = (b-a).crossProduct(p1-a);
  RealPoint cp2 = (b-a).crossProduct(p2-a);
  return cp1.dot(cp2) >= 0;
}


bool
isInsideFaceTriangle(const RealPoint &p, const RealPoint &q, const RealPoint &r, const RealPoint &aPoint ){
  if (sameSide(aPoint, p, q, r) && sameSide(aPoint, q, p, r) && sameSide(aPoint, r, p, q))
  {
    RealPoint vc1 =  (p-q).crossProduct(p-r);
    return std::abs((p-aPoint).dot(vc1)) <= approxSamePlane;
  }
  return false;
}

RealPoint
getProjectedPoint(const RealPoint &normal, const RealPoint &aPlanePt, const RealPoint &p ){
  double d = -(normal[0]*aPlanePt[0]+normal[1]*aPlanePt[1]+normal[2]*aPlanePt[2]);
  double dist = -(normal[0]*p[0] + normal[1]*p[1] + normal[2]*p[2] + d)/
  (normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]);
  return (dist*normal+p);
}

int main( int argc, char** argv )
{

  // parse command line ----------------------------------------------
  po::options_description general_opt("Allowed options are: ");
  general_opt.add_options()
  ("help,h", "display this message")
  ("input,i", po::value<std::string>(), "input file: mesh (off,obj)" )
  ("inputPointCloud,p", po::value<std::string>(), "point cloud to extract (xyz)" )
  ("output,o", po::value<std::string>()->default_value("out.xyz"), "output point cloud corresponding to input mesh" );

  bool parseOK=true;
  bool cannotStart= false;
  
  po::variables_map vm;
  try{
    po::store(po::parse_command_line(argc, argv, general_opt), vm);
  }catch(const std::exception& ex){
    parseOK=false;
    trace.error()<< "Error checking program options: "<< ex.what()<< endl;
  }
  po::notify(vm);
  if(parseOK && ! vm.count("input"))
  {
      trace.error() << " The input mesh file name was not defined" << endl;
      cannotStart = true;
  }

  if(parseOK && ! vm.count("inputPointCloud"))
  {
      trace.error() << " The input point cloud file name was not defined" << endl;
      cannotStart = true;
  }

  string inputFilename = vm["input"].as<std::string>();
  string inputPointCloudFileName = vm["inputPointCloud"].as<std::string>();
  string outputFilename = vm["output"].as<std::string>();


  Mesh<Z3i::RealPoint>  aMesh(true);
  aMesh << inputFilename;
  std::vector<RealPoint> points = PointListReader<RealPoint>::getPointsFromFile(inputPointCloudFileName);
  assert(points.size() > 0);
  std::vector<int> insideMesh;
  for (unsigned int i = 0; i < points.size(); i++){
      RealPoint aPoint = points[i];
      trace.progressBar(i, points.size());
      for (unsigned int j = 0; j < aMesh.nbFaces(); j++){
        std::vector<unsigned int>  aFace = aMesh.getFace(j);
        RealPoint p0 = aMesh.getVertex(aFace.at(0));
        RealPoint p1 = aMesh.getVertex(aFace.at(1));
        RealPoint p2 = aMesh.getVertex(aFace.at(2));
        RealPoint normal = ((p0-p1).crossProduct(p2 - p1));
        RealPoint proj = getProjectedPoint(normal, p0, aPoint);
        if(isInsideFaceTriangle(p0, p1, p2, proj)){
            insideMesh.push_back(i);
        }
      }
  }

  std::ofstream outId;
  std::string outName = vm["output"].as<std::string>();
  outId.open(outName.c_str(), std::ofstream::out);
  for(unsigned int i = 0; i < insideMesh.size(); i++){
      outId <<points[insideMesh[i]][0]<< " "<<points[insideMesh[i]][1]<<" "<<points[insideMesh[i]][2]<<" "<<insideMesh[i]<<std::endl;
  }
    outId.close();
  return 0;
}

