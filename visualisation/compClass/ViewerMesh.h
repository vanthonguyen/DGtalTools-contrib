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
 * @file ViewerMesh.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2015/12/20
 *
 * An simple mesh viewer which allow basic edition (coloring, face removing).
 *
 * This file is part of the DGtal library.
 */

#if defined(ViewerMesh_RECURSES)
#error Recursive header files inclusion detected in ViewerMesh.h
#else // defined(ViewerMesh_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ViewerMesh_RECURSES


#if !defined ViewerMesh_h
#define ViewerMesh_h
/** Prevents repeated inclusion of headers. */


#ifndef Q_MOC_RUN
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/Display3D.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ConstImageAdapter.h"
#endif
#include <deque>


#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

enum EditMode {ERASE_MODE, COLOR_MODE};


template < typename Space = DGtal::Z3i::Space, typename KSpace = DGtal::Z3i::KSpace>
class ViewerMesh: public DGtal::Viewer3D <Space, KSpace>
{

  static const unsigned int MAXUNDO=10;
  typedef DGtal::Mesh<DGtal::Z3i::RealPoint> RealMesh;


public:
  
  ViewerMesh(RealMesh &aMesh, std::string outMeshName): myPenScale(1.0), myPenColor(DGtal::Color::Blue), 
                                                        myMesh(aMesh), complementMesh(aMesh), myOutMeshName(outMeshName),
                                                        myPenSize(5.0), myMode(COLOR_MODE) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPcl(new pcl::PointCloud<pcl::PointXYZ>);
    for (unsigned int i = 0; i < myMesh.nbFaces(); i++) {
        DGtal::Z3i::RealPoint p = myMesh.getFaceBarycenter(i);
        cloudPcl->points.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
    }
    kdtree.setInputCloud (cloudPcl);
  }
  
  
  void postSelection(const QPoint& point);
  void deleteFacesFromDist(DGtal::Z3i::RealPoint p);
  void addToDelete(DGtal::Z3i::RealPoint p);
  void removeFromSelection(DGtal::Z3i::RealPoint p);
  void deleteCurrents();
  void deleteOthers();
  void doInvertSelection();
  void displaySelectionOnMesh();
  void setDeleteMode();
  void setColorMode();
  void undo();
  void save();

  double myPenScale;
  DGtal::Color myPenColor;
  double myPenSize;

  
protected:
  virtual QString helpString() const;  
  virtual void keyPressEvent ( QKeyEvent *e );
  virtual void mousePressEvent ( QMouseEvent *e );
  virtual void init();
  void addCurrentMeshToQueue();  
  
  RealMesh &myMesh;
  RealMesh complementMesh;
  std::string myOutMeshName;
  EditMode myMode;
  std::vector<unsigned int> myVectFaceToDelete;
  std::deque<RealMesh> myUndoQueue;
  std::deque<std::vector<unsigned int>> myUndoQueueSelected;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  
};


#endif // undefined viewer3dimage
#undef ViewerMesh_RECURSES
#endif // else defined(ViewerMesh_RECURSES)

