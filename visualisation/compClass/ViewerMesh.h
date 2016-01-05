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


#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/Display3D.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ConstImageAdapter.h"
#include <deque>

enum EditMode {ERASE_MODE, COLOR_MODE};


template < typename Space = DGtal::Z3i::Space, typename KSpace = DGtal::Z3i::KSpace>
class ViewerMesh: public DGtal::Viewer3D <Space, KSpace>
{

  static const unsigned int MAXUNDO=10;
  typedef DGtal::Mesh<DGtal::Z3i::RealPoint> RealMesh;
  typedef DGtal::ImageContainerBySTLVector<DGtal::Z3i::Domain,  std::vector<DGtal::Z3i::RealPoint> > ImagePointAssociation;

public:
  
  ViewerMesh(RealMesh &aMesh, std::string outMeshName): myPenScale(1.0), myPenColor(DGtal::Color::Blue), 
                                                        myMesh(aMesh), myOutMeshName(outMeshName),
                                                        myPenSize(5.0), myMode(COLOR_MODE) {
    std::vector<DGtal::Z3i::RealPoint::Component> vectX;
    std::vector<DGtal::Z3i::RealPoint::Component> vectY;
    std::vector<DGtal::Z3i::RealPoint::Component> vectZ;
    for( RealMesh::VertexStorage::const_iterator it= myMesh.vertexBegin(); it!=myMesh.vertexEnd(); it++){
        vectX.push_back((*it)[0]);
        vectY.push_back((*it)[1]);
        vectZ.push_back((*it)[2]);
    }

    DGtal::Z3i::RealPoint::Component valMaxX = *(std::max_element(vectX.begin(), vectX.end() )); 
    DGtal::Z3i::RealPoint::Component valMaxY = *(std::max_element(vectY.begin(), vectY.end() ));
    DGtal::Z3i::RealPoint::Component valMaxZ = *(std::max_element(vectZ.begin(), vectZ.end() )); 

    DGtal::Z3i::RealPoint::Component valMinX = *(std::min_element(vectX.begin(), vectX.end() )); 
    DGtal::Z3i::RealPoint::Component valMinY = *(std::min_element(vectY.begin(), vectY.end() )); 
    DGtal::Z3i::RealPoint::Component valMinZ = *(std::min_element(vectZ.begin(), vectZ.end() )); 

    DGtal::Z3i::RealPoint minPt (valMinX, valMinY, valMinZ);
    DGtal::Z3i::RealPoint maxPt (valMaxX, valMaxY, valMaxZ);
    boudingBox.first= minPt;
    boudingBox.second= maxPt;

    DGtal::Z3i::Domain dom(DGtal::Z3i::Point((int) minPt[0], (int) minPt[1], (int) minPt[2]),
    DGtal::Z3i::Point((int) maxPt[0], (int) maxPt[1], (int) maxPt[2]));
    unsigned int domSize = (int)(maxPt-minPt)[0]*(int)(maxPt-minPt)[1]*(int)(maxPt-minPt)[2];
    image3d = ImagePointAssociation(dom);
       
  }
  
  
  void postSelection(const QPoint& point);
  void deleteFacesFromDist(DGtal::Z3i::RealPoint p);
  void addToDelete(DGtal::Z3i::RealPoint p);
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
  virtual void init();
  void addCurrentMeshToQueue();  
  
  RealMesh &myMesh;
  //to speed up the selection
  ImagePointAssociation image3d;

  std::pair<DGtal::Z3i::RealPoint, DGtal::Z3i::RealPoint> boudingBox;

  std::string myOutMeshName;
  EditMode myMode;
  std::vector<unsigned int> myVectFaceToDelete;
  std::deque<RealMesh> myUndoQueue;
  std::deque<std::vector<unsigned int>> myUndoQueueSelected;

  std::pair<DGtal::Z3i::RealPoint, DGtal::Z3i::RealPoint> getBoundingBox();
  
};


#endif // undefined viewer3dimage
#undef ViewerMesh_RECURSES
#endif // else defined(ViewerMesh_RECURSES)

