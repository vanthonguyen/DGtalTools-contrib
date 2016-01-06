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
 * @file meshViewerEdit.cpp
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2015/12/20
 *
 * An simple mesh viewer which allow basic edition (coloring, face removing).
 *
 * This file is part of the DGtal library.
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


#include "meshViewerEdit.h"
#include "ui_meshViewerEdit.h"


using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
namespace po = boost::program_options;


typedef DGtal::ImageContainerBySTLVector<DGtal::Z3i::Domain,  std::vector<unsigned int> > ImagePointAssociation;
typedef DGtal::Mesh<DGtal::Z3i::RealPoint> RealMesh;


MainWindow::MainWindow(ViewerMesh<> *aViewer,
                       QWidget *parent, Qt::WindowFlags flags) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  myViewer(aViewer)
{
  ui->setupUi(this);
  ui->verticalLayout->addWidget(aViewer);  
  QObject::connect(ui->scaleSlider, SIGNAL(valueChanged(int)), this, SLOT(updatePenSize()));
  QObject::connect(ui->deleteButton, SIGNAL(clicked()), this, SLOT(setDeleteMode()));
  QObject::connect(ui->colorButton, SIGNAL(clicked()), this, SLOT(setColorMode()));
  QObject::connect(ui->saveButton, SIGNAL(clicked()), this, SLOT(save()));
  QObject::connect(ui->undoButton, SIGNAL(clicked()), this, SLOT(undo()));
  updatePenSize();
}


void 
MainWindow::setDeleteMode(){
  myViewer->setDeleteMode();
}

void 
MainWindow::setColorMode(){
  myViewer->setColorMode();
}

void 
MainWindow::undo(){
  myViewer->undo();
}

void 
MainWindow::save(){
  myViewer->save();
}


void 
MainWindow::updatePenSize(){
  (*myViewer).myPenSize = ui->scaleSlider->value();
  stringstream s; s << ui->scaleSlider->value();
  ui->labelPenSize->setText(QString(s.str().c_str()));
}


MainWindow::~MainWindow()
{
  delete ui;
}

int main( int argc, char** argv )
{

  // parse command line ----------------------------------------------
  po::options_description general_opt("Allowed options are: ");
  general_opt.add_options()
  ("help,h", "display this message")
  ("input,i", po::value<std::string>(), "input file: mesh (off,obj)" )
  ("scalePen,s", po::value<double>(), "change the scale factor of the pen size (by default 1.0, real size: penSize*scale)" )
  ("penColor,c", po::value<std::vector<unsigned int>>()->multitoken(), "change the scale factor of the pen size (by default 1.0)" )

  ("output,o", po::value<std::string>()->default_value("out.off"), "output file: mesh (off,obj)" );

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
      trace.error() << " The input file name was not defined" << endl;
      cannotStart = true;
    }

  string inputFilename = vm["input"].as<std::string>();
  string outputFilename = vm["output"].as<std::string>();
  
  
  Mesh<Z3i::RealPoint>  aMesh(true);
  aMesh << inputFilename;
  std::vector<DGtal::Z3i::RealPoint::Component> vectX;
  std::vector<DGtal::Z3i::RealPoint::Component> vectY;
  std::vector<DGtal::Z3i::RealPoint::Component> vectZ;
  for( RealMesh::VertexStorage::const_iterator it= aMesh.vertexBegin(); it!=aMesh.vertexEnd(); it++){
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

  DGtal::Z3i::Domain dom(DGtal::Z3i::Point((int) minPt[0], (int) minPt[1], (int) minPt[2]),
          DGtal::Z3i::Point((int) maxPt[0], (int) maxPt[1], (int) maxPt[2]));
  unsigned int domSize = (int)(maxPt-minPt)[0]*(int)(maxPt-minPt)[1]*(int)(maxPt-minPt)[2];

  ImagePointAssociation anImage3d(dom);

  QApplication application(argc,argv);

  ViewerMesh<> *viewer = new ViewerMesh<> (aMesh, outputFilename, anImage3d);
  if (vm.count("scalePen")){
    viewer->myPenScale = vm["scalePen"].as<double>();
  }
  if(vm.count("penColor")){
    std::vector<unsigned int> colors = vm["penColor"].as<std::vector<unsigned int> >();
    if (colors.size()!=4){
      trace.warning() << "you need to precise R,G,B, A values, taking default blue color..." << std::endl;
    }else{
      viewer->myPenColor = DGtal::Color(colors[0], colors[1], colors[2], colors[3]);
    }
  }
  
  MainWindow w(viewer, 0,0);
  w.setWindowTitle("Simple Mesh Edit");
  w.show();
  *viewer << aMesh;
  
  *viewer << Viewer3D<>::updateDisplay;
  return application.exec();
}

