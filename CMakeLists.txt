#Bertrand Kerautret <bertrand.kerautret@loria.fr>
# -----------------------------------------------------------------------------
# Project definition
# -----------------------------------------------------------------------------
PROJECT(DGtalTools-contrib)

cmake_minimum_required (VERSION 2.6) 

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


FIND_PACKAGE(DGtal 0.9.3 REQUIRED)
INCLUDE_DIRECTORIES(${DGTAL_INCLUDE_DIRS})
LINK_DIRECTORIES(${DGTAL_LIBRARY_DIRS})

 # -------------------------------------------------------------------------
 # This test is for instance used for ITK v3.x. ITK forces a limited
 # template depth. We remove this option.
 # --------------------------------------------------------------------------
 if (CMAKE_CXX_FLAGS MATCHES "-ftemplate-depth-[0-9]*")
    message( "Warning: some package has enabled a limited template depth for the C++ compiler." )
    message( "         Disabling option -ftemplate-depth-xx in CMAKE_CXX_FLAGS." )
    set( CMAKE_CXX_FLAGS_TMP ${CMAKE_CXX_FLAGS} )
    STRING( REGEX REPLACE "-ftemplate-depth-[0-9]*" ""
    CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_TMP}" )
     message ("         CMAKE_CXX_FLAGS=" ${CMAKE_CXX_FLAGS} )
  endif (CMAKE_CXX_FLAGS MATCHES "-ftemplate-depth-[0-9]*")


if(USE_OPENCV)
   find_package( OpenCV REQUIRED )
endif()

# -----------------------------------------------------------------------------
# Define variables and cmake parameters, and display some information
# -----------------------------------------------------------------------------
# SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
SET(DGtalToolsContrib_VERSION_MAJOR 0)
SET(DGtalToolsContrib_VERSION_MINOR 9)
SET(DGtalToolsContrib_VERSION_PATCH 3)
SET(DGTALTOOLSContrib_VERSION "${DGtalToolsContrib_VERSION_MAJOR}.${DGtalToolsContrib_VERSION_MINOR}.${DGtalToolsContrib_VERSION_PATCH}")
SET(PROJECT_VERSION "${DGtalToolsContrib_VERSION_MAJOR}.${DGtalToolsContrib_VERSION_MINOR}.${DGtalToolsContrib_VERSION_PATCH}")
SET(VERSION ${DGtalToolsContrib_VERSION_MAJOR}.${DGtalToolsContrib_VERSION_MINOR}.${DGtalToolsContrib_VERSION_PATCH})

SET(Boost_USE_STATIC_LIBS   ON)
SET(Boost_USE_MULTITHREADED ON)
SET(Boost_USE_STATIC_RUNTIME OFF)
SET(Boost_FOUND FALSE)
FIND_PACKAGE(Boost 1.46.0 REQUIRED COMPONENTS program_options)
if ( Boost_FOUND )
  ADD_DEFINITIONS(${BOOST_DEFINITIONS} -DBOOST_ALL_NO_LIB)
  message(STATUS "Boost and boost_program_options found.")
  include_directories( ${Boost_INCLUDE_DIRS} )
  SET(DGtalToolsContribLibDependencies ${DGtalToolsContribLibDependencies} 
     ${Boost_LIBRAIRIES}  
     ${Boost_PROGRAM_OPTIONS_LIBRARY})
   SET(DGtalLibInc ${Boost_INCLUDE_DIRS})
endif( Boost_FOUND )

#------Tools subdirectories ------
add_subdirectory(geometry2d)
add_subdirectory(geometry3d)
add_subdirectory(visualisation)


