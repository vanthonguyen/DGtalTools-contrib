


# DGtalTools-contrib  0.9.4

- *Geometry2d*
  - LUTBasedNSDistanceTransform: Removed the dependency to libNetPBM. (Nicolas Normand,
  [#32](https://github.com/DGtal-team/DGtalTools-contrib/pull/32))

- *Geometry3d*
  - off2sdp: a new tool to convert a mesh into a set of points (.sdp). It can
    extract the mesh vertices (by default) or the face centers. 
    (Bertrand Kerautret [33](https://github.com/DGtal-team/DGtalTools-contrib/pull/32)) 
  
  
# DGtalTools-contrib  0.9.3

- *visualisation*
  - displayLineSegments: add a new tool allowing to display line segment in an output image. (Bertrand Kerautret)
  - meshViewerEdit: add a button to invert the current selection (Bertrand Kerautret)
  
- *Geometry2d*
  - houghLineDetect: to detect line segment from Hough transform (using OpenCV). 
  

# DGtalTools-contrib  0.9.2

- *visualisation*
  - graphViewer: add a new tool allowing to display graph from edges, vertex and radii.
    (Adrien Krahenbuhl)

- *Geometry3d*
  - xyzScale: a basic tool to adjust the scale of an xyz file. (Bertrand Kerautret)


# DGtalTools-contrib  0.9.1

- *Geometry2d*
 - meaningFullThickness: to display the meaningful thickness of digital contour. (Bertrand Kerautret)
 - LUTBasedNSDistanceTransform: Compute the 2D translated neighborhood-sequence distance transform of a binary image. (Nicolas Normand)
 - CumulativeSequenceTest and RationalBeattySequenceTest: tests from LUTBasedNSDistanceTransform. (Nicolas Normand)

- *Geometry3d*
 - basicEditMesh: to apply basic mesh edition (scale change, mesh face contraction, face filtering). (Bertrand Kerautret)
 - computeMeshDistances: computes for each face of a mesh A the minimal distance to another mesh B. (Bertrand Kerautret)
 - volLocalMax: extract the local maximas of a vol image within a spherical kernel. (Bertrand Kerautret)
 - basicMorphoFilter: apply basic morpho filter from a ball structural element. (Bertrand Kerautret)


- *visualisation*
 - displayTgtCoverAlphaTS: to display alpha-thick segment given on a simple contour. (Bertrand Kerautret)
 - meshViewerEdit: tool to visualize a mesh and to apply simple edits (face removal, color edits...). (Bertrand Kerautret)

