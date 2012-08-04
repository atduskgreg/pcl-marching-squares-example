#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_hoppe.h>

using namespace pcl;
using namespace std;

int
  main (int argc, char** argv)
{
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

  if(io::loadPLYFile<PointXYZ> (argv[1], *cloud) == -1){
    cout << "ERROR: couldn't find file" << endl;
    return (1);
  } else {
    cout << "loaded" << endl;

    NormalEstimationOMP<PointXYZ, Normal> ne;
    search::KdTree<PointXYZ>::Ptr tree1 (new search::KdTree<PointXYZ>);
    tree1->setInputCloud (cloud);
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree1);
    ne.setKSearch (20);
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    ne.compute (*normals);
         
    // Concatenate the XYZ and normal fields*
    PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
    concatenateFields(*cloud, *normals, *cloud_with_normals);
 
    // Create search tree*
    search::KdTree<PointNormal>::Ptr tree (new search::KdTree<PointNormal>);
    tree->setInputCloud (cloud_with_normals);
    
    cout << "begin marching cubes reconstruction" << endl;    

    MarchingCubesHoppe<PointNormal> mc;
    PolygonMesh::Ptr triangles(new PolygonMesh);
    mc.setInputCloud (cloud_with_normals);
    mc.setSearchMethod (tree);
    mc.reconstruct (*triangles);

    cout << triangles->polygons.size() << " triangles created" << endl;
  }
  return (0);
}