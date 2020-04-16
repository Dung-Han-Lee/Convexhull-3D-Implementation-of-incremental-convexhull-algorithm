## Incremental 3D-Convexhull algorithm
This repository contains an C++ implementation of 3D-ConvexHull algorithm from the  
book [Computational Geometry in C by O'Rourke](http://crtl-i.com/PDF/comp_c.pdf). RVIZ is used for visualization but is  
not required to use this package. This code is implemented with C++11 STL conta-  
iners only. The files are well documented, feel free to poke around.

[![Example video](https://media.giphy.com/media/hsV1GgRby1M4kDbAgm/giphy.gif)](https://youtu.be/DDgGc7_fEyU)

## Build and Run (using RVIZ)
    catkin_make
    source devel/setup.bash
    roscore
    rosrun convexhull_demo convex_node
    rviz

## Build and Run (text output only)
    cd /src
    g++ -std=c++11 convexhull.cpp plain_demo.cpp -o test.o && ./test.o

## Example Usage
Although in this example pcl pointcloud was used, the interface is a template  
function that takes any vector with points (expect to have field x, y and z)

    func(pcl::PointCloud<pcl::PointXYZI>::Ptr PC, Point3D& p)   
    {  
       Convexhull ch(PC->points);  
       if(ch.Contains(p)) // surface point is consider outside  
       {  
         //do something  
        }  
        auto vertices = ch.GetVertices();   
        auto faces = ch.GetFaces();  
        ...  
    }  

## Who calls whom
    ConvexHull()
    --ConstructHull()
      -- BulildFirstHull()
         -- Colinear()
         -- Coplanar()
         -- AddOneFace()
      -- IncreHull()
         -- VolumeSign()
         -- AddOneFace()
      -- CleanUp()
      -- ExtractExteriorPoints()

## License
  MIT License
