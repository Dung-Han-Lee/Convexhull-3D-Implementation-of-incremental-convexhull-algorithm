/*The MIT License

Copyright (c) 2020 Dung-Han Lee
dunghanlee@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE. */

/* This work is an implementation of incremental convex hull algorithm from
the book Computational Geometry in C by O'Rourke */

#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include "utility.h"


// Defined in CCW
struct Face
{
  Face(const Point3D& p1, const Point3D& p2, const Point3D& p3): visible(false)
      { vertices[0] = p1; vertices[1] = p2; vertices[2] = p3;};

  void Reverse(){std::swap(vertices[0], vertices[2]); };

  friend std::ostream& operator<<(std::ostream& os, const Face& f)
  {
    os << "[face pt1 = "   << f.vertices[0].ToString()
       << " | face pt2 = " << f.vertices[1].ToString()
       << " | face pt3 = " << f.vertices[2].ToString()<<"] ";
    return os;
  }

  bool visible;
  Point3D vertices[3];
};

struct Edge
{
  Edge(const Point3D& p1, const Point3D& p2): 
      adjface1(nullptr), adjface2(nullptr), remove(false) 
      { endpoints[0] = p1; endpoints[1] = p2; };
  
  void LinkAdjFace(Face* face) 
  {
    if( adjface1 != NULL && adjface2 != NULL )
    {
      std::cout<<"warning: property violated!\n";
    }
    (adjface1 == NULL ? adjface1 : adjface2)= face;
  };

  void Erase(Face* face) 
  {
    if(adjface1 != face && adjface2 != face) return;
    (adjface1 == face ? adjface1 : adjface2) = nullptr;
  };

  friend std::ostream& operator<<(std::ostream& os, const Edge& e)
  {
    os << "[edge pt1 = "   << e.endpoints[0].ToString()
       << " | edge pt2 = " << e.endpoints[1].ToString() << " ]";
    return os;
  }

  bool remove; 
  Face *adjface1, *adjface2;
  Point3D endpoints[2];
};

class ConvexHull
{
  public:
    template<typename T> ConvexHull(const std::vector<T>& points);
    // All major works are conducted upon calling of constructor

    ~ConvexHull() = default;

    template<typename T> bool Contains(T p) const;
    // In out test for a query point (surface point is considered outside)

    const std::list<Face>& GetFaces() const {return this->faces;};
    
    const std::vector<Point3D> GetVertices() const \
        {return this->exterior_points;};
    // Return exterior vertices than defines the convell hull

    void Print(const std::string mode);
    // mode {face, edge, vertice}

    int Size() const {return this->exterior_points.size();};

  private:

    bool Colinear(const Point3D& a, const Point3D& b, const Point3D& c) const;

    bool CoPlanar(Face& f, Point3D& p);

    int VolumeSign(const Face& f, const Point3D& p) const;
    // A point is considered outside of a CCW face if the volume of tetrahedron
    // formed by the face and point is negative. Note that origin is set at p.

    size_t Key2Edge(const Point3D& a, const Point3D& b) const;
    // Hash key for edge. hash(a, b) = hash(b, a)

    void AddOneFace(const Point3D& a, const Point3D& b, 
        const Point3D& c, const Point3D& inner_pt);
    // Inner point is used to make the orientation of face consistent in counter-
    // clockwise direction

    bool BuildFirstHull(std::vector<Point3D>& pointcloud);
    // Build a tetrahedron as first convex hull

    void IncreHull(const Point3D& p);

    void ConstructHull(std::vector<Point3D>& pointcloud);

    void CleanUp();

    void ExtractExteriorPoints();

    Point3D FindInnerPoint(const Face* f, const Edge& e);
    // for face(a,b,c) and edge(a,c), return b

    std::vector<Point3D> pointcloud = {};
    std::vector<Point3D> exterior_points = {};
    std::list<Face> faces = {};
    std::list<Edge> edges = {};
    std::unordered_map<size_t, Edge*> map_edges;
};

template<typename T> ConvexHull::ConvexHull(const std::vector<T>& points)
{
  const int n = points.size();
  this->pointcloud.resize(n);
  for(int i = 0; i < n; i++)
  { 
    this->pointcloud[i].x = points[i].x;
    this->pointcloud[i].y = points[i].y;
    this->pointcloud[i].z = points[i].z;
  }
  this->ConstructHull(this->pointcloud);
}

template<typename T> bool ConvexHull::Contains(T p) const
{
  Point3D pt(p.x, p.y, p.z);
  for(auto& face : this->faces)
  {
    if(VolumeSign(face, pt) <= 0) return false;
  }
  return true;
}
#endif