#include "convexhull.h"
#include <vector>

int main()
{
  std::vector<Point3D> tests = \
    {{3,0,0}, {0,3,0}, {0,0,3}, {3,3,3}};
  ConvexHull C(tests);
  std::cout<<"vertices (x, y, z, intensity)\n"; C.Print("vertice");
  std::cout<<"faces\n";  C.Print("face");

  return 0;
}