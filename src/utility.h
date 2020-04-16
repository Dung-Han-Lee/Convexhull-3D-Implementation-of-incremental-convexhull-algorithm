#include <vector>

struct Point3D {
  float x;
  float y;
  float z;
  float intensity;
  bool processed;
  Point3D() = default;
  Point3D(float _x, float _y, float _z):\
      x(_x), y(_y), z(_z), intensity(0), processed(false) {}
  Point3D(float _x, float _y, float _z, float _i):\
      x(_x), y(_y), z(_z), intensity(_i), processed(false) {}

  bool operator ==(const Point3D& pt) const
  {
      return x == pt.x && y == pt.y && z == pt.z;
  }

  float operator *(const Point3D& pt) const
  {
      return x * pt.x + y * pt.y + z * pt.z;
  }

  Point3D operator *(const float factor) const
  {
      return Point3D(x*factor, y*factor, z*factor);
  }

  Point3D operator /(const float factor) const
  {
      return Point3D(x/factor, y/factor, z/factor);
  }

  Point3D operator +(const Point3D& pt) const
  {
      return Point3D(x+pt.x, y+pt.y, z+pt.z);
  }
  Point3D operator -(const Point3D& pt) const
  {
      return Point3D(x-pt.x, y-pt.y, z-pt.z);
  }

  std::string ToString() const
  {
    return std::to_string(x).substr(0,4) + ", " + 
           std::to_string(y).substr(0,4) + ", " + 
           std::to_string(z).substr(0,4) + ", " + 
           std::to_string(intensity).substr(0,4);
  };
  
  friend std::ostream& operator<<(std::ostream& os, const Point3D& p)
  {
    os << "["<< p.ToString() << "] ";
    return os;
  }
  
};

typedef std::vector<Point3D> PointStack;

struct point_hash
{
  std::size_t operator() (const Point3D& p) const
  {
      std::string sx, sy, sz;
      sx = std::to_string(p.x);
      sy = std::to_string(p.y);
      sz = std::to_string(p.z);
      return std::hash<std::string>{}(sx+sy+sz);
  }
};
