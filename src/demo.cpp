#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include "convexhull.h"

class Master{
  public:
    Master(ros::NodeHandle& nh);

    ~Master() = default;
    
    void VizPolyhedron(const std::list<Face>& triangles) const;

    void VizPoints(const std::vector<Point3D>& points, 
        const ConvexHull& hull) const;

    int Loop();

  private:

    void SetMarker(visualization_msgs::Marker& scan_marker,
        float scale = 1.f) const;

    std::vector<std::vector<double>> tests;

    ros::Publisher convex_pub, point_pub;

    int cnt;
    
    bool random = 1;
};

Master::Master(ros::NodeHandle& nh){
  this->convex_pub = \
      nh.advertise<visualization_msgs::Marker>("convex_hull", 10);
  this->point_pub = \
      nh.advertise<visualization_msgs::Marker>("points", 10);
  this->cnt = 4;
}

int Master::Loop(){
  while(ros::ok())
  { 
    if(this->random)
    {
      double x = (std::rand()%10 - 5) * 1.f;
      double y = (std::rand()%10 - 5) * 1.f;
      double z = (std::rand()%10) * 1.f;
      std::vector<double> tmp = {x, y, z};
      this->tests.push_back(tmp);
    }
    else
    {
      this->tests = { 
          {1,2,1}, {2,1,1}, {1,1,1}, {2,2,1}, {1,2,2}, {2,1,2}, {1,1,2}, {2,2,2},  
          {0,3,0}, {3,0,0}, {0,0,0}, {3,3,0}, {0,3,3}, {3,0,3}, {0,0,3}, {3,3,3}}; 
    }
    std::vector<Point3D> vecs;
    int n = this->cnt >= tests.size() ? tests.size() : this->cnt;
    for(int i =0 ; i < n; i++)
      vecs.emplace_back(tests[i][0], tests[i][1], tests[i][2]);
    ConvexHull c(vecs);
    this->VizPolyhedron(c.GetFaces());
    if(c.Size() >= 4) this->VizPoints(vecs, c);
    this->cnt++;
    ros::Rate loop_rate(10);
    ros::spinOnce();
    loop_rate.sleep();
  }    
}

void Master::SetMarker(\
    visualization_msgs::Marker& scan_marker, float scale) const
{ 
  scan_marker.header.frame_id = "map";
  scan_marker.header.stamp = ros::Time::now();
  scan_marker.ns = "scan";
  scan_marker.action = visualization_msgs::Marker::ADD;
  scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z = scale;
  scan_marker.pose.orientation.x = 0.0;
  scan_marker.pose.orientation.y = 0.0;
  scan_marker.pose.orientation.z = 0.0;
  scan_marker.pose.orientation.w = 1.0;
  scan_marker.pose.position.x = 0.0;
  scan_marker.pose.position.y = 0.0;
  scan_marker.pose.position.z = 0.0;
  std_msgs::ColorRGBA scan_color;
  scan_color.a = 0.5f;
  scan_color.r = 0.f;
  scan_color.g = 0.9f;
  scan_color.b = 0.9f;
  scan_marker.color = scan_color;
}

void Master::VizPolyhedron(const std::list<Face>& triangles) const
{
  visualization_msgs::Marker scan_marker;
  this->SetMarker(scan_marker);
  scan_marker.id = 0;
  scan_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

  geometry_msgs::Point temp;
  for (const auto& triangle : triangles) 
  {
    for(int i = 0; i < 3; i++)
    {
      temp.x = triangle.vertices[i].x; 
      temp.y = triangle.vertices[i].y; 
      temp.z = triangle.vertices[i].z;
      scan_marker.points.push_back(temp);
    }
  }
  this->convex_pub.publish(scan_marker);
}

void Master::VizPoints(const std::vector<Point3D>& points, 
    const ConvexHull& hull) const
{
  visualization_msgs::Marker scan_marker;
  this->SetMarker(scan_marker, 0.1);
  scan_marker.id = 1;
  scan_marker.type = visualization_msgs::Marker::POINTS;

  geometry_msgs::Point temp;
  for (const auto& pt : points) 
  {
    temp.x = pt.x;  temp.y = pt.y;  temp.z = pt.z;
    scan_marker.points.push_back(temp);

    bool inside = hull.Contains(pt);
    std_msgs::ColorRGBA c;
    c.a = 1.0f;
    c.r = inside ? 0.0f : 0.9f;
    c.b = 0.f;
    c.g = inside ? 0.9f : 0.f;
    scan_marker.colors.push_back(c);
  }
  this->point_pub.publish(scan_marker);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh;
  Master master(nh);
  master.Loop();
}