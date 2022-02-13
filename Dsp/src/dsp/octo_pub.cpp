#include<ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
int main(int argc, char **argv)
{
  ros::init (argc, argv, "octomap_publisher");
  ros::NodeHandle n;
  ros::Publisher octomap_pub = n.advertise<octomap_msgs::Octomap>("/octomap_full", 1);
  ros::Rate loop_rate(10);
  octomap_msgs::Octomap octomap;
  octomap::OcTree myOctomap("/home/yifan/Downloads/obstacle_room.bt");
  while (ros::ok())
  {
    octomap_msgs::binaryMapToMsg(myOctomap, octomap);
    octomap_pub.publish(octomap);
    loop_rate.sleep();
   }

}
