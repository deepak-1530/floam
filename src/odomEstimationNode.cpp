// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <math.h>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimationClass.h"

#include<pcl/io/pcd_io.h>
#include<pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_types.h>
#include<pcl/PCLPointCloud2.h>

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry;
ros::Publisher pubLaserPose;

sensor_msgs::NavSatFix carGps;
Eigen::Vector3d mapGps;
Eigen::Vector3d mapXYZ;
float mapYaw;

float carYaw;

// publish the perscan computation time on a topic to store it for visualization
ros::Publisher computeTime;
bool is_odom_inited = false;
bool gotFirstPoint = false;

pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr initScan (new pcl::PointCloud<pcl::PointXYZ>);

pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

const double  _earth_a=6378137.00000;   // [m] WGS84 equator radius
const double  _earth_b=6356752.31414;   // [m] WGS84 epolar radius
const double  _earth_e=8.1819190842622e-2; //  WGS84 eccentricity
const double _aa=_earth_a*_earth_a;
const double _ee=_earth_e*_earth_e;

bool gotGps = false;
bool relocalize = false;
bool gps_initialization = false;

Eigen::Vector3d gpsXYZ(double lat, double lon, double alt)
{

    double r = 6371000 + alt;
    double x = r*cos(lat*3.14/180)*cos(lon*3.14/180);
    double y = r*cos(lat*3.14/180.0)*sin(lon*3.14/180.0);
    double z = r*sin(lat*3.14/180.0);
    Eigen::Vector3d pose;
    pose(0) = x;
    pose(1) = y;
    pose(2) = z;
    return pose;
}

Eigen::Vector3d gpsToXYZ(double lat, double lon, double alt)
{
    double  a,b,x,y,z,h,l,c,s;

    a=lon;
    b=lat;
    h=alt;
    c=cos(b);
    s=sin(b);
    // WGS84 from eccentricity
    l=_earth_a/sqrt(1.0-(_ee*s*s));
    x=(l+h)*c*cos(a);
    y=(l+h)*c*sin(a);
    z=(((1.0-_ee)*l)+h)*s;

    Eigen::Vector3d coordinate;

    coordinate(0) = x;
    coordinate(1) = y;
    coordinate(2) = z;

    //std::cout<<"Car Coordinates "<<coordinate.transpose()<<std::endl;

    return coordinate;
}

double getDistance(double lat1, double lon1, double lat2, double lon2)
{
    double earthRadius = 6371000;
    double dLat = (lat2-lat1)*3.14/180.0;
    double dLon = (lon2-lon1)*3.14/180.0;
    
    lat1 = lat1*3.14/180.0;
    lat2 = lat2*3.14/180.0;
    lon1 = lon1*3.14/180.0;

    double a = sin(dLat/2) * sin(dLat/2) +
          sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
  double c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  return abs(earthRadius * c);
}

void compassHandler(std_msgs::Float64 msg)
{
    carYaw = msg.data;
}

void gpsHandler(sensor_msgs::NavSatFix msg)
{
    carGps = msg;
    gotGps=true;
}

void velodyneHandler(sensor_msgs::PointCloud2 msg)
{
    if(!is_odom_inited)
    {

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *initScan);
        gotFirstPoint=true;

    }
}

void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

bool initialize = false;

double total_time =0;
int total_frame=0;

void odom_estimation(){

    while(1){

        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            //if time aligned 
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            // perform initialization here
            if(is_odom_inited == false)
            {
                /* If performing localization on pre-made maps */
                if(relocalize)
                {
                    double x_           = 0.0;
                    double y_           = 0.0;
                    double angleDiff    = 0.0;
                    double currDistance = 0.0;

                    while(!gotFirstPoint)
                    {
                        ROS_INFO("Waiting for first point-cloud. ");
                        ros::spinOnce();
                        continue;
                    }

                    ndt.setInputTarget (initScan);
                    
                    /* If using GPS for initialization with NDT */
                    if (gps_initialization)
                    {
                        Eigen::Vector3d carGnss;

                        carGnss(0) = double(carGps.latitude);
                        carGnss(1) = double(carGps.longitude);
                        carGnss(2) = double(carGps.altitude);

                        while(carGnss(0) + carGnss(1) == 0)
                        {
                            std::cout<<"GPS not yet updated "<<carGnss.transpose()<<std::endl;
                            ros::spinOnce();
                            carGnss(0) = double(carGps.latitude);
                            carGnss(1) = double(carGps.longitude);
                            carGnss(2) = double(carGps.altitude);
                        }

                        /** Distance and angle of current gps coordinates from those of map's origin **/
                        currDistance = getDistance(mapGps(0), mapGps(1), carGnss(0), carGnss(1));
                        angleDiff = carYaw - mapYaw;
                        Eigen::Vector3d carXYZ = gpsXYZ(carGnss(0), carGnss(1), carGnss(2));
                        Eigen::Vector3d mapXYZ = gpsXYZ(mapGps(0), mapGps(1), mapGps(2));
                        Eigen::Vector3d diffPose = carXYZ - mapXYZ;
                        std::cout<<"Distance from X,Y,Z is "<<(carXYZ - mapXYZ).transpose()<<std::endl;
                        
                        angleDiff = atan2(diffPose(1), diffPose(0))*180/3.14;
                       // double dLon = (carGnss(1) - mapGps(1))*3.14/180.0;
                       // double y__  = sin(dLon)*cos(carGnss(0)*3.14/180.0);
                       // double x__  = cos(mapGps(0)*3.14/180.0)*sin(carGnss(0)*3.14/180.0) - sin(mapGps(0)*3.14/180.0)*cos(carGnss(0)*3.14/180.0)*cos(dLon);
                       // angleDiff   = atan2(y__, x__)*180.0/3.14;
                      //  angleDiff = (360 - ((angleDiff + 360.0) % 360.0));
                    
                        if(angleDiff > 180)
                        {
                            angleDiff -= 360.0;
                        }
                    
                        if(angleDiff < -180)
                        {
                            angleDiff += 360.0;
                        }
                        
                        angleDiff *= 3.14/180.0;

                        std::cout<<"Angle Difference between "<<carYaw<<" and "<<mapYaw<<" is "<<angleDiff*180.0/3.14 + carYaw-mapYaw<<std::endl;
                        std::cout<<"Distance is "<<currDistance<<std::endl;

                        //angleDiff = 0;
                        x_ = currDistance*cos(angleDiff);
                        y_ = currDistance*sin(angleDiff);
                        //x_ = diffPose(0);
                        //y_ = diffPose(1);
                    }
                
                    Eigen::AngleAxisf init_rotation (angleDiff, Eigen::Vector3f::UnitZ ());
                    Eigen::Translation3f init_translation (x_, y_, 0);
                    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

                    pcl::PointCloud<pcl::PointXYZ> Final;
                
                    ndt.align(Final, init_guess);

                    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()<< " score: " << ndt.getFitnessScore() << std::endl;

                    auto T = ndt.getFinalTransformation();
                
                    Eigen::Transform<float, 3, Eigen::Affine> Tf(T);
                   // Eigen::Transform<double, 3, Eigen::Isometry3d> Td(T);
//                    Eigen::MatrixXd  Td = Tf.cast<double>();
                    
                    float x, y, z, roll, pitch, yaw;
                    
                    pcl::getTranslationAndEulerAngles(Tf, x, y, z, roll, pitch, yaw);

                    std::cout<<"Initial coordinates passed to NDT "<<x_<<" "<<y_<<std::endl;
                    std::cout<<"Final coordinates after NDT X, Y, Z ["<<x<<", "<<y<<", "<<z<<"]"<<std::endl;
                    
                    std::cout<<"Roll, pitch, yaw ["<<roll*180.0/3.14<<", "<<pitch*180/3.14<<", "<<yaw*180.0/3.14<<"]"<<std::endl;

                    odomEstimation.odom.translation().x() = x;
                    odomEstimation.odom.translation().y() = y;
                    odomEstimation.odom.translation().z() = z;

                    odomEstimation.last_odom.translation().x() = x;
                    odomEstimation.last_odom.translation().y() = y;
                    odomEstimation.last_odom.translation().z() = z;
                    
                    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
                    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());

                    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
                    q.normalize();
                    Eigen::Matrix3d rotationMatrix = q.matrix();

                    odomEstimation.odom.linear() = rotationMatrix;
                    odomEstimation.last_odom.linear() = rotationMatrix;

                   // std::cout<<"Odometry Matrix "<<odomEstimation.odom<<std::endl;

                    //odomEstimation.odom.translation() = Tf.translation();
                    //odomEstimation.odom.rotation()    = Tf.rotation();
                }    

                    odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                    is_odom_inited = true;
                    ROS_INFO("odom inited");
            }

            else
            {
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();

                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                ROS_INFO("average odom estimation time %f and frame count is %f ms \n \n", total_time/total_frame, total_frame);
              
            }

            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            //q_current.normalize();
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "base_link";
            laserOdometry.header.stamp = ros::Time::now();//pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            pubLaserOdometry.publish(laserOdometry);

            geometry_msgs::Pose carCoord;
            carCoord.position.x = t_current.x();
            carCoord.position.y = t_current.y();
            carCoord.position.z = t_current.z();
            carCoord.orientation.x = q_current.x();
            carCoord.orientation.y = q_current.y();
            carCoord.orientation.z = q_current.z();
            carCoord.orientation.w = q_current.w();
            pubLaserPose.publish(carCoord);

        }
        
        //sleep 2 ms every time
        std::chrono::milliseconds dura(1);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    double ndt_transform_epsilon = 0;
    double ndt_step_size = 0.0;
    double ndt_resolution = 0.0;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);
    nh.getParam("/relocalization", relocalize);
    nh.getParam("/gps_initialization", gps_initialization);
    nh.getParam("/ndt_transform_epsilon", ndt_transform_epsilon);
    nh.getParam("/ndt_step_size", ndt_step_size);
    nh.getParam("/ndt_resolution", ndt_resolution);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    ndt.setTransformationEpsilon (0.01);
    ndt.setStepSize (0.1);
    ndt.setResolution (1.0);

    ndt.setMaximumIterations (100);
    
    odomEstimation.init(lidar_param, map_resolution);

    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 1, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 1, velodyneSurfHandler);

    ros::Subscriber velodynePoints    = nh.subscribe<sensor_msgs::PointCloud2>("/transformed_points",1, velodyneHandler);

    ros::Subscriber gnss              = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global",1, gpsHandler);
    ros::Subscriber compass           = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg",1, compassHandler);

    // store the map here
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/deepak/IIITD/catkin_ws/src/floam/maps/floamMap_Test_12Nov_bag.pcd", *map);
    ndt.setInputSource (map);

    mapGps(0) = 28.5487534;
    mapGps(1) = 77.2740308;
    mapGps(2) = 152.927592849;

    mapXYZ = gpsToXYZ(mapGps(0), mapGps(1), mapGps(2));
    mapYaw = 235.58;

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pubLaserPose     = nh.advertise<geometry_msgs::Pose>("/lidarPose",1);
    computeTime      = nh.advertise<std_msgs::Float64>("/computeTime", 1);

    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}

