
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

class DataGrabber
{
public:
    DataGrabber(ros::NodeHandle &nh):nh_(nh)
    {
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/RobotAtHome/Scan", 1, true);
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/RobotAtHome/PointCloud", 1, true);
    }

    void GrabData(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD, const sensor_msgs::LaserScanConstPtr &msgScan);
    void PublishTF();
    sensor_msgs::PointCloud2 GeneratePointCloud(const cv::Mat &im, const cv::Mat &imD);
    sensor_msgs::LaserScan GenerateScanMsg(const sensor_msgs::LaserScanConstPtr &msgScan);
private:
    ros::NodeHandle nh_;
    ros::Publisher scan_pub_;
    ros::Publisher pointcloud_pub_;

public:
    static long nData;
};
long DataGrabber::nData = 0;

void DataGrabber::GrabData(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD, const sensor_msgs::LaserScanConstPtr &msgScan)
{
    ros::Duration tstamp_diff(msgRGB->header.stamp - msgScan->header.stamp);
    double time_difference = abs(tstamp_diff.toSec());
    if (time_difference > 0.1)
    {
        return;
    }
    DataGrabber::nData++;
    std::cout << "[ROS mixed] New Set Get! n:" << nData << " Time difference:" << time_difference << std::endl;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::cout << "Generate CLOUD topic" << std::endl;
    sensor_msgs::PointCloud2 RGBDcloud = GeneratePointCloud(cv_ptrRGB->image, cv_ptrD->image);
    std::cout << "Generate SCAN topic" << std::endl;
    sensor_msgs::LaserScan SCAN = GenerateScanMsg(msgScan);
    PublishTF();
    pointcloud_pub_.publish(RGBDcloud);
    scan_pub_.publish(SCAN);

    std::cout << std::endl;
}

void DataGrabber::PublishTF()
{
    tf2::Matrix3x3 tf_laser_rotation(1, 0, 0,
                                     0, 1, 0,
                                     0, 0, 1);
    tf2::Vector3 tf_laser_translation(0.205, 0, 0.31);
    tf2::Transform tf_laser_to_base(tf_laser_rotation, tf_laser_translation);

    tf2::Matrix3x3 tf_camera_rotation( 0,  0, 1,
                                      -1,  0, 0,
                                       0, -1, 0);
    tf2::Vector3 tf_camera_translation(0.285, 0, 1.045);
    tf2::Transform tf_camera_to_base(tf_camera_rotation, tf_camera_translation);

    static tf2_ros::TransformBroadcaster tf_broadcaster;

    // Make LiDAR message
    tf2::Stamped<tf2::Transform> tf_map2laser_stamped;
    tf_map2laser_stamped = tf2::Stamped<tf2::Transform>(tf_laser_to_base, ros::Time::now(), "RobotAtHome_Base");
    geometry_msgs::TransformStamped msg_lidar = tf2::toMsg(tf_map2laser_stamped);
    msg_lidar.child_frame_id = "RobotAtHome_Laser";
    tf_broadcaster.sendTransform(msg_lidar);

    // Make Camera message
    tf2::Stamped<tf2::Transform> tf_map2camera_stamped;
    tf_map2camera_stamped = tf2::Stamped<tf2::Transform>(tf_camera_to_base, ros::Time::now(), "RobotAtHome_Base");
    geometry_msgs::TransformStamped msg_camera = tf2::toMsg(tf_map2camera_stamped);
    msg_camera.child_frame_id = "RobotAtHome_Camera";
    tf_broadcaster.sendTransform(msg_camera);
}

sensor_msgs::PointCloud2 DataGrabber::GeneratePointCloud(const cv::Mat &im, const cv::Mat &imD)
{
    cv::Mat depthmap = imD;
    // Data Preprocess
    float mDepthMapFactor = 6553.5;
    if(fabs(mDepthMapFactor)<1e-5)
        mDepthMapFactor=1;
    else
        mDepthMapFactor = 1.0f/mDepthMapFactor;
    if((fabs(mDepthMapFactor-1.0f)>1e-5) || depthmap.type()!=CV_32F)
    depthmap.convertTo(depthmap,CV_32F,mDepthMapFactor);

    // Count Valid Data
    int num_data = 0;
    for(int i=0; i<depthmap.rows; i++)
    {
        for(int j=0; j<depthmap.cols; j++)
        {
            float z = depthmap.at<float>(i,j);
            if(z>0)
            {
                num_data++;
            }
        }
    }

    // Intrinsic
    float fx = 271.36999;
    float invfx = 1/fx;
    float fy = 286.441384;
    float invfy = 1/fy;
    float cx = 120.0802295;
    float cy = 157.3245865;

    // Generate Point Cloud
    sensor_msgs::PointCloud2 cloud;

    const int num_channels = 3; // x y z
    const int min_observations_per_point_ = 2;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "RobotAtHome_Camera";
    cloud.height = 1;
    cloud.width = num_data;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[num_channels];

    int idx = 0;
    for(int i=0; i<depthmap.rows; i++)
    {
        for(int j=0; j<depthmap.cols; j++)
        {
            float z = depthmap.at<float>(i,j);
            if(z>0)
            {
                const float u = j;
                const float v = i;
                const float x = (u-cx)*z*invfx;
                const float y = (v-cy)*z*invfy;
                data_array[0] = x;
                data_array[1] = y;
                data_array[2] = z;
                memcpy(cloud_data_ptr + (idx * cloud.point_step), data_array, num_channels * sizeof(float));
                idx++;
            }
        }
    }
    return cloud;
}

sensor_msgs::LaserScan DataGrabber::GenerateScanMsg(const sensor_msgs::LaserScanConstPtr &msgScan)
{
    sensor_msgs::LaserScan SCAN;
    SCAN.header.frame_id = string("RobotAtHome_Laser");
    SCAN.header.stamp = ros::Time::now();
    SCAN.angle_min = msgScan->angle_min;
    SCAN.angle_max = msgScan->angle_max;
    SCAN.angle_increment = msgScan->angle_increment;
    SCAN.time_increment = msgScan->time_increment;
    SCAN.scan_time = msgScan->scan_time;
    SCAN.range_min = msgScan->range_min;
    SCAN.range_max = msgScan->range_max;
    SCAN.ranges = msgScan->ranges;
    return SCAN;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RobotAtHome_ROS_Viewer");
    ros::start();

    string strLaserTopic = "/scan";
    string strRGBTopic = "/camera/rgb";
    string strDepthTopic = "/camera/depth";

    cout << "[RobotAtHome ROS Viewer] Listen to topic " << strLaserTopic << endl;
    cout << "[RobotAtHome ROS Viewer] Listen to topic " << strRGBTopic << endl;
    cout << "[RobotAtHome ROS Viewer] Listen to topic " << strDepthTopic << endl;
    
    ros::NodeHandle nh;

    DataGrabber igb(nh);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, strRGBTopic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, strDepthTopic, 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, strLaserTopic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::LaserScan> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(1), rgb_sub, depth_sub, scan_sub);
    sync.registerCallback(boost::bind(&DataGrabber::GrabData, &igb, _1, _2, _3));

    ros::spin();

    ros::shutdown();

    return 0;
}
