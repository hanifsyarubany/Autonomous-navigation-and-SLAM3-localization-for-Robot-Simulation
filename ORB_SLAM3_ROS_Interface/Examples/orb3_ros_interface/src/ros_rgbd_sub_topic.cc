/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
// #include <sensor_msgs/image_encodings.h>


#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <System.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

cv::Mat g_new_color_image, g_new_depth_image;
void DrawFeature(cv::Mat& im_feature, const cv::Mat im,std::vector<cv::KeyPoint> keypoints, float imageScale, vector<bool> mvbVO,vector<bool> mvbMap);
void PublishPointCloud(vector<Eigen::Matrix<float,3,1>>& global_points, vector<Eigen::Matrix<float,3,1>>& local_points,
ros::Publisher& global_pc_pub, ros::Publisher& local_pc_pub);
void imageCallback(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::ImageConstPtr& depth_image);
bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;

}

int main(int argc, char **argv) {


    // if (argc < 3 || argc > 4) {
    if (argc < 3 ) { 
        cerr << endl
             << "Number of arguments : " << argc << endl
             << argv[0] << endl
             << argv[1] << endl 
             << argv[2] << endl
             << argv[3] << endl
             << argv[4] << endl
             << "End of arguments" <<endl
             << "Usage: ./mono_inertial_realsense_D435i path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }
    // ros

    ros::init(argc, argv,"ros_rgbd_realsense");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

    // ros param setting
    bool enable_pangolin;
    if (!nh_param.getParam("/rgbd_sub_topic/enable_pangolin",enable_pangolin))
    {
        std::cout<<"It has not been decided whether to use Pangolin."<<std::endl
        <<"shut down the program"<<std::endl;
        return 1;
    }
    
    std::string rgb_topic, depth_topic;
    if (!nh_param.getParam("/rgbd_sub_topic/rgb_topic", rgb_topic)) {
        std::cout<<"Failed to read ROS param '/rgbd_sub_topic/rgb_topic\n";
        return 1;
    }

    if (!nh_param.getParam("/rgbd_sub_topic/depth_topic", depth_topic)) {
        std::cout<<"Failed to read ROS param '/rgbd_sub_topic/depth_topic\n";
        return 1;
    }

    std::string pose_topic, odom_topic;
    if (!nh_param.getParam("/rgbd_sub_topic/pose_topic", pose_topic)) {
        std::cout<<"Failed to read ROS param '/rgbd_sub_topic/pose_topic\n";
        return 1;
    }

    if (!nh_param.getParam("/rgbd_sub_topic/odom_topic", odom_topic)) {
        std::cout<<"Failed to read ROS param '/rgbd_sub_topic/odom_topic\n";
        return 1;
    }

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_topic,1);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic,1);
    ros::Publisher global_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/ORB3/globalmap",1);
    ros::Publisher  local_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/ORB3/localmap",1);

    // for image handling
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image         = it.advertise("/camera/color/image_raw_from_orb", 1);
    image_transport::Publisher pub_image_feature = it.advertise("/orb3_feature_image_from_orb", 1);
    image_transport::Publisher pub_depth         = it.advertise("/camera/depth/image_raw_from_orb", 1);
    sensor_msgs::ImagePtr image_msg;
    sensor_msgs::ImagePtr image_feature_msg;
    sensor_msgs::ImagePtr depth_msg;
        
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh,depth_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;


    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    string file_name;
    bool bFileName = false;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
        bFileName = true;
        cout<<"file_name : "<<file_name<<endl;
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    // I will not open pangolin viewer!
    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD, true, 0, file_name);
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD, enable_pangolin, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    double timestamp;
    cv::Mat im, depth, im_feature;

    double t_resize = 0.f;
    double t_track = 0.f;

    Sophus::SE3f output;
    
    // Eigen::Matrix4f current_camera_pose, current_base_pose;
    Eigen::Matrix4f changer;
    /*
    changer << 0.0f, -1.0f,  0.0f,  0.0f,
               0.0f,  0.0f, -1.0f,  0.0f,
               1.0f,  0.0f,  0.0f,  0.0f,
               0.0f,  0.0f,  0.0f,  1.0f; // x 90, and z 90
    */
    changer <<  0.0f,  0.0f,  1.0f,  0.0f,
               -1.0f,  0.0f,  0.0f,  0.0f,
                0.0f, -1.0f,  0.0f,  0.0f,
                0.0f,  0.0f,  0.0f,  1.0f; // x 90, and z 90
    
    // main loop
    int print_index=0;
    geometry_msgs::PoseStamped current_pose;
    nav_msgs::Odometry current_odom;

    vector<Eigen::Matrix<float,3,1>> global_points;
    vector<Eigen::Matrix<float,3,1>> local_points;


    while (!SLAM.isShutDown() && ros::ok())
    {
        
        ros::spinOnce(); // this is to get new image!
        // ros::spinOnce;

        if(imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(depth, depth, cv::Size(width, height));

        }

        // Pass the image to the SLAM system
        // ******************************************************************************
        // output = SLAM.TrackRGBD(im, depth, timestamp); //, vImuMeas); depthCV
        output = SLAM.TrackRGBD(g_new_color_image, g_new_depth_image, timestamp); //, vImuMeas); depthCV
        // ******************************************************************************

        // ROS
        //publish
        // current pose based on camera frame, whose z axis is going foward!
        // current_camera_pose = output.inverse().matrix() * changer;
        Eigen::Matrix4f current_camera_pose = output.inverse().matrix();
        Sophus::SE3f current_base_pose(changer * current_camera_pose);
        Eigen::Quaternionf q(0.5, 0.5, -0.5, 0.5);
        q = current_base_pose.so3().unit_quaternion() * q;



        // Publish pose and topic!
        current_pose.header.stamp = ros::Time::now();
        current_pose.header.frame_id = "odom";
        current_pose.pose.position.x = current_base_pose.translation()(0,0);
        current_pose.pose.position.y = current_base_pose.translation()(1,0);
        current_pose.pose.position.z = current_base_pose.translation()(2,0);
        
        current_pose.pose.orientation.x = q.x();
        current_pose.pose.orientation.y = q.y();
        current_pose.pose.orientation.z = q.z();
        current_pose.pose.orientation.w = q.w();

        current_odom.header.stamp = ros::Time::now();
        current_odom.header.frame_id = "odom";
        current_odom.pose.pose.position.x = current_base_pose.translation()(0,0);
        current_odom.pose.pose.position.y = current_base_pose.translation()(1,0);
        current_odom.pose.pose.position.z = current_base_pose.translation()(2,0);
        
        current_odom.pose.pose.orientation.x = q.x();
        current_odom.pose.pose.orientation.y = q.y();
        current_odom.pose.pose.orientation.z = q.z();
        current_odom.pose.pose.orientation.w = q.w();

        pose_pub.publish(current_pose);
        odom_pub.publish(current_odom);
        
                // Publish image
        
        std::vector<cv::KeyPoint> keypoints = SLAM.GetTrackedKeyPointsUn();
        vector<bool> mvbMap, mvbVO;
        int N = keypoints.size();
        mvbVO = vector<bool>(N,false);
        mvbMap = vector<bool>(N,false);

        SLAM.GetVOandMap(mvbVO,mvbMap);
        DrawFeature(im_feature,g_new_color_image,keypoints,imageScale,mvbVO,mvbMap);

        image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im).toImageMsg();
        image_feature_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im_feature).toImageMsg();
        depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth).toImageMsg();

        // draw features in the image
        // pub_image.publish(image_msg);
        pub_image_feature.publish(image_feature_msg);
        // pub_depth.publish(depth_msg);



        
    
        

        // pub pointcloud
        vector<Eigen::Matrix<float,3,1>> global_points, local_points;
        SLAM.GetPointCloud(global_points,local_points);
        PublishPointCloud(global_points,local_points,global_pc_pub,local_pc_pub);

        
        if (!ros::ok())
        {
            std::cout<<"ros shutdown!"<<std::endl;
            break;
        }
        // show output
        print_index++;
        if(ros::ok() && print_index >  5 )
        {
            // Eigen::Matrix3f rotation_matrix = output.so3().matrix();
            
            // std::cout<<"translation vector : "<< output.translation()

            /*
            inline void getEulerAnglesFromQuaterniondepth_registered(const Eigen::Quaternion<double>& q,
                                         Eigen::Vector3d* euler_angles) {
            {
                assert(euler_angles != NULL);

                *euler_angles << std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                                    1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),

                    std::asin(2.0 * (q.w() * q.y() - q.z() * q.x())),

                    std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
                }
            }
            */
            // rpy[0] = std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
            //                         1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));

            // rpy[1] = std::asin(2.0 * (q.w() * q.y() - q.z() * q.x()));

            // rpy[2] = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
            //             1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));



            std::cout<<"current pose"<<std::endl
            
            <<"x : "<<current_base_pose.translation()(0,0)<<std::endl
            <<"y : "<<current_base_pose.translation()(1,0)<<std::endl
            <<"z : "<<current_base_pose.translation()(2,0)<<std::endl;

            // <<"========================"<<std::endl
            //<<"translation vector : "<< current_camera_pose.translation() <<std::endl
            // << " rotation : "<<rpy[0] <<", "<<rpy[1]<<", "<<rpy[2] << std::endl;
            // << " quaternion(x,y,z,w) : "<<q.x() <<", "<<q.y()<<", "<<q.z() <<", "<<q.w()<< std::endl;
            print_index=0;
        }

        // end of the loop
    }
    cout << "System shutdown!\n";
}

void DrawFeature(cv::Mat& im_feature, const cv::Mat im,std::vector<cv::KeyPoint> keypoints, float imageScale, vector<bool> mvbVO,vector<bool> mvbMap)
{
    // copy IMAGE
    im.copyTo(im_feature);

    cv::Point2f point(100,100);
    // cv::circle(im_feature,point,2,cv::Scalar(0,255,0),-1);   


    
    std::vector<cv::KeyPoint> keypoints_ = keypoints;
    std::vector<bool>         vbVO = mvbVO;
    std::vector<bool>         vbMap = mvbMap;
    const float r = 5;
    int n = keypoints_.size();
    
    for(int i=0;i<n;i++)
    {
        if(vbVO[i] || vbMap[i])
        {
            cv::Point2f pt1,pt2;
            cv::Point2f point;
            
            point = keypoints_[i].pt / imageScale;
            float px = keypoints_[i].pt.x / imageScale;
            float py = keypoints_[i].pt.y / imageScale;
            pt1.x=px-r;
            pt1.y=py-r;
            pt2.x=px+r;
            pt2.y=py+r;
            
            cv::rectangle(im_feature,pt1,pt2,cv::Scalar(0,255,0));
            cv::circle(im_feature,point,2,cv::Scalar(0,255,0),-1);
        }
    }
    
}

void PublishPointCloud(vector<Eigen::Matrix<float,3,1>>& global_points, vector<Eigen::Matrix<float,3,1>>& local_points,
ros::Publisher& global_pc_pub, ros::Publisher& local_pc_pub)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());

    //global
    for(int i=0; i<global_points.size();i++)
    {
        pcl::PointXYZ pt;
        pt.x =  global_points[i](2,0);
        pt.y = -global_points[i](0,0);
        pt.z = -global_points[i](1,0);
        global_pointcloud->points.push_back(pt);
    }

    for(int i=0;i<local_points.size();i++)
    {
        pcl::PointXYZ pt;
        pt.x =  local_points[i](2,0);
        pt.y = -local_points[i](0,0);
        pt.z = -local_points[i](1,0);
        global_pointcloud->points.push_back(pt);
         local_pointcloud->points.push_back(pt); 
    }
    sensor_msgs::PointCloud2 global_map_msg;
    sensor_msgs::PointCloud2 local_map_msg;
    pcl::toROSMsg(*global_pointcloud,global_map_msg);
    pcl::toROSMsg(*local_pointcloud,local_map_msg);
    
    global_map_msg.header.frame_id = "odom";
    global_map_msg.header.stamp = ros::Time::now();
    global_pc_pub.publish(global_map_msg);
    
    local_map_msg.header.frame_id = "odom";
    local_map_msg.header.stamp = ros::Time::now();
    local_pc_pub.publish(local_map_msg);
}
void imageCallback(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::ImageConstPtr& depth_image)
{
    std::cout<<"Image Callback!"<<std::endl;
  // Convert RGB image to cv::Mat format
  cv_bridge::CvImagePtr cv_rgb_ptr;
  try
  {
    cv_rgb_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Convert depth image to cv::Mat format
  cv_bridge::CvImagePtr cv_depth_ptr;
  try
  {
    cv_depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Display RGB and depth images
  g_new_color_image = cv_rgb_ptr->image;
  g_new_depth_image = cv_depth_ptr->image;
}