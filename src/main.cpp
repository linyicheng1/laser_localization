#include "localization/localization.h"
#include "global_localization.h"
#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include "eigen3/Eigen/Core"
#include "unistd.h"
#include "thread"
#include "fstream"
#include "laser_odometry.h"
#include "localization/tools.h"
#include "std_srvs/srv/set_bool.hpp"


using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
    using PointT = pcl::PointXYZI;
public:
    Publisher()
            :Node("my_localization")
    {
        // params
        this->declare_parameter<double>("bbs/max_range", 15.0);
        this->declare_parameter<double>("bbs/min_tx", -50.0);
        this->declare_parameter<double>("bbs/max_tx", 50.0);
        this->declare_parameter<double>("bbs/min_ty", -50.0);
        this->declare_parameter<double>("bbs/max_ty", 50.0);
        this->declare_parameter<double>("bbs/min_theta", -3.14);
        this->declare_parameter<double>("bbs/max_theta", 3.14);
        this->declare_parameter<double>("bbs/map_min_z", -0.2);
        this->declare_parameter<double>("bbs/map_max_z", 1.2);
        this->declare_parameter<int>("bbs/map_width", 512);
        this->declare_parameter<int>("bbs/map_height", 512);
        this->declare_parameter<double>("bbs/map_resolution", 0.5);
        this->declare_parameter<int>("bbs/max_points_pre_cell", 5);
        this->declare_parameter<int>("bbs/map_pyramid_level", 6);
        this->declare_parameter<double>("bbs/scan_min_z", -0.2);
        this->declare_parameter<double>("bbs/scan_max_z", 1.2);
        this->declare_parameter<double>("bbs/map_filter_resolution", 0.5);
        this->declare_parameter<double>("bbs/scan_filter_resolution", 0.5);
        this->declare_parameter<double>("global_map_width", 100.0);
        this->declare_parameter<double>("global_map_height", 100.0);

        this->declare_parameter<double>("ndt/resolution", 0.8);
        this->declare_parameter<double>("ndt/step_size", 0.2);
        this->declare_parameter<double>("ndt/epsilon", 0.01);
        this->declare_parameter<double>("ndt/max_iterations", 30.0);
        this->declare_parameter<double>("ndt/frame_resolution", 0.5);
        this->declare_parameter<double>("odom/kf_distance", 2.0);
        this->declare_parameter<int>("odom/local_map_size", 10);
        this->declare_parameter<double>("ndt/local_map_resolution", 0.5);

        this->declare_parameter<int>("localization/mode", 2);
        this->declare_parameter<std::string>("global_map", "/home/sa/code/robot_ws/src/inspection_3d_robot/laser_localization/map/map.pcd");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_link_frame", "base_link");
        this->declare_parameter<std::string>("laser_frame", "os_sensor");
        this->declare_parameter<std::string>("laser_topic", "/points");
        this->declare_parameter<std::string>("pose_save", "/home/sa/code/robot_ws/src/inspection_3d_robot/laser_localization/map/pos.txt");
        this->declare_parameter<int>("correct_count", 5);
        this->declare_parameter<double>("global_resolution", 1.3);
        this->declare_parameter<double>("global_frame_resolution", 1);
        this->declare_parameter<double>("global_view_resolution", 3);
        this->declare_parameter<double>("local_map_size", 100);

        this->declare_parameter<double>("localization/ndt_resolution", 1);
        this->declare_parameter<double>("localization/ndt_step_size", 0.1);
        this->declare_parameter<double>("localization/ndt_epsilon", 0.01);
        this->declare_parameter<std::string>("initial_pose_save", "/home/sa/code/robot_ws/src/inspection_3d_robot/laser_localization/map/init_pos.txt");

        localization_ = (new laser_localization::localization(this));
        global_localization_ = (new global_localization::GlobalLocalizationBBS(this));
        odometry = new laserOdometry(this);

        // localization mode
        this->get_parameter("localization/mode", location_mode_);
        this->get_parameter("odom_frame", odom_frame);
        this->get_parameter("base_link_frame", base_link_frame);
        this->get_parameter("laser_frame", laser_frame);
        std::string map_path;
        this->get_parameter("global_map", map_path);
        this->get_parameter("correct_count", correct_count);

        // global localization
        pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *map) == -1)
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        }
        global_localization_->set_global_map(map);

        using std::placeholders::_1;
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        // ros2 service call /set_init_pose std_srvs/srv/SetBool "{data: true}"
        initial_service_ = this->create_service<std_srvs::srv::SetBool>(
                "set_init_pose",
                [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response){
                    set_initial_position();
                    response->success = true;
                    response->message = "set initial position !!!";
                });

        // ros2 service call /relocalization std_srvs/srv/SetBool "{data: true}"
        relocalization_service_ = this->create_service<std_srvs::srv::SetBool>(
                "relocalization",
                [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response){
                    manual_relocalization = true;
                    std::string save_txt;
                    this->get_parameter("initial_pose_save", save_txt);
                    // wait for new frames
                    while (correct_frames.empty()){
                        std::cout<<"wait frames "<<std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds (1));
                    }
                    lock_.lock();
                    auto frame = correct_frames.front();
                    Eigen::Matrix4f frame_odom = correct_frames_pose.front();
                    if (location_mode_ == 2){
                        frame_odom = odom_pose.front();
                    }
                    lock_.unlock();

                    std::cout<<"start global localization "<<std::endl;
                    // global localization
                    Eigen::Matrix4f T;
                    if (request->data && global_localization_->globalLocalization(frame, save_txt, T)){
                        localization_->set_inv_odom_base(frame_odom.inverse());
                        localization_->init_localization(T);
                        std::cout<<"global localization success !!!"<<std::endl;
                        localization_state = 1;
                        response->message = "relocalization position success!!!";
                    }else {
                        std::cout<<" re localization again "<<std::endl;
                        if (!global_localization_->globalLocalization(frame, T)){
                            localization_state = 2;
                            response->message = "relocalization position failed!!!";
                        }else{
                            localization_->set_inv_odom_base(frame_odom.inverse());
                            localization_->init_localization(T);
                            std::cout<<"global localization success !!!"<<std::endl;
                            response->message = "relocalization position success!!!";
                            localization_state = 1;
                        }
                    }
                    localization_->update_local_map(T);
                    manual_relocalization = false;
                });

        if(location_mode_ == 3){// fake localization
            subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
                    "cmd_vel", 10, std::bind(&Publisher::cmd_callback, this, _1));
        }else if (location_mode_ == 1 || location_mode_ == 2){
            int try_cnt = 0;
            while (rclcpp::ok()){
                try{
                    auto pos = buffer_->lookupTransform(
                            base_link_frame,
                            laser_frame, tf2::TimePointZero);
                    laser_base_ = transform2Matrix(pos);
                    break;
                }
                catch (tf2::TransformException &ex){
                    RCLCPP_INFO(this->get_logger(),"Could not transform %s",
                                ex.what());
                }
                sleep(1);
                try_cnt ++;
                if (try_cnt > 10) {
                    laser_base_ = Eigen::Matrix4f::Identity();
                    break;
                }
            }
            std::string topic;
            this->get_parameter("laser_topic", topic);
            laser_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic, rclcpp::SensorDataQoS(), std::bind(&Publisher::laser_callback, this, _1));

            if (location_mode_ == 2){
                while (rclcpp::ok())
                {
                    try{
                        auto pos = buffer_->lookupTransform(
                                odom_frame,
                                base_link_frame,
                                tf2::TimePointZero);
                        last_trans_ = transform2Matrix(pos);
                        break;
                    }
                    catch (tf2::TransformException &ex){
                        RCLCPP_INFO(this->get_logger(),"Could not transform %s",
                                    ex.what());
                    }
                    sleep(1);
                    odom_ready = true;
                }
            }
        }
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", 10);
        aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned", 10);

        // 5 hz
        timer_ = this->create_wall_timer(100ms, [this] { timer_callback();});
        // start thread
        t1 = std::make_shared<std::thread>(&Publisher::laser_thread, this);
    }
private:

    void laser_thread()
    {
        Eigen::Matrix4f last_pos = Eigen::Matrix4f::Identity();
        std::string save_txt;
        this->get_parameter("pose_save", save_txt);
        while (true){
//            if (!laser_ready)
//                continue;
//            if (location_mode_ == 2 && !odom_ready)
//                continue;
            if(!correct_frames.empty() && !manual_relocalization){
                lock_.lock();
                auto frame = correct_frames.front();
                Eigen::Matrix4f frame_odom = correct_frames_pose.front();
                Eigen::Matrix4f odom;
                if (location_mode_ == 2){odom = odom_pose.front();}
                lock_.unlock();

                if (localization_state == 0){
                    std::cout<<"start global localization "<<std::endl;
                    // global localization
                    Eigen::Matrix4f T;
                    if (global_localization_->globalLocalization(frame, save_txt, T)){
                        if (location_mode_ == 2){
                            localization_->set_inv_odom_base(odom.inverse());
                        }else {
                            localization_->set_inv_odom_base(frame_odom.inverse());
                        }
                        localization_->init_localization(T);
                        last_pos = frame_odom;
                        std::cout<<"global localization success !!!"<<std::endl;
                        localization_state = 1;
                    }else {
                        std::cout<<" re localization again "<<std::endl;
                        if (!global_localization_->globalLocalization(frame, T)){
                            localization_state = 2;
                        }else{
                            if (location_mode_ == 2){
                                localization_->set_inv_odom_base(odom.inverse());
                            }else {
                                localization_->set_inv_odom_base(frame_odom.inverse());
                            }
                            localization_->init_localization(T);
                            last_pos = frame_odom;
                            std::cout<<"global localization success !!!"<<std::endl;
                            localization_state = 1;
                        }
                    }
                    localization_->update_local_map(T);
                }
                if (localization_state == 1){

                    localization_->update_local_map(localization_->get_estimate_pos());
                    auto s = std::chrono::system_clock::now();
                    // predict
                    Eigen::Matrix4f delta = last_pos.inverse() * frame_odom;
                    localization_->predict(delta);
                    last_pos = frame_odom;
                    // set odom
                    if (location_mode_ == 2){
                        localization_->set_inv_odom_base(odom.inverse());
                    }else {
                        localization_->set_inv_odom_base(frame_odom.inverse());
                    }
                    // correct
                    if (!localization_->correct(frame, 30)){
                        localization_state = 2;
                    }
                    auto e = std::chrono::system_clock::now();
                    std::cout<<"cost "<<std::chrono::duration_cast<std::chrono::milliseconds>(e-s).count()<<" ms"<<std::endl;
                }else {
                    std::cout<<" try global localization"<<std::endl;
                    Eigen::Matrix4f predict = localization_->get_estimate_pos();
                    Eigen::Matrix4f T;
                    if (global_localization_->globalLocalization(frame, predict, T)){
                        if (location_mode_ == 2){
                            localization_->set_inv_odom_base(odom.inverse());
                        }else {
                            localization_->set_inv_odom_base(frame_odom.inverse());
                        }
                        localization_->init_localization(T);
                        last_pos = frame_odom;
                        std::cout<<"global localization success !!!"<<std::endl;
                        localization_state = 1;
                    }else {
                        std::cout<<" re localization again "<<std::endl;
                        if (!global_localization_->globalLocalization(frame, T)){
                            localization_state = 2;
                        }else{
                            if (location_mode_ == 2){
                                localization_->set_inv_odom_base(odom.inverse());
                            }else {
                                localization_->set_inv_odom_base(frame_odom.inverse());
                            }
                            localization_->init_localization(T);
                            last_pos = frame_odom;
                            std::cout<<"global localization success !!!"<<std::endl;
                            localization_state = 1;
                        }
                    }
                }
                write_position(save_txt, localization_->get_estimate_pos());
                lock_.lock();
                correct_frames_pose.pop();
                correct_frames.pop();
                if (location_mode_ == 2){odom_pose.pop();}
                lock_.unlock();
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }


    void timer_callback()
    {
        static int cnt = 0;
        if(cnt%100 == 0)
        {// publish map
            sensor_msgs::msg::PointCloud2 map;
            pcl::toROSMsg(*localization_->get_view_map(), map);
            map.header.stamp = this->get_clock()->now();
            map.header.frame_id = "map";
            map_pub_->publish(map);
        }
        if(cnt%3 == 0 && localization_->get_aligned() != nullptr)
        {
            sensor_msgs::msg::PointCloud2 map;
            if(localization_->get_aligned().get()->points.size() == localization_->get_aligned().get()->width * localization_->get_aligned().get()->height)
            {
                pcl::toROSMsg(*localization_->get_aligned(), map);
                map.header.stamp = this->get_clock()->now();
                map.header.frame_id = "map";
                aligned_pub_->publish(map);
            }
        }

        cnt ++;
    }


    void publish_pos(geometry_msgs::msg::Transform pos)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "odom";
        t.transform = pos;
        if (localization_state != 1){
            t.transform.translation.z = 4;
        }
        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        static auto last_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        float dt = (float)std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
        static auto Zero = localization_->get_pos();
        localization_->update_pos(msg, dt / 1000000.f);
        last_time = now;

        // 发布定位信息
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";
            t.transform = localization_->get_pos();
            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        }
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id = "odom";
            t.transform = Zero;
            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        }
    }

    Eigen::Matrix4f transform2Matrix(const geometry_msgs::msg::TransformStamped& trans)
    {
        Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
        Eigen::Quaternionf q;

        M(0, 3) = (float)trans.transform.translation.x;
        M(1, 3) = (float)trans.transform.translation.y;
        M(2, 3) = (float)trans.transform.translation.z;
        q.x() = (float)trans.transform.rotation.x;
        q.y() = (float)trans.transform.rotation.y;
        q.z() = (float)trans.transform.rotation.z;
        q.w() = (float)trans.transform.rotation.w;
        M.block<3,3>(0,0) = q.toRotationMatrix();

        return M;
    }


    // laser odom
    void laser_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!laser_ready)
            laser_ready = true;
//        std::cout<<"get laser points "<<std::endl;
        auto s = std::chrono::system_clock::now();
        static int cnt = correct_count;

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        // to pcl points
        pcl::fromROSMsg(*msg, *pcl_cloud);
        if(pcl_cloud->empty()) {
            std::cerr<<"cloud is empty"<<std::endl;
            return;
        }
        //std::cout<<"size: "<<filter->points.size()<<std::endl;
        // tf to base frame
        pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, laser_base_);
        Eigen::Matrix4f current_pos;
        // normal localization
        //if ((localization_state == 0 && correct_frames.empty()) || localization_state == 1)
        {
            Eigen::Matrix4f odom;
            if (location_mode_ == 1) {
                odom = odometry->addFrame(pcl_cloud);
                odom = odom * laser_base_;
                // 发布定位信息
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = odom_frame;
                t.child_frame_id = base_link_frame;
                geometry_msgs::msg::Transform transform;
                Eigen::Quaternionf q1(odom.block<3,3>(0,0));
                transform.translation.x = odom(0, 3);
                transform.translation.y = odom(1, 3);
                transform.translation.z = odom(2, 3);
                transform.rotation.x = q1.x();
                transform.rotation.y = q1.y();
                transform.rotation.z = q1.z();
                transform.rotation.w = q1.w();
                t.transform = transform;
                // Send the transformation
                tf_broadcaster_->sendTransform(t);
            }else if (location_mode_ == 2){
                // with odometry
                // 1. get last pos --> current pos
                geometry_msgs::msg::TransformStamped trans;
                while(!buffer_->canTransform(
                        odom_frame,
                        base_link_frame, tf2::TimePointZero));//tf2::TimePoint(std::chrono::nanoseconds(msg->header.stamp.nanosec) + std::chrono::seconds(msg->header.stamp.sec))));
                try{
                    trans = buffer_->lookupTransform(
                            odom_frame,
                            base_link_frame, tf2::TimePointZero);//tf2::TimePoint(std::chrono::nanoseconds(msg->header.stamp.nanosec) + std::chrono::seconds(msg->header.stamp.sec)));
                    current_pos = transform2Matrix(trans);
                }
                catch (tf2::TransformException &ex){
                    RCLCPP_INFO(this->get_logger(),"Could not transform %s",
                                ex.what());
                }
                bool accept = false;
//                std::cout<<"current odom pose: "<<current_pos<<std::endl;
                Eigen::Matrix4f delta_pos = last_trans_.inverse() * current_pos;
                odom = odometry->addFrame(pcl_cloud, delta_pos, accept);
//                std::cout<<"delta_pos "<<delta_pos<<"odom"<<odom<<std::endl;
                last_trans_ = current_pos;
            }

            if (cnt > correct_count && correct_frames.size() < 2){
                lock_.lock();
                correct_frames.push(pcl_cloud);
                correct_frames_pose.push(odom);
                if (location_mode_ == 2){
                    odom_pose.emplace(current_pos);
                }
                lock_.unlock();
                cnt = 0;
            }
            cnt ++;
        }
        auto e = std::chrono::system_clock::now();
        //std::cout<<"frame time "<<(float)std::chrono::duration_cast<std::chrono::milliseconds>(e - s).count()/1000.f<<" s"<<std::endl;

        // publish map --> odom
        publish_pos(localization_->get_pos());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void set_initial_position(){
        std::string save_txt;
        this->get_parameter("initial_pose_save", save_txt);
        write_position(save_txt, localization_->get_estimate_pos());
    }

    std::string odom_frame = "odom";
    std::string base_link_frame = "base_link";
    std::string laser_frame = "os_sensor";

    Eigen::Matrix4f last_trans_;
    Eigen::Matrix4f laser_base_;
    global_localization::GlobalLocalizationBBS* global_localization_;
    laserOdometry* odometry;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    // 订阅cmd,用于测试
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    // 订阅激光雷达数据
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_sub_;

    // 订阅里程计数据
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // 获取激光雷达与里程计的相对位置关系
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
    laser_localization::localization* localization_;

    std::queue<pcl::PointCloud<PointT>::Ptr> correct_frames;
    std::queue<Eigen::Matrix4f> correct_frames_pose;
    std::queue<Eigen::Matrix4f> odom_pose;
    std::shared_ptr<std::thread> t1;
    std::mutex lock_;
    // 用于调试模式，利用cmd_vel指令推测当前位置，用于测试路径规划算法
    int location_mode_ = 1;

    // 0 wait initial
    // 1 normal
    // 2 error
    int localization_state = 0;
    int correct_count = 10;

    bool manual_relocalization = false;

    // service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr initial_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr relocalization_service_;

    bool odom_ready = false;
    bool laser_ready = false;
};

// read msg then publish to ros2
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}





