/**
   TNK controller.
   @author Ryod Tanaka
*/
#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <mutex>
#include <vector>
#include <map>

using namespace cnoid;
using namespace std;

class TNKController : public SimpleController
{
    // For ROS
    ros::NodeHandle node_;
    ros::Subscriber twist_sub_;
    ros::Subscriber mode_sub_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber joint_trajectory_sub_;

    mutex twist_mtx_;
    mutex mode_mtx_;
    mutex joint_state_mtx_;
    mutex joint_trajectory_mtx_;

    geometry_msgs::Twist cmd_twist_;
    std_msgs::String cmd_mode_;
    sensor_msgs::JointState cmd_js_;
    trajectory_msgs::JointTrajectory cmd_jt_;

    string name_space;

    // For Simple Control
    vector<string> joint_names_;
    vector<Link*> joints_;
    vector<double> init_pose_;
    map<string, int> joint_map_;

    
public:
    // initialize
    virtual bool initialize(SimpleControllerIO* io) override
        {
            // to avoid "velocity " string.....
            string tmp = io->optionString();
            for(int i=9; i<tmp.size(); i++)
                name_space += tmp[i];

            // This is a manual Joint names
            joint_names_.push_back("WHEEL_L0");
            joint_names_.push_back("WHEEL_R0");
            joint_names_.push_back("MFRAME");
            joint_names_.push_back("BLOCK");
            joint_names_.push_back("BOOM");
            joint_names_.push_back("ARM");
            joint_names_.push_back("TOHKU_PITCH");
            joint_names_.push_back("TOHKU_ROLL");
            joint_names_.push_back("TOHKU_TIP_01");
            joint_names_.push_back("TOHKU_TIP_02");
            joint_names_.push_back("UFRAME");
            joint_names_.push_back("MNP_SWING");
            joint_names_.push_back("MANIBOOM");
            joint_names_.push_back("MANIARM");
            joint_names_.push_back("MANIELBOW");
            joint_names_.push_back("YAWJOINT");
            joint_names_.push_back("HANDBASE");
            joint_names_.push_back("PUSHROD");
        
            // get body info & initialize joint
            Body* body = io->body();
            for(int i=0; i<joint_names_.size(); i++){
                joints_.push_back(body->link(joint_names_[i].c_str()));
                io->enableInput(joints_[i], JOINT_ANGLE | JOINT_VELOCITY);
                joints_[i]->setActuationMode(Link::JOINT_ANGLE);
                io->enableOutput(joints_[i]);
                // io->enableIO(joints_[i]);
                init_pose_.push_back(joints_[i]->q());
                joint_map_[joint_names_[i]] = i;
            }

            return true;
        }
    // when the plugin starts
    virtual bool start() override
        {
            // set initial pose
            for(int i=0; i<joints_.size(); i++)
                joints_[i]->q() = init_pose_[i];

            // ROS subscriber
            twist_sub_ = node_.subscribe(name_space+"/command/velocity", 1,
                                         &TNKController::velocityCallback, this);
            if(!(bool)twist_sub_){
                ROS_ERROR("TNKController : Failed to set velocity subscriber.");
                return false;
            }
            mode_sub_ = node_.subscribe(name_space+"/command/mode", 1,
                                        &TNKController::modeCallback, this);
            if(!(bool)mode_sub_){
                ROS_ERROR("TNKController : Failed to set mode subscriber.");
                return false;
            }
            joint_state_sub_ = node_.subscribe(name_space+"/command/joint_state", 1,
                                               &TNKController::joint_stateCallback, this);
            if(!(bool)joint_state_sub_){
                ROS_ERROR("TNKController : Failed to set joint_state subscriber.");
                return false;
            }
            joint_trajectory_sub_ = node_.subscribe(name_space+"/command/joint_trajectory", 1,
                                                    &TNKController::joint_stateCallback, this);
            if(!(bool)joint_trajectory_sub_){
                ROS_ERROR("TNKController : Failed to set joint_trajectory subscriber.");
                return false;
            }

            return true;

        }
    // when the plugin stops
    virtual void stop() override
        {
            ROS_WARN("TNKController : Stop");
        }

    // control loop
    virtual bool control() override
        {
            return false;

        }
    

    // Callback Functions
    void velocityCallback(const geometry_msgs::Twist& msg)
        {
            ROS_INFO("velocity callback");
            vector<double> cmd_vel = calcCMDVel(msg.linear.x, msg.angular.z);
            joints_[joint_map_["WHEEL_L0"]]->dq() = joints_[joint_map_["WHEEL_L0"]]->dq_upper() * cmd_vel[0];
            joints_[joint_map_["WHEEL_R0"]]->dq() = joints_[joint_map_["WHEEL_R0"]]->dq_upper() * cmd_vel[1];

            ROS_INFO("Left : %lf", joints_[joint_map_["WHEEL_L0"]]->dq());
            ROS_INFO("Right : %lf", joints_[joint_map_["WHEEL_R0"]]->dq());      
        }

    void modeCallback(const std_msgs::String &msg)
        {
            mode_mtx_.lock();
            cmd_mode_ = msg;
            mode_mtx_.unlock();
        }

    void joint_stateCallback(const sensor_msgs::JointState &msg)
        {
            joint_state_mtx_.lock();
            cmd_js_ = msg;
            joint_state_mtx_.unlock();
        }

        
    void joint_trajectoryCallback(const trajectory_msgs::JointTrajectory &msg)
        {
            joint_trajectory_mtx_.lock();
            cmd_jt_ = msg;
            joint_trajectory_mtx_.unlock();

        }

    // Wheel velocity calculation
    vector<double> calcCMDVel(const double linear, const double angular)
        {
            vector<double> result(2,0);
            double tread = 1.2;
            result[0] = linear - (tread/2.0)*angular;
            result[1] = linear + (tread/2.0)*angular;

            return result;
        }    
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TNKController)
