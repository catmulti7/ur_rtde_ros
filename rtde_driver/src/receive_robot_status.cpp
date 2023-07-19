//
// Created by catmulti7 on 14/07/23.
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/string.hpp"
#include <boost/lockfree/queue.hpp>
#include <queue>
#include <algorithm>

//sensor_msgs/msg/JointState

#include <thread>
#include <chrono>
#include <memory>

using namespace ur_rtde;
using namespace std::chrono;
using std::cout, std::endl;
using std::placeholders::_1;
using std::vector, std::queue, std::string;


sensor_msgs::msg::JointState joint_state;

template<typename T>
class sharedObj
{
private:
    std::mutex mtx;
    T obj;

public:
    void operator()(T& other)
    {
        mtx.lock();
        obj.swap(other);
        mtx.unlock();
    }
};

class RTDEDriver : public rclcpp::Node
{
public:
    std::unique_ptr<RTDEControlInterface> rtde_control;
    std::unique_ptr<RTDEReceiveInterface> rtde_receive;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_status_pub;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub;
    std::thread* control_thread;
    vector<vector<double>> mid_traj;
    vector<vector<double>> trajectory;
    sharedObj<queue<vector<double>>> trajectory_share;
    queue<vector<double>> traj;

    double velocity = 0.5;
    double acceleration = 0.5;
    double dt = 1.0/500; // 2ms
    double lookahead_time = 0.1;
    double gain = 300;


    std::atomic<bool> new_traj;
    std::mutex traj_lock;
    

    vector<string> name = {"shoulder_pan_joint",
                            "shoulder_lift_joint", 
                            "elbow_joint", 
                            "wrist_1_joint", 
                            "wrist_2_joint", 
                            "wrist_3_joint"
                            };


    RTDEDriver() : Node("rtde_driver")
    {
        new_traj = false;
        rtde_control = std::make_unique<RTDEControlInterface>("192.168.56.101");
        rtde_receive = std::make_unique<RTDEReceiveInterface>("192.168.56.101");

        joint_status_pub = this -> create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
        trajectory_sub = this -> create_subscription<trajectory_msgs::msg::JointTrajectory>
                            ("joint_trajectory", 10, std::bind(&RTDEDriver::trajRecCallback, this,_1));
        
        control_thread = new std::thread(&RTDEDriver::controlArm, this);

        timer_ = this->create_wall_timer(
                    8ms, std::bind(&RTDEDriver::receiveCallback, this));

    }

    void controlArm();
    void receiveCallback();
    void print_vector(vector<double>& v);
    void swap_idx(vector<double>& v);
    void trajRecCallback(const trajectory_msgs::msg::JointTrajectory& rec_traj);
    
};

void RTDEDriver::controlArm()
{
    while(true)
    {
        // if(new_traj)
        // {
        //     traj_lock.lock();
        //     trajectory.clear();
        //     trajectory.insert(trajectory.end(), mid_traj.begin(), mid_traj.end());
        //     mid_traj.clear();
        //     traj_lock.unlock();
        //     new_traj = false;
        // }

        // if(trajectory.empty())
        // {
        //     cout<<"no trajectory to execute"<<endl;
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // }


        // for(auto p: trajectory)
        // {
        //     if(new_traj)
        //         continue;
        //     cout<<"controlling"<<endl;
        //     steady_clock::time_point t_start = rtde_control -> initPeriod();
        //     rtde_control -> servoJ(p, velocity, acceleration, dt, lookahead_time, gain);
        //     rtde_control -> waitPeriod(t_start);
        // }
        // trajectory.clear();


    if(new_traj)
    {
        trajectory_share(traj);
        new_traj = false;
    }
    if(traj.empty())
    {
        //cout<<"no trajectory to excute"<<endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
    }
        vector<double> p;
        p = traj.front();
        traj.pop();
        //cout<<"controlling"<<endl;
        steady_clock::time_point t_start = rtde_control -> initPeriod();
        rtde_control -> servoJ(p, velocity, acceleration, dt, lookahead_time, gain);
        rtde_control -> waitPeriod(t_start);
    }
   
}

void RTDEDriver::receiveCallback()
{
    vector<double> position, velocity, effort;
    position = rtde_receive -> getActualQ();
    velocity = rtde_receive -> getActualQd();
    effort = rtde_receive -> getTargetCurrent();
    // swap_idx(position);
    // swap_idx(velocity);
    // swap_idx(effort);
    joint_state.header.stamp = this -> get_clock()-> now();
    joint_state.name = name;
    joint_state.position = position;
    joint_state.velocity = velocity;
    // joint_state.effort = effort;
    // cout<<"position :"<<endl;
    // print_vector(position);
    // cout<<"velocity :"<<endl;
    // print_vector(velocity);
    // cout<<"effort :"<<endl;
    // print_vector(effort);
    // joint_state.effort = effort;
    // cout<<"position :"<<endl;
    // print_vector(position);
    // cout<<"velocity :"<<endl;
    // print_vector(velocity);
    // cout<<"effort :"<<endl;
    // print_vector(effort);
    joint_status_pub -> publish(joint_state);
}

void RTDEDriver::trajRecCallback(const trajectory_msgs::msg::JointTrajectory& rec_traj)
{
    cout<<"receive one traj, size is: "<<rec_traj.points.size()<<endl;
    queue<vector<double>> tmp_traj;
    for(auto& p : rec_traj.points)
    {
        // swap_idx(p.positions);
        tmp_traj.push(p.positions);
    }

    trajectory_share(tmp_traj);


    // traj_lock.lock();
    // mid_traj.insert(mid_traj.end(),tmp_traj.begin(),tmp_traj.end());
    // traj_lock.unlock();
    new_traj = true;

}


void RTDEDriver::print_vector(vector<double>& v)
{
    for(auto it: v)
    {
        cout<<"- "<<it<<endl;
    }
}

void RTDEDriver::swap_idx(vector<double>& v)
{
    vector<double> tmp;
    tmp.push_back(v[5]);
    tmp.push_back(v[0]);
    tmp.push_back(v[1]);
    tmp.push_back(v[2]);
    tmp.push_back(v[3]);
    tmp.push_back(v[4]);
    v = tmp;
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RTDEDriver>());
  
    /*
    shoulder_pan
    shoulder_lift
    elbow
    wrist 1
    wrist 2
    wrist 3
    */

    rclcpp::shutdown();
    return 0;
}
