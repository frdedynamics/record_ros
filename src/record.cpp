#include "record_ros/record.h"
#include "record_ros/record_status.h"
#include <ros/ros.h>
#include <boost/thread.hpp>


std::string FILENAME_PREFIX;
std::string START_STAMP;

std::string stampToString(const ros::Time& stamp, const std::string format="%Y-%m-%d-%H-%M-%S")
   {
      const int output_size = 100;
      char output[output_size];
      std::time_t raw_time = static_cast<time_t>(stamp.sec + stamp.nsec/1e9);
      struct tm* timeinfo = localtime(&raw_time);
      std::strftime(output, output_size, format.c_str(), timeinfo);
      return std::string(output);
    }

static void cb_shutdown(const ros::TimerEvent&){
  ros::shutdown();
}

Record::Record(ros::NodeHandle &nh,rosbag::RecorderOptions const& options):
    rosbag::Recorder(options)
{
    service_srv = nh.advertiseService("cmd",&Record::string_command,this);
    b_record    = false;
    shutdown_timer = nh.createTimer(ros::Duration(0.1), cb_shutdown,true,false);
    record_status_pub = nh.advertise<record_ros::record_status>("record_status", 1);
    FILENAME_PREFIX = options.prefix;
    record_status_msg.filename = options.prefix;

}

void Record::wait_for_callback(){
    ros::Rate r(100); // 60 hz
    while (!b_record && ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}

bool Record::string_command(record_ros::String_cmd::Request& req, record_ros::String_cmd::Response& res){
    std::string cmd = req.cmd;
    ROS_INFO("Record callback");
    if(cmd == "record"){
        if(b_record){
            shutdown_timer.start();
            res.res = "stopping recorder";
        }else{
            b_record = true;
            res.res  = "starting recorder";
            START_STAMP = stampToString(ros::Time::now());
        }
        return true;
    }else if(cmd == "stop"){
        shutdown_timer.start();
        res.res = "stopping recorder";
        record_status_msg.stamp = ros::Time::now();
        record_status_msg.filename_with_start_stamp = FILENAME_PREFIX + "_" + START_STAMP + ".bag";
        record_status_pub.publish(record_status_msg);
        return true;
    }else{
        res.res = "No such command[" + cmd + "] in [Record::string_command]";
        ROS_WARN_STREAM(res.res);
        return false;
    }
}
