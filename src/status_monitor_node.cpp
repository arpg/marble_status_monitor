#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include "ros_type_introspection/ros_introspection.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <numeric>
#include <iomanip>

using namespace RosIntrospection;
using topic_tools::ShapeShifter;

struct topicInfo
{
    double freq_expected;
    double tolerance;
    bool initialized = false;
    bool is_publishing = false;

    ros::Time last_time;
    ros::Time curr_time;
    double last_avg_freq = 0.0;
    double avg_freq = 0.0;
    double curr_freq = 0.0;
    int horizon = 100;
    // std::vector<double> pub_times = std::vector<double>(horizon);
    std::vector<double> pub_times; // has to be a better way to do this
    
    topicInfo(double freq_ex, double tol)
    : freq_expected(freq_ex),
      tolerance(tol)
    {};
};

XmlRpc::XmlRpcValue monitor_list;
std::string vehicle_name;
std::map<std::string, topicInfo> topics;

void topicCallback(const ShapeShifter::ConstPtr& msg, const std::string& topic_name,
                   RosIntrospection::Parser& parser)
{
    std::map<std::string, topicInfo>::iterator it = topics.find(topic_name);
    auto info = &(it->second);
    if (!info->initialized) {
        // check if topic has been initialized
        info->last_time = ros::Time::now();
        info->initialized = true;
    }
    else {
        // if topic has been initialized then calculate publishing rate statistics
        info->last_avg_freq = info->avg_freq;
        ros::Time curr_time = ros::Time::now();
        ros::Duration duration = curr_time - info->last_time;
        double curr_freq = 1 / duration.toSec();
        // check if pub_times vector is equal to horizon length
        if (info->pub_times.size() == info->horizon) {
            info->pub_times.erase(info->pub_times.begin());
            info->pub_times.push_back(curr_freq);
        }
        else
            info->pub_times.push_back(curr_freq);
        info->curr_freq = curr_freq;
        info->last_time = curr_time;
    }
}

void monitorCallback(const ros::TimerEvent& event)
{   
    ROS_INFO("\033\143"); // This clears the terminal so it appears like it's updating instead of printing continuously
    int num_topics = topics.size();
    std::string color;
    for (auto it = topics.begin(); it != topics.end(); ++it) {
        topicInfo* topic = &(it->second);
        topic->last_avg_freq = topic->avg_freq;
        double avg_freq = std::accumulate(topic->pub_times.begin(), topic->pub_times.end(), 0.0) / topic->pub_times.size();
        topic->avg_freq = avg_freq;
        if(isnan(avg_freq)) {
            topic->avg_freq = 0.0;
            ROS_INFO("\033[1;31m NO MSG\t%.1f\t%s\033[0m", topic->avg_freq, it->first.c_str());
        }
        else if((topic->avg_freq > topic->freq_expected + topic->tolerance) || 
            (topic->avg_freq < topic->freq_expected - topic->tolerance)) {
            ROS_INFO("\033[1;31m FAIL\t%.1f\t%s\033[0m", topic->avg_freq, it->first.c_str());
            topic->is_publishing = true;
        }
        else {
            ROS_INFO("\033[1;32m PASS\t%.1f\t%s\033[0m", topic->avg_freq, it->first.c_str());
        }
    }
}

void loadFromConfig(ros::NodeHandle& nh, std::vector<ros::Subscriber>& subs)
{
    for (int i = 0; i < monitor_list.size(); i++) {
        XmlRpc::XmlRpcValue monitor_iter = monitor_list[i];
        ROS_ASSERT(monitor_iter.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        // verify topic name is type string 
        ROS_ASSERT(monitor_iter["topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string topic_name = monitor_iter["topic"];
		topic_name = "/" + vehicle_name + "/" + topic_name;

        // verify expected publishing rate is type double
        ROS_ASSERT(monitor_iter["expected_hz"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double expected_hz = monitor_iter["expected_hz"];

        // verify publishing rate tolerance is type double
        ROS_ASSERT(monitor_iter["tolerance"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double tolerance = monitor_iter["tolerance"];

        // create new struct to hold topic information
        topicInfo newTopic(expected_hz, tolerance);
        topics.insert(std::pair<std::string, topicInfo> (topic_name, newTopic));
        
        // create callback for topic and add to subscriber list
        Parser parser;
        boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
        callback = [&parser, topic_name](const topic_tools::ShapeShifter::ConstPtr& msg)
        {
            topicCallback(msg, topic_name, parser);
        };
        ros::Subscriber sub = nh.subscribe(topic_name, 10, callback);
        subs.push_back(sub);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "status_monitor");
    ROS_INFO("\033[1;34m----> Topic monitor started <----\033[0m");
    ros::NodeHandle nh;
    nh.getParam("vehicle_name", vehicle_name);
    nh.getParam("topic_list", monitor_list);
    ROS_INFO("\033[1;35mMonitoring %d topics \033[0m", monitor_list.size());

    std::vector<ros::Subscriber> topic_subs;
    loadFromConfig(nh, topic_subs);

    // for debugging
    // ros::master::V_TopicInfo advertized_topics;
    // ros::master::getTopics(advertized_topics);
    // for (const auto& topic_info: advertized_topics) {
    //     if ((topic_info.name == "/rosout") || (topic_info.name == "/rosout_agg"))
    //         continue;
    //     ROS_INFO("\033[1;35m--> Advertised: %s \033[0m", topic_info.name.c_str());
    // }
    
    ros::Timer monTimer = nh.createTimer(ros::Duration(1), monitorCallback);
    ros::spin();
	
    return 0;
}

