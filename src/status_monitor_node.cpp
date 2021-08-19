#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include "ros_type_introspection/ros_introspection.hpp"

#include <iostream>
#include <string>
#include <vector>

using namespace RosIntrospection;
using topic_tools::ShapeShifter;

struct topicInfo
{
    double freq_expected;
    double freq_tol = 2.0;
    bool initialized = false;
    bool is_publishing;

    ros::Time last_time;
    double curr_freq = 0.0;

    topicInfo(double freq_ex)
    : freq_expected(freq_ex)
    {};
};

XmlRpc::XmlRpcValue monitor_list;
std::map<std::string, topicInfo> topics;

void topicCallback(const ShapeShifter::ConstPtr& msg, const std::string& topic_name,
                   RosIntrospection::Parser& parser)
{
    std::map<std::string, topicInfo>::iterator it = topics.find(topic_name);
    auto info = &(it->second);
    if (!info->initialized) {
        info->last_time = ros::Time::now();
        info->initialized = true;
    }
    else {
        ros::Time curr_time = ros::Time::now();
        ros::Duration duration = curr_time - info->last_time;
        info->curr_freq = 1 / duration.toSec();
        info->last_time = curr_time;
    }
}

void monitorCallback(const ros::TimerEvent& event)
{   
    int num_topics = topics.size();
    int correct_pub = 0;
    std::vector<std::string> incorrect_pub_topics;

    for (const auto& t : topics) {
        topicInfo topic = t.second;
        if ((topic.curr_freq > topic.freq_expected - topic.freq_tol) && 
            (topic.curr_freq < topic.freq_expected + topic.freq_tol)) {
            correct_pub += 1;
        }
        else {
            incorrect_pub_topics.push_back(t.first);
        }
    }
    ROS_INFO("\033[1;32m%i / %i topics publishing correctly\033[0m", correct_pub, num_topics);
    if (correct_pub != num_topics) {
        ROS_INFO("\033[1;33m--> Topics not publishing correctly:\033[0m");
        for (auto t : incorrect_pub_topics)
            ROS_INFO("\033[1;33m    %s\033[0m", t.c_str());
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

        // verify expected publishing rate is type double
        ROS_ASSERT(monitor_iter["expected_hz"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double expected_hz = monitor_iter["expected_hz"];

        // for debugging
        // ROS_INFO("Topic name: %s - expected rate: %f. ", topic_name.c_str(), expected_hz);

        // create new struct to hold topic information
        topicInfo newTopic(expected_hz);
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
    // nh.getParam("pub_sub", monitor_list);
    nh.getParam("H02_topics", monitor_list);
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

