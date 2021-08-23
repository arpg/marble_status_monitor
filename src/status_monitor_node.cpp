#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include "ros_type_introspection/ros_introspection.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <numeric>
#include <iomanip>

using namespace RosIntrospection;
using namespace std;
using topic_tools::ShapeShifter;

struct topicInfo
{
    string label_name;
    double freq_expected;
    double tolerance;
    bool initialized = false;

    double freq;
    int counter;
    // vector<double> pub_times = vector<double>(horizon);
    vector<double> pub_times; // has to be a better way to do this
    
    topicInfo(string lab, double freq_ex, double tol)
    : label_name(lab),
      freq_expected(freq_ex),
      tolerance(tol)
    {};
};

bool init;
string vehicle_name;
float print_rate;
XmlRpc::XmlRpcValue label_list;
XmlRpc::XmlRpcValue topic_list;
map<string, topicInfo> topics;

void topicCallback(const ShapeShifter::ConstPtr& msg, const string& topic_name,
                   RosIntrospection::Parser& parser)
{
    map<string, topicInfo>::iterator it = topics.find(topic_name);
    auto info = &(it->second);
    info->counter++;
}

void monitorCallback(const ros::TimerEvent& event)
{
    if(!init) {
        printf("\033\143"); // This clears the terminal so it appears like it's updating instead of printing continuously
        int num_topics = topics.size();
        string color;
        for (auto it = topics.begin(); it != topics.end(); ++it) {
            topicInfo* topic = &(it->second);
            topic->freq = topic->counter * print_rate;
            if(isnan(topic->freq)) {
                topic->freq = 0.0;
            }
            else if((topic->freq > topic->freq_expected + topic->tolerance) || 
                (topic->freq < topic->freq_expected - topic->tolerance)) {
            }
            else {
            }
            topic->counter = 0;
        }
        init = 1;
    }
    else {
        printf("\033\143"); // This clears the terminal so it appears like it's updating instead of printing continuously
        int num_topics = topics.size();
        string color;
        for (auto it = topics.begin(); it != topics.end(); ++it) {
            topicInfo* topic = &(it->second);
            topic->freq = topic->counter * print_rate;
            if(isnan(topic->freq)) {
                topic->freq = 0.0;
                printf("\033[1;31m NO MSG\t%.1f\t%s\t%s\033[0m \n", topic->freq, topic->label_name.c_str(), it->first.c_str());
            }
            else if((topic->freq > topic->freq_expected + topic->tolerance) || 
                (topic->freq < topic->freq_expected - topic->tolerance)) {
                printf("\033[1;31m FAIL\t%.1f\t%s\t%s\033[0m \n", topic->freq, topic->label_name.c_str(), it->first.c_str());
            }
            else {
                printf("\033[1;32m PASS\t%.1f\t%s\t%s\033[0m \n", topic->freq, topic->label_name.c_str(), it->first.c_str());
            }
            topic->counter = 0;
        }
    }
}

void loadFromConfig(ros::NodeHandle& nh, vector<ros::Subscriber>& subs)
{
    for (int i = 0; i < topic_list.size(); i++) {
        XmlRpc::XmlRpcValue monitor_iter = topic_list[i];
        ROS_ASSERT(monitor_iter.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        // verify topic name is type string 
        ROS_ASSERT(monitor_iter["label"].getType() == XmlRpc::XmlRpcValue::TypeString);
        string label_name = monitor_iter["label"];

        // verify topic name is type string 
        ROS_ASSERT(monitor_iter["topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
        string topic_name = monitor_iter["topic"];
		topic_name = "/" + vehicle_name + "/" + topic_name;

        // verify expected publishing rate is type double
        ROS_ASSERT(monitor_iter["expected_hz"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double expected_hz = monitor_iter["expected_hz"];

        // verify publishing rate tolerance is type double
        ROS_ASSERT(monitor_iter["tolerance"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double tolerance = monitor_iter["tolerance"];

        // create new struct to hold topic information
        topicInfo newTopic(label_name, expected_hz, tolerance);
        topics.insert(pair<string, topicInfo> (topic_name, newTopic));
        
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
    ros::NodeHandle nh;
    nh.getParam("print_rate", print_rate);
    nh.getParam("vehicle_name", vehicle_name);
    nh.getParam("label_list", label_list);
    nh.getParam("topic_list", topic_list);
    vector<ros::Subscriber> topic_subs;
    loadFromConfig(nh, topic_subs);

    // for debugging
    // ros::master::V_TopicInfo advertized_topics;
    // ros::master::getTopics(advertized_topics);
    // for (const auto& topic_info: advertized_topics) {
    //     if ((topic_info.name == "/rosout") || (topic_info.name == "/rosout_agg"))
    //         continue;
    //     cout << "\033[1;35m--> Advertised: %s \033[0m", topic_info.name.c_str());
    // }
    
    ros::Timer monTimer = nh.createTimer(ros::Duration(1 / print_rate), monitorCallback);
    ros::spin();
	
    return 0;
}

