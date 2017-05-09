#pragma once

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
#include <val_footstep/ValkyrieWalker.h>

using namespace decision_making;

#define foreach BOOST_FOREACH

FSM_HEADER(val_task2)

class valTask2 {

    private:
    ros::NodeHandle nh_;

    // object for the walker api
    ValkyrieWalker* walker_;

    static valTask2 *currentObject;

    public:

    // goal location for the panel
    geometry_msgs::Pose2D panel_walk_goal_;

    // default constructor and destructor
    valTask2(ros::NodeHandle nh);
    ~valTask2();
    static valTask2* getValTask2(ros::NodeHandle nh);
    bool preemptiveWait(double ms, decision_making::EventQueue& queue);
    decision_making::TaskResult initTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult orientPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult pickPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult placePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectButtonTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult deployPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult dtectCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult pickCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult plugCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectfinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult endTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    geometry_msgs::Pose2D getPanelWalkGoal();
    void setPanelWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal_);
};
