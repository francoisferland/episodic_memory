/*
 * episodic_memory_planner_node.cpp
 *
 *  Created on: 2013-05-29
 *      Author: frank
 */

#include <episodic_memory_planner.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "episodic_memory_planner");

    ros::NodeHandle n, np("~");
    ros_episodic_memory::EpisodicMemoryPlanner node(n, np);
    ros::spin();
}
