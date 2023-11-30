#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <beginner_tutorials/ComputeDistance.h>
#include <cmath>

ros::ServiceClient spawnClient;

bool spawnTurtle() {
    turtlesim::Spawn spawn;
    spawn.request.x = 5.5;
    spawn.request.y = 5.5;
    spawn.request.theta = 0.0;
    spawn.request.name = "turtle2";

    if (spawnClient.call(spawn)) {
        ROS_INFO("Turtle 'turtle2' spawned successfully");
        return true;
    } else {
        ROS_ERROR("Failed to spawn turtle 'turtle2'");
        return false;
    }
}

bool computeDistance(beginner_tutorials::ComputeDistance::Request &req,
                     beginner_tutorials::ComputeDistance::Response &res) {
    double dx = req.x2 - req.x1;
    double dy = req.y2 - req.y1;
    res.distance = sqrt(dx * dx + dy * dy);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_distance_node");
    ros::NodeHandle nh;

    spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");
    ros::service::waitForService("spawn");

    if (spawnTurtle()) {
        ROS_INFO("Spawning of turtle2 successful");
    } else {
        ROS_ERROR("Failed to spawn turtle2. Exiting.");
        return 1;
    }

    ros::ServiceServer distanceServer = nh.advertiseService("calculate_distance", computeDistance);

    ros::spin();
    return 0;
}
