#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

float vel_linear = 0.0;
float vel_angular = 1.0;

// Callback para manejar mensajes cmd_vel
void cmdVelCallback(const geometry_msgs::Twist& msg) {
    ROS_INFO_STREAM("Received cmd_vel message - Linear: " << msg.linear.x << ", Angular: " << msg.angular.z);
    vel_linear = msg.linear.x;
    vel_angular = msg.angular.z;
}   

int main(int argc, char **argv) {
    ros::init(argc, argv, "spawn_turtle_plus");
    ros::NodeHandle nh;

    // Cliente de servicio para spawn
    ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");
    // Cliente de servicio para toggle_forward
    ros::ServiceClient toggleForwardClient = nh.serviceClient<std_srvs::Empty>("/toggle_forward");

    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;

    std_srvs::Empty::Request toggleReq;
    std_srvs::Empty::Response toggleResp;

    // Configuración de la posición y orientación de la nueva tortuga
    req.x = 5;
    req.y = 8;
    req.theta = M_PI / 2;
    req.name = "Dinho-10";

    // Intenta llamar al servicio de spawn
    bool success = spawnClient.call(req, resp);

    if (success) {
        ROS_INFO_STREAM("Spawned a turtle named " << resp.name);
    } else {
        ROS_ERROR_STREAM("Failed to spawn.");
        return 1;
    }

    // Suscripción al tema "turtle1/cmd_vel" para recibir comandos de velocidad
    ros::Subscriber sub = nh.subscribe("turtle1/cmd_vel", 1000, cmdVelCallback);

    // Publicador para el tema "Leo/cmd_vel" de la nueva tortuga
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("Leo/cmd_vel", 1000);
    
    // Intenta llamar al servicio toggle_forward
    bool toggleSuccess = toggleForwardClient.call(toggleReq, toggleResp);
    ros::Rate rate(2);

    if (toggleSuccess) {
        ROS_INFO_STREAM("Turtles are now rotating.");
        while (ros::ok()) {
            geometry_msgs::Twist msg;
            msg.linear.x = vel_linear;
            msg.angular.z = vel_angular;
            pub.publish(msg);
            ros::spinOnce();
            rate.sleep();

            ROS_INFO_STREAM("Sending random velocity command:"
            << " linear=" << msg.linear.x
            << " angular=" << msg.angular.z);
        }
    } else {
        ROS_ERROR_STREAM("Failed to toggle rotation.");
        return 1;
    }

    return 0;
}
