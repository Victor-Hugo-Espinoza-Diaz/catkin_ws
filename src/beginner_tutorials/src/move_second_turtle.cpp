#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

class MoveSecondTurtle {
public:
    MoveSecondTurtle() {
        // Inicializar el nodo
        ros::NodeHandle nh;

        // Suscribirse al topic "turtle1/cmd_vel"
        cmd_vel_sub = nh.subscribe("turtle1/cmd_vel", 10, &MoveSecondTurtle::cmdVelCallback, this);

        // Publicar en el topic "second_turtle/cmd_vel"
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("second_turtle/cmd_vel", 10);

        // Crear una nueva tortuga
        spawnSecondTurtle();
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // Invertir la velocidad
        geometry_msgs::Twist inverted_twist;
        inverted_twist.linear.x = -msg->linear.x;
        inverted_twist.linear.y = -msg->linear.y;
        inverted_twist.angular.z = -msg->angular.z;

        // Publicar la velocidad invertida en "second_turtle/cmd_vel"
        cmd_vel_pub.publish(inverted_twist);
    }

    void spawnSecondTurtle() {
        // Crear una nueva tortuga en la misma posición que la primera
        ros::service::waitForService("spawn");
        turtlesim::Spawn spawn;
        spawn.request.x = 5.0;
        spawn.request.y = 5.0;
        spawn.request.theta = 0;
        spawn.request.name = "second_turtle";

        if (ros::service::call("spawn", spawn)) {
            ROS_INFO("Segunda tortuga creada con éxito");
        } else {
            ROS_ERROR("Error al crear la segunda tortuga");
        }
    }

private:
    ros::Subscriber cmd_vel_sub;
    ros::Publisher cmd_vel_pub;
};

int main(int argc, char** argv) {
    // Inicializar ROS
    ros::init(argc, argv, "move_second_turtle");

    // Crear el objeto MoveSecondTurtle
    MoveSecondTurtle moveSecondTurtle;

    // Manejar eventos y esperar mensajes
    ros::spin();

    return 0;
}
