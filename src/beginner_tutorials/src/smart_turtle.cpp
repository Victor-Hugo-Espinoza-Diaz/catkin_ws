#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // Para geometry_msgs::Twist
#include <turtlesim/Pose.h>      // Para turtlesim::Pose
#include <stdlib.h>              // Para rand() y RAND_MAX


bool smartturtle = true;
void callback(const turtlesim::Pose& msg) 
{
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed
    << "position=(" <<  msg.x << "," << msg.y << ")"
    << " direction=" << msg.theta);
    if(abs(msg.x) >= 9.0 || abs(msg.y) >= 9.0)
    {
        smartturtle = true;
    }
    else
        smartturtle = false;
}

int main(int argc, char **argv) {
 // Inicializa el sistema ROS y se convierte en un nodo.
 ros::init(argc, argv, "smart_turtle");
 ros::NodeHandle nh;

 // Crea un objeto editor.
 ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "turtle1/cmd_vel", 1000);

 // Obtén las coordenadas x e y de la tortuga en tiempo real.
 ros::Subscriber pose_sub = nh.subscribe(
    "turtle1/pose", 1000,&callback);

 // Inicializa el generador de números aleatorios.
 srand(time(0));

 // Bucle a 2 Hz hasta que el nodo se cierre.
 ros::Rate rate(2);
 while(ros::ok()) {
    // Crea y llena el mensaje. Los otros cuatro
    // campos, que son ignorados por turtlesim, se establecen en 0 por defecto.
    geometry_msgs::Twist msg;

    // Determina si la tortuga está chocando contra una pared.
    if (smartturtle) {
      msg.linear.x = 0.0; // No avanza.
      msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1; // Gira aleatoriamente.
    } else {
      msg.linear.x = 1.0; // Avanza a una velocidad constante de 1.0.
      msg.angular.z = 0.0; // No gira.
    }

    // Publica el mensaje.
    pub.publish(msg);

    // Envía un mensaje a rosout con los detalles.
    ROS_INFO_STREAM("Enviando comando de velocidad:"
      << " linear=" << msg.linear.x
      << " angular=" << msg.angular.z);

    // Espera hasta que sea el momento de otra iteración.
    rate.sleep();
    ros::spinOnce();
 }
}
