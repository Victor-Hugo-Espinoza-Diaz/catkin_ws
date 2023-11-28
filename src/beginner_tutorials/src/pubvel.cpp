// Este programa publica mensajes de velocidad generados aleatoriamente
// para turtlesim.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // Para geometry_msgs::Twist
#include <stdlib.h> // Para rand() y RAND_MAX

int main(int argc, char **argv) 
{
  // Inicializa el sistema ROS y se convierte en un nodo.
  ros::init(argc, argv, "publish_velocity");
  ros::NodeHandle nh;

  // Crea un objeto editor.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "turtle1/cmd_vel", 1000);

  // Inicializa el generador de números aleatorios.
  srand(time(0));

  // Bucle a 2 Hz hasta que el nodo se cierre.
  ros::Rate rate(2);
  while(ros::ok()) 
  {
    // Crea y llena el mensaje. Los otros cuatro
    // campos, que son ignorados por turtlesim, se establecen en 0 por defecto.
    geometry_msgs::Twist msg;
    msg.linear.x = double(rand())/double(RAND_MAX);
    msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;

    // Publica el mensaje.
    pub.publish(msg);

    // Envía un mensaje a rosout con los detalles.
    ROS_INFO_STREAM("Enviando comando de velocidad aleatorio:"
      << " linear=" << msg.linear.x
      << " angular=" << msg.angular.z);

    // Espera hasta que sea el momento de otra iteración.
    rate.sleep();
  }
}
