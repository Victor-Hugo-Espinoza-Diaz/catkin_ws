#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // Para geometry_msgs::Twist
#include <turtlesim/Pose.h>      // Para turtlesim::Pose
#include <stdlib.h>              // Para rand() y RAND_MAX

bool safezone = true;
void callback(const turtlesim::Pose& msg) 
{
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed
        << "position=(" <<  msg.x << "," << msg.y << ")"
        << " direction=" << msg.theta);
        if(msg.x >= 2.5 && msg.x <= 7 && msg.y >= 2.5 && msg.y <= 7)
        {
            safezone = true;
        }
        else
            safezone = false;
}

int main(int argc, char **argv) {
 // Inicializa el sistema ROS y se convierte en un nodo.
 ros::init(argc, argv, "safe_zone_vel_publisher");
 ros::NodeHandle nh;

 // Crea un objeto editor.
 ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "turtle1/cmd_vel", 1000);

 // Obtén las coordenadas x e y de la tortuga en tiempo real.
 ros::Subscriber sub = nh.subscribe(
    "turtle1/pose", 1000,&callback);

 // Inicializa el generador de números aleatorios.
 srand(time(0));

 // Bucle a 2 Hz hasta que el nodo se cierre.
 ros::Rate rate(2);
 while(ros::ok()) 
 {
    // Crea y llena el mensaje. Los otros cuatro
    // campos, que son ignorados por turtlesim, se establecen en 0 por defecto.
    geometry_msgs::Twist msg;

    msg.linear.x = double(rand())/double(RAND_MAX); // Velocidad lineal aleatoria.
    msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1; // Velocidad angular aleatoria.

    // Determina si la tortuga está dentro del cuadrado seguro.
    if (safezone) 
    {
      msg.linear.x = 1.0; // Velocidad lineal fija de 1.0.
      msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1; // Velocidad angular aleatoria.
    } 
    else 
    {
      msg.linear.x = double(rand())/double(RAND_MAX); // Velocidad lineal aleatoria.
      msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1; // Velocidad angular aleatoria.
    }

    // Publica el mensaje.
    pub.publish(msg);

    // Envía un mensaje a rosout con los detalles.
    ROS_INFO_STREAM("Enviando comando de velocidad:"
      << " linear=" << msg.linear.x
      << " angular=" << msg.angular.z);

    // Espera hasta que sea el momento de otra iteración.
    rate.sleep();
 }
}
