#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/MyCustomMessageType.h"
#include <sstream>

ros::Publisher chatter_pub;

// Se incluyen las bibliotecas necesarias para el uso de ROS y los mensajes estándar.

// Se declara un publicador global llamado chatter_pub.

// Implementa la función de devolución de llamada del temporizador
void timerCallback(const ros::TimerEvent&)
{
    // Inicializa un mensaje vacío del tipo personalizado.
    beginner_tutorials::MyCustomMessageType msg;

    // Rellena los campos del mensaje.
    msg.isValid = true;
    msg.restartsRemaining = 42;
    msg.distanceToStart = 4.2;
    msg.distanceToEnd = 42.42;
    msg.taskDescription = "Staying alive";

    // "Push back" datos en el campo vector.
    msg.distancesToOtherRobots.push_back(1.1);
    msg.distancesToOtherRobots.push_back(2.3);
    msg.distancesToOtherRobots.push_back(2.7);

    // Publica el mensaje.
    chatter_pub.publish(msg);
}

int main(int argc, char **argv)
{
    // Inicializa el nodo ROS con los argumentos proporcionados.
    ros::init(argc, argv, "talker_custom_msg");

    // Crea un objeto NodeHandle para interactuar con el sistema ROS.
    ros::NodeHandle n;

    // Asocia el publicador al tema "great_custom_topic" con una cola máxima de 1000 mensajes.
    chatter_pub = n.advertise<beginner_tutorials::MyCustomMessageType>("great_custom_topic", 1000);

    // Crea un temporizador que ejecutará la función de devolución de llamada cada 1 segundo.
    ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);

    // Gira para mantener vivo el nodo y procesar devoluciones de llamada.
    ros::spin();

    // Devuelve 0 para indicar una terminación exitosa.
    return 0;
}
