//Se incluye librerias necesarias para ROS
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
void number_callback(const std_msgs::Int32::ConstPtr& msg)
//
{
    ROS_INFO("Received [%d]",msg->data);
// La función se encarga de imprimir el mensaje recibido
}
int main(int argc, char **argv)
//Inicializar ROS y suscriptor
{
    ros::init(argc, argv,"demo_topic_subscriber");
//Establece el nombre del nodo ROS, que en este caso es "demo_topic_subscriber". 
//También es importante notar que esto es una llamada necesaria a cualquier programa ROS que no sea un nodo.
    ros::NodeHandle node_obj;
//Es importante tener un NodeHandle para comunicarse con el ROS Master y suscriptores y publicadores.
    ros::Subscriber number_subscriber =
    node_obj.subscribe("/numbers",10,number_callback);
//Crea un suscriptor en el tópico "/numbers". La función de devolución de llamada (callback)
//que se ejecutará cuando se reciba un mensaje en este tópico es "number_callback". 
//La cantidad máxima de mensajes en la cola del suscriptor es 10.
    ros::spin();
//Esta función se bloquea hasta que el nodo reciba una señal de finalización. 
//Procesa todas las callbacks que estén listas en la cola.
    return 0;
}