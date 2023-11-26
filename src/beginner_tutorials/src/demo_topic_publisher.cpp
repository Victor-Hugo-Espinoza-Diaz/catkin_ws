//Se incluye librerias necesarias para ROS
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
int main(int argc, char **argv)
{
//inicializa el nodo de ROS y registra su nombre como"demo_topic_publisher".
    ros::init(argc, argv,"demo_topic_publisher");
//Crea un objeto NodeHandle para interactuar con el master de ROS.                            
    ros::NodeHandle node_obj;
//Advertise (anunciar) un Publisher que enviará mensajes de tipo Int32 al topic 
//llamado"/numbers".Se reserva espacio para hasta 10 mensajes en cola.
    ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers",10);
//Crea un objeto Rate para controlar la tasa de publicación. Aquí,la tasa de 
//publicación se establece en 10 Hz.
    ros::Rate loop_rate(10);
//Inicializa un contador entero que se utilizará para generar el número en el mensaje.
    int number_count = 0;
//Inicia un bucle while que se ejecutará siempre que ROS esté funcionando correctamente.
    while (ros::ok())
    {
    std_msgs::Int32 msg;//Crea un objeto mensaje de tipo Int32.
        msg.data = number_count;//Establece el valor del número en el mensaje.
        ROS_INFO("%d",msg.data);//Imprime el valor del número en el mensaje en la terminal.
        number_publisher.publish(msg);//Publica el mensaje en el topic "/numbers".
        ros::spinOnce();//Ejecuta una ronda de publicación y suscripción de mensajes de ROS.
        loop_rate.sleep();//Hace una pausa hasta que haya pasado el tiempo suficiente para cumplir con la tasa de publicación deseada.
        ++number_count;//Incrementa el contador de números en 1.
    }
    return 0;
}