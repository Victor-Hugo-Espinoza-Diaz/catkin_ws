#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/MyCustomMessageType.h"

// Implementa la función de devolución de llamada del suscriptor
void subscriberCallback(const beginner_tutorials::MyCustomMessageType& msg)
{
    // Extrae los datos del mensaje
    bool is_valid = msg.isValid;
    uint restarts_remaining = msg.restartsRemaining;
    std::string taskDescription = msg.taskDescription;
    // y así sucesivamente para los demás campos.

    // Imprime los elementos del vector
    std::vector<float> distances_vector;
    distances_vector.assign(msg.distancesToOtherRobots.begin(), msg.distancesToOtherRobots.end());
    int vector_size = distances_vector.size();
    ROS_INFO_STREAM("*********[NODO CPP DEL SUSCRIPTOR]*************");
    ROS_INFO("El parámetro is_valid es: %s", is_valid ? "true" : "false");
    ROS_INFO("El parámetro restarts_remaining es: [%i]", restarts_remaining);
    ROS_INFO("El parámetro taskDescription es: %s", taskDescription.c_str());
    for (int i=0; i<vector_size; i++)
    {
        ROS_INFO_STREAM("[NODO CPP DEL SUSCRIPTOR] distancesToOtherRobots["<< i << "] = " << msg.distancesToOtherRobots.at(i));
    }
}

int main(int argc, char **argv)
{
    // Inicializa el nodo ROS con los argumentos proporcionados.
    ros::init(argc, argv, "listener_custom_msg");

    // Crea un objeto NodeHandle para interactuar con el sistema ROS.
    ros::NodeHandle n;

    // Crea un objeto suscriptor para el tema "great_custom_topic" con una cola máxima de 1000 mensajes y 
    // lo asocia a la función de devolución de llamada del suscriptor.
    ros::Subscriber sub = n.subscribe("great_custom_topic", 1000, subscriberCallback);

    // Gira para mantener vivo el nodo y procesar devoluciones de llamada.
    ros::spin();

    // Devuelve 0 para indicar una terminación exitosa.
    return 0;
}

//En resumen, este nodo suscriptor escucha el tema "great_custom_topic" y, 
//cuando recibe mensajes, extrae datos específicos del mensaje y los imprime en la consola.