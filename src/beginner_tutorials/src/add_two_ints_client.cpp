#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
//Crea un objeto NodeHandle, que representa la conexión 
//del programa cliente con el core de ROS.
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
//Crea un objeto ServiceClient, que permite al programa cliente 
//interactuar con el servicio "add_two_ints".
  beginner_tutorials::AddTwoInts srv;
//Crea una variable del tipo de servicio AddTwoInts, que se 
//utilizará para interactuar con el servicio.
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
//Convierte los dos argumentos del programa (cadenas de texto) en números enteros 
//de 64 bits y los asigna a los campos "a" y "b" de la estructura del servicio.
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}

//El programa "add_two_ints_client" es un cliente de servicio ROS que 
//utiliza el servicio "add_two_ints" para sumar dos números enteros. 
//Para utilizarlo, se debe proporcionar exactamente dos argumentos al 
//programa (los números a sumar) más el nombre del programa.