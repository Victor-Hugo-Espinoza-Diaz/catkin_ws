#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
//Esta función toma dos argumentos de referencia: la solicitud y 
//la respuesta del servicio. La función suma los dos números enteros 
//que se encuentran en la solicitud y devuelve el resultado en la respuesta.
{
  res.sum = req.a + req.b;
  //Realiza la suma de los dos números enteros que se 
  //encuentran en la solicitud del servicio.
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //Imprime un mensaje en la línea de comandos que muestra los dos números 
  //enteros que se encuentran en la solicitud del servicio.
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  //Imprime un mensaje en la línea de comandos que muestra el resultado de la suma.
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;//Crea un objeto NodeHandle, que representa la 
  //conexión del programa servidor con el core de ROS.

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  //Imprime un mensaje en la línea de comandos que indica que el 
  //programa está listo para ofrecer el servicio "add_two_ints".
  ros::spin();//va actulizando todos los nodos-topic´s
  return 0;
}

//El programa "add_two_ints_server" es un servidor de servicio ROS 
//que ofrece el servicio "add_two_ints" para sumar dos números enteros. 
//Al ejecutar este programa, el servicio "add_two_ints" estará disponible 
//para otros programas en el sistema ROS que deseen utilizarlo.