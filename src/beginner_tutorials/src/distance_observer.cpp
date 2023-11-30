#include <ros/ros.h>  // Incluye la librería de ROS (Robot Operating System).
#include <turtlesim/Pose.h>  // Incluye la definición del mensaje de posición de turtlesim.
#include <beginner_tutorials/ComputeDistance.h>  // Incluye la definición del servicio ComputeDistance definido en el paquete beginner_tutorials.
#include <cmath>  // Incluye la librería para funciones matemáticas.

float turtle1_x, turtle1_y, turtle2_x, turtle2_y;  // Variables globales para almacenar las posiciones de las dos tortugas.

// Callback que se ejecuta cada vez que se recibe un mensaje de posición de la tortuga1.
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr &pose) {
    turtle1_x = pose->x;
    turtle1_y = pose->y;
}

// Callback que se ejecuta cada vez que se recibe un mensaje de posición de la tortuga2.
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr &pose) {
    turtle2_x = pose->x;
    turtle2_y = pose->y;
}

// Función principal del nodo.
int main(int argc, char **argv) {
    ros::init(argc, argv, "distance_observer_node");  // Inicializa el nodo con el nombre "distance_observer_node".
    ros::NodeHandle nh;  // Crea un objeto NodeHandle para manejar la comunicación con ROS.

    ros::Subscriber sub1 = nh.subscribe("/turtle1/pose", 1000, turtle1PoseCallback);  // Suscribe la función turtle1PoseCallback al topic /turtle1/pose.
    ros::Subscriber sub2 = nh.subscribe("/turtle2/pose", 1000, turtle2PoseCallback);  // Suscribe la función turtle2PoseCallback al topic /turtle2/pose.

    ros::ServiceClient distanceClient = nh.serviceClient<beginner_tutorials::ComputeDistance>("calculate_distance");  // Cliente para llamar al servicio calculate_distance.
    ros::service::waitForService("calculate_distance");  // Espera a que el servicio calculate_distance esté disponible.

    beginner_tutorials::ComputeDistance distanceSrv;  // Objeto de tipo ComputeDistance para almacenar la solicitud y la respuesta.

    ros::Rate loop_rate(2.0);  // Frecuencia del bucle principal.

    while (ros::ok()) {
        distanceSrv.request.x1 = turtle1_x;  // Establece la posición x de la tortuga1 en la solicitud.
        distanceSrv.request.y1 = turtle1_y;  // Establece la posición y de la tortuga1 en la solicitud.
        distanceSrv.request.x2 = turtle2_x;  // Establece la posición x de la tortuga2 en la solicitud.
        distanceSrv.request.y2 = turtle2_y;  // Establece la posición y de la tortuga2 en la solicitud.

        if (distanceClient.call(distanceSrv)) {
            ROS_INFO("Distance between turtles: %.2f", distanceSrv.response.distance);  // Imprime la distancia calculada entre las tortugas.
        } else {
            ROS_ERROR("Failed to call service 'calculate_distance'");  // Imprime un mensaje de error si no se pudo llamar al servicio.
        }

        ros::spinOnce();  // Va actulizando todos los nodos-topic´s
        loop_rate.sleep();  // Espera según la frecuencia especificada.
    }

    return 0;
}
