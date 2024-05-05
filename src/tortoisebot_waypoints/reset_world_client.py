#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty  # Importar el tipo de mensaje de servicio apropiado

def reset_world_client():
    rospy.wait_for_service('/gazebo/reset_world')  # Esperar a que el servicio esté disponible
    try:
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)  # Crear un cliente del servicio
        reset_world()  # Llamar al servicio sin pasar argumentos
        rospy.loginfo("World reset request sent successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('reset_world_client')  # Inicializar el nodo
    reset_world_client()  # Llamar a la función para enviar la sol