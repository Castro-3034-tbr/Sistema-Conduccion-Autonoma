#!/usr/bin/env python3
import rospy
import numpy as np
#import torch

from geometry_msgs.msg import Twist
from TFM_Vision.srv import MensajeControlarRobot , MensajeControlarRobotResponse

#region ConduccionAutonoma
def ConduccionAutonoma(Datos):
    """Funcion que analiza los datos recibidos y devuelve una respuesta"""
    global motores
    
    rospy.loginfo("Peticion recibida --> Calculo de velocidad iniciada")
    
    #Obtenemos los datos que nos llegan 
    Distancia = Datos.Distancia
    Angulo = Datos.Angulo
    distancia_laser=Datos.dist_laser
    
    puntos300 = Datos.Puntos300
    puntos400 = Datos.Puntos400
    
    #Definimos tamaño de la imagen
    altura = 480
    ancho = 640
    
    #Creamos el mensaje de control
    velocidades = Twist()
    VL = 0.075
    
    if len(Distancia)!=0:
        #Calculamos la distancia minima y calculamos el angulo que le corresponde
        DistanciaMinima = min(Distancia)
        AnguloMinimo = Angulo[Distancia.index(DistanciaMinima)]
    
        if DistanciaMinima < 0.5:
            if 5.76 < AnguloMinimo < 6.28  or 0 < AnguloMinimo < 0.52:
                VL = 0.0
    
    if distancia_laser<0.25:
        VL=0.0


    #TODO: Control para mantenernos en el carril
    
    #Calculamos los centros 
    Centro = ancho // 2
    
    if puntos300[0] != -1.0  and puntos300[1] != -1.0:
        Centro300 = (puntos300[0] + puntos300[1]) // 2
    elif puntos300[0] == -1.0:
        Centro300 = (0 + puntos300[1]) // 2
    elif puntos300[1] == -1.0:
        Centro300 = (puntos300[0] + ancho) // 2

    if puntos400[0] != -1.0 and puntos400[1] != -1.0:
        Centro400 = (puntos400[0] + puntos400[1]) // 2
    elif puntos400[0] == -1.0:
        Centro400 = (0 + puntos400[1]) // 2
    elif puntos400[1] == -1.0:
        Centro400 = (puntos400[0] + ancho) // 2

    centros = [Centro300, Centro400]
    
    #Calculamos los anglos 
    Angulo300 = np.arctan2((Centro - centros[0]), (altura-300))
    Angulo400 = np.arctan2((Centro - centros[1]),(altura-400))

    #Calculamos las velocidades 
    Kp300 = 2
    Kp400 = 1.5
    V300 = Kp300 * Angulo300
    V400 = Kp400 * Angulo400
    VA = (V300 + V400)/2

    #Imprimimos la informacion
    rospy.loginfo("Angulos:\n\t Angulo 300 ", Angulo300 , "\n\t Angulo 400 ", Angulo400)
    rospy.loginfo("Velocidades:\n\t  Linear: ", VL, "\n\t Angular: ", VA)

    #Actualizamos los valores de las velocidades
    velocidades.linear.x = VL
    velocidades.angular.z = VA
    
    motores.publish(velocidades)
    
    rospy.loginfo("Calculo acabado --> Velocidades enviadas")
    
    return MensajeControlarRobotResponse()


#region Main

#Creacion del nodo 
rospy.init_node('ConduccionAutonoma')
rospy.loginfo("Nodo Conduccion Autonoma creado y corriendo")

#Creacion del servidor
servidorConduccionAutonoma = rospy.Service('/ConduccionAutonoma', MensajeControlarRobot, ConduccionAutonoma)
rospy.loginfo("Servidor Conduccion Autonoma creado y corriendo")


#Creacion del controlador del robot
motores = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.loginfo("Publicador de velocidades creado")

#Bucle de ejecucion
rospy.loginfo("------Servidor Conduccion Autonoma corriendo------")
rospy.spin()

