#!/usr/bin/env python3
import rospy
import torch
from sensor_msgs.msg import LaserScan, Image
from time import sleep

from TFM_Vision.srv import MensajeAnalizarDatos , MensajeAnalizarDatosRequest , MensajeAnalizarDatosResponse
from TFM_Vision.srv import MensajeControlarRobot , MensajeControlarRobotRequest, MensajeControlarRobotResponse

#region Callbacks
def CallbackCamara(msg):
    """Funcion que se ejecuta cada vez que se recibe un mensaje de la camara"""
    #Variables globales
    global CamaraRead , Imagen
    
    #Marcamos que hemos recibido el primer mensaje de la camara
    CamaraRead = True
    
    #Guardamos los datos en una variable global
    Imagen = msg

#region CallbackLaser
def CallbackLaser(msg):
    """Funcion que se ejecuta cada vez que se recibe un mensaje del laser"""
    #Variables globales 
    global LaserRead , DatosLaser
    
    #Marcamos que hemos recibido el primer mensaje del laser
    LaserRead = True
    
    #Guardamos los datos en una variable global
    DatosLaser = msg

#region Main

#Creacion del nodo 
rospy.init_node('Main')
rospy.loginfo("Nodo Main creado y corriendo")

#Conexion con el servidor de Analizar Datos 
rospy.wait_for_service("/AnalizarDatos")
AnalizarDatos = rospy.ServiceProxy('/AnalizarDatos', MensajeAnalizarDatos)
rospy.loginfo("Conexion con el servidor de Analizar Datos establecida")

#Conexion con el servidor de Controlar Robot
rospy.wait_for_service("/ConduccionAutonoma")
ControlarRobot = rospy.ServiceProxy('/ConduccionAutonoma', MensajeControlarRobot)
rospy.loginfo("Conexion con el servidor de Controlar Robot establecida")


#Creacion del subscriptor del laser 
LaserRead= False
DatosLaser = LaserScan()
laser = rospy.Subscriber('/scan', LaserScan, CallbackLaser)
while not LaserRead:
    sleep(1)
rospy.loginfo("Subscriptor del laser creado y corriendo")

#Creacion del subcrito de la camara
CamaraRead = False
Imagen = Image()
camara = rospy.Subscriber('/csi_cam_0/image_raw', Image, CallbackCamara)
while not CamaraRead:
    sleep(1)
rospy.loginfo("Subscriptor de la camara creado y corriendo")

#Creamos el rate
rate = rospy.Rate(2)

#Bucle principal
rospy.loginfo("------Nodo Main corriendo------")

#region Bucle principal
while not rospy.is_shutdown():
    
    #TODO: Parte de deteccion de objetos
    
    #Creamos el mensaje 
    Mensaje = MensajeAnalizarDatosRequest()
    
    #Cambiamos los datos del mensaje
    Mensaje.DatosLaser = DatosLaser
    Mensaje.Imagen = Imagen
    
    #Enviamos el mensaje al servidor de Analizar Datos
    rospy.loginfo("Datos recibidos --> Enviando al servidor de Analizar Datos")
    RespuestaAnalizarDatos = AnalizarDatos(Mensaje)
    rospy.loginfo("Datos recibidos")
    
    #TODO: Parte de control del robot
    #Creamos el mensaje
    Mensaje = MensajeControlarRobotRequest()
    
    #Añadimos los datos al mensaje
    Mensaje.Distancia = RespuestaAnalizarDatos.Distancia
    Mensaje.Angulo = RespuestaAnalizarDatos.Angulo
    Mensaje.Clase = RespuestaAnalizarDatos.Clase
    Mensaje.Puntos300=RespuestaAnalizarDatos.Puntos300
    Mensaje.Puntos400=RespuestaAnalizarDatos.Puntos400
    Mensaje.dist_laser=RespuestaAnalizarDatos.dist_laser
    
    #Enviamos el mensaje al servidor de Controlar Robot
    rospy.loginfo("Enviando al servidor de Controlar Robot")
    ControlarRobot(Mensaje)
    
    #Esperamos un tiempo para no saturar el procesador
    rate.sleep()
    


