#!/usr/bin/env python3
import rospy

from TFM_Vision.srv import MensajeAnalizarDatos , MensajeAnalizarDatosResponse
from ultralytics import YOLO
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np

import torch
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor

#region CorrelacionarDatos
def CorrelacionarDatos(PosicionCamara,DatosLaser):
    """Funcion que correlaciona los datos recibidos de la camara y del laser"""
    
    global TamañoImagen

    #Obtemos la posicion de la caja
    x = PosicionCamara[0]
    w = PosicionCamara[2]
    
    #Calculamos la poscion inicial y final de la caja
    xi = int(x - w/2)
    xf = int(x + w/2)
    
    #Calculamos la correlacion con los datos del laser
    xLi = int(xi * len(DatosLaser) / TamañoImagen[1])
    xLf = int(xf * len(DatosLaser) / TamañoImagen[1])
    
    #Recortamos los datos del laser
    Medidas = DatosLaser[xLi:xLf]
    
    #Calculamos la media 
    Distancia = np.mean(Medidas)
    
    #Calculamos el angulo 
    Centro = (xLi + xLf) / 2
    Paso  = 0.334
    Angulo = 0 
    if Centro > len(DatosLaser)/2:
        Angulo = (len(DatosLaser) - Centro) * Paso
    elif Centro < len(DatosLaser)/2:
        Angulo = Centro * Paso - 80
    
    return Distancia , Angulo

#region AnalisisDeMascaras

def BuscarPuntoMasCercano(Puntos, CentroX):
    """Funcion que usaremos  para encontrar en punto mas cercano"""
    
    #Definimos las variables locales 
    Distancia = 1000000
    Punto = -1.0
    
    #Bucle para encontrar el mas cercano
    for i in range(len(Puntos)):
        if abs(Puntos[i][0] - CentroX) < Distancia:
            Punto = Puntos[i][0]
            Distancia = abs(Puntos[i][0] - CentroX)
    
    return Punto

def CalculoPuntos (mascara , altura = 400):
    """Funcion que usamos para analizar las mascaras"""
    
    #Calculamos el centro de la imagen
    CentroX = mascara.shape[1] // 2
    
    #Buscamos los contornos
    contours, _ = cv2.findContours(mascara, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #Buscamos los puntos que estan a la altura de altura
    Puntos = []
    for i in range(len(contours)):
        for j in range(len(contours[i])):
            if altura-2 <= contours[i][j][0][1] <= altura+2:
                Punto = contours[i][j][0][0]
                Puntos.append([Punto, contours[i][j][0][1]])
    
    #Ordenamos los puntos si estan a la derecha o a la izquierda
    PuntosDerecha = []
    PuntosIzquierda = []
    
    for i in range(len(Puntos)):
        if Puntos[i][0] > CentroX:
            PuntosDerecha.append(Puntos[i])
        else:
            PuntosIzquierda.append(Puntos[i])
    
    #Calculamos los puntos mas cercanos
    PuntoDerecha = BuscarPuntoMasCercano(PuntosDerecha, CentroX)
    PuntoIzquierda = BuscarPuntoMasCercano(PuntosIzquierda, CentroX)
    
    return contours, [PuntoIzquierda, PuntoDerecha]

def AnalisisDeMascaras(mask):
    "Funcion que analiza las mascaras para detectar carriles"
    
    #Calculamos los puntos a 300 de altura
    contornos, puntos300 = CalculoPuntos(mask, 300)
    #Calculamos los puntos a 400 de altura
    _ , puntos400 = CalculoPuntos(mask, 400)
    
    return contornos,[puntos300, puntos400]

#region VisualizacionImagen de las detecciones
def VisualizacionImagenDetecciones(Imagen,detecciones):
    """Funcion para publicar la imagen de las detecciones en un topic"""
    
    global pubImagenDetecciones
    
    #Dibujamos las cajas
    for deteccion in detecciones:
        x = int(deteccion[0])
        y = int(deteccion[1])
        w = int(deteccion[2])
        h = int(deteccion[3])
        clase = deteccion[6]
        confianza = deteccion[7]
        
        Imagen = np.copy(Imagen)
        Imagen.setflags(write=1)
        
        cv2.rectangle(Imagen, (x-w//2, y-h//2), (x+w//2, y+h//2), (36,255,12), 2)
        cv2.putText(Imagen, f"{clase} {confianza:0.2f}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
    
    #Creamos el mensaje de imagen
    msgImagen = Image()
    
    msgImagen.height = Imagen.shape[0]
    msgImagen.width = Imagen.shape[1]
    msgImagen.encoding = "bgr8"
    msgImagen.step = Imagen.shape[1]*3
    msgImagen.data = Imagen.tobytes()
    
    #Publicamos la imagen
    pubImagenDetecciones.publish(msgImagen)

#region VisualizacionImagen de las mascaras
def VisualizacionImagenMascaras(Imagen, contornos, puntos):
    """Funcion para publicar la imagen de las mascaras en un topic"""
    
    global pubImagenMascaras
    
    #Dibujamos los contornos
    #Imagen=cv2.cvtColor(Imagen, cv2.COLOR_GRAY2BGR)
    Imagen=Imagen.copy()
    Imagen = cv2.drawContours(Imagen, contornos, -1, (0,255,0), 3)
    
    #Dibujamos la linea 
    Imagen = cv2.line(Imagen, (0, 300), (Imagen.shape[1], 300), (0, 0, 255), 5)
    Imagen = cv2.line(Imagen, (0, 400), (Imagen.shape[1], 400), (0, 0, 255), 5)

    puntos300 = puntos[0]
    puntos400 = puntos[1]
    
    #Dibujamos los 4 puntos
    if puntos300[0] != -1.0:
        cv2.circle(Imagen, (puntos300[0], 300), 5, (255, 0, 0), -1)
    if puntos300[1] != -1.0:
        cv2.circle(Imagen, (puntos300[1], 300), 5, (255, 0, 0), -1)
    
    if puntos400[0] != -1.0:
        cv2.circle(Imagen, (puntos400[0], 400), 5, (255, 0, 0), -1)
    if puntos400[1] != -1.0:
        cv2.circle(Imagen, (puntos400[1], 400), 5, (255, 0, 0), -1)
    
    #Calculamos los centros 
    Centro300=-1
    Centro400=-1

    if puntos300[0] != -1.0  and puntos300[1] != -1.0:
        Centro300 = (puntos300[0] + puntos300[1]) // 2
    elif puntos300[0] == -1.0:
        Centro300 = (0 + puntos300[1]) // 2
    elif puntos300[1] == -1.0:
        Centro300 = (puntos300[0] + Imagen.shape[0]) // 2

    if puntos400[0] != -1.0 and puntos400[1] != -1.0:
        Centro400 = (puntos400[0] + puntos400[1]) // 2
    elif puntos400[0] == -1.0:
        Centro400 = (0 + puntos400[1]) // 2
    elif puntos400[1] == -1.0:
        Centro400 = (puntos400[0] + Imagen.shape[0]) // 2
    
    #Dibujamos los centros
    if Centro300!=-1:
        cv2.circle(Imagen, (Imagen.shape[1]//2,Imagen.shape[0]-10), 5, (0, 255, 255), -1)
        cv2.circle(Imagen, (int(Centro300), 300), 5, (0, 255, 255), -1)
    if Centro400!=-1:
        print(Centro400)
        cv2.circle(Imagen, (int(Centro400), 400), 5, (0, 255, 255), -1)
    
    #Convertimos la imagen a un mensaje
    msgImagen = Image()
    
    msgImagen.height = Imagen.shape[0]
    msgImagen.width = Imagen.shape[1]
    msgImagen.encoding = "bgr8"
    msgImagen.step = Imagen.shape[1]*3
    msgImagen.data = Imagen.tobytes()
    
    #Publicamos la imagen
    pubImagenMascaras.publish(msgImagen)


#region AnalizarDatosFuction
def AnalizarDatosFuction(request):
    """Funcion que analiza los datos recibidos y devuelve una respuesta"""
    rospy.loginfo("Peticion recibida --> Analisis en curso ...")
    #Variables globales
    global modeloDet  , modeloSeg, TamañoImagen
    
    #Obtenemos los datos de la imagen
    TamañoImagen = (request.Imagen.height, request.Imagen.width)
    ImagenDatos = np.frombuffer(request.Imagen.data, np.uint8)
    ImagenDatos = ImagenDatos.reshape(TamañoImagen[0], TamañoImagen[1] , 3)
    
    #Obtenemos los datos del laser
    LaserDatos = request.DatosLaser.ranges
    DatosLaserRecortados = (LaserDatos[907:len(LaserDatos)-1] + LaserDatos[0:240])

    distancia_laser=np.min(DatosLaserRecortados)
    
    #TODO:Analizamos los datos
    
    detecciones = []
    #Detectamos los objetos
    RespuestaModelo = modeloDet(ImagenDatos)
    #print(RespuestaModelo[0].boxes)
    for i in range(len(RespuestaModelo[0].boxes.conf)):
        conf = RespuestaModelo[0].boxes.conf[i].cpu().numpy()

        if conf>0.5:
            nombres=open("../ModelosVision/Yolo8/Models/coco.names")
            lineas=nombres.readlines()

            pos = RespuestaModelo[0].boxes.xywh[i].cpu().numpy()
        
            clase = str(lineas[int(RespuestaModelo[0].boxes.cls[i].cpu().numpy())].split('\n')[0])

            nombres.close()
        
            #Calculamos la correlacion con los datos del laser 
            Distancia , Angulo = CorrelacionarDatos( pos , DatosLaserRecortados)
        
            #Guardamos los datos
            detecciones.append([pos[0],pos[1],pos[2],pos[3],Distancia,Angulo,clase,conf])
    
    #TODO:Visualizamos la imagen
    VisualizacionImagenDetecciones(ImagenDatos,detecciones)
    
    #TODO:Responder al cliente
    #Creamos el mensaje de respuesta
    Respuesta = MensajeAnalizarDatosResponse()
    
    #Cambiamos los datos del mensaje
    for detecion in detecciones:
        Respuesta.Distancia.append(detecion[4])
        Respuesta.Angulo.append(detecion[5])
        Respuesta.Clase.append(detecion[6])

    Respuesta.dist_laser=distancia_laser   

    #Predecimos las mascaras con el modelo de MASKRCNN
    prediccion = modeloSeg(ImagenDatos)

    #Juntamos las mascaras
    mask = prediccion[0].masks
    if mask==None:
        mascara = np.zeros((480, 640), dtype=np.uint8)
    else:
        mascara = np.zeros((mask[0].shape[1], mask[0].shape[2]), dtype=np.uint8)
        for i in range(len(mask)):
            mascara+=mask[i].data.numpy()[0].astype(np.uint8)

    #Normalizamos la mascara 
    mascara = np.where(mascara > 0.5, 255, 0).astype(np.uint8)
    
    #Analisamos las mascaras
    contornos, puntos = AnalisisDeMascaras(mascara)
    
    #Cambiamos los datos del mensaje
    Respuesta.Puntos300 = puntos[0]
    Respuesta.Puntos400 = puntos[1]
    
    #Visualizamos la imagen
    VisualizacionImagenMascaras(ImagenDatos, contornos, puntos)
    
    rospy.loginfo("Analisis completado --> Respuesta enviada")
    return Respuesta

#region Main
#Inicializamos el nodo 
rospy.init_node('AnalizarDatos')
rospy.loginfo("Nodo Analizar Datos creado y corriendo")

#Creamos el publicador para visualizar la imagen
pubImagenDetecciones = rospy.Publisher('/ImagenDetecciones', Image, queue_size=10)
pubImagenMascaras = rospy.Publisher('/ImagenMascaras', Image, queue_size=10) 	
rospy.loginfo("Publicadores de imagenes creados")

#Cargamos el modelo YOLO 
modeloDet = YOLO("../ModelosVision/Yolo8/Models/yolov8n.pt")
rospy.loginfo("Modelo YOLO cargado")

#Cargamos el modelo Yolo para segmentacion 
modeloSeg = YOLO("../ModelosVision/Yolo8/Models/yolov8-seg.pt")
rospy.loginfo("Modelo YOLO cargado")

#Creamos el servidor
servidorAnalizarDatos = rospy.Service('/AnalizarDatos', MensajeAnalizarDatos , AnalizarDatosFuction)
rospy.loginfo("Servidor Analizar Datos creado y corriendo")

#Bucle de ejecucion 
rospy.loginfo("------Servidor Analizar Datos corriendo------")
rospy.spin()
