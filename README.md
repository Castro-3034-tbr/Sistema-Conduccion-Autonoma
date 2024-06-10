# Trabajo de fin materia basado en una prueba de un sistema de conduccion 
---

# Sistema de Conducción Autónoma

Este repositorio contiene el código para un sistema de conducción autónoma desarrollado como parte del proyecto final de la materia de Visión Artificial Avanzada del grado de robotica de la USC. El sistema está implementado utilizando ROS (Robot Operating System) y con la ayuda del modelo YOLOv8 para la detección y segmentación de objetos.

## Descripción

El proyecto tiene como objetivo crear una prueba de un sistema de conducción autónoma implementado en un robot Waveshare JetRacer ROS AI. El sistema utiliza una cámara monocular y un sensor RPLIDAR para la percepción del entorno. El NVIDIA Jetson Nano se utiliza para ejecutar los modelos de IA y gestionar los datos de los sensores.

## Características

- **Detección de Objetos**: Utiliza YOLOv8 para la detección de objetos en tiempo real, para poder evitar los objetos que se encuentran delante del robot 
- **Segmentación de Carriles**: Implementa YOLOv8 para la segmentación de imágenes para la identificación de marcas viales, en cocreto las lineas del carril.
- **Integración con ROS**: Aprovecha ROS para la gestión de datos de sensores y el control de actuadores.
- **Estructura de Código Modular**: Divide la funcionalidad en componentes modulares para facilitar el mantenimiento y la escalabilidad.

## Componentes

### Hardware

- Waveshare JetRacer ROS AI
- NVIDIA Jetson Nano
- Cámara Monocular
- Sensor RPLIDAR

### Software

- ROS (Robot Operating System)
- YOLOv8 para la detección y segmentación de objetos
- Scripts personalizados en Python para la gestión de datos de sensores y el control de actuadores

## Instalación

1. **Clonar el repositorio:**

    ```bash
    git clone https://github.com/yourusername/sistema-conduccion-autonoma.git
    cd sistema-conduccion-autonoma
    ```

2. **Instalar ROS:**

    Sigue las instrucciones en el [sitio web de ROS](http://wiki.ros.org/ROS/Installation) para instalar ROS en tu sistema.

3. **Instalar dependencias:**

    ```bash
    sudo apt-get update
    sudo apt-get install python3-pip
    pip3 install -r requirements.txt
    ```

4. **Configurar el espacio de trabajo de Catkin:**

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    ```

5. **Copiar el paquete en el espacio de trabajo de Catkin:**

    ```bash
    cp -r /path/to/sistema-conduccion-autonoma ~/catkin_ws/src/
    cd ~/catkin_ws/
    catkin_make
    ```
6. **Instalación de opencv**
   ``` bash
   pip install opencv-python
   ```
8. ** Instalación de ultralytics **
``` bash
pip install ultralytics
```
9. ** Instalacion del servidor para ver las imagenes
   ``` bash
   sudo apt-get install ros-melodic-web-video-server
   ```
## Uso

1. **Lanzar los nodos de ROS:**

   Inicio de los sensores del robot 

    ```bash
    #Incio del RPLIDAR
    roslaunch jetracer lidar.launch

    #Incio de la camara
    roslaunch jetracer csi_camera.launch
    ```

    Inicio de los motores y del nodo para mover el robot
   ```bash
   roslaunch jetracer jetracer.launch
   ```

    Inicio del servidor para publicar las imagenes
   ```bash
   rosrun web_video_server web_video_server
   ```
   

1. **Ejecutar los codigos necesarios**

    ```bash
    cd catkin_ws\src\sistema_conduccion_autonoma
    python3 main.py
    python3 AnalizarDatos.py 
    python3 ConduccionAutonoma.py
    ```


## Estructura del Código

- `src/`: Contiene los scripts principales en Python para los nodos de ROS.
  - `main.py`: Inicializa los nodos de ROS y gestiona el flujo de datos entre los sensores y los actuadores.
  - `AnalizarDatos.py`: Analizza todos los datos captados por los sensores.
  - `segmentation_node.py`: Calcula las velocidades necesarias y las publicas en el topic necesario .
- `srv/`: Contiene los mensajes necesario para los diferentes servicio.
  - `MensajeAnalizarDatos.srv`: Mensaje de activacion para el servicio de analizar datos .
  - `MensajeControlarRobot.srv`: Mensaje de activacion para el servicio de conduccion autonoma .
- `config/`: Archivos de configuración para los parámetros de ROS.
- `models/`: Contine todo los necesario para entrenar y guardar los medelos que usamos

---
#Resultados 
- ** Entramiento del modelo de segmentacion** 
- **Segmentación de Carriles**: El sistema segmenta con precisión las marcas viales, facilitando la navegación autónoma.
- **Rendimiento del Sistema**: El robot demuestra capacidades robustas de conducción autónoma en varios escenarios de prueba.

## Conclusión

Este proyecto demuestra el uso efectivo de ROS y YOLOv8 en el desarrollo de un sistema de conducción autónoma. El enfoque modular garantiza que el sistema sea mantenible y escalable para futuras mejoras.
