# Sistema-Conduccion-Autonoma
---

# Sistema de Conducción Autónoma

Este repositorio contiene el código para un sistema de conducción autónoma desarrollado como parte del proyecto final de la materia de Visión Artificial Avanzada. El sistema está implementado utilizando ROS (Robot Operating System) y YOLOv8 para la detección y segmentación de objetos.

## Descripción

El proyecto tiene como objetivo crear un sistema de conducción autónoma implementado en un robot Waveshare JetRacer ROS AI. El sistema utiliza una cámara monocular y un sensor RPLIDAR para la percepción del entorno. El NVIDIA Jetson Nano se utiliza para ejecutar los modelos de IA y gestionar los datos de los sensores.

## Características

- **Detección de Objetos**: Utiliza YOLOv8 para la detección de objetos en tiempo real.
- **Segmentación de Carriles**: Implementa YOLOv8 para la segmentación de imágenes y la identificación de marcas viales.
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

## Uso

1. **Lanzar los nodos de ROS:**

    Iniciar el nodo maestro de ROS:

    ```bash
    roscore
    ```

    En una nueva terminal, navega al espacio de trabajo de Catkin y lanza el paquete:

    ```bash
    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch sistema_conduccion_autonoma main.launch
    ```

2. **Ejecutar los nodos de detección y segmentación:**

    ```bash
    rosrun sistema_conduccion_autonoma detection_node.py
    rosrun sistema_conduccion_autonoma segmentation_node.py
    ```

3. **Visualizar los resultados:**

    Usa RViz para visualizar los datos de los sensores y los resultados de la detección:

    ```bash
    rviz
    ```

## Estructura del Código

- `src/`: Contiene los scripts principales en Python para los nodos de ROS.
  - `main.py`: Inicializa los nodos de ROS y gestiona el flujo de datos entre los sensores y los actuadores.
  - `detection_node.py`: Gestiona la detección de objetos utilizando YOLOv8.
  - `segmentation_node.py`: Gestiona la segmentación de carriles utilizando YOLOv8.
- `launch/`: Contiene archivos de lanzamiento de ROS.
  - `main.launch`: Archivo de lanzamiento para iniciar todos los nodos necesarios.
- `config/`: Archivos de configuración para los parámetros de ROS.
- `models/`: Modelos de YOLOv8 para detección y segmentación.

## Resultados

- **Detección de Objetos**: El modelo YOLOv8 detecta varios objetos con alta precisión.
- **Segmentación de Carriles**: El sistema segmenta con precisión las marcas viales, facilitando la navegación autónoma.
- **Rendimiento del Sistema**: El robot demuestra capacidades robustas de conducción autónoma en varios escenarios de prueba.

## Conclusión

Este proyecto demuestra el uso efectivo de ROS y YOLOv8 en el desarrollo de un sistema de conducción autónoma. El enfoque modular garantiza que el sistema sea mantenible y escalable para futuras mejoras.
