# Lab Aprendiendo Docker, ROS, y Robótica 

**Autores:** Oscar Grande — Didier Posse
**Fecha:** Octubre 2025

Este laboratorio documenta la implementación y simulación de sistemas robóticos complejos utilizando **Docker** para garantizar un entorno consistente y reproducible. El proyecto se centra en la aplicación práctica de herramientas esenciales en robótica, el Internet de las Cosas (IoT) y la Industria 4.0.

---

## 🛠️ Herramientas Fundamentales

El documento se enfoca en el desarrollo y la integración de las siguientes herramientas de software y tecnologías de sensado.

### 1. Desarrollo de Software para Robótica

| Herramienta | Concepto Clave | Funcionalidad Principal |
| :--- | :--- | :--- |
| **ROS (Robot Operating System)** | Framework de software (Middleware) para el desarrollo y control de robots. | Facilita la comunicación modular entre componentes (nodos) mediante topics, servicios o acciones. |
| **MoveIt!** | Plataforma de ROS especializada en planificación, control y manipulación de robots. | Se encarga de la **planificación de trayectorias** y la **evitación de colisiones** de forma inteligente y segura. |
| **Gazebo** | Simulador 3D de robots y entornos físicos. | Crea un **entorno virtual con física realista** (gravedad, fricción, colisiones) para probar algoritmos de control sin hardware. |

### 2. Tecnologías de Sensado y Navegación

| Tecnología | Definición | Aplicación Clave en Robótica/IoT |
| :--- | :--- | :--- |
| **LIDAR** | Tecnología que utiliza pulsos láser para medir distancias y construir un **mapeo 3D detallado** del entorno. | Envía datos de distancias y formas a la nube o servidor IoT, permitiendo la toma de decisiones coordinadas por otros dispositivos. |
| **SLAM** (Localización y Mapeo Simultáneo) | Proceso por el cual un robot construye un mapa de su entorno mientras calcula su propia posición en él, sin depender de GPS. | Los mapas generados pueden **compartirse con otros dispositivos por red**, facilitando la cooperación entre múltiples robots y el almacenamiento en la nube. |

---

## Implementación y Dockerización

El laboratorio demuestra la ejecución de dos simulaciones completas encapsuladas en contenedores Docker, lo que asegura la portabilidad y elimina problemas de dependencias entre sistemas operativos.

### 1. Simulación de Cuadrúpedo con PyBullet

Se implementa un robot cuadrúpedo virtual utilizando la librería **PyBullet**, un motor de física para robótica y simulación.

* **Archivos de Configuración:** `minotaur.py` (código de simulación), `requirements.txt` (librerías usadas), y `Dockerfile`.

* **Proceso de Despliegue Dockerizado (GUI):**
    * La simulación se ejecuta con Docker, configurado para mostrar la interfaz gráfica (GUI) de PyBullet en el sistema anfitrión.
    * Esto se logra mediante el mapeo del socket **X11** (`-v /tmp/.X11-unix:/tmp/.X11-unix`) y la exportación de la variable de entorno **DISPLAY** (`--env="DISPLAY"`).

* **Ejemplo de Comandos:**
    ```bash
    # 1. Construir la imagen (Asumiendo que el nombre es 'cuadrupedo-sim')
    docker build -t cuadrupedo-sim .

    # 2. Ejecutar el contenedor con soporte para GUI
    docker run -it --rm --env="DISPLAY" -v/tmp/.X11-unix:/tmp/.X11-unix --name cuadrupedo_container cuadrupedo-sim
    ```

### 2. Simulación de TurtleBot3 con LIDAR y SLAM

Esta aplicación integra ROS (Noetic), Gazebo, y el algoritmo SLAM (`gmapping`) para el mapeo.

* **Entorno:** Se basa en una imagen ROS **Noetic** e incluye paquetes esenciales como `turtlebot3-simulations`, `slam-gmapping`, y `turtlebot3-navigation`.
* **Archivos Clave:** `Turtlebot.py` (código de control/lógica) y `Dockerfile`.

#### Pasos de Ejecución (en contenedores)

Para interactuar con la simulación, se utiliza el patrón de **múltiples terminales** para ejecutar diferentes nodos de ROS dentro del mismo contenedor que aloja la simulación de Gazebo.

1.  **Construcción y Ejecución de Gazebo:**
    Se construye la imagen (`turtlebot3-sim`) y se ejecuta el contenedor para lanzar la simulación del TurtleBot3 en Gazebo.
    ```bash
    # Ejecución de la simulación base en Gazebo (con soporte para GUI y red)
    sudo docker run -it --rm --env="DISPLAY" --env="TURTLEBOT3_MODEL=burger" --network=host -v/tmp/.X11-unix:/tmp/.X11-unix --name turtlebot3-sim turtlebot3-sim
    ```
2.  **Lanzamiento del Mapeo SLAM (Gmapping):**
    En una terminal separada, se utiliza `docker exec` para iniciar el nodo que procesa los datos simulados del LIDAR y construye el mapa.
    ```bash
    # Ejecución del nodo SLAM para mapeo dentro del contenedor
    sudo docker exec -it turtlebot3-sim bash -c "source /opt/ros/noetic/setup.bash && roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping"
    ```
3.  **Control del Robot (Teleoperación):**
    En una tercera terminal, se ejecuta el nodo de teleoperación para controlar el movimiento del robot mediante teclado, alimentando los datos al proceso SLAM.
    ```bash
    # Ejecución del control por teclado para mover el TurtleBot3
    sudo docker exec -it turtlebot3-sim bash -c "source /opt/ros/noetic/setup.bash && rosrun turtlebot3_teleop turtlebot3_teleop_key"
    ```

El resultado final es una simulación robusta que permite la **visualización, el mapeo dinámico con detección de obstáculos** y el control en tiempo real del TurtleBot3.
