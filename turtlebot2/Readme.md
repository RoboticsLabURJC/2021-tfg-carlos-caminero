# Turtlebot 2 Simulado (Versión en ROS Foxy)
## Introducción
Esta es una versión exportada a mano para ROS Foxy del robot Turtlebot 2 de Yujin.  
El robot posee 2 sensores que se pueden quitar o colocar a gusto del usuario a través del fichero [./turtlebot2/urdf/turtlebot2.urdf.xacro](https://github.com/RoboticsLabURJC/2021-tfg-carlos-caminero/blob/main/turtlebot2/turtlebot2/urdf/turtlebot2.urdf.xacro):

* Una cámara RGBD simulada para obtener una **nube de puntos** de la escena captada en cada instante de tiempo.
* Un LIDAR simulado de (360 º) para determinar la distancia con los obstáculos que rodean al robot.

![](https://github.com/RoboticsLabURJC/2021-tfg-carlos-caminero/blob/main/docs/images/turtlebot2-sim-v2.png)

## Modo de uso.
Una vez que compilemos el repositorio, podrás probar el Turtlebot2 simulado lanzando cada uno de estos comandos en una terminal distinta (el último comando es opcional):
~~~
ros2 launch turtlebot2 empty_world.launch.py
ros2 launch turtlebot2 spawn_model.launch.py
ros2 launch kobuki_keyop kobuki_keyop.launch.py
~~~

