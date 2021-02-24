## Simulador de algoritmos de control implementado en Ros-Webots##
## Para mayor informacion visitar #

import cv2
from controller import *
import numpy as np
from numpy.linalg import inv
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from scipy.spatial.transform import Rotation as R
# Declaracion de variables globales para la comunicacion
vx_c=0
wz_c=0

# Definicion de las constantes del sistema radio de las ruedas y distancia entre ruedas
r = 0.07 / 2
L = 0.24

def velocityCallback(velocity_message):
    global vx_c
    global wz_c

    vx_c=velocity_message.linear.x
    wz_c=velocity_message.angular.z

def conversion(v,r,L):
    T=np.array([[r/2,r/2],[r/L,-r/L]])
    tranformacion_ruedas=np.linalg.inv(T)@v
    return tranformacion_ruedas[0,0],tranformacion_ruedas[1,0]

def conversion_u(v,r,L):
    T=np.array([[r/2,r/2],[r/L,-r/L]])
    tranformacion_ruedas=T@v
    return tranformacion_ruedas[0,0],tranformacion_ruedas[1,0]

def tranformacion_cordenadas(x,y,z,phi):
    T1=np.array([[np.cos(phi/2),-np.sin(phi/2),0],[np.sin(phi/2),np.cos(phi/2),0],[0,0,1]])
    T2=np.array([[1,0,0],[0,np.cos(phi/2),-np.sin(phi/2)],[0,np.sin(phi/2),np.cos(phi/2)]])
    relativo=np.array([[x],[y],[z]])
    real=T1@T2@relativo
    return real[0,0],real[1,0],real[2,0]

def infinito(lista):
    for n,i in enumerate(lista):
        if i==2.0:
            lista[n]=float('inf')
    invert = lista[::-1]
    return invert
def envio_odometria(gps,imu,wheels):
    
    # lectura de los datos generados por el gps
    position = gps.getValues()
    # Transformacion de los valores del sensor a los valores reales
    x_real, y_real, z_real = tranformacion_cordenadas(position[0], position[1], position[2], np.pi)

    # Obtener los valores de los angulos del robot mobil
    roll = -imu.getRollPitchYaw()[0]
    pitch = -imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]
    quat=R.from_euler('z',yaw,degrees=False)
    rotacion=quat.as_quat()
    # tranformacion de los angulos a quaternios
    # Lectura de las velocidades de las ruedas para aplicar al tranformacion a velocidades generales del sistema
    w_r_real = wheels[0].getVelocity()
    w_l_real = wheels[1].getVelocity() * (-1)

    # generacion de las velocidades generales a partir de las ruedas
    reales = np.array([[w_r_real], [w_l_real]])
    u_general, w_general = conversion_u(reales, r, L)
    # lectura de datos del sensor lidar

    # Envio de datos por ros en el packete de odometria
    odometry_message.pose.pose.position.x = x_real
    odometry_message.pose.pose.position.y = y_real
    odometry_message.pose.pose.position.z = z_real

    odometry_message.pose.pose.orientation.x = rotacion[0]
    odometry_message.pose.pose.orientation.y = rotacion[1]
    odometry_message.pose.pose.orientation.z = rotacion[2]
    odometry_message.pose.pose.orientation.w = rotacion[3]

    odometry_message.twist.twist.linear.x = u_general
    odometry_message.twist.twist.linear.y = 0
    odometry_message.twist.twist.linear.z = 0

    odometry_message.twist.twist.angular.x = 0
    odometry_message.twist.twist.angular.y = 0
    odometry_message.twist.twist.angular.z = w_general

    # Envio de datos a travez de ros
    rospy.loginfo("Enviando Odometria..")
    odometry_publisher.publish(odometry_message)
    return None

def cmd(wheels):
    # Lectura de las velocidades generales deseadas del sistema
    u_sp = vx_c
    w_sp = wz_c
    v = np.array([[u_sp], [w_sp]])

    # OBTENER EL EQUIVALENTE EN VELOCIDAD DE CADA RUEDA
    w_r, w_l = conversion(v, r, L)
    # Accion sobre las ruedas
    wheels[0].setVelocity(w_r)
    wheels[1].setVelocity(w_l * (-1))
    return None
def envio_lidar(lidar_id):
    # Lectura de datos del sensor lidar
    lectura_lidar = infinito(lidar_id[0].getRangeImage())
    scan_message.angle_max= 6.2831898
    scan_message.angle_min = 0
    scan_message.angle_increment = 0.0175
    scan_message.range_min = 0.01
    scan_message.range_max = 2
    scan_message.ranges=lectura_lidar
    scan_message.intensities=[0]*len(lectura_lidar)
    rospy.loginfo("Enviando valores del laser")
    scan_publisher.publish(scan_message)


def bucle(robot,odometry_publisher,odometry_message, scan_publisher,scan_message):
    # Obtencion del tiempo minimo para la simulacion
    timestep = int(robot.getBasicTimeStep())

    # Definicion de los nombres de las ruedas
    wheels = []
    wheelsNames = ['whel1_joint', 'whel2_joint']
    # Bucle para poder acceder a las ruedas del sistema
    for i in range(2):
        wheels.append(robot.getMotor(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(0.0)

    # Definicion del gps del sistema
    gps = GPS("gps")
    gps.enable(timestep)

    # Definicion de la imu del sistema
    imu = InertialUnit("inertial unit")
    imu.enable(timestep)

    # Defincion del lidar del robot
    lidar_id = []
    lidar = ['lidar']
    for i in range(1):
        lidar_id.append(robot.getLidar(lidar[i]))
        lidar_id[i].enable(timestep)
        lidar_id[i].enablePointCloud()


    # Deficion de el sample time de las simulacion
    loop_rate = rospy.Rate(200)

    while robot.step(timestep)!=-1:
        current_time = rospy.get_time()
        # envio datos de odometria del robot
        envio_odometria(gps, imu, wheels)
        # envio de datos del lidar del robot a los nodos ros
        envio_lidar(lidar_id)
        # envio de comandos a los motores
        cmd(wheels)
        # especificar el timepo de muestreo en hz
        last_time=current_time
        loop_rate.sleep()

    return None

if __name__=='__main__':
    try:
        # Inicializacion del nodo
        rospy.init_node('Webots_simulador', anonymous=True)

        # Topico para escuchar los comandos de las velocidades
        velocidad_topic = '/cmd_vel'
        pose_subscriber = rospy.Subscriber(velocidad_topic, Twist, velocityCallback)

        #Topico para publicar la odometria del robot
        odometria_topic = '/odom'
        odometry_message=Odometry()
        odometry_publisher = rospy.Publisher(odometria_topic, Odometry, queue_size=10)

        scan_topic = '/scan'
        scan_message=LaserScan()
        scan_publisher = rospy.Publisher(scan_topic,LaserScan, queue_size=50)

        # generar el objeto del robot
        robot=Robot()

        # Definir el bucle de simulacin
        bucle(robot, odometry_publisher, odometry_message, scan_publisher, scan_message)

    except KeyboardInterrupt:
        print("Press Ctrl-C to terminate while statement")
        pass
