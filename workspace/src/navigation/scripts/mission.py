#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import math

class MisionRescateLabyrinth(Node):
    def __init__(self):
        super().__init__('mision_rescate_nodo')
        
        # Suscriptor al láser para detectar el "Punto B"
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',  # Asegúrate de que este sea el tópico 2D de tu Unitree L2
            self.scan_callback,
            10
        )
        
        # Inicializar el comandante de Nav2
        self.nav = BasicNavigator()
        
        # Estados de la misión
        self.ESTADO_EXPLORANDO = 0
        self.ESTADO_PUNTO_B_ENCONTRADO = 1
        self.ESTADO_REGRESANDO = 2
        
        self.estado_actual = self.ESTADO_EXPLORANDO
        self.punto_b_detectado = False

        # Configuración de detección de la "U" (Muros a los lados y al frente)
        self.distancia_umbral = 0.6  # Metros. Si hay paredes a menos de 60cm en las 3 direcciones, es el Punto B
        
        # Punto A (Inicio). Asumimos que el mapa de RTAB-Map inicia en (0,0)
        self.punto_a = PoseStamped()
        self.punto_a.header.frame_id = 'map'
        self.punto_a.pose.position.x = 0.0
        self.punto_a.pose.position.y = 0.0
        self.punto_a.pose.orientation.w = 1.0 # Mirando al frente
        
        self.get_logger().info("Nodo de Misión Iniciado. Esperando a Nav2...")
        self.nav.nav_to_pose_client.wait_for_server()
        #self.nav.waitUntilNav2Active(localizer='controller_server', navigator='bt_navigator')
        self.get_logger().info("Nav2 Activo. ¡Iniciando exploración!")

        # Timer principal del bucle de la misión (1 Hz)
        self.timer = self.create_timer(1.0, self.bucle_mision)

    def scan_callback(self, msg):
        # Solo buscamos el Punto B si estamos explorando
        if self.estado_actual != self.ESTADO_EXPLORANDO:
            return

        # El arreglo 'ranges' tiene todas las distancias. 
        # Calculamos los índices para Frente (0°), Izquierda (90°) y Derecha (-90°)
        num_lecturas = len(msg.ranges)
        indice_frente = int(num_lecturas / 2)
        indice_izquierda = int(num_lecturas * 0.75)
        indice_derecha = int(num_lecturas * 0.25)

        dist_frente = msg.ranges[indice_frente]
        dist_izq = msg.ranges[indice_izquierda]
        dist_der = msg.ranges[indice_derecha]

        # Ignorar valores infinitos (fuera de rango)
        if math.isinf(dist_frente) or math.isinf(dist_izq) or math.isinf(dist_der):
            return

        # LÓGICA DE DETECCIÓN: Trampa geométrica (Punto B)
        if (dist_frente < self.distancia_umbral and 
            dist_izq < self.distancia_umbral and 
            dist_der < self.distancia_umbral):
            
            self.get_logger().info("¡PUNTO B DETECTADO! (Muro en U)")
            self.punto_b_detectado = True
            self.estado_actual = self.ESTADO_PUNTO_B_ENCONTRADO

    def bucle_mision(self):
        if self.estado_actual == self.ESTADO_EXPLORANDO:
            # Aquí el nodo 'explore_lite' debería estar mandando comandos.
            # Nuestro script solo observa el láser pasivamente.
            pass

        elif self.estado_actual == self.ESTADO_PUNTO_B_ENCONTRADO:
            self.get_logger().info("Cancelando exploración. Tomando control manual...")
            
            # 1. Cancelar cualquier ruta que esté haciendo explore_lite o Nav2
            self.nav.cancelTask()
            
            # 2. Accionar el flipper
            self.tocar_muro_con_flipper()
            
            # 3. Cambiar estado a regresar
            self.estado_actual = self.ESTADO_REGRESANDO
            
            # 4. Enviar a Nav2 la orden de regresar al Punto A
            self.get_logger().info("Iniciando ruta de regreso al Punto A...")
            self.nav.goToPose(self.punto_a)

        elif self.estado_actual == self.ESTADO_REGRESANDO:
            # Revisar si ya llegamos al Punto A
            if self.nav.isTaskComplete():
                resultado = self.nav.getResult()
                if resultado == TaskResult.SUCCEEDED:
                    self.get_logger().info("¡Misión Cumplida! Hemos vuelto al Punto A.")
                    
                    # Preparar para el siguiente ciclo
                    self.get_logger().info("Girando el robot para prepararse...")
                    # Aquí podrías usar self.nav.spin() para girar 180 grados
                    time.sleep(3)
                    
                    self.get_logger().info("Reiniciando ciclo de exploración.")
                    self.punto_b_detectado = False
                    self.estado_actual = self.ESTADO_EXPLORANDO
                    # IMPORTANTE: Aquí tendrías que volver a encender explore_lite
                else:
                    self.get_logger().error("Nav2 falló intentando regresar al Punto A.")

    def tocar_muro_con_flipper(self):
        self.get_logger().info(">>> ACTIVANDO FLIPPER FRONTAL <<<")
        # --- Aquí va el código para comunicarte con tus motores ---
        # Ejemplo: publicar en un tópico /flipper_cmd un ángulo de 90 grados
        time.sleep(2) # Simula el tiempo que tarda el flipper en subir y bajar
        self.get_logger().info(">>> FLIPPER RETRAÍDO <<<")

def main(args=None):
    rclpy.init(args=args)
    nodo_mision = MisionRescateLabyrinth()
    
    try:
        rclpy.spin(nodo_mision)
    except KeyboardInterrupt:
        pass
    finally:
        nodo_mision.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()