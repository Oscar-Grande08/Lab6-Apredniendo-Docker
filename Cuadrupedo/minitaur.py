#!/usr/bin/env python3
"""
Simulación simplificada del robot cuadrúpedo Minitaur usando PyBullet
No requiere el repositorio completo de motion_imitation
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Configuración del Minitaur
NUM_LEGS = 4
NUM_MOTORS = 8  # 2 motores por pata
MOTOR_NAMES = [
    "motor_front_leftL_joint",
    "motor_front_leftR_joint",
    "motor_back_leftL_joint",
    "motor_back_leftR_joint",
    "motor_front_rightL_joint",
    "motor_front_rightR_joint",
    "motor_back_rightL_joint",
    "motor_back_rightR_joint",
]

class MinitaurSimple:
    def __init__(self, render=True):
        """Inicializa la simulación del Minitaur"""
        # Conectar a PyBullet
        if render:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        
        # Configurar cámara
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=45,
            cameraPitch=-20,
            cameraTargetPosition=[0, 0, 0]
        )
        
        # Cargar plano
        self.plane = p.loadURDF("plane.urdf")
        
        # Cargar Minitaur
        start_pos = [0, 0, 0.2]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        
        try:
            # Intentar cargar modelo Minitaur de PyBullet
            self.robot = p.loadURDF(
                "quadruped/minitaur.urdf",
                start_pos,
                start_orientation
            )
            print("✓ Minitaur cargado exitosamente")
        except:
            # Si no está disponible, usar cuadrúpedo alternativo
            try:
                self.robot = p.loadURDF(
                    "laikago/laikago.urdf",
                    start_pos,
                    start_orientation
                )
                print("✓ Laikago cargado como alternativa")
            except:
                print("❌ No se encontró modelo de cuadrúpedo")
                print("Creando cuadrúpedo simple...")
                self.robot = self.create_simple_quadruped(start_pos)
        
        # Obtener información de articulaciones
        self.num_joints = p.getNumJoints(self.robot)
        self.motor_indices = []
        self.joint_info = []
        
        print(f"\nNúmero total de articulaciones: {self.num_joints}")
        print("\nArticulaciones del robot:")
        
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot, i)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]
            
            if joint_type == p.JOINT_REVOLUTE:
                self.motor_indices.append(i)
                self.joint_info.append({
                    'index': i,
                    'name': joint_name,
                    'lower': info[8],
                    'upper': info[9]
                })
                print(f"  {i}: {joint_name} (motor)")
        
        print(f"\nTotal de motores controlables: {len(self.motor_indices)}")
        
        # Parámetros de marcha
        self.t = 0
        self.gait_period = 0.5  # Período del ciclo de marcha
        self.step_length = 0.1  # Longitud del paso
        
    def create_simple_quadruped(self, position):
        """Crea un cuadrúpedo simple si no hay modelos disponibles"""
        # Crear cuerpo principal
        body_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.3, 0.15, 0.08]
        )
        body_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.3, 0.15, 0.08],
            rgbaColor=[0.6, 0.6, 0.8, 1]
        )
        
        # Crear robot simple
        robot = p.createMultiBody(
            baseMass=5.0,
            baseCollisionShapeIndex=body_collision,
            baseVisualShapeIndex=body_visual,
            basePosition=position
        )
        
        # Agregar patas simples (cilindros)
        leg_positions = [
            [0.2, 0.15, -0.08],   # Frontal izquierda
            [0.2, -0.15, -0.08],  # Frontal derecha
            [-0.2, 0.15, -0.08],  # Trasera izquierda
            [-0.2, -0.15, -0.08], # Trasera derecha
        ]
        
        for pos in leg_positions:
            leg_collision = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=0.02,
                height=0.15
            )
            leg_visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=0.02,
                length=0.15,
                rgbaColor=[0.3, 0.3, 0.3, 1]
            )
            
        return robot
    
    def generate_trot_gait(self):
        """Genera un patrón de trote para las 4 patas"""
        # Patrón de trote: diagonal legs move together
        # FR-BL (grupo 1), FL-BR (grupo 2)
        
        phase = (self.t / self.gait_period) % 1.0
        
        # Ángulos para cada motor (simplificado)
        motor_angles = []
        
        if len(self.motor_indices) >= 8:
            # Minitaur real tiene 8 motores
            amplitude = 0.5
            offset = 0.3
            
            for i in range(len(self.motor_indices)):
                # Alternar fase entre grupos diagonales
                leg_phase = phase if i in [0, 1, 4, 5] else (phase + 0.5) % 1.0
                angle = offset + amplitude * math.sin(2 * math.pi * leg_phase)
                motor_angles.append(angle)
        else:
            # Para otros modelos, usar patrón simple
            for i in range(len(self.motor_indices)):
                angle = 0.3 * math.sin(2 * math.pi * phase + i * math.pi / 2)
                motor_angles.append(angle)
        
        return motor_angles
    
    def step(self):
        """Ejecuta un paso de simulación con control de marcha"""
        # Generar ángulos de las articulaciones
        target_angles = self.generate_trot_gait()
        
        # Aplicar control a los motores
        for i, motor_idx in enumerate(self.motor_indices):
            if i < len(target_angles):
                p.setJointMotorControl2(
                    self.robot,
                    motor_idx,
                    p.POSITION_CONTROL,
                    targetPosition=target_angles[i],
                    force=5.0,
                    maxVelocity=10.0
                )
        
        # Avanzar simulación
        p.stepSimulation()
        self.t += 1./240.
    
    def run(self, duration=10.0):
        """Ejecuta la simulación por un tiempo determinado"""
        print(f"\n{'='*60}")
        print("SIMULACIÓN CUADRÚPEDO MINITAUR")
        print(f"{'='*60}")
        print(f"Duración: {duration} segundos")
        print("Presiona Ctrl+C para detener\n")
        
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                self.step()
                time.sleep(1./240.)
                
                # Mostrar progreso cada 2 segundos
                if int(time.time() - start_time) % 2 == 0 and \
                   abs((time.time() - start_time) - int(time.time() - start_time)) < 0.02:
                    pos, _ = p.getBasePositionAndOrientation(self.robot)
                    vel, _ = p.getBaseVelocity(self.robot)
                    print(f"Tiempo: {time.time() - start_time:.1f}s | "
                          f"Posición: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] | "
                          f"Velocidad: {np.linalg.norm(vel):.2f} m/s")
        
        except KeyboardInterrupt:
            print("\n\n⏹️  Simulación detenida por el usuario")
        
        print(f"\n{'='*60}")
        print("SIMULACIÓN COMPLETADA")
        print(f"{'='*60}\n")
        
        time.sleep(2)
        p.disconnect()
    
    def run_interactive(self):
        """Modo interactivo con sliders para control manual"""
        print(f"\n{'='*60}")
        print("MODO INTERACTIVO - CONTROL MANUAL")
        print(f"{'='*60}\n")
        
        # Crear sliders para cada motor
        sliders = []
        for joint in self.joint_info:
            slider = p.addUserDebugParameter(
                paramName=f"Motor {joint['index']}: {joint['name']}",
                rangeMin=joint['lower'],
                rangeMax=joint['upper'],
                startValue=0
            )
            sliders.append(slider)
        
        print("Usa los sliders para controlar cada motor del robot")
        print("Presiona Ctrl+C para salir\n")
        
        try:
            while True:
                # Leer valores de sliders y aplicar
                for i, slider in enumerate(sliders):
                    target_pos = p.readUserDebugParameter(slider)
                    motor_idx = self.motor_indices[i]
                    
                    p.setJointMotorControl2(
                        self.robot,
                        motor_idx,
                        p.POSITION_CONTROL,
                        targetPosition=target_pos,
                        force=5.0
                    )
                
                p.stepSimulation()
                time.sleep(1./240.)
        
        except KeyboardInterrupt:
            print("\n⏹️  Modo interactivo terminado")
            p.disconnect()


def main():
    """Función principal"""
    import sys
    
    print("\n🤖 SIMULADOR DE CUADRÚPEDO MINITAUR")
    print("="*60)
    
    # Crear simulación
    sim = MinitaurSimple(render=True)
    
    # Preguntar modo
    print("\nModos disponibles:")
    print("  1. Marcha automática (trote)")
    print("  2. Control manual interactivo")
    
    # Si hay argumentos, usar el primero
    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("\nSelecciona modo (1 o 2): ").strip()
    
    if choice == "2":
        sim.run_interactive()
    else:
        sim.run(duration=15.0)


if __name__ == "__main__":
    main()
