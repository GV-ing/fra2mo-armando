#!/usr/bin/env python3
"""
IK-based Pick & Place per fra2mo + Armando
Sistema semplificato con hard-coded positions ma usando cinematica inversa
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np
import time

try:
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
except ImportError:
    print("ERRORE: ikpy non installato. Esegui: pip3 install ikpy")
    exit(1)


class ArmandoIKChain:
    """Definizione catena cinematica di Armando per IK"""
    
    def __init__(self):
        # Catena cinematica basata su URDF analisi
        # Link lengths estratte da armando_macro.xacro
        self.chain = Chain(name='armando', links=[
            OriginLink(),  # Base fissa
            
            # J0: Base rotation (Z-axis)
            URDFLink(
                name="arm_j0",
                origin_translation=[0, 0.0155, 0.063],  # Somma offset base
                origin_orientation=[0, 0, -1.57],
                rotation=[0, 0, 1],  # Ruota attorno Z
                bounds=(-2.57, 2.57)
            ),
            
            # J1: Shoulder (Z-axis dopo rotazioni)
            URDFLink(
                name="arm_j1",
                origin_translation=[-0.047, 0, 0.0385],
                origin_orientation=[0, 1.57, 1.57],
                rotation=[0, 0, 1],
                bounds=(-3.14, 3.14)
            ),
            
            # J2: Elbow (Z-axis)
            URDFLink(
                name="arm_j2",
                origin_translation=[-0.004, 0.0645, -0.046],
                origin_orientation=[-1.57, 3.14, 1.57],
                rotation=[0, 0, 1],
                bounds=(-3.14, 3.14)
            ),
            
            # J3: Wrist (Z-axis)
            URDFLink(
                name="arm_j3",
                origin_translation=[-0.004, 0.065, -0.017],
                origin_orientation=[-1.57, 3.14, 1.57],
                rotation=[0, 0, 1],
                bounds=(-3.14, 3.14)
            ),
            
            # End effector (gripper center)
            URDFLink(
                name="gripper",
                origin_translation=[0.015, 0, 0.05],  # Offset al centro gripper
                origin_orientation=[0, 0, 1.57],
                rotation=[0, 0, 0]
            ),
        ])
    
    def compute_ik(self, target_position, initial_joints=None):
        """
        Calcola IK per raggiungere target_position [x, y, z]
        Returns: [j0, j1, j2, j3] o None se fallisce
        """
        if initial_joints is None:
            initial_joints = [0, 0, 0, 0, 0, 0]  # Include origin e gripper
        
        try:
            # ikpy usa matrice 4x4 per target pose
            target_pose = np.eye(4)
            target_pose[:3, 3] = target_position
            
            # Calcola IK
            ik_solution = self.chain.inverse_kinematics(
                target_pose,
                initial_position=initial_joints,
                orientation_mode=None  # Solo posizione, non orientamento
            )
            
            # Estrai solo i 4 joint attivi (esclude origin e gripper)
            joint_angles = ik_solution[1:5]  # j0, j1, j2, j3
            
            return joint_angles
            
        except Exception as e:
            print(f"IK fallita: {e}")
            return None


class PickPlaceTask(Node):
    """Task Pick & Place con cinematica inversa"""
    
    def __init__(self):
        super().__init__('ik_pick_place_task')
        
        # Publisher per controllo braccio e base
        self.arm_pub = self.create_publisher(
            Float64MultiArray, 
            '/position_controller/commands', 
            10
        )
        self.base_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # IK solver
        self.ik_chain = ArmandoIKChain()
        
        # Configurazione task (HARD-CODED per demo)
        self.ARM_BASE_HEIGHT = 0.24  # Altezza base braccio rispetto ground (top_mount_link)
        self.PILLAR_HEIGHT = 0.60    # Altezza pilastro
        self.OBJECT_HEIGHT = 0.05    # Altezza oggetto
        self.BASKET_HEIGHT = 0.30    # Altezza basket
        
        # Posizioni pilastri (x, y in map frame) - DA CONFIGURARE
        self.pillars = [
            {'x': 2.0, 'y': 1.5, 'name': 'pillar_1'},
            {'x': 2.0, 'y': -1.5, 'name': 'pillar_2'},
            # Aggiungi altri pilastri...
        ]
        
        # Posizione basket
        self.basket_pos = {'x': -2.0, 'y': 0.0}
        
        self.get_logger().info("IK Pick&Place Task inizializzato")
        self.get_logger().info(f"Pilastri configurati: {len(self.pillars)}")
    
    def move_arm(self, joint_angles, gripper_pos=0.03):
        """Muove il braccio alla configurazione specificata"""
        msg = Float64MultiArray()
        msg.data = [
            float(joint_angles[0]),
            float(joint_angles[1]),
            float(joint_angles[2]),
            float(joint_angles[3]),
            float(gripper_pos)
        ]
        self.arm_pub.publish(msg)
        self.get_logger().info(f"Arm command: {msg.data}")
    
    def move_base(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        """Muove la base per duration secondi"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        rate = self.create_rate(10)  # 10 Hz
        start = time.time()
        
        while time.time() - start < duration:
            self.base_pub.publish(msg)
            rate.sleep()
        
        # Stop
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.base_pub.publish(msg)
    
    def calculate_object_position_relative(self, pillar_x, pillar_y):
        """
        Calcola posizione oggetto rispetto alla base del braccio
        Assumiamo robot posizionato davanti al pilastro
        """
        # Posizione robot (semplificato: assume robot allineato con pilastro)
        robot_x = pillar_x - 0.4  # 40cm davanti al pilastro
        robot_y = pillar_y
        
        # Posizione oggetto in map frame
        obj_x_map = pillar_x
        obj_y_map = pillar_y
        obj_z_map = self.PILLAR_HEIGHT + self.OBJECT_HEIGHT / 2
        
        # Trasforma in frame braccio (relativo a base_link del robot)
        # Semplificazione: assume braccio montato su asse X del robot
        obj_x_rel = obj_x_map - robot_x
        obj_y_rel = obj_y_map - robot_y
        obj_z_rel = obj_z_map - self.ARM_BASE_HEIGHT
        
        return np.array([obj_x_rel, obj_y_rel, obj_z_rel])
    
    def pick_object_from_pillar(self, pillar):
        """Sequenza pick per un singolo pilastro"""
        self.get_logger().info(f"\n=== PICK da {pillar['name']} ===")
        
        # 1. Calcola posizione target oggetto
        target_pos = self.calculate_object_position_relative(
            pillar['x'], pillar['y']
        )
        self.get_logger().info(f"Target position (rel): {target_pos}")
        
        # 2. Calcola IK per raggiungere oggetto
        home_joints = [0.0, 0.0, 0.0, 0.0]  # Posizione iniziale
        target_joints = self.ik_chain.compute_ik(target_pos, home_joints)
        
        if target_joints is None:
            self.get_logger().error("IK fallita! Impossibile raggiungere oggetto")
            return False
        
        self.get_logger().info(f"IK solution: {target_joints}")
        
        # 3. Apri gripper
        self.get_logger().info("Apertura gripper...")
        self.move_arm(home_joints, gripper_pos=0.06)  # Gripper aperto
        time.sleep(2.0)
        
        # 4. Muovi braccio verso oggetto
        self.get_logger().info("Movimento verso oggetto...")
        self.move_arm(target_joints, gripper_pos=0.06)
        time.sleep(4.0)  # Attendi movimento
        
        # 5. Chiudi gripper
        self.get_logger().info("Chiusura gripper...")
        self.move_arm(target_joints, gripper_pos=0.0)  # Gripper chiuso
        time.sleep(2.0)
        
        # 6. Solleva oggetto
        self.get_logger().info("Sollevamento oggetto...")
        lift_pos = target_pos.copy()
        lift_pos[2] += 0.10  # Solleva 10cm
        lift_joints = self.ik_chain.compute_ik(lift_pos, target_joints)
        if lift_joints is not None:
            self.move_arm(lift_joints, gripper_pos=0.0)
            time.sleep(3.0)
        
        # 7. Ritorna a home con oggetto
        self.get_logger().info("Ritorno a home position...")
        self.move_arm(home_joints, gripper_pos=0.0)
        time.sleep(3.0)
        
        return True
    
    def place_object_in_basket(self):
        """Sequenza place nel basket"""
        self.get_logger().info("\n=== PLACE nel basket ===")
        
        # Calcola posizione sopra basket
        basket_rel_pos = np.array([
            self.basket_pos['x'],  # Semplificato
            self.basket_pos['y'],
            self.BASKET_HEIGHT + 0.15  # 15cm sopra bordo basket
        ])
        
        # IK per posizione sopra basket
        home_joints = [0.0, 0.0, 0.0, 0.0]
        basket_joints = self.ik_chain.compute_ik(basket_rel_pos, home_joints)
        
        if basket_joints is None:
            self.get_logger().error("IK fallita per basket!")
            return False
        
        # Muovi sopra basket
        self.get_logger().info("Posizionamento sopra basket...")
        self.move_arm(basket_joints, gripper_pos=0.0)
        time.sleep(4.0)
        
        # Rilascia oggetto
        self.get_logger().info("Rilascio oggetto...")
        self.move_arm(basket_joints, gripper_pos=0.06)  # Apri gripper
        time.sleep(2.0)
        
        # Ritorna home
        self.get_logger().info("Ritorno a home...")
        self.move_arm(home_joints, gripper_pos=0.03)
        time.sleep(3.0)
        
        return True
    
    def run_task(self):
        """Esegue task completo pick&place per tutti i pilastri"""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("INIZIO TASK PICK & PLACE")
        self.get_logger().info("="*50 + "\n")
        
        # Home position iniziale
        self.get_logger().info("Posizione home iniziale...")
        self.move_arm([0.0, 0.0, 0.0, 0.0], gripper_pos=0.03)
        time.sleep(3.0)
        
        for pillar in self.pillars:
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"Pilastro: {pillar['name']}")
            self.get_logger().info(f"Posizione: x={pillar['x']}, y={pillar['y']}")
            self.get_logger().info(f"{'='*50}\n")
            
            # TODO: Navigazione verso pilastro (usando Nav2)
            # Per ora: movimento semplificato base
            self.get_logger().info("Navigazione verso pilastro...")
            # self.navigate_to_pillar(pillar)  # Da implementare
            time.sleep(1.0)
            
            # Pick
            success = self.pick_object_from_pillar(pillar)
            if not success:
                self.get_logger().error(f"Pick fallito per {pillar['name']}")
                continue
            
            # TODO: Navigazione verso basket
            self.get_logger().info("Navigazione verso basket...")
            time.sleep(1.0)
            
            # Place
            success = self.place_object_in_basket()
            if not success:
                self.get_logger().error("Place fallito!")
                continue
            
            self.get_logger().info(f"\n✓ {pillar['name']} completato!\n")
        
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("TASK COMPLETATO!")
        self.get_logger().info("="*50 + "\n")


def main(args=None):
    rclpy.init(args=args)
    
    task = PickPlaceTask()
    
    # Attendi che i publisher siano pronti
    time.sleep(2.0)
    
    try:
        # Esegui task
        task.run_task()
    except KeyboardInterrupt:
        task.get_logger().info("Task interrotto da utente")
    finally:
        task.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
