#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import time

class TransformPublisher(Node):
    def __init__(self):
        super().__init__('transform_publisher')
        
        # Créer les publishers pour les deux topics
        self.tcp_right_pub = self.create_publisher(TransformStamped, '/tcp_right', 10)
        self.tcp_left_pub = self.create_publisher(TransformStamped, '/tcp_left', 10)
        
        # Timer pour publier les messages à intervalles réguliers
        self.timer = self.create_timer(0.1, self.publish_transforms)  # 10Hz
        
        # Index pour cycler à travers les transformations tcp_right
        self.tcp_right_index = 0
        
        # Données pour tcp_right (4 transformations différentes)
        self.tcp_right_data = [
            {
                'translation': {'x': -0.19701637412670062, 'y': 0.6629855708551466, 'z': 0.18097976022893575},
                'rotation': {'x': -4.9543361280280835e-05, 'y': 0.7071128000788099, 'z': -4.170087857335249e-05, 'w': 0.7071007592777686}
            },
            {
                'translation': {'x': -0.19699118285366932, 'y': 0.6629995817882632, 'z': 0.18098074795485658},
                'rotation': {'x': -1.1320677202587838e-05, 'y': 0.7071128119874167, 'z': -1.3041593224087133e-05, 'w': 0.7071007501233522}
            },
            {
                'translation': {'x': -0.1970101825762322, 'y': 0.6630062281043148, 'z': 0.18099467172982237},
                'rotation': {'x': -4.344198473663422e-05, 'y': 0.7071119393892533, 'z': -2.931893651430921e-05, 'w': 0.707101621003914}
            },
            {
                'translation': {'x': -0.19698962229903091, 'y': 0.6629909703110299, 'z': 0.1809650515478592},
                'rotation': {'x': -3.06434459419842e-05, 'y': 0.7071179624377539, 'z': -4.569216644537219e-05, 'w': 0.7070955976182318}
            }
        ]
        
        # Données pour tcp_left (une seule transformation)
        self.tcp_left_data = {
            'translation': {'x': 0.13184608168136697, 'y': 0.6705640136911637, 'z': 0.13943102937867935},
            'rotation': {'x': -0.018400000820701312, 'y': 0.7068966723938777, 'z': -0.018481559354360928, 'w': 0.7068358837043378}
        }
        
        self.get_logger().info('Transform Publisher Node démarré')

    def publish_transforms(self):
        # Publier tcp_right
        self.publish_tcp_right()
        
        # Publier tcp_left
        self.publish_tcp_left()

    def publish_tcp_right(self):
        msg = TransformStamped()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'cam_bassa_base_frame'
        msg.child_frame_id = 'ur_right_cable'
        
        # Utiliser les données cycliques
        data = self.tcp_right_data[self.tcp_right_index]
        
        # Transform
        msg.transform.translation.x = data['translation']['x']
        msg.transform.translation.y = data['translation']['y']
        msg.transform.translation.z = data['translation']['z']
        
        msg.transform.rotation.x = data['rotation']['x']
        msg.transform.rotation.y = data['rotation']['y']
        msg.transform.rotation.z = data['rotation']['z']
        msg.transform.rotation.w = data['rotation']['w']
        
        # Publier le message
        self.tcp_right_pub.publish(msg)
        
        # Incrémenter l'index pour le prochain message
        self.tcp_right_index = (self.tcp_right_index + 1) % len(self.tcp_right_data)

    def publish_tcp_left(self):
        msg = TransformStamped()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'cam_bassa_base_frame'
        msg.child_frame_id = 'ur_left_cable'
        
        # Transform
        msg.transform.translation.x = self.tcp_left_data['translation']['x']
        msg.transform.translation.y = self.tcp_left_data['translation']['y']
        msg.transform.translation.z = self.tcp_left_data['translation']['z']
        
        msg.transform.rotation.x = self.tcp_left_data['rotation']['x']
        msg.transform.rotation.y = self.tcp_left_data['rotation']['y']
        msg.transform.rotation.z = self.tcp_left_data['rotation']['z']
        msg.transform.rotation.w = self.tcp_left_data['rotation']['w']
        
        # Publier le message
        self.tcp_left_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = TransformPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()