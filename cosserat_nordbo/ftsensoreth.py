import struct
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty
import argparse

IP_ADDR = '192.168.0.100'
PORT = 2001

CMD_TYPE_SENSOR_TRANSMIT 	= '07'
SENSOR_TRANSMIT_TYPE_START = '01'
SENSOR_TRANSMIT_TYPE_STOP 	= '00'

CMD_TYPE_SET_CURRENT_TARE 	= '15'
SET_CURRENT_TARE_TYPE_NEGATIVE	= '01'

class NordboForceSensor(Node):

	def __init__(self, tare):
		super().__init__('nordbo_sensor')

		self.get_logger().info('Nordbo Force Sensor: Configuration Start')

		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.settimeout(2.0)
		self.socket.connect((IP_ADDR, PORT))

		self.wrench_pub = self.create_publisher(WrenchStamped, 'nordbo/wrench_data', 10)

		self.start_srv = self.create_service(Empty, 'nordbo/start_sensor', self.start_sensor_cb)
		self.stop_srv = self.create_service(Empty, 'nordbo/stop_sensor', self.stop_sensor_cb)
		self.tare_srv = self.create_service(Empty, 'nordbo/tare_sensor', self.tare_sensor_cb)

		self.sensor_active = True

		if tare:
			send_data = '03' + CMD_TYPE_SET_CURRENT_TARE + SET_CURRENT_TARE_TYPE_NEGATIVE
			send_data = bytearray.fromhex(send_data)
			self.socket.send(send_data)
			self.recv_msg()

		send_data = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_START
		send_data = bytearray.fromhex(send_data)
		self.socket.send(send_data)
		self.recv_msg()

		self.get_logger().info('Nordbo Force Sensor: Configuration Completed')

		self.publish_wrench_data()

	def start_sensor_cb(self, request, response):
		self.get_logger().info('Nordbo Force Sensor: Start Requested')

		send_data = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_START
		send_data = bytearray.fromhex(send_data)
		self.socket.send(send_data)
		self.recv_msg()

		return response

	def stop_sensor_cb(self, request, response):
		self.get_logger().info('Nordbo Force Sensor: Stop Requested')

		send_data = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_STOP
		send_data = bytearray.fromhex(send_data)
		self.socket.send(send_data)
		self.recv_msg()

		return response

	def tare_sensor_cb(self, request, response):
		self.get_logger().info('Nordbo Force Sensor: Tare Requested')

		send_data = '03' + CMD_TYPE_SET_CURRENT_TARE + SET_CURRENT_TARE_TYPE_NEGATIVE
		send_data = bytearray.fromhex(send_data)
		self.socket.send(send_data)
		self.recv_msg()

		self.get_logger().info('Nordbo Force Sensor: Tare Ok')
		return response

	def recv_msg(self):
		recv_data = bytearray(self.socket.recv(2))

		while len(recv_data) < recv_data[0]:
			recv_data += bytearray(self.socket.recv(recv_data[0] - len(recv_data)))

		return recv_data

	def publish_wrench_data(self):
		while self.sensor_active:
			new_wrench = WrenchStamped()
			new_wrench.header.stamp = self.get_clock().now().to_msg()

			recv_data = self.recv_msg()
			self.get_logger().info('Nordbo Force Sensor: Received data')
			self.get_logger().info('Nordbo Force Sensor: ' + str(recv_data))
			new_wrench.wrench.force.x = struct.unpack('!d', recv_data[2:10])[0]
			new_wrench.wrench.force.y = struct.unpack('!d', recv_data[10:18])[0]
			new_wrench.wrench.force.z = struct.unpack('!d', recv_data[18:26])[0]
			new_wrench.wrench.torque.x = struct.unpack('!d', recv_data[26:34])[0]
			new_wrench.wrench.torque.y = struct.unpack('!d', recv_data[34:42])[0]
			new_wrench.wrench.torque.z = struct.unpack('!d', recv_data[42:50])[0]
			print(f"Force: {new_wrench.wrench.force.x}, {new_wrench.wrench.force.y}, {new_wrench.wrench.force.z}")
			print(f"Torque: {new_wrench.wrench.torque.x}, {new_wrench.wrench.torque.y}, {new_wrench.wrench.torque.z}")

			self.wrench_pub.publish(new_wrench)


def main(args=None):
	parser = argparse.ArgumentParser(description='Nordbo Force Sensor Node')
	parser.add_argument('--tare', action='store_true', help='Set tare to true (default: false)')
	parsed_args = parser.parse_args()

	rclpy.init(args=args)
	sensor = NordboForceSensor(tare=parsed_args.tare)
	rclpy.spin(sensor)
	sensor.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
