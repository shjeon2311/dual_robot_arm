import rclpy
from sensor_msgs.msg import JointState
import serial

def joint_state_callback(msg):
    joint_positions = msg.position
    send_to_arduino(joint_positions)

def send_to_arduino(joint_positions):
    serial_port = '/dev/ttyACM1'  # 아두이노의 시리얼 포트
    baud_rate = 9600
    serial_connection = serial.Serial(serial_port, baud_rate)

    for position in joint_positions:
        data_to_send = str(position) + '\n'
        serial_connection.write(data_to_send.encode())

def main():
    rclpy.init()
    node = rclpy.create_node('ros_to_arduino')
    subscription = node.create_subscription(JointState, '/arm1/joint_states', joint_state_callback, 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
