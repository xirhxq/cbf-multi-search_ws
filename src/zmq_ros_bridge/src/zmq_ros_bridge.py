#!/usr/bin/env python3

import zmq
import rospy
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
import struct
from functools import partial
import socket as skt
from datetime import datetime


def now_ms():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]

class ZMQ_ROS_BRIDGE:
    def __init__(self):
        rospy.init_node('zmq_ros_bridge', anonymous=True)
        self.name_ip_dict = eval(rospy.get_param('name_ip_dict'))
        self.server = rospy.get_param('server')
        print(len(self.name_ip_dict))

    def connect(self):
        self.s = skt.socket(skt.AF_INET, skt.SOCK_DGRAM)
        self.s.connect((self.server, 80))

        self.my_ip = self.s.getsockname()[0]
        print('My IP: ', self.my_ip)

        self.my_name = ''
        psb_name = [name for name, socket in self.name_ip_dict.items() if self.my_ip in socket]
        if len(psb_name) != 1:
            print('Can\'t find right name for this IP')
            raise AssertionError
        else:
            self.my_name = psb_name[0]
            print('My name: ' + self.my_name)

        self.context = zmq.Context()
        self.poller = zmq.Poller()

        self.sub_sockets = {}
        for name, socket in self.name_ip_dict.items():
            if name == self.my_name:
                continue
            self.sub_socket = self.context.socket(zmq.SUB)
            self.sub_socket.connect(socket)
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, self.my_name)
            self.sub_sockets[name] = self.sub_socket
            self.poller.register(self.sub_socket, zmq.POLLIN)
            print(f'Listen to {name} @ {socket}')

        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind("tcp://*:5555")

        self.ros_sub = rospy.Subscriber(
            '/tx',  
            Float32MultiArray, 
            self.float_array_callback
        )
        print(f'Listen to ROS topic /tx on {self.my_name}\'s ROS network')

        self.ros_pub = rospy.Publisher('/rx', Float32MultiArray, queue_size=10)
        print(f'Will pub /rx on {self.my_name}\'s ROS network')
        

    def float_array_callback(self, msg):
        data = msg.data
        who = msg.layout.dim[0].label
        msg = struct.pack('f' * len(data), *data)
        print(f'ROS->TCP | {self.my_name}(me) to {who} | {now_ms()} | {data}')
        self.pub_socket.send_multipart([who.encode(), msg])


    def main_loop(self):
        while not rospy.is_shutdown():
            socks = dict(self.poller.poll(10))

            for name, sub_socket in self.sub_sockets.items():
                if sub_socket in socks and socks[sub_socket] == zmq.POLLIN:
                    message = sub_socket.recv_multipart()[1]
                    data = struct.unpack('f' * (len(message) // 4), message)
                    print(f'TCP->ROS | {name} to {self.my_name}(me) | {now_ms()} | {data}')
                    msg = Float32MultiArray(data=data)
                    msg.layout.dim.append(MultiArrayDimension(label=name))
                    self.ros_pub.publish(msg)

        self.context.term()

if __name__ == '__main__':
    zrb = ZMQ_ROS_BRIDGE()
    zrb.connect()
    zrb.main_loop()