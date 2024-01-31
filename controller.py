#!/usr/bin/env python
import rclpy
from std_msgs.msg import Float32MultiArray
import sys
import numpy as np

controllerPubData = None  # controller's publisher


def displayDataDescription(data):
    print('leg0: ', data[0])
    print('leg1: ', data[1])
    print('leg2: ', data[2])
    print('leg3: ', data[3])
    return


# Emulate Interface Communication

def emulateInterface(data):
    global controllerPubData

    print('\033[94m Received data from topic /TIRREX/microparallel/simulationData: \033[0m')
    displayDataDescription(data.data)
    # Send commands value to /TIRREX/microparallel/interfaceData
    msg = Float32MultiArray()  # wrapper for ROS primitive types
    # see : https://github.com/ros2/common_interfaces/tree/master/std_msgs
    msg.data = [20., 10., 0., 5.]
    controllerPubData.publish(msg)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    controllerROSnode = rclpy.create_node('MicroInterface')
    controllerROSnode.get_logger().info('Created node')

    subData = controllerROSnode.create_subscription(Float32MultiArray, "/TIRREX/microparallel/simulationData", emulateInterface, 10)
    controllerPubData = controllerROSnode.create_publisher(Float32MultiArray, "/TIRREX/microparallel/interfaceData", 10)

    rclpy.spin(controllerROSnode)

    controllerROSnode.destroy_node()
    rclpy.shutdown()


import Sofa.Core


# SOFA Communication Part

class SofaROSInterface(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.name = "SOFAROSInterface"

        # Arguments
        self.robot = args[0]

        # Initialize a node ROS
        rclpy.init()
        self.node = rclpy.create_node('SOFASimulation')
        self.node.get_logger().info('Created node')

        # Set up subscription to the interface: get actuator commands
        self.subData = self.node.create_subscription(Float32MultiArray, "/TIRREX/microparallel/interfaceData", self.callback, 10)
        self.dataInterface = None
        self.dataSimulation = None

        # Set up publisher: send the current data
        self.pubData = self.node.create_publisher(Float32MultiArray, "/TIRREX/microparallel/simulationData", 10)

    def callback(self, data):
        self.dataInterface = data.data
        print('\033[94m Received commands from topic /TIRREX/microparallel/interfaceData: \033[0m')
        displayDataDescription(self.dataInterface)

    def onAnimateBeginEvent(self, event):

        # Receive command from topic "/TIRREX/microparallel/interfaceData"
        rclpy.spin_once(self.node, timeout_sec=0.001)  # Enables callbacks
        if self.dataInterface is not None:
            self.__processDataReceived()
            self.dataInterface = None

        # Publish data through "/TIRREX/microparallel/simulationData" topic
        msg = Float32MultiArray()
        # wrapper for ROS primitive types, see : https://github.com/ros2/common_interfaces/tree/master/std_msgs
        # "This should generally not be relied upon for long-term use"
        data = self.__processDataToSend()
        msg.data = [float(d) for d in data]
        self.pubData.publish(msg)

    def __processDataReceived(self):
        positions = np.copy(self.robot.anchor.dofs.position.value)
        positions[0][1] = self.dataInterface[0]
        positions[1][1] = self.dataInterface[1]
        positions[2][1] = self.dataInterface[2]
        positions[3][1] = self.dataInterface[3]
        self.robot.anchor.dofs.position.value = positions

    def __updateDataSimulation(self):
        positions = self.robot.node.dofs.position.value
        self.dataSimulation = list([
            positions[2][1],
            positions[3][1],
            positions[4][1],
            positions[5][1],
        ])

    def __processDataToSend(self):
        self.__updateDataSimulation()
        return self.dataSimulation


