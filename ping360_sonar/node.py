#!/usr/bin/env python
import sys

import numpy as np
# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import time
from cv_bridge import CvBridge
# from dynamic_reconfigure.server import Server
# from ping360_sonar_old.cfg import sonarConfig
from ping360_sonar.sensor import Ping360
from ping360_sonar_msgs.msg import SonarEcho

class ping360_node_ros(Node):

    def __init__(self):

        super().__init__('ping360_sonar')
        self.declare_parameter('device', "/dev/ping360")
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('gain', 0)
        self.declare_parameter('numberOfSamples', 200)
        self.declare_parameter('transmitFrequency', 740)
        self.declare_parameter('sonarRange', 7)
        self.declare_parameter('speedOfSound', 1500)
        self.declare_parameter('debug', False)
        self.declare_parameter('threshold', 200)
        self.declare_parameter('enableDataTopic', True)
        self.declare_parameter('maxAngle', 400)
        self.declare_parameter('minAngle', 0)
        self.declare_parameter('oscillate', True)
        self.declare_parameter('step', 2)
        self.declare_parameter('imgSize', 500)
        self.declare_parameter('queueSize', 1)

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.gain = self.get_parameter('gain').get_parameter_value().integer_value
        self.numberOfSamples = self.get_parameter('numberOfSamples').get_parameter_value().integer_value
        self.transmitFrequency = self.get_parameter('transmitFrequency').get_parameter_value().integer_value
        self.sonarRange = self.get_parameter('sonarRange').get_parameter_value().integer_value
        self.speedOfSound = self.get_parameter('speedOfSound').get_parameter_value().integer_value
        self.step = self.get_parameter('step').get_parameter_value().integer_value
        self.imgSize = self.get_parameter('imgSize').get_parameter_value().integer_value
        self.queue_size = self.get_parameter('queueSize').get_parameter_value().integer_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().integer_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.enableDataTopic = self.get_parameter('enableDataTopic').get_parameter_value().bool_value
        self.maxAngle = self.get_parameter('maxAngle').get_parameter_value().integer_value
        self.minAngle = self.get_parameter('minAngle').get_parameter_value().integer_value
        self.oscillate = self.get_parameter('oscillate').get_parameter_value().bool_value


        print(self.get_parameter('speedOfSound').get_parameter_value().integer_value)
        self.samplePeriod = calculateSamplePeriod(self.sonarRange, self.numberOfSamples, self.speedOfSound)

        self.transmitDuration = adjustTransmitDuration(
            self.sonarRange, self.samplePeriod, self.speedOfSound)
        FOV = self.maxAngle - self.minAngle  # The sonars field of view
        self.sign = 1
        if self.step >= FOV:
            rclpy.logerr(
                """
                The configured step is bigger then the set FOV (maxAngle - minAngle)
                Current settings:
                    step: {} - minAngle: {} - maxAngle: {} - FOV: {}""".format(self.step,
                                                                           self.maxAngle,
                                                                           self.minAngle,
                                                                           FOV))
            rclpy.shutdown()
            return

        # Initialize sensor
        self.sensor = Ping360(self.device, self.baudrate)
        while not self.sensor.initialize():
            print("Initialized Sensor: %s" % self.sensor.initialize())
            print("didnt work trying again in 1 second")
            time.sleep(2)
            self.sensor = Ping360(self.device, self.baudrate)
            if not rclpy.ok():
                exit()
        print("Initialized Sensor: %s" % self.sensor.initialize())

        # Global Variables
        self.angle = self.minAngle
        self.bridge = CvBridge()


        self.rawPub = self.create_publisher(SonarEcho,"sonar/intensity", self.queue_size)
        # laserPub = rclpy.Publisher(
        #     "/ping360_node/sonar/scan", LaserScan, queue_size=queue_size)

        # Initialize and configure the sonar
        updateSonarConfig(self.sensor, self.gain, self.transmitFrequency,
                          self.transmitDuration, self.samplePeriod, self.numberOfSamples)
        self.image = np.zeros((self.imgSize, self.imgSize, 1), np.uint8)

        # Initial the LaserScan Intensities & Ranges
        self.ranges = [0]
        self.intensities = [0]

        # Center point coordinates
        self.center = (float(self.imgSize / 2), float(self.imgSize / 2))

        timer_period = 1.0/100.0
        # print("test2")
        self.timer = self.create_timer(timer_period, self.sonarMeasurements)

        self.add_on_set_parameters_callback(self.parameters_callback)

    def sonarMeasurements(self):
        if updated:
            updateSonarConfig(self.sensor, self.gain, self.transmitFrequency,
                              self.transmitDuration, self.samplePeriod, self.numberOfSamples)
            angle = self.minAngle
        # Get sonar response
        data = getSonarData(self.sensor, self.angle)

        # Contruct and publish Sonar data msg
        if self.enableDataTopic:
            rawDataMsg = generateRawMsg(self,self.angle, data, self.gain, self.numberOfSamples, self.transmitFrequency, self.speedOfSound, self.sonarRange, self.step)
            self.rawPub.publish(rawDataMsg)

        # calculate next angle step
        self.angle += self.sign * self.step
        # print(angle)

        self.angle = wrap_angle(self.angle)

        if self.maxAngle > self.minAngle:
            if self.angle >= self.maxAngle:
                if not self.oscillate:
                    self.angle = self.minAngle
                else:
                    self.angle = self.maxAngle
                    sign = -1

            if self.angle <= self.minAngle and self.oscillate:
                self.sign = 1
                self.angle = self.minAngle
        else:
            if self.angle >= self.maxAngle and self.angle < self.minAngle:
                if not self.oscillate:
                    self.angle = self.minAngle
                    self.sign = 1
                else:
                    self.angle = self.maxAngle
                    self.sign = -1

    def parameters_callback(self, params):
        global updated

        for param in params:
            success = False
            if param.name == 'device':
                if param.type_ == Parameter.Type.STRING:
                    if param.value.startswith('/dev/tty'):
                        success = True
                        self.device = param.value
            if param.name == 'baudrate':
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    self.baudrate = param.value
            if param.name == 'gain':
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    self.gain = param.value
            if param.name == 'numberOfSamples':
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    self.numberOfSamples = param.value
            if param.name == 'transmitFrequency':
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    self.transmitFrequency = param.value
            if param.name == 'sonarRange':
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    self.sonarRange = param.value
            if param.name == 'speedOfSound':
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    self.speedOfSound = param.value
            if param.name == 'step':
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    self.step = param.value
            if param.name == 'debug':
                if param.type_ == Parameter.Type.BOOL:
                    success = True
                    self.debug = param.value
            if param.name == 'enableDataTopic':
                if param.type_ == Parameter.Type.BOOL:
                    success = True
                    self.enableDataTopic = param.value
            if param.name == 'maxAngle':
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    self.maxAngle = param.value
            if param.name == 'minAngle':
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    self.minAngle = param.value
            if param.name == 'oscillate':
                if param.type_ == Parameter.Type.BOOL:
                    success = True
                    self.oscillate = param.value
        if success == True:
            updated = True
        return SetParametersResult(successful=success)

def main():
    rclpy.init(args=sys.argv)

    ping360NodeObject = ping360_node_ros()
    rclpy.spin(ping360NodeObject)

def getSonarData(sensor, angle):
    """
    Transmits the sonar angle and returns the sonar intensities
    Args:
        sensor (Ping360): Sensor class
        angle (int): Gradian Angle
    Returns:
        list: Intensities from 0 - 255
    """
    sensor.transmitAngle(angle)
    data = bytearray(getattr(sensor, '_data'))
    return [k for k in data]


def generateRawMsg(node,angle, data, gain, numberOfSamples, transmitFrequency, speedOfSound, sonarRange,stepSize):
    """
    Generates the raw message for the data topic
    Args:
        angle (int): Gradian Angle
        data (list): List of intensities
        gain (int)
        numberOfSamples (int)
        transmitFrequency (float)
        speedOfSound (int)
        sonarRange (int)
    Returns:
        SonarEcho: message
     """
    msg = SonarEcho()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'sonar_frame'
    msg.angle = float(angle)
    msg.gain = gain
    msg.number_of_samples = numberOfSamples
    msg.transmit_frequency = transmitFrequency
    msg.speed_of_sound = speedOfSound
    msg.range = sonarRange
    msg.step_size = stepSize
    msg.intensities = data
    return msg


def calculateRange(numberOfSamples, samplePeriod, speedOfSound, _samplePeriodTickDuration=25e-9):
    # type: (float, int, float, float) -> float
    """
      Calculate the range based in the duration
     """
    return numberOfSamples * speedOfSound * _samplePeriodTickDuration * samplePeriod / 2


def calculateSamplePeriod(distance, numberOfSamples, speedOfSound, _samplePeriodTickDuration=25e-9):
    # type: (float, int, int, float) -> float
    """
      Calculate the sample period based in the new range
     """
    return 2 * distance / (numberOfSamples * speedOfSound * _samplePeriodTickDuration)


def adjustTransmitDuration(distance, samplePeriod, speedOfSound, _firmwareMinTransmitDuration=5):
    # type: (float, float, int, int) -> float
    """
     @brief Adjust the transmit duration for a specific range
     Per firmware engineer:
     1. Starting point is TxPulse in usec = ((one-way range in metres) * 8000) / (Velocity of sound in metres
     per second)
     2. Then check that TxPulse is wide enough for currently selected sample interval in usec, i.e.,
          if TxPulse < (2.5 * sample interval) then TxPulse = (2.5 * sample interval)
        (transmit duration is microseconds, samplePeriod() is nanoseconds)
     3. Perform limit checking
     Returns:
        float: Transmit duration
     """
    duration = 8000 * distance / speedOfSound
    transmit_duration = max(
        2.5 * getSamplePeriod(samplePeriod) / 1000, duration)
    return max(_firmwareMinTransmitDuration, min(transmitDurationMax(samplePeriod), transmit_duration))


def transmitDurationMax(samplePeriod, _firmwareMaxTransmitDuration=500):
    # type: (float, int) -> float
    """
    @brief The maximum transmit duration that will be applied is limited internally by the
    firmware to prevent damage to the hardware
    The maximum transmit duration is equal to 64 * the sample period in microseconds

    Returns:
        float: The maximum transmit duration possible
    """
    return min(_firmwareMaxTransmitDuration, getSamplePeriod(samplePeriod) * 64e6)


def getSamplePeriod(samplePeriod, _samplePeriodTickDuration=25e-9):
    # type: (float, float) -> float
    """  Sample period in ns """
    return samplePeriod * _samplePeriodTickDuration


def updateSonarConfig(sensor, gain, transmitFrequency, transmitDuration, samplePeriod, numberOfSamples):
    global updated
    sensor.set_gain_setting(gain)
    sensor.set_transmit_frequency(transmitFrequency)
    sensor.set_transmit_duration(int(transmitDuration))
    sensor.set_sample_period(int(samplePeriod))
    sensor.set_number_of_samples(numberOfSamples)
    updated = False

def wrap_angle(angle):
    while angle < 0:
        angle += 400
    while angle >= 400:
        angle -= 400
    return angle
