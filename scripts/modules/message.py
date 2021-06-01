import rospy
import math

from std_msgs.msg import UInt8                 # flight state, ground mission cmd
from sensor_msgs.msg import NavSatFix          # RTK pos
from std_msgs.msg import Int16                 # rtk yaw
from geometry_msgs.msg import Vector3Stamped   # gimbal state,  UAV RTK velocity

from dji_osdk_ros.msg import iuslTarState      # target state
from dji_osdk_ros.msg import iuslMyState       # my state for bag
from dji_osdk_ros.msg import iuslUAVCtrlCmd    # my control cmd
from dji_osdk_ros.msg import iuslDetectionResult  # detect state for fire


class MessageSubscriber(object):
    def __init__(self, topic, msg_type):
        rospy.Subscriber(topic, msg_type, self.callback) 

    def callback(self):
        pass

class UAVCtlCmdPublisher(object):
    def __int__(self):
        self.__pub = rospy.Publisher(
            '/iusl/UAV_control_cmd', 
            iuslUAVCtrlCmd, 
            queue_size=5
        )

        self.iuslUAVCtrlCmd_data = iuslUAVCtrlCmd()
        self.iuslUAVCtrlCmd_data.task = 0    # 1 takeoff, 2 gohome, 3 hover, 4 control
        self.iuslUAVCtrlCmd_data.mode = 0    # 1 for pos ctrl, 2 for vel ctrl
        self.iuslUAVCtrlCmd_data.x = 0.
        self.iuslUAVCtrlCmd_data.y = 0.
        self.iuslUAVCtrlCmd_data.z = 0.
        self.iuslUAVCtrlCmd_data.yaw = 0.

    def update(self, data):
        self.iuslUAVCtrlCmd_data = data

    def publish(self, cmd):
        self.__pub.publish(cmd)


class TargetSubscriber(MessageSubscriber):
    def __init__(self, topic):
        super(TargetSubscriber, self).__init__(
            topic=topic, 
            msg_type=iuslTarState, 
        )
        self.tar_OK = False
        self.x = 0.
        self.y = 0.
        self.z = 0.
        self.vx = 0.
        self.vy = 0.

    def callback(self, msg):
        self.tar_OK = msg.tar_OK
        self.x = msg.tar_x
        self.y = msg.tar_y
        self.z = msg.tar_z
        self.vx = msg.tar_vx
        self.vy = msg.tar_vy

class UInt8Subscriber(MessageSubscriber):
    def __init__(self, topic):
        super(UInt8Subscriber, self).__init__(
            topic=topic, 
            msg_type=UInt8
        )
        self.num = 0

    def callback(self, msg):
        self.num = msg.data


class DetectionResultSubscriber(MessageSubscriber):
    def __init__(self, topic):
        super(DetectionResultSubscriber, self).__init__(
            topic=topic, 
            msg_type=iuslDetectionResult
        )
        self.measure_is_new = False      # detect state for fire
        self.measure_is_in_center =False
        self.measure_gim_pitch = 0.
        self.measure_gim_yaw = 0.
        self.measure_laser_dis = 0.

    def callback(self, msg):
        self.measure_is_new = True
        dx = 1536/2 - msg.center_x
        dy = 864/2 - msg.center_y
        self.measure_is_in_center = abs(dx) < msg.max_length/3 and abs(dy) < msg.max_length/4
        self.measure_gim_pitch = msg.pitch
        self.measure_gim_yaw = msg.yaw
        self.measure_laser_dis = msg.laser_dis


class RTKPositionSubscriber(MessageSubscriber):
    # static member
    __orig_lat = math.radians(30.129161866939494) # math.radians(30.12974118425157)   # aliyun gross
    __orig_lon = math.radians(120.07418009681075) # math.radians(120.07765047891777)
    __orig_alt = 0.             # 16.5

    def __init__(self, topic):
        super(RTKPositionSubscriber, self).__init__(
            topic=topic, 
            msg_type=NavSatFix
        )
        self.my_UAV_x = 0.          # my UAV pos
        self.my_UAV_y = 0.
        self.my_UAV_z = 0.
        self.my_UAV_home_x = 0.
        self.my_UAV_home_y = 0.
        self.my_UAV_home_z = 0.
        self.my_UAV_home_OK = False

    def callback(self, msg):
        UAV_lat = math.radians(msg.latitude)
        UAV_lon = math.radians(msg.longitude)
        UAV_alt = msg.altitude
        Ec = 6378137. * (1 - 21412./6356725. * (math.sin(UAV_lat)**2) ) + UAV_alt
        Ed = Ec * math.cos(UAV_lat)
        d_lat = UAV_lat - RTKPositionSubscriber.__orig_lat
        d_lon = UAV_lon - RTKPositionSubscriber.__orig_lon

        self.my_UAV_x = d_lat * Ec
        self.my_UAV_y = d_lon * Ed
        self.my_UAV_z = RTKPositionSubscriber.__orig_alt - UAV_alt

        if not self.my_UAV_home_OK:
            self.my_UAV_home_x = self.my_UAV_x
            self.my_UAV_home_y = self.my_UAV_y
            self.my_UAV_home_z = self.my_UAV_z
            self.my_UAV_home_OK = True

class VectorSubscriber(MessageSubscriber):
    def __init__(self, topic):
        super(VectorSubscriber, self).__init__(
            topic=topic, 
            msg_type=Vector3Stamped, 
        )
        self.x = 0
        self.y = 0
        self.z = 0
    
    def callback(self, msg):
        self.x = msg.vector.x
        self.y = msg.vector.y
        self.z = msg.vector.z

class Int16Subscriber(MessageSubscriber):
    def __init__(self, topic):
        super(Int16Subscriber, self).__init__(
            topic=topic, 
            msg_type=Int16, 
        )
        self.my_UAV_yaw = 0

    def callback(self, msg):
        my_UAV_yaw = (msg.data + 90)
        if my_UAV_yaw > 180:
            my_UAV_yaw = my_UAV_yaw - 360

        self.my_UAV_yaw = my_UAV_yaw

