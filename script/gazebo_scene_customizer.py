from gazebo_topic_publisher import *
import pygazebo.msg.physics_pb2
import pygazebo.msg.light_pb2
import copy
class GazeboPhysicsSetting(GazeboTopicPublisher):
    def __init__(self):
        super(self.__class__, self).__init__('/gazebo/default/physics','gazebo.msgs.Physics')
        self.default_message_contents = pygazebo.msg.physics_pb2.Physics()
        self.default_message_contents.type = 1
        self.default_message_contents.iters = 50
        self.default_message_contents.sor = 1.3
        self.default_message_contents.cfm = 0
        self.default_message_contents.erp = 0.2
        self.default_message_contents.contact_max_correcting_vel = 100
        self.default_message_contents.contact_surface_layer = 0.001
        self.default_message_contents.gravity.x = 0
        self.default_message_contents.gravity.y = 0
        self.default_message_contents.gravity.z = -9.8
        self.default_message_contents.enable_physics = True
        self.default_message_contents.real_time_update_rate = 1000
        self.default_message_contents.max_step_size = 0.001

    def setEnablePhysics(self, enable = True):
        message = copy.copy(self.default_message_contents)
        message.enable_physics = enable
        self.publishMessage(message)

class GazeboLightSetting(GazeboTopicPublisher):
    def __init__(self):
        super(self.__class__, self).__init__('/gazebo/default/light','gazebo.msgs.Light')
        self.light_type_dict = {'spot' : 0, 'point' : 1, 'directional' : 2}

        self.default_message_contents = pygazebo.msg.light_pb2.Light()
        self.default_message_contents.pose.position.x = 0
        self.default_message_contents.pose.position.y = 0
        self.default_message_contents.pose.position.z = 1
        self.default_message_contents.pose.orientation.x = 0
        self.default_message_contents.pose.orientation.y = 0
        self.default_message_contents.pose.orientation.z = 0
        self.default_message_contents.pose.orientation.w = 1
        self.default_message_contents.diffuse.r = 0.9
        self.default_message_contents.diffuse.g = 0.9
        self.default_message_contents.diffuse.b = 0.9
        self.default_message_contents.diffuse.a = 1.0
        self.default_message_contents.specular.r = 0.9
        self.default_message_contents.specular.g = 0.9
        self.default_message_contents.specular.b = 0.9
        self.default_message_contents.specular.a = 1.0
        self.default_message_contents.attenuation_constant = 1
        self.default_message_contents.attenuation_linear = 0
        self.default_message_contents.attenuation_quadratic = 0
        self.default_message_contents.range = 50
        self.default_message_contents.name = "my_light"
        self.default_message_contents.type = self.light_type_dict['point']

    def setPointSourcePose(self, name, light_type, x, y, z, qx, qy, qz, qw):
        message = copy.copy(self.default_message_contents)
        message.name = name
        message.type = self.light_type_dict[light_type]
        # message.pose = pose
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        self.publishMessage(message)

