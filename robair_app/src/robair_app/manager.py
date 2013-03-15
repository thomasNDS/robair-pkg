import rospy
from .xmpp.client import ClientXMPP
from .xmpp.rpc import remote
# from robair_msgs.msg import Command


class RobotManager(ClientXMPP):
    def __init__(self, node_name):
        jid = rospy.get_param('robot_jabber_id')
        password = rospy.get_param('robot_jabber_password')
        super(RobotManager, self).__init__(jid, password)
        rospy.init_node(node_name)

    @remote
    def div(self, a, b):
        return a / b

    @remote
    def echo(self, message):
        return message

    @remote
    def add(self, *args):
        return sum((int(i) for i in args))

    @remote
    def whoami(self):
        return self.current_rpc_session().client_jid


class ClientManager(ClientXMPP):
    def __init__(self, node_name):
        jid = rospy.get_param('tv_jabber_id')
        password = rospy.get_param('tv_jabber_password')
        super(ClientManager, self).__init__(jid, password)
        rospy.init_node(node_name)
        self.robot_jid = rospy.get_param('robot_jabber_id')
        self.proxy_robot = self.get_proxy(self.robot_jid)


    #     self.topic_name = "/info/battery"
    #     rospy.Subscriber(self.topic_name, Command, self.callback)
    #     rospy.spin()

    # def callback(self, data):
    #     rospy.loginfo("%s: I heard  speed %s - curve %s :D"
    #                   % (rospy.get_name(), data.speed, data.angle))
    #     topic_to_serialize = {"topic": self.topic_name, "data": data}
    #     msg = cPickle.dumps(topic_to_serialize)
    #     self.send_message(self.robot_jid, msg)
