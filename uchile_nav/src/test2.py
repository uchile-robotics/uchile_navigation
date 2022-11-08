from move_to import *
from geometry_msgs.msg import PoseWithCovarianceStamped
class Pose():
    def __init__(self):
        self.subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._callback)
        

    def _callback(self, msg):
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        pose_z = msg.pose.pose.position.z
        ori_x  = msg.pose.pose.orientation.x
        ori_y  = msg.pose.pose.orientation.y
        ori_z  = msg.pose.pose.orientation.z
        ori_w  = msg.pose.pose.orientation.w
        ori = [ori_x,ori_y,ori_z,ori_w]
        orie = tf.transformations.euler_from_quaternion(ori)
        
        # print(orie[0])
        # print(orie[1])
        # print(orie[2])

def main():
    rospy.init_node('Pose')
    Pose()
    rospy.spin()

if __name__ == '__main__':
    main()