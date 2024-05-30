from move_to import *
from actionlib_msgs.msg import GoalStatusArray #GoalStatusArray

class nav_status():
    def __init__(self):
        self.subscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, self._callback)
        self.status = 0

    def _callback(self, msg):
        msj = msg.status_list
        #status = msj.status
        print("----------------------------")
        print(msj)
        print( type(msj))
        print("status:", status)
        #self.status = status



def main():
    rospy.init_node('Pose', anonymous=False)
    #nav_st = nav_status()
    
    print("*************")
    #print("status act:" , nav_st.status)
    rospy.spin()

if __name__ == '__main__':
    main()

