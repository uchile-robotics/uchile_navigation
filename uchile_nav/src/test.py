#!/usr/bin/env python
from move_to import *
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan
import traceback

def main():
    rospy.init_node('fish')
    m = Move()
    x = input()
    y = input()
    w = input()
    m.set_pose(x,y,w) 
    m.go()
    

    # get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
    # print(get_plan)
    # req = GetPlan()
    # req.start = (0,0,0)
    # req.goal = (2.561, 0.068,1)
    # req.tolerance = .5
    # try:
    #     resp = get_plan(req.start, req.goal, req.tolerance)

    # except:
    #     error = traceback.format_exc()
    #     print("error: ")
    #     print(error)
    # print(resp)

#if __name__ == '__main__':
#    main()
'''
if __name__ == '__main__':
    rospy.init_node('fish')
    m = Move()
    m.set_pose(1, 3, 60)
    #m.get_pose()
    m.go()
'''


def main_w():
    rospy.init_node('fish')
    rooms = {"0": (2.561, 0.068), "1": (5.729, 1.341), "2": (6.114, 3.559), "3":(1.853, 3.303)}
    m = Move()
    w=90
    print("where would you like to go? (please select as number)")
    print("avaliable: 0=hall, 1=living, 2=kitchen, 3=bedroom")
    place = input()
    place = str(place)
    print("you choose:"+place)
    room = rooms.get(place, rooms["0"])
    #verificar si estoy mirando un obstaculo 

    m.set_pose(room[0], room[1], w)
    m.go()    


if __name__ == '__main__':
    main()
    #main_w()