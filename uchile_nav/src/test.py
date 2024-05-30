#!/usr/bin/env python
from move_to import *
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan
import traceback
import time 

def main():
    rospy.init_node('fish')
    m = Move()
    x = input()
    y = input()
    w = input()
    m.set_pose(x,y,w) 
    m.go()
    


# if __name__ == '__main__':
#     rospy.init_node('fish')
#     m = Move()
#     m.set_pose(1, 3, 60)
#     #m.get_pose()
#     m.go()
# '''


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


# if __name__ == '__main__':
#     main()
#     #main_w()


##si, lo escribi a la mala xd
##lo arreglo dps
##pendiente, revisar pq robot parte como el orto y arreglar eso 
def demo():
    print("inciando nodo")
    rospy.init_node('fish')
    rooms = {"inicio": (-1.3478, 5.6232, 0.0), "refri": (-1.9517, 2.4806, 0.45), "pieza": (-0.4331, 1.4848, 0.0), "sofa": (1.3478, 5.6232, 1.0)}
    print("nodo iniciado!")
    m = Move()

    print("Starting demo .... ")
    print("First goal is the refri")
    room = rooms["refri"]
    print("refri is at: ", room)
    m.set_pose(room[0], room[1], room[2])
    m.go()

    print("waiting 5 second before then next goal...")
    time.sleep(5)
    print("next goal is pieza")
    room = rooms["pieza"]
    print("pieza is at:  ", room)
    m.set_pose(room[0], room[1], room[2])
    m.go()

    print("waiting 5 second before then next goal...")
    time.sleep(5)
    print("next goal is sofa")
    room = rooms["sofa"]
    print("sofa is at:  ", room)
    m.set_pose(room[0], room[1], room[2])
    m.go()

    print("waiting 5 second before returning to starting point...")
    time.sleep(5)
    print("returnig")
    room = rooms["inicio"]
    print("inicio is at:  ", room)
    m.set_pose(room[0], room[1], room[2])
    m.go()

    print("DEMO IS FINISHEEEEEEEEEEEED C:")

    return

if __name__ == '__main__':
    demo()
