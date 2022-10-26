#!/usr/bin/env python
from move_to import *



def main():
    rospy.init_node('fish')
    m = Move()
    x = input()
    y = input()
    w = input()
    m.set_pose(x,y,w) 
    m.go()

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
    w=1
    print("where would you like to go? (please select as number)")
    print("avaliable: 0=hall, 1=living, 2=kitchen, 3=bedroom")
    place = input()
    place = str(place)
    print("you choose:"+place)
    room = rooms.get(place, rooms["0"])
    #default es el hall
    m.set_pose(room[0], room[1], w)
    m.go()    


if __name__ == '__main__':
    main()
    main_w()