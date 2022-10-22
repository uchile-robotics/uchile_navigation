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

if __name__ == '__main__':
    main()
'''
if __name__ == '__main__':
    rospy.init_node('fish')
    m = Move()
    m.set_pose(1, 3, 60)
    #m.get_pose()
    m.go()
'''
