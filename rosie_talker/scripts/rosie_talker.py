#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('rosie_talker')
import rospy
import sys
import os
import random

# Node example class.
class rosie_talker():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # Get the ~private namespace parameters from command line or launch file.
        rate = float(rospy.get_param('~rate', '1.0'))
        filename = rospy.get_param('~sentences', None)
        if filename == None:
            print "didnt get sentences"
            return
        sentences = open(os.path.abspath(filename), 'r')
        lines = sentences.readlines()
        length = len(lines)
        rospy.loginfo('rate = %d', rate)
        rospy.loginfo('file = %s', filename)
        # Main while loop.
        last = -1
        while not rospy.is_shutdown():
            nbr = random.randint(0, length-1)
            while nbr == last:
                nbr = random.randint(0, length-1)
            last = nbr
            line = lines[nbr]
            os.system('espeak "' + line + '"')
            # Sleep for a while before publishing new messages. Division is so rate != period.
            if rate:
                rospy.sleep(1/rate)
            else:
                rospy.sleep(1.0)

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('rosie_talker')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = rosie_talker()
    except rospy.ROSInterruptException: pass
