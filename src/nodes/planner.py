#!/usr/bin/env python3
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal

import numpy as np
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import json


class Planner:
    def __init__(self):
        super().__init__()

        # Initialize Node
        rospy.init_node('Planner', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Initialize subscriber
        self.map_sub = rospy.Subscriber('/map', String, self.map_callback)
        self.map = None
        
        # Intialize publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.cmd = None
        self.rate = rospy.Rate(10)  # Publisher frequency

        # TODO BEGIN MRSS: Add attributes (If needed)

        # END MRSS

    def map_callback(self, msg):
        self.map = json.loads(msg.data)

        # TODO BEGIN MRSS: Use map for planning
       
        # Attractive forces
        # END MRSS
        delta_X = self.map["/goal"][0]
        delta_Y = self.map["/goal"][1]

        delta_Z = np.arctan2(delta_Y, delta_X)

        vel_X = 0.0
        vel_Y = 0.0
        vel_Z = 0.0
        if np.abs(delta_Z) > 0.3:
            vel_Z = 0.8*np.sign(delta_Z)
        else:
            if np.abs(delta_X) > 0.2:
                vel_X = 0.2*np.sign(delta_X)
            if np.abs(delta_Y) > 0.2:
                vel_Y = 0.2*np.sign(delta_Y)
            vel_Z = 0.5*np.sign(delta_Z)
        
        # Repulsive forces
        obs_X = self.map["/obstacle1"][0]
        obs_Y = self.map["/obstacle1"][1]

        d = np.linalg.norm(obs_X, obs_Y)
        # Twist
        self.cmd = geometry_msgs.msg.Twist()

        # TODO BEGIN MRSS: Update the current command
        self.cmd.linear.x = vel_X
        self.cmd.linear.y = vel_Y
        self.cmd.angular.z = vel_Z
        # END MRSS

    def spin(self):
        '''
        Spins the node.
        '''
        try:
            while not rospy.is_shutdown():
                if self.cmd is not None:
                    # Publish
                    self.cmd_pub.publish(self.cmd)
                else:
                    rospy.logwarn("SKIP")

                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down planner.")

    def on_shutdown(self):
        '''
        Called on node shutdown.
        '''
        pass


if __name__ == '__main__':
    try:
        node = Planner()
        node.spin()
    except rospy.ROSInterruptException:
        pass
