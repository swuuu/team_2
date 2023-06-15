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
        self.dist_min = 1.5
        self.vel_max = 1.2
        self.vel_max_X = 1.0
        self.vel_max_Y = 0.4
        self.k_rep = 0.2
        self.k_atr = 0.2
        # END MRSS
    
    def get_repulsive_force(self, delta_pos):
       
        # print("self.pos_obs=", self.pos_obs, "pos_fbk=", pos_fbk )
        d = np.linalg.norm( delta_pos )
        
        if d > self.dist_min: # Far enough away, ignore the obstacle
	        return np.array ([0,0])
        else:
            dd_dq 	= 2 * (delta_pos)           
            vel_des = -self.k_rep / (d*d) * (1/d - 1/self.dist_min) * dd_dq            
            
            # normalize it if the norm is too large
            d = np.linalg.norm(vel_des) 
            if d > self.vel_max:
                vel_des = vel_des / d * self.vel_max
            return vel_des
    
    ################################################# OLD FUNCTION
    #  def map_callback(self, msg):
    #     self.map = json.loads(msg.data)
       
    #     # Attractive forces
    #     delta_X = self.map["/goal"][0]
    #     delta_Y = self.map["/goal"][1]

    #     vel_X = 0.0
    #     vel_Y = 0.0
    #     ang_vel_Z = 0.0
    #     if np.abs(delta_Z) > 0.3:
    #         ang_vel_Z = 0.4*delta_Z
    #     else:
    #         if np.abs(delta_X) > 0.1:
    #             vel_X = 0.2*delta_X
    #         if np.abs(delta_Y) > 0.1:
    #             vel_Y = 0.2*delta_Y
    #         ang_vel_Z = 0.2*delta_Z
        
    #     if np.abs(delta_X) > 0.05:
    #         att_vel_X = self.k_atr*delta_X
    #     if np.abs(delta_Y) > 0.05:
    #         att_vel_Y = self.k_atr*delta_Y

    #     # normalize it if the norm is too large
    #     vel_des = np.array([att_vel_X, att_vel_Y])
    #     d = np.linalg.norm(vel_des)
    #     if d > self.vel_max:
    #         vel_des = vel_des / d * self.vel_max

    #     if vel_des[1] > self.vel_max_Y: # TODO: Check this one
    #         vel_des = vel_des / vel_des[1] * self.vel_max
        
    #     # Obstacles: repulsive forces
    #     # for obs in ['/obstacle1', '/obstacle2', '/obstacle3']:
    #     for obs in ['/obstacle1']:
    #         try:
    #             obs_X = self.map[obs][0]
    #             obs_Y = self.map[obs][1]
    #             repulsive_vel = self.get_repulsive_force(np.array([obs_X, obs_Y]))
    #             print('##############################################################')
    #             rospy.logerr(repulsive_vel)
    #             print(repulsive_vel)
    #             vel_X = vel_des[0] + repulsive_vel[0]
    #             vel_Y = vel_des[1] + repulsive_vel[1]
    #         except Exception as e:
    #             rospy.logerr(e)


    #     # Twist
    #     self.cmd = geometry_msgs.msg.Twist()
    #     self.cmd.linear.x = vel_X
    #     self.cmd.linear.y = vel_Y
    #     self.cmd.angular.z = ang_vel_Z
        

    def map_callback(self, msg):
        self.map = json.loads(msg.data)
       
        # Attractive forces
        delta_X = self.map["/goal"][0]
        delta_Y = self.map["/goal"][1]

        vel_X = 0.0
        vel_Y = 0.0
        ang_vel_Z = 0.0
        
        if np.abs(delta_X) > 0.05:
            att_vel_X = self.k_atr*delta_X
        if np.abs(delta_Y) > 0.05:
            att_vel_Y = self.k_atr*delta_Y

        # normalize it if the norm is too large
        vel_des = np.array([att_vel_X, att_vel_Y])
        d = np.linalg.norm(vel_des)
        if d > self.vel_max:
            vel_des = vel_des / d * self.vel_max

        if vel_des[1] > self.vel_max_Y: # TODO: Check this one
            vel_des = vel_des / vel_des[1] * self.vel_max
        
        # Obstacles: repulsive forces
        # for obs in ['/obstacle1', '/obstacle2', '/obstacle3']:
        for obs in ['/obstacle1']:
            try:
                obs_X = self.map[obs][0]
                obs_Y = self.map[obs][1]
                repulsive_vel = self.get_repulsive_force(np.array([obs_X, obs_Y]))
                rospy.logerr(repulsive_vel)
                vel_des[0] = vel_des[0] + repulsive_vel[0]
                vel_des[1] = vel_des[1] + repulsive_vel[1]
            except Exception as e:
                rospy.logerr(e)

        # normalize again it if the norm is too large
        d = np.linalg.norm(vel_des)
        if d > self.vel_max:
            vel_des = vel_des / d * self.vel_max
        
        delta_Z = np.arctan2(vel_des[1], vel_des[0])
        if np.abs(delta_Z) > 0.3:
            ang_vel_Z = 0.7*delta_Z
            vel_X = 0.0
            vel_Y = 0.0
        else:
            ang_vel_Z = 0.2*delta_Z
            vel_X = vel_des[0]
            vel_Y = vel_des[1]

        # Twist
        self.cmd = geometry_msgs.msg.Twist()
        self.cmd.linear.x = vel_X
        self.cmd.linear.y = vel_Y
        self.cmd.angular.z = ang_vel_Z

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