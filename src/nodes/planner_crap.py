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
        self.dist_min = 1.2
        self.vel_max = 0.8
        self.vel_max_X = 1.0
        self.vel_max_Y = 0.4
        self.k_rep = 0.2
        self.k_atr = 0.2
        self.sideStepAngle = 0.2 # Angle between obs and goal to enable side stepping
        self.k_side = 0.4 # Gain to decide amount of side stepping
        
        # Parameters for continuous modulation of repulsive force
        self.min_rep_scale = 0.2
        self.rotation_scale = 1.0
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
            rospy.logerr("4")
            
            return vel_des
        
    def modify_repulsive_force(self, goal_pos, obs_pos, repulsive_vel):
        # Make sure the value assignments are proper
        alignment_diff = np.arctan2(goal_pos[1], goal_pos[0]) - np.arctan2(obs_pos[1], obs_pos[0])
        rospy.logerr("Alignment_diff: ")
        rospy.logerr(alignment_diff)

        # If obstacle is on the side, we can ignore it
        if np.abs(alignment_diff) > 1.0:
            repulsive_vel = 0.5 * repulsive_vel # Make the repulsive_vel weaker

        # If obstacle is in the way, move sideways
        side_X = 0.0
        side_Y = 0.0
        if np.abs(alignment_diff) < self.sideStepAngle:
            if alignment_diff > 0: # TODO: Check. I think positive difference should be 'pass left'
                side_X =  - repulsive_vel[1] # Note that these rotations are written knowing that repulsive_vel is in the opposite direction of the obstacle
                side_Y =  + repulsive_vel[0]
            else: # And this should be 'pass right'
                side_X =  + repulsive_vel[1]
                side_Y =  - repulsive_vel[0] 

        rospy.logerr("Side velocity: ")
        rospy.logerr([side_X, side_Y])

        repulsive_vel[0] = repulsive_vel[0] + self.k_side * side_X
        repulsive_vel[1] = repulsive_vel[1] + self.k_side * side_Y
        
        d = np.linalg.norm(repulsive_vel) 
        if d > self.vel_max:
            repulsive_vel = repulsive_vel / d * self.vel_max

        return repulsive_vel
    
    ################################################# OLD FUNCTION
    def map_callback(self, msg):
        self.map = json.loads(msg.data)
       
        # Attractive forces
        delta_X = self.map["/goal"][0]
        delta_Y = self.map["/goal"][1]

        delta_Z = np.arctan2(delta_Y, delta_X)
        vel_X = 0.0
        vel_Y = 0.0
        att_vel_X = 0.0
        att_vel_Y = 0.0
        ang_vel_Z = 0.0

        if np.abs(delta_X) > 0.1 or np.abs(delta_Y) > 0.1: # If not at goal, only then try to move
            if np.abs(delta_Z) > 0.3: # and np.linalg.norm(np.array([delta_X, delta_Y])) > 0.3: 
                # If the robot is at least some distance far from the goal and looking away, then turn first
                ang_vel_Z = 0.4*delta_Z
            else:
                ang_vel_Z = 0.2*delta_Z
                if np.abs(delta_X) > 0.1:
                    att_vel_X = self.k_atr*delta_X
                if np.abs(delta_Y) > 0.1:
                    att_vel_Y = self.k_atr*delta_Y

            # normalize it if the norm is too large
            attractive_vel = np.array([att_vel_X, att_vel_Y])
            d = np.linalg.norm(attractive_vel)
            if d > self.vel_max:
                attractive_vel = attractive_vel / d * self.vel_max
            
            rospy.logerr("Attractive_vel: ")
            rospy.logerr(attractive_vel)
            rospy.logerr(delta_X)
            vel_X = attractive_vel[0]
            vel_Y = attractive_vel[1]

            # Obstacles: repulsive forces
            for obs in ['/obstacle1', '/obstacle2', '/obstacle3']:
            # for obs in ['/obstacle1']:
                try:
                    obs_X = self.map[obs][0]
                    obs_Y = self.map[obs][1]
                    repulsive_vel = self.get_repulsive_force(np.array([obs_X, obs_Y]))
                    rospy.logerr("Repulsive_vel: ")
                    rospy.logerr(repulsive_vel)
                    
                    # repulsive_vel = self.modify_repulsive_force(np.array([delta_X, delta_Y]), np.array([obs_X, obs_Y]), repulsive_vel)
                    repulsive_vel = self.modify_repulsive_force_continuous(np.array([delta_X, delta_Y]), np.array([obs_X, obs_Y]), repulsive_vel)
                    
                    rospy.logerr("Repulsive_vel after modification: ")
                    rospy.logerr(repulsive_vel)
                    
                    vel_X = vel_X + repulsive_vel[0]
                    vel_Y = vel_Y + repulsive_vel[1]

                except Exception as e:
                    rospy.logerr(e)
            
        # Normalize the final velocity
        vel_final = np.array([vel_X, vel_Y])
        rospy.logerr("Final velocity: ")
        rospy.logerr(vel_final)
        d = np.linalg.norm(vel_final)
        if d > self.vel_max:
            vel_final = vel_final / d * self.vel_max

        if np.abs(delta_X) <= 0.1 and np.abs(delta_Y) <= 0.1:
            vel_final[0] = 0.0
            vel_final[1] = 0.0
        # Twist
        self.cmd = geometry_msgs.msg.Twist()
        self.cmd.linear.x = vel_final[0]
        self.cmd.linear.y = vel_final[1]
        self.cmd.angular.z = ang_vel_Z
        
    def modify_repulsive_force_continuous(self, goal_pos, obs_pos, repulsive_vel):
        # Make sure the value assignments are proper
        alignment_diff = np.arctan2(goal_pos[1], goal_pos[0]) - np.arctan2(obs_pos[1], obs_pos[0])
        rospy.logerr("Alignment_diff: ")
        rospy.logerr(alignment_diff)

        # Change magnitude for larger diffrence, so that we don't push when we're on the side and past the obstacle
        repulsive_vel = repulsive_vel * max(self.min_rep_scale, (np.pi/2 - np.abs(alignment_diff))/(np.pi/2)) # Max at 0 degrees, nin at 90 degrees or higher
        
        # Rotate vector for smaller angle
        if np.sign(alignment_diff) == 0:
            alignment_diff += 0.001
        rotation_angle =  - self.rotation_scale * np.abs(np.pi/2 - np.abs(alignment_diff)) * np.sign(alignment_diff) # Positive (CCW) rotation for negative difference

        rot_matrix = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle)], [np.sin(rotation_angle), np.cos(rotation_angle)]])

        repulsive_vel = np.dot(rot_matrix, repulsive_vel)
        
        d = np.linalg.norm(repulsive_vel) 
        if d > self.vel_max:
            repulsive_vel = repulsive_vel / d * self.vel_max

        return repulsive_vel
    
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