#!/usr/bin/env python

import numpy as np
import random
import math
import rospy

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):
        
      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

      # Read the velocities architecture
      self.velocity_arch = rospy.get_param("velocities_architecture")
      print "The selected velocities architecture is " + self.velocity_arch

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
      # Choose architecture
      if self.velocity_arch == "subsumption":
        self.produceSpeedsSubsumption()
      else:
        self.produceSpeedsMotorSchema()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0 
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
        print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produce speeds from sonars
    def produceSpeedsSonars(self):
      # Get the sonars' measurements
      front   = self.sonar_aggregation.sonar_front_range
      left    = self.sonar_aggregation.sonar_left_range
      right   = self.sonar_aggregation.sonar_right_range
      r_left  = self.sonar_aggregation.sonar_rear_left_range
      r_right = self.sonar_aggregation.sonar_rear_right_range
        
      linear  = 0
      angular = 0

      # YOUR CODE HERE ------------------------------------------------------
      # Adjust the linear and angular velocities using the five sonars values
      '''e = 0.0001 * random.randint(1,10)

      if front >= 280:
	front  = 10e16
      if left >= 280:
	left = 10e16
      if right >= 280:
	right  = 10e16
      if r_left >= 280:
	r_left = 10e16
      if r_right >= 280:
	r_right  = 10e16
  

      force_front = 1/(front + e)**2
      force_left = 1/(left + e)**2
      force_right = 1/(right + e)**2
      force_r_left = 1/(r_left + e)**2
      force_r_right = 1/(r_right + e)**2


      linear = 0.7 * front/(r_left+r_right + front + e)
      angular = ((0.3 * left + 0.2 * left/(r_left+e) * random.randint(1,10) * 0.1) - (0.3 * right + 0.2 * right/(r_right+e) * random.randint(1,10) * 0.1)) * force_front
      if linear <= 0.10 and angular <= 0.01 :
	if angular >= 0:
	     angular = - 0.01 * random.randint(10,20) 
	else:
	     angular = + 0.01 * random.randint(10,20)'''

      # ---------------------------------------------------------------------

      return [linear, angular]

    # Produces speeds from the laser
    def produceSpeedsLaser(self):

      # Get the laser scan
      scan   = self.laser_aggregation.laser_scan
        
      linear  = 0
      angular = 0

      # YOUR CODE HERE ------------------------------------------------------
      # Adjust the linear and angular velocities using the laser scan
      scan = np.clip(scan,0,10)
      degrees = np.matrix(scan) * 667/240 
      deg = np.cos(np.array(np.arange(-166,167,1))/166)
      f = np.array(np.arange(-166,167,1))/166
      f2 = np.array(np.arange(-150,150,1))/150
      gaussianLinear = np.exp(-f**2/(2*(0.5 ** 2)))
      linear = 0.6 * (np.mean(scan[166:499] * gaussianLinear))/10

      gaussianAngular = np.exp(-f2**2/(2*(0.5 ** 2)))
      #deg2 = np.cos(np.array(np.arange(-100,100,1))/100)
      left = (np.mean(scan[0:300] * gaussianAngular))/10

      #deg3 = np.cos(np.array(np.arange(-100,100,1))/100)
      right = np.mean(scan[367:667] * gaussianAngular)/10
	
      angular = (1/linear) * (-left + right)/1.5

      # ---------------------------------------------------------------------

      return [linear, angular]

    # Combines the speeds into one output using a subsumption approach
    def produceSpeedsSubsumption(self):
      
      # Produce target if not existent
      if self.move_with_target == True and self.navigation.target_exists == False:
        self.navigation.selectTarget()

      # Get the submodules' speeds
      [l_sonar, a_sonar] = self.produceSpeedsSonars()
      [l_laser, a_laser] = self.produceSpeedsLaser()
      [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Combine the speeds following the subsumption architecture
      # YOUR CODE HERE ------------------------------------------------------
      '''front_sonar = self.sonar_aggregation.sonar_front_range
      left_sonar = self.sonar_aggregation.sonar_left_range
      right_sonar = self.sonar_aggregation.sonar_right_range
      
      # linear velocity
      if front_sonar >= 0.5 or front_sonar >= 2.5:
	self.linear_velocity = l_laser
      else :
	self.linear_velocity = l_sonar

      # angular velocity
      if (left_sonar >= 0.5 or left_sonar >= 2.5) or (right_sonar >= 0.5 or right_sonar >= 2.5):
	self.angular_velocity = a_laser
      else :
	self.angular_velocity = a_sonar'''

      # ---------------------------------------------------------------------

    # Combines the speeds into one output using a motor schema approach
    def produceSpeedsMotorSchema(self):
 
      # Produce target if not existent
      if self.move_with_target == True and self.navigation.target_exists == False:
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_sonar, a_sonar] = self.produceSpeedsSonars()
      [l_laser, a_laser] = self.produceSpeedsLaser()
      [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()

      self.linear_velocity  = l_laser
      self.angular_velocity = a_laser
        
      # Get the speeds using the motor schema approach
      # YOUR CODE HERE ------------------------------------------------------
      '''self.linear_velocity = (l_sonar + l_laser)/2
      self.angular_velocity = (a_sonar + a_laser)/2'''
      # ---------------------------------------------------------------------

    # Assistive functions - Do you have to call them somewhere?
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
