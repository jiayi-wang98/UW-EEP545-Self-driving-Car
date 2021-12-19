#!/usr/bin/env python

import rospy
import numpy as np
import math
import sys

import utils

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

SCAN_TOPIC = '/car/scan' # The topic to subscribe to for laser scans
CMD_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation' # The topic to publish controls to
POSE_TOPIC = '/car/car_pose' # The topic to subscribe to for current pose of the car
                                  # NOTE THAT THIS IS ONLY NECESSARY FOR VIZUALIZATION
VIZ_TOPIC = '/laser_wanderer/rollouts' # The topic to publish to for vizualizing
                                       # the computed rollouts. Publish a PoseArray.

MAX_PENALTY = 10000 # The penalty to apply when a configuration in a rollout
                    # goes beyond the corresponding laser scan
                    

'''
Wanders around using minimum (steering angle) control effort while avoiding crashing
based off of laser scans. 
'''
class LaserWanderer:

  '''
  Initializes the LaserWanderer
    rollouts: An NxTx3 numpy array that contains N rolled out trajectories, each
              containing T poses. For each trajectory, the t-th element represents
              the [x,y,theta] pose of the car at time t+1
    deltas: An N dimensional array containing the possible steering angles. The n-th
            element of this array is the steering angle that would result in the 
            n-th trajectory in rollouts
    speed: The speed at which the car should travel
    compute_time: The amount of time (in seconds) we can spend computing the cost
    laser_offset: How much to shorten the laser measurements
  '''
  def __init__(self, rollouts, deltas, speed, compute_time, laser_offset):
    # Store the params for later
    self.rollouts = rollouts
    self.deltas = deltas
    self.speed = speed
    self.compute_time = compute_time
    self.laser_offset = laser_offset
    
    # YOUR CODE HERE
    self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size=10)# Create a publisher for sending controls
    self.laser_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.wander_cb,queue_size=10)# Create a subscriber to laser scans that uses the self.wander_cb callback
    self.viz_pub = rospy.Publisher(VIZ_TOPIC, PoseArray, queue_size=10)# Create a publisher for vizualizing trajectories. Will publish PoseArrays
    self.viz_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.viz_sub_cb,queue_size=10)# Create a subscriber to the current position of the car
    # NOTE THAT THIS VIZUALIZATION WILL ONLY WORK IN SIMULATION. Why?
    # A: Cause the real robot cannot get its pose information.
  '''
  Vizualize the rollouts. Transforms the rollouts to be in the frame of the world.
  Only display the last pose of each rollout to prevent lagginess
    msg: A PoseStamped representing the current pose of the car
  '''  
  def viz_sub_cb(self, msg):
    # Create the PoseArray to publish. Will contain N poses, where the n-th pose
    # represents the last pose in the n-th trajectory
    pa = PoseArray()
    pa.header.frame_id = '/map'
    pa.header.stamp = rospy.Time.now()
 
    # Transform the last pose of each trajectory to be w.r.t the world and insert into
    # the pose array
    # YOUR CODE HERE
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.orientation)])
    #print(self.rollouts[0][1])
    
    
    for i in range(len(self.deltas)):
      roll_xy=np.array([[self.rollouts[i][-1][0]],[self.rollouts[i][-1][1]]])
      theta0=self.rollouts[i][-1][2]
      pub_pose=Pose()

      #rotation
      roll_xy=utils.rotation_matrix(cur_pose[2])*roll_xy
      #add offset
      pub_pose.position.x=roll_xy[0][0].sum()+cur_pose[0]
      pub_pose.position.y=roll_xy[1][0].sum()+cur_pose[1]
      pub_pose.orientation=utils.angle_to_quaternion(theta0+cur_pose[2])

      #print(self.rollouts[i,-1,:])
      #print(roll_xy,theta0+cur_pose[2])
      
      pa.poses.append(pub_pose)
      #print(i,pa)      
    
    self.viz_pub.publish(pa)
    
  '''
  Compute the cost of one step in the trajectory. It should penalize the magnitude
  of the steering angle. It should also heavily penalize crashing into an object
  (as determined by the laser scans)
    delta: The steering angle that corresponds to this trajectory
    rollout_pose: The pose in the trajectory 
    laser_msg: The most recent laser scan
  '''  
  def compute_cost(self, delta, rollout_pose, laser_msg, laser_range):
  
    # Initialize the cost to be the magnitude of delta
    # Consider the line that goes from the robot to the rollout pose
    # Compute the angle of this line with respect to the robot's x axis
    # Find the laser ray that corresponds to this angle
    # Add MAX_PENALTY to the cost if the distance from the robot to the rollout_pose 
    # is greater than the laser ray measurement - np.abs(self.laser_offset)
    # Return the resulting cost
    # Things to think about:
    #   What if the angle of the pose is less (or greater) than the angle of the
    #   minimum (or maximum) laser scan angle
    #   What if the corresponding laser measurement is NAN or 0?
    # NOTE THAT NO COORDINATE TRANSFORMS ARE NECESSARY INSIDE OF THIS FUNCTION
    
    # YOUR CODE HERE

    
    #compute cost
    cost=np.abs(delta)
    theta=np.pi/2
    
    #side trajectory  
    theta_left=np.pi/2
    theta_right=np.pi/2
    
    #car width
    width_2=0.1
    
    #distance of pose and center of the car
    distance=np.sqrt(rollout_pose[0]**2+rollout_pose[1]**2)

    # if y==0, then theta=0 or pi
    if (rollout_pose[0]!=0):
      theta=np.arctan(rollout_pose[1]/rollout_pose[0])
    elif rollout_pose[1]<0:
      theta=-np.pi/2
    
    # add the width impact
    # left side theta
    if (rollout_pose[0]!=0):
      theta_left=np.arctan((rollout_pose[1]+width_2)/rollout_pose[0])
    elif rollout_pose[1]+width_2<0:
      theta_left=-np.pi/2


    #right side theta
    if (rollout_pose[0]!=0):
      theta_right=np.arctan((rollout_pose[1]-width_2)/rollout_pose[0])
    elif rollout_pose[1]-width_2<0:
      theta_right=-np.pi/2

    #find laser ray
    '''
    std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
    float32 angle_min
    float32 angle_max
    float32 angle_increment
    float32 time_increment
    float32 scan_time
    float32 range_min
    float32 range_max
    float32[] ranges
    float32[] intensities

    '''
    #calculate idx
    idx=int(((theta-laser_msg.angle_min)%(2*np.pi))/laser_msg.angle_increment)
    idx_left=int(((theta_left-laser_msg.angle_min)%(2*np.pi))/laser_msg.angle_increment)
    idx_right=int(((theta_right-laser_msg.angle_min)%(2*np.pi))/laser_msg.angle_increment)


    d_dist=0
    d_dist_left=0
    d_dist_right=0

    #calculate distances diff of laser
    d_dist=laser_range[idx]-np.abs(self.laser_offset)-distance
    d_dist_left=laser_range[idx_left]-np.abs(self.laser_offset)-distance
    d_dist_right=laser_range[idx_right]-np.abs(self.laser_offset)-distance
    if (d_dist<0 or d_dist_left<0 or d_dist_right<0):
      cost=MAX_PENALTY #if any side hit the wall, cost is MAX_PENALTY
    #else:
      #cost+=np.exp(10*np.abs(delta))+1/(1+float(d_dist))+1/(1+float(d_dist_left))+1/(1+float(d_dist_right))
    #print(delta,laser_range[idx],d_dist,cost)
    return cost
    
  '''
  Controls the steering angle in response to the received laser scan. Uses approximately
  self.compute_time amount of time to compute the control
    msg: A LaserScan
  '''
  def wander_cb(self, msg):
    start = rospy.Time.now().to_sec() # Get the time at which this function started
    
    
    # A N dimensional matrix that should be populated with the costs of each
    # trajectory up to time t <= T
    delta_costs = np.zeros(self.deltas.shape[0], dtype=np.float) 
    traj_depth = 0

    # Evaluate the cost of each trajectory. Each iteration of the loop should calculate
    # the cost of each trajectory at time t = traj_depth and add those costs to delta_costs
    # as appropriate
    
    # Pseudo code
    # while(you haven't run out of time AND traj_depth < T):
    #   for each trajectory n:
    #       delta_costs[n] += cost of the t=traj_depth step of trajectory n
    #   traj_depth += 1 
    # YOUR CODE HERE

    #laser_msg pre process, in real robot it has intensities which might be 0.0, first translate these data into the average of the nearest left-side and right-side none zero values.
    # in sim, it may be nan, which can be processed the same as 0.0 intensities in real robot.
    laser_no_nan=np.zeros(len(msg.ranges))
    
    if len(msg.intensities)<len(msg.ranges): #simulation
      for i in range(int(0.25*len(msg.ranges)),int(0.75*len(msg.ranges))):  #-90 degree to 90 degree, can expand if compute time is longer
        if np.isnan(msg.ranges[i])==False: #in real robot: use msg.intensities[i]!=0.0)
          laser_no_nan[i]=msg.ranges[i]
        else:
          k=1
          m=1
          while (np.isnan(msg.ranges[i-k])):
            k+=1
          while (np.isnan(msg.ranges[(i+m)%len(msg.ranges)])):
            m+=1
          laser_no_nan[i]=0.5*(msg.ranges[i-k]+msg.ranges[(i+m)%len(msg.ranges)])    #take the average
      
    else: #real robot
      for i in range(int(0.25*len(msg.ranges)),int(0.75*len(msg.ranges))):
        if msg.intensities[i]!=0.0: #sim: np.isnan(msg.ranges[i]==False)
          laser_no_nan[i]=msg.ranges[i]
        else:
          k=1
          m=1
          while (msg.intensities[i-k]==0.0):
            k+=1
          while (msg.intensities[(i+m)%len(msg.ranges)]==0.0):
            m+=1
          laser_no_nan[i]=0.5*(msg.ranges[i-k]+msg.ranges[(i+m)%len(msg.ranges)]) 
 
    #Take the spacial average to decrease the impact of random noise
    avg_range=3
    window=laser_no_nan[:avg_range]
    laser_range_avg=np.zeros(len(msg.ranges))
    
    for i in range(int(0.25*len(msg.ranges)),int(0.75*len(msg.ranges))):
      laser_range_avg[i]=max(min(window.sum()/avg_range,msg.range_max),msg.range_min)
      window=np.delete(window,0)
      window=np.append(window,laser_no_nan[i+(avg_range+1)/2])
      #print((msg.angle_min+i*msg.angle_increment)/np.pi*180,msg.ranges[i],laser_no_nan[i],laser_range_avg[i])
  
    #collect cost
    while ((rospy.Time.now().to_sec()-start<self.compute_time)and(traj_depth<self.rollouts.shape[1])):
      for i in range(self.deltas.shape[0]):
        delta_costs[i]+= self.compute_cost(self.deltas[i], self.rollouts[i][traj_depth], msg,laser_range_avg)
      traj_depth += 1 
    
    print(traj_depth)
    print(delta_costs)
    #print(self.deltas)
    # Find the delta that has the smallest cost and execute it by publishing
    # YOUR CODE HERE
    
    idx=np.argmin(delta_costs) #default delta=0
    delta = self.deltas[idx]
    print(delta)

    # Setup the control message
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed
    
    # Send the control message
    self.cmd_pub.publish(ads)
    
'''
Apply the kinematic model to the passed pose and control
  pose: The current state of the robot [x, y, theta]
  control: The controls to be applied [v, delta, dt]
  car_length: The length of the car
Returns the resulting pose of the robot
'''
def kinematic_model_step(pose, control, car_length):
  # Apply the kinematic model
  # Make sure your resulting theta is between 0 and 2*pi
  # Consider the case where delta == 0.0
  # YOUR CODE HERE
  [x0,y0,theta0]=pose
  steer_angle=control[1]
  steer_length=control[0]*control[2]
  new_pose=np.zeros(3)

  if steer_angle==0:
    new_pose[0]=x0+steer_length*np.cos(theta0)
    new_pose[1]=y0+steer_length*np.sin(theta0)
    new_pose[2]=theta0

  else:
    steer_r=car_length/np.tan(steer_angle)
    d_theta=steer_length/steer_r
    new_pose[0]=x0+steer_length*np.cos(theta0+d_theta*0.5)   #x+dx
    new_pose[1]=y0+steer_length*np.sin(theta0+d_theta*0.5)   #y+dy
    new_pose[2]=(theta0+d_theta)%(2*np.pi)  #theta+d_theta
  
  return new_pose
    
'''
Repeatedly apply the kinematic model to produce a trajectory for the car
  init_pose: The initial pose of the robot [x,y,theta]
  controls: A Tx3 numpy matrix where each row is of the form [v,delta,dt]
  car_length: The length of the car
Returns a Tx3 matrix where the t-th row corresponds to the robot's pose at time t+1
'''
def generate_rollout(init_pose, controls, car_length):
  # YOUR CODE HERE
  [x0,y0,theta0]=init_pose
  rollout=np.zeros((controls.shape))
 
  for i in range(rollout.shape[0]):

    #the first use initial pose to calculate
    if i==0:
      rollout[i]=kinematic_model_step(init_pose, controls[i], car_length)   #x+dx
    else:
      rollout[i]=kinematic_model_step(rollout[i-1], controls[i], car_length)  #x+dx

  return rollout
'''
Helper function to generate a number of kinematic car rollouts
    speed: The speed at which the car should travel
    min_delta: The minimum allowed steering angle (radians)
    max_delta: The maximum allowed steering angle (radians)
    delta_incr: The difference (in radians) between subsequent possible steering angles
    dt: The amount of time to apply a control for
    T: The number of time steps to rollout for
    car_length: The length of the car
Returns a NxTx3 numpy array that contains N rolled out trajectories, each
containing T poses. For each trajectory, the t-th element represents the [x,y,theta]
pose of the car at time t+1
'''
def generate_mpc_rollouts(speed, min_delta, max_delta, delta_incr, dt, T, car_length):

  deltas = np.arange(min_delta, max_delta, delta_incr)
  N = deltas.shape[0]
  
  init_pose = np.array([0.0,0.0,0.0], dtype=np.float)
  
  rollouts = np.zeros((N,T,3), dtype=np.float)
  for i in xrange(N):
    controls = np.zeros((T,3), dtype=np.float)
    controls[:,0] = speed
    controls[:,1] = deltas[i]
    controls[:,2] = dt
    rollouts[i,:,:] = generate_rollout(init_pose, controls, car_length)
  
  print(rollouts[:,-1,:])
  return rollouts, deltas

    

def main():

  rospy.init_node('laser_wanderer', anonymous=True)

  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LaserWanderer class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system  
  # YOUR CODE HERE
  speed = rospy.get_param ('speed')# Default val: 1.0
  min_delta = rospy.get_param ('min_delta')# Default val: -0.34
  max_delta = rospy.get_param ('max_delta')# Default val: 0.341
  delta_incr = rospy.get_param ('delta_incr')# Starting val: 0.34/3 (consider changing the denominator) 
  dt = rospy.get_param ('dt')# Default val: 0.01
  T = rospy.get_param ('T')# Starting val: 300
  compute_time = rospy.get_param ('compute_time')# Default val: 0.09
  laser_offset = rospy.get_param ('laser_offset')# Starting val: 1.0
  
  # DO NOT ADD THIS TO YOUR LAUNCH FILE, car_length is already provided by teleop.launch
  car_length = rospy.get_param("/car/vesc/chassis_length", 0.33) 
  
  # Generate the rollouts
  rollouts, deltas = generate_mpc_rollouts(speed, min_delta, max_delta,
                                           delta_incr, dt, T, car_length)
  
  # Create the LaserWanderer                                         
  lw = LaserWanderer(rollouts, deltas, speed, compute_time, laser_offset)
  
  # Keep the node alive
  rospy.spin()
  

if __name__ == '__main__':
  main()
