# emcl: mcl with expansion resetting

emcl is an alternative Monte Carlo localization (MCL) package to amcl (http://wiki.ros.org/amcl). Differently from amcl, KLD-samping and adaptive MCL are not implemented. Instead, the expansion resetting is implemented[^1].

## Nodes

### mcl_node

This node transforms laser scans and odometry transform messages to pose estimations by using an occupancy grid map. 

#### Subscrbed Topics 

* scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html))
    * laser scans
* tf ([tf/tfMessage](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html))
    * transforms
* initialpose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
    * pose of particles for replacement

#### Published Topics

* mcl_pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
    * the mean pose of the particles with covariance
* particlecloud ([geometry_msgs/PoseArray](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))
    * poses of the particles
* tf ([tf/tfMessage](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html))
    * the transform from odom (which can be remapped via the ~odom_frame_id parameter) to map


#### Services Called

* static_map ([nav_msgs/GetMap](http://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html))
    * mcl initializes the map for localization

#### Parameters

* ~odom_freq (int, default: 20 [Hz])
    * frequency of odometry update
* ~num_particles (int, default: 1000)
    * number of particles
* ~odom_frame_id (string, default: "odom")
    * the frame for odometry
* ~base_frame_id (string, default: "base_footprint")
    * the frame of the localized robot's base
* ~global_frame_id (string, default: "map")
    * the frame for localization 
* ~initial_pose_x (double, default: 0.0 [m])
    * initial x coordinate of particles
* ~initial_pose_y (double, default: 0.0 [m])
    * initial y coordinate of particles
* ~initial_pose_a (double, default: 0.0 [rad])
    * initial yaw coordinate of particles
* ~odom_fw_dev_per_fw (double, default: 0.19 [m/m])
    * standard deviation of forward motion noise by forward motion
* ~odom_fw_dev_per_rot (double, default: 0.0001 [m/rad])
    * standard deviation of forward motion noise by rotational motion
* ~odom_rot_dev_per_fw (double, default: 0.13 [m/m])
    * standard deviation of rotational motion noise by forward motion
* ~laser_likelihood_max_dist (double, default: 0.2 meters)
    * maximum distance to inflate occupied cells on the likelihood field map

## Notes

### likelihood field

This implemenation uses an ad-hoc likelihood field model. Occupied cells on the map are inflated so that each collision detection between a laser beam and an occupied cell is relaxed. The likelihood for each cell is given with a pyramidal kernel function. The parameter `~laser_likelihood_max_dist` gives the length from the center cell to the edge of the pyramid.


### citation

[^1]: R. Ueda, T. Arai, K. Sakamoto, T. Kikuchi, S. Kamiya: Expansion resetting for recovery from fatal error in Monte Carlo localization - comparison with sensor resetting methods, IEEE/RSJ IROS, pp.2481-2486, 2004. 
