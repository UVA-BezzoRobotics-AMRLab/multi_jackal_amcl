"""Control multiple jackals using APF class and helpful functions"""
import rospy
import numpy as np
import math
from gazebo_msgs.srv import GetModelState

# Get the true model pose in gazebo (if in simulation)
def get_model_pose(model_name):

    # Wait for the service to become available
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        # Create a service proxy
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Request the model state
        resp = get_model_state(model_name, "")

        if resp.success:
            return resp.pose
        else:
            rospy.logerr("Failed to get model state for model: %s", model_name)
            return None

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None

class PotentialField:
    def __init__(self, kappa_attr, kappa_rep_obs, kappa_rep_veh, d0, d1):
        self.kappa_attr = kappa_attr # kappa_attr is the gain for attraction to the goal
        self.kappa_rep_obs = kappa_rep_obs # kappa_rep_obs is the gain for repulsion from obstacles
        self.kappa_rep_veh = kappa_rep_veh # kappa_rep_veh is the gain for repulsion from other vehicles
        self.d0 = d0 # d0 is the distance threshold for repulsion from obstacles
        self.d1 = d1 # d1 is the distance threshold for repulsion from other vehicles

    def attraction(self, q, q_goal):
        return self.kappa_attr * (q_goal - q)/(np.linalg.norm(q_goal - q) + 1e-6)

    def repulsion(self, q, q_obs, kappa_rep, d_thresh):
        dist = np.linalg.norm(q_obs - q)
        if dist < d_thresh:
            obs_return = (1.0 / dist - 1.0 / d_thresh) * (1.0 / dist**2)*(q_obs-q)
            return kappa_rep * obs_return/(np.linalg.norm(obs_return) + 1e-6)
        return np.array([0, 0])

    def total_force(self, q, q_goal, q_obs_list, q_veh_list):
        force = self.attraction(q, q_goal)
        for q_obs in q_obs_list:
            force -= self.repulsion(q, q_obs, self.kappa_rep_obs, self.d0)
        for q_veh in q_veh_list:
            force -= self.repulsion(q, q_veh, self.kappa_rep_veh, self.d1)
        return force

    def get_velocities(self, q, q_ori, q_goal, q_obs_list, q_veh_list, max_speed):
        force = self.total_force(q, q_goal, q_obs_list, q_veh_list)
        # Find desired difference in orientation (and wrap between -pi and pi)
        theta_desired = ((math.atan2(force[1], force[0]) - q_ori) + math.pi) % (2 * math.pi) - math.pi
        # Normalize force to max_speed
        if max_speed < np.linalg.norm(force):
            force = max_speed * (force / (np.linalg.norm(force) + 1e-6))
        elif np.linalg.norm(force) < 0.1:
            force = np.array([0, 0])
            theta_desired = 0
        if math.isnan(theta_desired) or math.isnan(force[0]) or math.isnan(force[1]):
            print("NAN")
            return 0, 0
        # Finalize velocities
        linear_velocity = np.linalg.norm(force)
        angular_velocity = 3 * theta_desired # 5 is a gain
        angular_velocity = max(min(angular_velocity, math.pi/2), -math.pi/2) # Limit angular velocity to [-pi/2, pi/2]
        linear_velocity = math.cos(angular_velocity) * linear_velocity # Limit linear velocity based on angular velocity
        return linear_velocity, angular_velocity

# Example usage:
# field = PotentialField(kappa_attr=1, kappa_rep_obs=1, kappa_rep_veh=1, d0=2.0, d1=3.0)
# q = np.array([0, 0])
# q_ori = 0
# q_goal = np.array([5, 5])
# q_obs_list = [np.array([2, 2])]
# q_veh_list = [np.array([1, 4])]
# max_speed = 1.0
# linear_velocity, angular_velocity = field.get_velocities(q, q_ori, q_goal, q_obs_list, q_veh_list, max_speed)
# print("Linear Velocity:", linear_velocity)
# print("Angular Velocity:", angular_velocity)