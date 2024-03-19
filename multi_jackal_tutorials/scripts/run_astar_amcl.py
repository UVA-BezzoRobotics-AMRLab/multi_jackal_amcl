#!/usr/bin/env python3
import numpy as np
import math
import matplotlib.pyplot as plt
from PIL import Image
import heapq
import random
from itertools import cycle
import os
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
string_msg = {'new_run': False}
random.seed(0) #Set seed to get same results every time

# Initialize the dictionary for the environment
# string_msg = {
#     'numAgents': 1,
#     'numCities': 5,
#     'startPose': [[2,4]],
#     'vels': [1],
#     'cityCoordinates': [[2,3],[2,5],[-2,-0.5],[14,3],[-10,2]],
#     'numGenerations': 10,
#     'populationSize': 10,
#     'mutationRate': 0.1,
#     'new_run': True
# }

def string_callback(msg):  
    global string_msg  
    string_msg = eval(msg.data)

class GA:
    def __init__(self,rate):
        # self.occupancy_grid = (image_data > threshold).astype(int)
        self.rate = rate
        self.numAgents = string_msg['numAgents']
        self.numCities = string_msg['numCities']
        self.startPose = np.array(string_msg['startPose'])
        self.vels = string_msg['vels']
        self.cityCoordinates = np.array(string_msg['cityCoordinates'])
        self.priorities = string_msg['priorities']
        self.taskTimes = string_msg['taskTimes']
        self.precedence = string_msg['precedence']
        self.start_pub = rospy.Publisher("start_point",PoseWithCovarianceStamped,queue_size=10)
        self.target_pub = rospy.Publisher("final_point",PoseStamped,queue_size=10)
        self.path_sub = rospy.Subscriber("/nav_path",Path, self.nav_path_cb)

        self.numGenerations = string_msg['numGenerations']
        self.populationSize = string_msg['populationSize']
        self.mutationRate = string_msg['mutationRate']
        # Initialize the population
        self.population = []
        # Genetic algorithm:
        self.path_cache = {}
        self.path_received = False
        self.path_from_msg = []
        self.path_cost = 0

    def nav_path_cb(self,msg):
        self.path_received = True
        self.path_from_msg = np.zeros((len(msg.poses), 2))
        for idx,pose in enumerate(msg.poses):
            self.path_from_msg[idx,0] = pose.pose.position.x
            self.path_from_msg[idx,1] = pose.pose.position.y
            if idx == 0:
                self.path_cost = 0
            else:
                self.path_cost += np.linalg.norm(self.path_from_msg[idx-1,:]-self.path_from_msg[idx,:])
            # print(pose.pose.position.x, pose.pose.position.y)

    def publish_start(self,x,y,sub):
        start = PoseWithCovarianceStamped()
        start.header.stamp = rospy.get_rostime()
        start.header.frame_id = "map"
        start.pose.pose.position.x = x
        start.pose.pose.position.y = y
        quaternion = [0,0,0,1]

        start.pose.pose.orientation.w = quaternion[0]
        start.pose.pose.orientation.x = quaternion[1]
        start.pose.pose.orientation.y = quaternion[2]
        start.pose.pose.orientation.z = quaternion[3]
        sub.publish(start)

    def publish_target(self,x,y,sub):
        target = PoseStamped()
        target.header.stamp = rospy.get_rostime()
        target.header.frame_id = "map"
        target.pose.position.x = x
        target.pose.position.y = y
        quaternion = [0,0,0,1]

        target.pose.orientation.w = quaternion[0]
        target.pose.orientation.x = quaternion[1]
        target.pose.orientation.y = quaternion[2]
        target.pose.orientation.z = quaternion[3]
        sub.publish(target)

    def generateUniquePaths(self):
        paths = np.zeros((self.numCities, self.numAgents), dtype=int) - 1
        remainingCities = np.arange(0, self.numCities)

        # agent_cycle = cycle(range(self.numAgents))
        
        for _ in range(self.numCities):
            randomChoice = random.choice(range(len(remainingCities)))
            agent = random.choice(range(self.numAgents))
            
            # Find the next available slot for the agent
            available_slot = np.where(paths[:, agent] == -1)[0][0]

            paths[available_slot, agent] = remainingCities[randomChoice]
            remainingCities = np.delete(remainingCities, randomChoice)

        return paths
    
    def totalDistance(self, path):
        global path_received
        totalDist = np.zeros(path.shape[1])
        for k in range(path.shape[1]):
            pathInds = path[:, k][path[:, k] >= 0]
            numCities = len(pathInds)
            current_dist = 0
            for i in range(numCities+1):
                if i == 0:
                    whichCity1 = -k-1
                    city1 = self.startPose[k]
                else:
                    whichCity1 = pathInds[i-1]
                    city1 = self.cityCoordinates[whichCity1]
                if i == numCities:
                    whichCity2 = -k-1
                    city2 = self.startPose[k]
                else:
                    whichCity2 = pathInds[i]
                    city2 = self.cityCoordinates[whichCity2]

                # Use cached results if available
                path_key = (whichCity1, whichCity2)
                if path_key in self.path_cache:
                    dist = self.path_cache[path_key]["cost"]
                    # priority based on total distance:
                    if whichCity2 >= 0:
                        priority = self.priorities[whichCity2]
                        if priority > 0:
                            dist = (current_dist+dist) * (1+priority)
                    current_dist += dist
                else:
                    iters = 0
                    while not self.path_received and iters < 10:
                        self.publish_start(city1[0],city1[1],self.start_pub)
                        self.publish_target(city2[0],city2[1],self.target_pub)
                        rospy.sleep(.15)
                        iters+=1
                        # rospy.wait_for_message("/nav_path",Path)
                    if iters >= 10:
                        print("Path not received")
                        return float('inf')
                    self.path_received = False
                    dist = self.path_cost / self.vels[k]

                    # Add task times using velocities
                    if whichCity2 >= 0:
                        task_cost = self.taskTimes[whichCity2] * self.vels[k]
                        dist += task_cost

                    # Prioritization
                    # priority = self.priorities[whichCity2]
                    # path_index = i
                    # steepness = 1.5
                    # index_weight = .5
                    # midpoint = numCities/2 #numCities/2 for len(path)/2, string_msg['numCities']/2 for all cities

                    # Logarithmic:
                    # if numCities > 1:
                    #     dist = dist * (1 + priority * (np.log(1+path_index)/np.log(numCities)))
                    # else:
                    #     dist = dist

                    # Logistic:
                    # if priority > 0:
                    #     dist = dist * (1+math.exp(-steepness*((1/priority) + (index_weight*(path_index-midpoint)))))
                    # else:
                    #     dist = dist
                    # print(dist)
                    # print(f"Agent {k+1} Path: {pathInds} Total Distance: {dist}")

                    self.path_cache[path_key] = {"cost": dist, "path": self.path_from_msg.tolist()}
                    # Can't reuse distances due to priorities
                    # self.path_cache[(path_key[1], path_key[0])] = {"cost": dist, "path": self.path_from_msg.tolist()[::-1]}

                totalDist[k] += dist
                print(f"Agent {k+1} Path: {pathInds} Total Distance: {dist}")
            
        return np.max(totalDist)
    
    
    def swapMutation(self, path):
        mutatedPath = np.copy(path)
        for k in range(path.shape[1]):
            self.numCities = len(path[path[:, k] > 0])
            if self.numCities > 2:
                indices = random.sample(range(self.numCities), 2)
                mutatedPath[indices, k] = path[indices[::-1], k]
        return mutatedPath
    
    def orderCrossover(self,parent1, parent2):
        numAgents = parent1.shape[1]
        numCities = parent1.shape[0]
        child1 = np.copy(parent1)
        child2 = np.copy(parent2)

        for k in range(self.numAgents):
            crossoverPoint1 = random.randint(1, numCities - 1)
            crossoverPoint2 = random.randint(crossoverPoint1 + 1, numCities)

            child1[crossoverPoint1:crossoverPoint2, k] = parent2[crossoverPoint1:crossoverPoint2, k]
            child2[crossoverPoint1:crossoverPoint2, k] = parent1[crossoverPoint1:crossoverPoint2, k]

        return child1, child2

    def repairChild(self, child):
        numAgents = child.shape[1]
        numCities = child.shape[0]
        # Remove duplicates
        for agent in range(numAgents):
            for c in range(numCities):
                whichIdx = np.where(child[:, agent] == c)[0]
                if len(whichIdx) > 1:
                    emptyIdx = random.choice(whichIdx)
                    child[emptyIdx, agent] = -1
        
        # Add missing cities
        for c in range(numCities):
            if np.sum(child == c) == 0:
                emptyAgents = np.where(np.sum(child == -1, axis=0) > 0)[0]
                agentChoice = random.choice(emptyAgents)
                emptySlots = np.where(child[:,agentChoice] == -1)[0]
                fillIdx = random.choice(emptySlots)
                child[fillIdx, agentChoice] = c
            if np.sum(child == c) > 1:
                # original:
                assignedAgents = np.where(np.sum(child == c, axis=0) > 0)[0]
                keepAgent = random.choice(assignedAgents)
                for agent in assignedAgents:
                    if agent != keepAgent:
                        whichIdx = np.where(child[:, agent] == c)[0]
                        child[whichIdx, agent] = -1
        
        # Ensure precedence constraints are satisfied
        # for c in range(numCities):
        #     for constraint in self.precedence[c]:
        #         constraintIdxs = np.where(child == constraint)[0]
        #         cIdxs = np.where(child == c)[0]
        #         # If constraint city is missing or comes after c, adjust the path
        #         if len(constraintIdxs) == 0 or (len(cIdxs) > 0 and constraintIdxs[0] > cIdxs[0]):
        #             # Swap c and constraint city in the same agent's path
        #             for agent in range(numAgents):
        #                 if c in child[:, agent] and constraint in child[:, agent]:
        #                     cAgentIdx = np.where(child[:, agent] == c)[0][0]
        #                     constraintAgentIdx = np.where(child[:, agent] == constraint)[0][0]
        #                     child[cAgentIdx, agent], child[constraintAgentIdx, agent] = constraint, c
        #                     break

        return child
    
    # Define the fitness function
    def fitnessFunction(self, x):
        return self.totalDistance(x)
    
    def run_ga(self):
        # Initialize the population
        self.population = [self.generateUniquePaths() for _ in range(self.populationSize)]
        # Run the genetic algorithm
        for generation in range(self.numGenerations):
            # Evaluate fitness of the population
            fitness = np.array([self.fitnessFunction(p) for p in self.population])

            # Perform selection (tournament selection)
            selectedIndices = np.zeros(self.populationSize, dtype=int)
            for i in range(self.populationSize):
                tournamentIndices = random.sample(range(self.populationSize), 2)
                winnerIndex = tournamentIndices[np.argmin(fitness[tournamentIndices])]
                selectedIndices[i] = winnerIndex

            # Initialize offspring with zeros
            offspring = [np.zeros((self.numCities, self.numAgents), dtype=int) for _ in range(self.populationSize)]

            # Perform crossover (order crossover)
            for i in range(0, self.populationSize, 2):
                parent1 = self.population[selectedIndices[i]]
                parent2 = self.population[selectedIndices[i + 1]]
                child1, child2 = self.orderCrossover(parent1, parent2)
                child1 = self.repairChild(child1)
                child2 = self.repairChild(child2)
                offspring[i] = child1
                offspring[i + 1] = child2

            # Perform mutation (swap mutation)
            for i in range(self.populationSize):
                if random.random() < self.mutationRate:
                    offspring[i] = self.swapMutation(offspring[i])

            # Replace the population with the offspring
            self.population = offspring
    
    def publish_results(self):
        bestPaths = [[] for i in range(self.numAgents)]
        agentCities = [[] for i in range(self.numAgents)]
        for agent in range(self.numAgents):
            bestIndex = 0
            bestFitness = self.fitnessFunction(self.population[0])
            for i in range(0, self.populationSize):
                currentFitness = self.fitnessFunction(self.population[i])
                if currentFitness < bestFitness:
                    bestIndex = i
                    bestFitness = currentFitness
            # Get astar paths
            agentCities[agent] = self.population[bestIndex][:, agent][self.population[bestIndex][:, agent] > -1]
            for i in range(len(agentCities[agent]) + 1):
                if i == 0:
                    whichCity1 = -agent-1
                    city1 = self.startPose[agent]
                else:
                    whichCity1 = agentCities[agent][i-1]
                    city1 = self.cityCoordinates[whichCity1]
                if i == len(agentCities[agent]):
                    whichCity2 = -agent-1
                    city2 = self.startPose[agent]
                else:
                    whichCity2 = agentCities[agent][i]
                    city2 = self.cityCoordinates[agentCities[agent][i]]

                # Use cached results - should always be available
                path_key = (whichCity1, whichCity2)
                if i == 0:
                    bestPaths[agent].extend(self.path_cache[path_key]["path"])
                    # print(bestPaths[agent])
                else:
                    bestPaths[agent].extend(self.path_cache[path_key]["path"][1:])

        # Print the best city-paths for each agent
        for agent in range(self.numAgents):
            print(f'Agent {agent + 1} Path:', agentCities[agent])

        return bestPaths,agentCities

def main():
    rospy.init_node('string_subscriber')
    rospy.Subscriber('string_msg', String, string_callback)
    path_pub = rospy.Publisher('best_paths', String, queue_size=1,latch=True)
    goal_pub = rospy.Publisher('assigned_tasks', String, queue_size=1,latch=True)
    rate = rospy.Rate(10)
    best_paths = String()
    assigned_tasks = String()
    while not rospy.is_shutdown():
        if string_msg['new_run']:
            # image_data = load_image()
            ga_path = GA(rate)
            ga_path.run_ga()
            best_path,cities = ga_path.publish_results()
            # Publish the best paths
            best_paths.data = str(best_path)
            assigned_tasks.data = str([cities[i].tolist() for i in range(len(cities))])
            path_pub.publish(best_paths)
            goal_pub.publish(assigned_tasks)
            string_msg['new_run'] = False   
        else:
            pass
        rate.sleep()

if __name__ == '__main__':
    main()






# Find the best solution for each agent

    
