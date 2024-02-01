import numpy as np
import math
import matplotlib.pyplot as plt
from PIL import Image
import heapq
import random
from itertools import cycle
import os
import rospy
from std_msgs.msg import String
global string_msg
string_msg = {'new_run': False}
random.seed(0) #Set seed to get same results every time

# Initialize the dictionary for the environment
# string_msg = {
#     'numAgents': 2,
#     'numCities': 4,
#     'startPose': [[30,30],[30,30]],
#     'vels': [1, 1],
#     'cityCoordinates': [[2,2],[1,1],[1, 10],[10,10]],
#     'numGenerations': 10,
#     'populationSize': 10,
#     'mutationRate': 0.1,
#     'image_path': "",#"obstacleMap2.png",
#     'new_run': True
# }

def string_callback(msg):  
    global string_msg  
    string_msg = eval(msg.data)

def load_image():
    try:
        image_path = os.path.dirname(os.path.realpath(__file__)) + "/" + string_msg['image_path']
        image = Image.open(image_path).convert('L')
        image_data = np.array(image)
    except:
        print('Image not found')
        image_data = np.ones((50,50))*255 # Default empty image
    return image_data

# A* algorithm:
class Node:
    def __init__(self, position, parent=None, g=float('inf'), h=float('inf')):
        self.position = position
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h

    def __lt__(self, other):
        return self.f < other.f

def heuristic(position, goal):
    # return math.sqrt((position[0] - goal[0])**2 + (position[1] - goal[1])**2)
    return abs(position[0] - goal[0]) + abs(position[1] - goal[1])

def a_star(occupancy_grid, start, goal):
    open_list = []
    closed_set = set()

    start_node = Node(start, None, 0, heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if tuple(current_node.position) == tuple(goal):
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        closed_set.add(tuple(current_node.position))

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor_position = (int(current_node.position[0] + dx), int(current_node.position[1] + dy))

                if (0 <= neighbor_position[0] < occupancy_grid.shape[0] and
                    0 <= neighbor_position[1] < occupancy_grid.shape[1] and
                    occupancy_grid[neighbor_position[0], neighbor_position[1]] == 1 and
                    neighbor_position not in closed_set):

                    movement_cost = 1.414 if dx != 0 and dy != 0 else 1
                    new_g = current_node.g + movement_cost

                    # Check for the neighbor in open list
                    in_open_list = [node for node in open_list if node.position == neighbor_position]

                    if in_open_list:
                        if new_g < in_open_list[0].g:
                            open_list.remove(in_open_list[0])
                            in_open_list[0].g = new_g
                            in_open_list[0].f = new_g + in_open_list[0].h
                            in_open_list[0].parent = current_node
                            heapq.heappush(open_list, in_open_list[0])
                    else:
                        neighbor_node = Node(neighbor_position, current_node, new_g, heuristic(neighbor_position, goal))
                        heapq.heappush(open_list, neighbor_node)

    return None  # Path not found

class AStar:
    def __init__(self, image_data, threshold):
        self.occupancy_grid = (image_data > threshold).astype(int)
        self.numAgents = string_msg['numAgents']
        self.numCities = string_msg['numCities']
        self.startPose = np.array(string_msg['startPose'])
        self.vels = string_msg['vels']
        self.cityCoordinates = np.array(string_msg['cityCoordinates'])

        self.numGenerations = string_msg['numGenerations']
        self.populationSize = string_msg['populationSize']
        self.mutationRate = string_msg['mutationRate']
        # Initialize the population
        self.population = []
        # Genetic algorithm:
        self.path_cache = {}

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
        totalDist = np.zeros(path.shape[1])
        for k in range(path.shape[1]):
            pathInds = path[:, k][path[:, k] >= 0]
            numCities = len(pathInds)
            for i in range(numCities+1):
                if i == 0:
                    city1 = self.startPose[k]
                else:
                    city1 = self.cityCoordinates[pathInds[i - 1]]
                if i == numCities:
                    city2 = self.startPose[k]
                else:
                    city2 = self.cityCoordinates[pathInds[i]]

                # Use cached results if available
                path_key = (tuple(city1), tuple(city2))
                if path_key in self.path_cache:
                    dist = self.path_cache[path_key]["cost"]
                else:
                    a_star_path = a_star(self.occupancy_grid, tuple(map(int, city1)), tuple(map(int, city2)))
                    if a_star_path is None:
                        return float('inf')
                    dist = len(a_star_path) / self.vels[k]
                    self.path_cache[path_key] = {"cost": dist, "path": a_star_path}

                totalDist[k] += dist
            
            a_star_path = a_star(self.occupancy_grid, city2, tuple(map(int, self.startPose[k])))
            if a_star_path is None:
                return float('inf')
            dist = len(a_star_path) / self.vels[k]
            totalDist[k] += dist
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
                assignedAgents = np.where(np.sum(child == c, axis=0) > 0)[0]
                keepAgent = random.choice(assignedAgents)
                for agent in assignedAgents:
                    if agent != keepAgent:
                        whichIdx = np.where(child[:, agent] == c)[0]
                        child[whichIdx, agent] = -1
        return child
    
    # Define the fitness function
    def fitnessFunction(self, x):
        return self.totalDistance(x)
    
    def run_astar(self):
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
                    city1 = self.startPose[agent]
                else:
                    city1 = self.cityCoordinates[agentCities[agent][i - 1]]
                if i == len(agentCities):
                    city2 = self.startPose[agent]
                else:
                    city2 = self.cityCoordinates[agentCities[agent][i]]

                # Use cached results - should always be available
                path_key = (tuple(city1), tuple(city2))
                if i == 0:
                    bestPaths[agent].extend(self.path_cache[path_key]["path"])
                else:
                    bestPaths[agent].extend(self.path_cache[path_key]["path"][1:])

        # Print the best city-paths for each agent
        for agent in range(self.numAgents):
            print(f'Agent {agent + 1} Path:', agentCities[agent])
        

        return bestPaths, agentCities

def main():
    rospy.init_node('string_subscriber')
    rospy.Subscriber('string_msg', String, string_callback)
    path_pub = rospy.Publisher('best_paths', String, queue_size=1,latch=True)
    goal_pub = rospy.Publisher('assigned_tasks', String, queue_size=1,latch=True)
    rate = rospy.Rate(10)
    threshold = 180
    best = []
    best_paths = String()
    assigned_tasks = String()
    while not rospy.is_shutdown():
        if string_msg['new_run']:
            image_data = load_image()
            astar_path = AStar(image_data,threshold)
            astar_path.run_astar()
            best_path, cities = astar_path.publish_results()
            # Publish the best paths
            best_paths.data = str(best_path)
            assigned_tasks.data = str(cities)
            path_pub.publish(best_paths)
            goal_pub.publish(assigned_tasks)
            string_msg['new_run'] = False   
        else:
            pass
        rate.sleep()

if __name__ == '__main__':
    main()