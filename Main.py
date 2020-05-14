import random
import math
import sys
import pygame
import numpy as np 
import time

x_dim = 800
y_dim = 600
window_size = [x_dim, y_dim]

animation = True
fps_clock = pygame.time.Clock()
screen = pygame.display.set_mode(window_size)
pygame.display.set_caption('Performing RRT_MultiGoal_WallE')

class Node():
    """
    RRT Node`
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None
        self.children = set()

class RRT_MultiGoal_WE():
    """
    RRT Star Planning with Multi-Goal.
    1. Primary focus is to reach the primary goal.
            1.1. Detect if the goals have been found while developing the tree.
    2. Find nodes within the tree that's closest to the tree and then focus on growing branches to find these goals
            2.1. If the secondary goals are not found, find the nodes closest to the main path and regrow these branches.
            2.2. Do the same until all the goals are connected to the main tree.
            2.3.1.Try to connect to the primary goal 
            2.3.2. Try to connect the secondary goals back to the primary tree. 
    3. Stop exploring and start exploiting.
    """
    def __init__(self, start_node, primary_goal, sgoal_list, n_goals, obstacle_list, rand_area, expand_dist = 15.0, goal_sample_rate = 10, max_nodes = 500, nearNeigbhors = 5):
        self.start = start_node
        self.primary_goal = primary_goal
        self.goal_list = sgoal_list
        self.x_rand = rand_area[0]
        self.y_rand = rand_area[1]

        self.start_node_list = {}
        self.nearNeighbors = nearNeigbhors
        self.expand_dist = expand_dist
        self.goal_sample_rate = goal_sample_rate
        self.max_nodes = max_nodes
        self.obstacle_list = obstacle_list
        self.node_list = {0: self.start}
        self.flag_pgoal = False
        
        self.sgoal_list = sgoal_list
        self.n_goals = n_goals 
        self.flag_s = np.zeros((self.n_goals), dtype=bool)
        self.idx_sgoal = 0
        #self.start_node_list[self.idx_sgoal] = start_node

    def Planning(self):
        print("Planning Started!")
        i = 0
        self.start_node_list[self.idx_sgoal] = self.start
        current_goal = self.primary_goal
        while True:
            #while not self.flag_pgoal: # Add a condition to stop this once the primary goal is found!
            #Step 1:
            rnd_pt = self.get_random_point(goal_node = self.primary_goal)
            nearInd = self.GetNearestListIndex(rnd = rnd_pt)
            newNode = self.steer(rnd = rnd_pt, nearInd = nearInd)
            print(i)
            #print("X: ", newNode.x)
            #time.sleep(0.5)
            #Performing Collision Check: Expensive!
            if self.__CollisionCheck(newNode, self.obstacle_list):
                #print("Collision Check Done!")
                nearInds = self.find_near_nodes(newNode, self.nearNeighbors)
                newNode = self.choose_parent(newNode = newNode, nearInds= nearInds)
                self.node_list[i+self.expand_dist] = newNode
                # Rewiring the tree to consider the new node

                self.rewire(i+self.expand_dist, newNode, nearInds)
                self.node_list[newNode.parent].children.add(i+self.expand_dist)
                #FN Implementation!
        
                # if len(self.node_list) > self.max_nodes:
                #     leaves = [ key for key, node in self.node_list.items() if len(node.children) == 0 and len(self.node_list[node.parent].children) > 1 and node is not self.primary_goal]
                #     if len(leaves) > 1:
                #         ind = leaves[random.randint(0, len(leaves)-1)]
                #         self.node_list[self.node_list[ind].parent].children.discard(ind)
                #         self.node_list.pop(ind)
                #     else:
                #         leaves = [ key for key, node in self.node_list.items() if len(node.children) == 0 ]
                #         ind = leaves[random.randint(0, len(leaves)-1)]
                #         self.node_list[self.node_list[ind].parent].children.discard(ind)
                #         self.node_list.pop(ind)
            i+=1

            if i>50 and self.find_goal(current_goal):
                    print("Found Primary goal!")
                    self.flag_pgoal = True

                    if self.idx_sgoal < self.n_goals and not self.flag_s[-1] and not self.flag_s[-2]:
                        self.idx_sgoal += 1

            if animation and i%5 == 0:
                #print("Inside Animation!")
                self.path_validation(current_goal = self.primary_goal)
                self.DrawGraph(start_node = self.start_node_list[0], idx = i, rnd = rnd_pt)
                    #self.DrawPath(idx = i)
            
            if not self.flag_s[-1] and self.idx_sgoal == self.n_goals:
                self.idx_sgoal -=1

            print("idx: ", self.idx_sgoal)
            #print("S Flag: ",  self.flag_s[self.idx_sgoal])
            if  not self.flag_s[-1]:
                #print("Inside secondary goal condition!")
                current_goal = self.sgoal_list[self.idx_sgoal]
                #time.sleep(0.5)
                #print(current_goal.x)
                if self.find_goal(current_goal=current_goal):
                    print("Goal {} exists within tree structure!".format(self.idx_sgoal+1))
                    self.flag_s[self.idx_sgoal] = True
                    if self.idx_sgoal < self.n_goals:
                        self.idx_sgoal +=1

                    start_node_ind = self.secondary_mapper(current_goal = current_goal)
                    self.start_node_list[self.idx_sgoal] = self.node_list[start_node_ind]
                    self.path_validation(current_goal = current_goal)
                    self.DrawGraph(start_node =  self.start_node_list[self.idx_sgoal], idx = i, rnd = rnd_pt)
                    current_goal_plot=self.sgoal_list[-1]
                    #Trial
                    #print("Current Goal: ", current_goal_plot.x)
                    lastIndex = self.get_best_last_index(current_goal=current_goal_plot)
                    if lastIndex is not None:
                        path = self.gen_final_course(start_node = self.start_node_list[self.idx_sgoal], goalind = lastIndex, current_goal=current_goal_plot)
                        ind = len(path)
                        #print("In Secondary, ind", ind)
                        while ind > 1:
                            pygame.draw.line(screen,(0,0,128),path[ind-2],path[ind-1])
                            ind-=1
                    #time.sleep(2)
                else:
                    print("Secondary goal {} doesn't exist within tree structure!".format(self.idx_sgoal+1))
                    nearInd = self.GetNearestListIndex(rnd = rnd_pt)
                    newNode = self.steer(rnd = rnd_pt, nearInd = nearInd)
                    idx_rnd = self.secondary_regrow(current_goal = current_goal, idx = i)
                    #self.flag_s[self.idx_sgoal] = False
                    #self.idx_sgoal +=1

            if (animation and i%5 == 0) and self.idx_sgoal < self.n_goals:
            #print("Inside Animation!")
                self.path_validation(current_goal = self.primary_goal)
                self.DrawGraph(start_node = self.start_node_list[0], idx = i, rnd = rnd_pt)

            if self.idx_sgoal == 1 or self.flag_s[1]:
                self.path_validation(current_goal = self.sgoal_list[1])
                start_second = list(self.start_node_list.keys())
                print(start_second)
                self.DrawGraph(start_node = start_second, idx = i, rnd = rnd_pt)

            if self.idx_sgoal == 2 and self.flag_s[-1]:
                self.path_validation(current_goal = self.sgoal_list[-1])
                start_last = list(self.start_node_list.keys())[-1]
                self.DrawGraph(start_node = start_last, idx = i, rnd = rnd_pt)

                #self.DrawPath(idx = i)
                #time.sleep(1)
                        
            i+=1

            for e in pygame.event.get():
                if e.type == pygame.MOUSEBUTTONDOWN:
                    if e.button == 1:
                        self.obstacle_list.append((e.pos[0],e.pos[1],30,30))
                        self.path_validation(current_goal=current_goal)
                    elif e.button == 3:
                        self.primary_goal = Node(e.pos[0], e.pos[1])

                if e.type == pygame.QUIT:
                        pygame.quit()
                        #self.flag_pgoal = False

    def secondary_mapper(self, current_goal):
        #This is to plan when the goal is within the tree structure.
        goal_ind = self.get_best_last_index(current_goal=self.primary_goal)
        current_goal_ind = self.get_best_last_index(current_goal = current_goal)
        main_tree = self.gen_final_course(start_node = self.start, current_goal= self.primary_goal, goalind=goal_ind)

        print(main_tree)
        #Finding the node that is closest to the current_goal but part of the main tree:
        dlist = np.subtract(np.array([ (node[0], node[1]) for node in main_tree]), (current_goal.x,current_goal.y))**2
        dlist = np.sum(dlist, axis=1)
        #list(self.node_list.keys())[np.argmin(dlist)]
        ind = np.array(dlist.argsort()[:3])
        print(ind)
        min_neighbors = [main_tree[i] for i in ind]
        #print(minind)
        print("main_tree: ", min_neighbors)
        #print
        #print(main_tree[int(minind)][1])
        #print("MinInd:", main_tree[int(minind)])
        #Unable to find the key corresponding to the elected node.

        min_key = [key for key, node in self.node_list.items() if node.x == min_neighbors[0][0] and node.y == min_neighbors[0][1]]
        print("Min Key", min_key)
        j = 1
        while len(min_key) == 0:
            min_key = [key for key, node in self.node_list.items() if node.x == min_neighbors[j][0] and node.y == min_neighbors[j][1]]
            j+=1

            if j>3:
                print("No Nearest Neigbhors!")

        min_key = int(min_key[0])
        
        #print("From NodeList: ", self.node_list[min_key].x)
        if np.min(dlist)<=self.expand_dist and  min_neighbors[j-1][0] == self.node_list[min_key].x and min_neighbors[j-1][1] == self.node_list[min_key].y:
            self.node_list[min_key].children.add(current_goal)
            self.node_list[current_goal_ind ].parent = min_key

        return min_key
        """
        else:
            idx = min_key
            #Regrow till we reach the current goal
            #nearInd = self.GetNearestListIndex(rnd = current_goal)
            tmp_rnd = self.get_random_point(goal_node=current_goal)
            newNode = self.steer(rnd = tmp_rnd, nearInd = min_key) 
            while(self.calc_dist_to_goal(newNode.x, newNode.y, current_goal)):
                tmp_rnd = self.get_random_point(goal_node=current_goal)
                nearInd = self.GetNearestListIndex(rnd = tmp_rnd)
                newNode = self.steer(rnd = tmp_rnd, nearInd = nearInd)
                rnd_pt = self.secondary_regrow(current_goal = current_goal, idx = idx)
                idx += self.expand_dist
        return idx
        """
    #"""
    def secondary_regrow(self, current_goal, idx):
        #This is to plan when the goal isn't within the tree structure.
            i = idx
            rnd_pt = self.get_random_point(goal_node = current_goal)
            nearInd = self.GetNearestListIndex(rnd = rnd_pt)
            newNode = self.steer(rnd = rnd_pt, nearInd = nearInd)
            #print(i)
            print("Secondary Regrow")
            #time.sleep(0.5)
            #Performing Collision Check: Expensive!
            if self.__CollisionCheck(newNode, self.obstacle_list):
                #print("Collision Check Done!")
                nearInds = self.find_near_nodes(newNode, self.nearNeighbors)
                newNode = self.choose_parent(newNode = newNode, nearInds= nearInds)
                self.node_list[i+self.expand_dist] = newNode
                # Rewiring the tree to consider the new node

                self.rewire(i+self.expand_dist, newNode, nearInds)
                self.node_list[newNode.parent].children.add(i+self.expand_dist)

            return i+self.expand_dist

    def get_random_point(self, goal_node):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(0, self.x_rand), random.uniform(0, self.y_rand)]
        else:
            rnd = [goal_node.x, goal_node.y]

        return rnd

    def GetNearestListIndex(self, rnd):
        dlist = np.subtract( np.array([ (node.x, node.y) for node in self.node_list.values() ]), (rnd[0],rnd[1]))**2
        dlist = np.sum(dlist, axis=1)
        minind = list(self.node_list.keys())[np.argmin(dlist)]
        return minind

    def steer(self, rnd, nearInd):
        nearNode = self.node_list[nearInd]
        theta = math.atan2(rnd[1] - nearNode.y, rnd[0] - nearNode.x)
        newNode = Node(nearNode.x, nearNode.y)
        newNode.x += self.expand_dist * math.cos(theta)
        newNode.y += self.expand_dist * math.sin(theta)

        newNode.cost = nearNode.cost + self.expand_dist
        newNode.parent = nearInd 
        #print("Steer!", newNode)
        return newNode

    def __CollisionCheck(self, node, obstacleList):

        for(sx,sy,ex,ey) in obstacleList:
            #sx,sy,ex,ey = sx+self.expand_dist*0.001,sy+self.expand_dist*0.001,ex+self.expand_dist*0.001,ey+self.expand_dist*0.001
            sx,sy,ex,ey = sx+0.5,sy+0.5,ex+0.5,ey+0.5
            if node.x > sx and node.x < sx+ex:
                if node.y > sy and node.y < sy+ey:
                    return False

        return True  # safe

    def find_near_nodes(self, newNode, value):
        r = self.expand_dist * value

        dlist = np.subtract( np.array([ (node.x, node.y) for node in self.node_list.values() ]), (newNode.x,newNode.y))**2
        dlist = np.sum(dlist, axis=1)
        nearinds = np.where(dlist <= r ** 2)
        nearinds = np.array(list(self.node_list.keys()))[nearinds]

        return nearinds

    def choose_parent(self, newNode, nearInds):
        if len(nearInds) == 0:
            return newNode

        dlist = []
        for i in nearInds:
            dx = newNode.x - self.node_list[i].x
            dy = newNode.y - self.node_list[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.node_list[i].x, self.node_list[i].y, theta, d):
                dlist.append(self.node_list[i].cost + d)
            else:
                dlist.append(float("inf"))
                print("Exiting since the cost is infinite!")
                #sys.exit()

        mincost = min(dlist)
        minind = nearInds[dlist.index(mincost)]

        if mincost == float("inf"):
            #print("NewNode:", newNode.x)
            #print("mincost is inf")
            newNode.cost = mincost
            newNode.parent = None
        else:
                    newNode.cost = mincost
        newNode.parent = minind

        newNode.cost = mincost
        newNode.parent = minind
        return newNode


    def check_collision_extend(self, nix, niy, theta, d):
        tmpNode = Node(nix,niy)

        for i in range(int(d/5)):
            tmpNode.x += self.expand_dist*0.5 * math.cos(theta)
            tmpNode.y += self.expand_dist*0.5 * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacle_list):
                return False

        return True

    def rewire(self, newNode_ind, newNode, nearInds):
        for i in nearInds:
            nearNode = self.node_list[i]
            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx**2 + dy**2 )
            tmp_cost = newNode.cost + d

            if tmp_cost < nearNode.cost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nix = nearNode.x, niy = nearNode.y, theta = theta, d = d ):
                    self.node_list[nearNode.parent].children.discard(i)
                    nearNode.parent = newNode_ind
                    nearNode.cost = tmp_cost
                    newNode.children.add(i)
    
    def path_validation(self, current_goal):
        #print("In path_validation!")
        lastIndex = self.get_best_last_index(current_goal)
        if lastIndex is not None:
            while self.node_list[lastIndex].parent is not None:
                nodeInd = lastIndex
                lastIndex = self.node_list[lastIndex].parent

                dx = self.node_list[nodeInd].x - self.node_list[lastIndex].x
                dy = self.node_list[nodeInd].y - self.node_list[lastIndex].y
                d = math.sqrt(dx ** 2 + dy ** 2)
                theta = math.atan2(dy, dx)
                if not self.check_collision_extend(self.node_list[lastIndex].x, self.node_list[lastIndex].y, theta, d):
                    self.node_list[lastIndex].children.discard(nodeInd)
                    self.remove_branch(nodeInd)
            
    def get_best_last_index(self, current_goal):
        #What if we considered cost?
        disglist = [(key, self.calc_dist_to_goal(node.x, node.y, current_goal)) for key, node in self.node_list.items()]
        goalInds = [key for key, distance in disglist if distance <= self.expand_dist]

        if len(goalInds) == 0:
            #print(self.flag_s[self.idx_sgoal])
            print("No goal found!")
            return None

        mincost = min([self.node_list[key].cost for key in goalInds])
        #This can be avoided. Is there a quicker way to find the index within minimum cost
        for i in goalInds:
            if self.node_list[i].cost == mincost:
                return i

    def calc_dist_to_goal(self, x, y, current_goal):
        return np.linalg.norm([x - current_goal.x, y - current_goal.y]) 

    def remove_branch(self, nodeInd):
        for ix in self.node_list[nodeInd].children:
            self.remove_branch(ix)
        self.node_list.pop(nodeInd)       
    
    def DrawGraph(self, idx, start_node, rnd=None):
        """
        Draw Graph
        """
        #print("Inside DrawGraph()")
        self.nodeList = self.node_list
        screen.fill((255, 255, 255))
        for node in self.node_list.values():
            if node.parent is not None:
                pygame.draw.line(screen,(0,255,0),[self.node_list[node.parent].x,self.node_list[node.parent].y],[node.x,node.y])

        for node in self.nodeList.values():
            if len(node.children) == 0: 
                pygame.draw.circle(screen, (255,0,255), [int(node.x),int(node.y)], 2)
                

        for(sx,sy,ex,ey) in self.obstacle_list:
            pygame.draw.rect(screen,(0,0,0), [(sx,sy),(ex,ey)])

        pygame.draw.circle(screen, (255,0,0), [self.start.x, self.start.y], 10)
        pygame.draw.circle(screen, (0,0,255), [self.primary_goal.x, self.primary_goal.y], 15)

        #Add condition for primary and secondary goals!
        for i in range(1, self.n_goals):
            pygame.draw.circle(screen, (0,0,255), [self.sgoal_list[i].x, self.sgoal_list[i].y], 10)

        #print(self.start_node_list)
        if self.flag_pgoal:
            for i in range(1, self.n_goals):
                if self.flag_s[i]:
                            print("Plotting path for primary goal")
                            start_node = self.start
                            if self.flag_pgoal:
                                lastIndex = self.get_best_last_index(current_goal=self.primary_goal)
                                if lastIndex is not None:
                                    #print("Start_node ", start_node)
                                    path_primary = self.gen_final_course(start_node = start_node, goalind = lastIndex, current_goal=self.primary_goal)
                                    #print(path_primary)

                                    ind = len(path_primary)
                                    while ind > 1:
                                        pygame.draw.line(screen,(255,0,0),path_primary[ind-2],path_primary[ind-1])
                                        ind-=1
                            
                            print("Plotting course for secondary goals!")

                            for i in range(1, self.n_goals):
                                print("Goal", i+1)
                                if self.flag_s[i]:
                                    current_goal=self.sgoal_list[i]
                                    #print("Current Goal: ", current_goal.x)
                                    lastIndex = self.get_best_last_index(current_goal=current_goal)
                                    if lastIndex is not None:
                                        path = self.gen_final_course(start_node = start_node, goalind = lastIndex, current_goal=current_goal)
                                        ind = len(path)
                                        #print("In Secondary, ind", ind)
                                        while ind > 1:
                                            pygame.draw.line(screen,(0,0,128),path[ind-2],path[ind-1])
                                            ind-=1
        pygame.display.update()

    #def DrawPath(self, start_node):
            #time.sleep(10)

    def gen_final_course(self, start_node, goalind, current_goal):
        path = [[current_goal.x, current_goal.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([start_node.x, start_node.y])
        return path

    def find_goal(self, current_goal):
        dist_goal = [(key, self.calc_dist_to_goal(node.x, node.y, current_goal)) for key, node in self.node_list.items()]
        goalInds = [key for key, distance in dist_goal if distance < self.expand_dist]
        #print(len(goalInds))
        if len(goalInds) !=0:
            return True
        else:
            return False

def main():
    print("Starting! RRT*FND Path Planning")

    # ====Search Path with RRT====
    
    obstacleList = [
        (400, 380, 400, 20),
        (400, 220, 20, 180),
        (500, 280, 150, 20),
        (0, 500, 100, 20),
        (500, 450, 20, 150),
        (400, 100, 20, 80),
        (100, 100, 100, 20)
    ]  # [x,y,size]
    # Set Initial parameters
    start_node = Node(20, 580)
    goal_node_1 = Node(510, 150)
    goal_node_2 = Node(350, 400)
    goal_node_3 = Node(600, 200)
    goal = [goal_node_1, goal_node_2, goal_node_3]
    #start_node, primary_goal, sgoal_list, n_goals, obstacle_list, rand_area, expand_dist = 15.0, goal_sample_rate = 10, max_nodes = 500, nearNeigbhors = 5):
    rrt = RRT_MultiGoal_WE(start_node=start_node, primary_goal = goal_node_1, sgoal_list=goal, rand_area=[x_dim, y_dim], n_goals = len(goal), obstacle_list= obstacleList)
    path = rrt.Planning()


if __name__ == '__main__':
    main()
