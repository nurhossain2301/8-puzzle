import queue as Q
import time
import sys
import math
from queue import LifoQueue
import cProfile
import pstats

profile = cProfile.Profile()



## The Class that Represents the Puzzle

class PuzzleState(object):

    """docstring for PuzzleState"""

    def __init__(self, config, n, parent=None, action="Initial", cost=0, depth = 0):
        if n*n != len(config) or n < 2:
            raise Exception("the length of config is not correct!")
        self.n = n
        self.cost = cost
        self.parent = parent
        self.action = action
        self.dimension = n
        self.config = config
        self.children = []
        self.heuristic = None
        self.depth = depth
        self.Max_depth = None


        for i, item in enumerate(self.config):
            if item == 0:
                self.blank_row = i // self.n
                self.blank_col = i % self.n
                break

    def display(self):
        for i in range(self.n):
            line = []
            offset = i * self.n
            for j in range(self.n):
                line.append(self.config[offset + j])
            print(line)

    def move_left(self):
        if self.blank_col == 0:
            return None
        else:
            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index - 1
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config), self.n, parent=self, action="Left", cost=self.cost + 1, depth = self.depth + 1
                               )

    def move_right(self):
        if self.blank_col == self.n - 1:
            return None
        else:
            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index + 1
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config), self.n, parent=self, action="Right", cost=self.cost + 1, depth = self.depth + 1
                               )

    def move_up(self):

        if self.blank_row == 0:
            return None

        else:
            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index - self.n
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config), self.n, parent=self, action="Up", cost=self.cost + 1, depth = self.depth + 1
                               )

    def move_down(self):

        if self.blank_row == self.n - 1:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col
            target = blank_index + self.n
            new_config = list(self.config)
            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]
            return PuzzleState(tuple(new_config), self.n, parent=self, action="Down", cost=self.cost + 1, depth = self.depth + 1
                               )

    def expand(self):
        if len(self.children) == 0:

            up_child = self.move_up()

            if up_child is not None:
                self.children.append(up_child)

            down_child = self.move_down()

            if down_child is not None:
                self.children.append(down_child)

            left_child = self.move_left()

            if left_child is not None:
                self.children.append(left_child)

            right_child = self.move_right()

            if right_child is not None:
                self.children.append(right_child)

        return self.children


def bfs_search(initial_state):
    frontier = Q.Queue(maxsize = 0)
    frontier.put(initial_state)
    explored = set([])
    while not frontier.empty():
        state = frontier.get()
        explored.add(state.config)
        if test_goal(state):
            path = path_to(state, initial_state)
            cost = calculate_total_cost(state)
            writeOutput(path, cost, len(explored)-1)
            return state
        state.expand()
        for neighbor in state.children:
            if not (neighbor.config in explored):
                frontier.put(neighbor)
    return False



def dfs_search(initial_state):
    frontier = LifoQueue(maxsize=0)
    frontier_copy = set([])
    frontier.put(initial_state)
    frontier_copy.add(initial_state.config)
    node = 0
    max_depth = -(math.inf)
    while not frontier.empty():
        state = frontier.get()
        if state.depth > max_depth:
            max_depth = state.depth
        if test_goal(state):
            path = path_to(state, initial_state)
            state.display()
            cost = (calculate_total_cost(state))
            writeOutput(path, cost, node)
            return state
        state.expand()
        node = node+1
        for neighbor in reversed(state.children):
            if not (neighbor.config in frontier_copy):
                frontier.put(neighbor)
                frontier_copy.add(neighbor.config)
    return False

def path_to(state, begin_state):
    path_to_goal = []
    while not test_parent(state, begin_state):
        path_to_goal.append(state.action)
        state = state.parent
    return path_to_goal


def test_parent(state, begin_state):
    x = state.config
    y = begin_state.config
    if all(map(lambda i, j: i == j, y, x)):
        return True
    return False

def calculate_total_cost(state):

    """calculate the total estimated cost of a state"""
    return state.cost



def test_goal(puzzle_state):
    x = puzzle_state.config
    goal_state = (0, 1, 2, 3, 4, 5, 6, 7, 8)
    if all(map(lambda i, j: i == j, goal_state, x)):
        return True
    return False


def A_star_search(initial_state):
    frontier = Q.PriorityQueue(maxsize=0)
    frontier_copy = set([])
    frontier.put((0, action_num(initial_state.action), 0, initial_state))
    frontier_copy.add(initial_state.config)
    explored = set([])
    id = 0
    max_depth = -(math.inf)
    node = 0
    while not frontier.empty():
        priority, action_n, id_g, state = frontier.get()
        explored.add(state.config)
        if state.depth > max_depth:
            max_depth = state.depth
        if test_goal(state):
            path = path_to(state, initial_state)
            state.display()
            cost = (calculate_total_cost(state))
            writeOutput(path, cost, len(explored), max_depth)
            return state
        state.expand()
        node = node + 1
        for neighbor in state.children:
            calculate_manhattan_dist(neighbor)
            id = id + 1
            if not neighbor.config in frontier_copy and not (neighbor.config in explored):
                frontier.put((neighbor.heuristic + calculate_total_cost(neighbor), action_num(neighbor.action), id, neighbor))
                frontier_copy.add(neighbor.config)

    return False

def calculate_manhattan_dist(state):
    #did not use manhattan distance for A star serach. Used number of wrong tile instead
    goal_state = (0,1,2,3,4,5,6,7,8)
    d=0
    for i in range(0,9):
        if state.config[i] != goal_state[i]:
            d+=1
    state.heuristic=d


def action_num(action):
    if action == 'Up':
        num = 0
    elif action == 'Down':
        num = 1
    elif action == 'Left':
        num = 2
    else:
        num = 3
    return num

if sys.platform == "win32":
    import psutil
    print("psutil", psutil.Process().memory_info().rss)

# def writeOutput(path, cost, nodes, max_depth):
#     file1 = open('output.txt', 'w')
#     path_to_goal = path
#     path_to_goal.reverse()
#     file1.write('path_to_goal: ')
#     file1.write(str(path_to_goal)+ '\n')
#     file1.write('cost_of_path: ' + str(cost) +'\n')
#     file1.write('nodes_expanded: '+  str(nodes) + '\n')
#     file1.write('search_depth: '+ str(len(path_to_goal)) + '\n')
#     file1.write('max_search_depth: '+  str(max_depth))
#     file1.close()

# begin_state = (8,6,4,2,1,3,5,7,0)
# size = int(math.sqrt(len(begin_state)))
# hard_state = PuzzleState(begin_state, size)
# A_star_search(hard_state)

def main():

    sm = sys.argv[1].lower()
    begin_state = sys.argv[2].split(",")
    begin_state = tuple(map(int, begin_state))
    size = int(math.sqrt(len(begin_state)))
    hard_state = PuzzleState(begin_state, size)
    if sm == "bfs":
        bfs_search(hard_state)

    elif sm == "dfs":
        dfs_search(hard_state)

    elif sm == "ast":
        A_star_search(hard_state)

    else:
        print("Enter valid command arguments !")

if __name__ == '__main__':
    main()

