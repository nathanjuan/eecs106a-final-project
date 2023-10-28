import math
import random

import numpy as np
from abb_robot_sandbox.col import *
from abb_robot_sandbox.util import RobotKinematics


def sample_config():
    """ Samples a random configuration """
    config = [0, 0, 0, 0, 0, 0]
    for i in range(6):
        # NOTE Each joint may have "limits"
        config[i] = random.randrange(-180, 180) * math.pi / 180
        # sample config[i] randomly a number between [-math.pi, math.pi]
    print("Sampled: " + str(config))
    return config


class RRT(object):
    def __init__(self, start_pos, goal_pos, start_orien, goal_orien, robot_cb, obstacle_cbs=[], seed=None):
        kinematics = RobotKinematics()
        self._goal_config = kinematics.ik(goal_pos, goal_orien)
        self._robot_cb = robot_cb
        self._start_config = kinematics.ik(start_pos, start_orien)
        if seed is not None:
            random.seed(seed)
        self._obstacle_cbs = obstacle_cbs
        self._col_handler = ColHandler()

    def plan(self, max_iters=2000):
        print("Starting Configuration: {}\nGoal Configuration: {}".
              format(self._start_config, self._goal_config))
        graph = Graph([self._start_config])
        q_new = self._start_config
        i = 0
        while i < max_iters:
            nodes = graph.get_nodes()
            q_rand = self.weighted_sample_config()
            q_near_index = graph.get_nearest_node(q_rand)
            q_near = nodes[q_near_index]
            q_new = self.new_config(q_near, q_rand)

            # Throw away any invalid sampled configurations.
            if not self.is_valid(q_new):
                i += 1
                continue

            if self.is_goal(q_new):
                print("Found Goal at " + str(q_new) + ", Total Iterations Used:", i)
                return nodes, graph.get_edges()
            graph.add_node(q_new)
            graph.add_edge(len(nodes) - 1, q_near_index)
            # print("Added {} to node list and edge list".format(q_new))
            # print("Node list length: " + str(len(graph._nodes)))
            i += 1

        return None

    def weighted_sample_config(self):
        """ Samples a random configuration and 5% of the time returns goal_config. """
        config = [0, 0, 0, 0, 0, 0]
        if random.randrange(0, 100) >= 5:
            for i in range(6):
                # NOTE Each joint may have "limits"
                config[i] = random.randrange(-180, 180) * math.pi / 180
                # sample config[i] randomly a number between [-math.pi, math.pi]

            return config
        else:
            return self._goal_config

    def is_goal(self, config):
        """ Checks if the configuration is at the goal. """
        # Phase 1 (c)
        # NOTE We want config to be "close enough" to self._goal_config
        for i in range(6):
            # the number multiplied by math.pi/180 is the max discrepancy allowed in degrees
            if abs(self._goal_config[i] - config[i]) > 5 * math.pi / 180:
                return False
        return True

    def is_valid(self, config):
        """ Checks if the configuration is valid (i.e. not in collision) """
        any_col = False
        for obstacle_cb in self._obstacle_cbs:
            if self._col_handler.col_bodies(config, self._robot_cb, obstacle_cb):
                any_col = True
                break

        return not any_col

        # v1 below
        # rp = RosPack()
        # path = rp.get_path('abb_robot_sandbox')
        # robot_cb = CollisionBody()
        # robot_cb.load(os.path.join(path, 'config/robot_collision_spheres.yaml'))
        # obstacle_cb = CollisionBody()
        # obstacle_cb.load(os.path.join(path, 'config/obstacle_collision_spheres.yaml'))
        # cl = col_handler()
        # print(cl.col_bodies(config, robot_cb, obstacle_cb))
        # return not cl.col_bodies(config, robot_cb, obstacle_cb)

    def is_valid_edge(self, config_1_list, config_2_list):
        """ Checks if the path from config_1 to config_2 is valid (i.e. not in collision) """
        config_1 = np.array(config_1_list)
        config_2 = np.array(config_2_list)
        change = 0.1
        for i in range(1, 10):
            if not self.is_valid(config_1 + change * i * (config_2 - config_1)):
                return False
        return True

    def new_config(self, q_near, q_rand):
        q_near = np.array(q_near)
        near_vec, rand_vec = np.array(q_near), np.array(q_rand)
        direction = rand_vec - near_vec
        np.true_divide(direction, np.linalg.norm(direction), out=direction, casting='unsafe')
        q_new = q_near + 0.1 * direction
        return q_new.tolist()

    def reconstruct_path(self, data):
        # Check to make sure the RRT found a path.
        if data is None:
            print("No path found!")
            return []

        print("Beginning Path Reconstruction")
        nodes = data[0]
        edges = data[1]
        end = len(nodes) - 1
        path = [end]
        path_step = 0
        while 0 not in path:
            path.append(edges[path[path_step]])
            # print("Added step #{} to path: {}".format(path_step, path[-1]))
            path_step += 1
        path.reverse()
        path_nodes = []
        for i in path:
            path_nodes.append(nodes[i])
        return path_nodes

    def shortcutting(self, path):
        for i in range(len(path) - 2):
            for j in range(len(path) - 1, 1, -1):
                if i < len(path) and self.is_valid_edge(path[i], path[j]):
                    # print("line between {} and {} does not contain obstacles".format(path[i], path[j]))
                    temp_path = path[:i]
                    temp_path.extend(path[j:])
                    # for x in temp_path:
                    #     print("temp_path: " + str(x))
                    path = temp_path[:]
                    break
        return path

    def get_coeffs(self, q_0, qd_0, q_1, qd_1, t_1):  # q(0), qdot(0), q(1), qdot(1), t_1
        th_3 = -qd_0 * t_1 - 2 * q_1 + 2 * q_0 + 2 * qd_0 * t_1 + qd_1 * t_1
        th_3 /= 9 * (t_1 ** 3)
        th_2 = q_1 - q_0 - qd_0 * t_1 - (t_1 ** 3) * th_3
        th_2 /= t_1 ** 2
        return [q_0, qd_0, th_2, th_3]

    def get_values(self, t, th_0, th_1, th_2, th_3):
        # print(t, th_0, th_1, th_2, th_3)
        q_t = th_0 + th_1 * t + th_2 * (t ** 2) + th_3 * (t ** 3)
        return q_t

    # 1. Fit a piecewise polynomial to the RRT's path
    # Input: path from the RRT (list of configurations)
    # Ouput: list of lists of coefficients
    #   Example: if the RRT's path has 10 points, then this will return a list of 9 sets of 24 coefficients
    def get_path_coeffs(self, path, duration):
        coeffs = []
        for i in range(len(path) - 1):
            ths = []
            for j in range(6):
                ths.extend(self.get_coeffs(path[i][j], 0, path[i + 1][j], 0, duration))
            coeffs.append(ths)
        # print(coeffs)
        return coeffs

    # 2. Produce samples along the smoothed RRT path
    # Input: output from 1, time step
    # Output: list of configs
    def get_configs(self, coeffs, dt, duration):
        configs = []
        for i in range(len(coeffs)):
            for j in range(int(duration / dt)):
                current_config = []
                for k in range(6):
                    current_config.append(
                        self.get_values(j * dt, coeffs[i][k * 4], coeffs[i][k * 4 + 1], coeffs[i][k * 4 + 2],
                                        coeffs[i][k * 4 + 3]))
                configs.append(current_config)
        return configs


class Graph(object):
    def __init__(self, nodes):
        self._nodes = nodes
        self._edges = {}

    def add_edge(self, q_new_index, q_near_index):
        self._edges[q_new_index] = q_near_index

    def add_node(self, node):
        self._nodes.append(node)

    def get_nearest_node(self, rand_node):
        distances = []
        nodes = self._nodes
        for i in range(len(nodes)):
            distance = 0.
            for j in range(len(nodes[i])):
                distance += (nodes[i][j] - rand_node[j]) ** 2
            distances.append(math.sqrt(distance))
        return distances.index(min(distances))

    def get_nodes(self):
        return self._nodes

    def get_edges(self):
        return self._edges
