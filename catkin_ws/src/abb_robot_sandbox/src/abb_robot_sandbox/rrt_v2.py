import math
import random
import rospy


class RRT(object):
    def __init__(self, start_config, goal_config, config_validator, robot_cb, obstacle_cbs=None, seed=None):
        """
        robot_cb: CollisionBody of robot
        config_validator: ConfigurationValidator object for checking if nodes are valid in RRT
        start_config: Joint angles of the robot to use for RRT start
        goal_config: Joint angles of the robot to use for RRT goal
        obstacle_cbs (optional): CollisionBody(s) of obstacles (for collision detection)
        seed (optional): Specify seed when sampling configurations
        """
        self._start_config = start_config
        self._goal_config = goal_config
        self._config_validator = config_validator
        self._robot_cb = robot_cb
        self._obstacle_cbs = obstacle_cbs if obstacle_cbs is not None else []
        
        if seed is not None:
            random.seed(seed)

        self._robot_joint_count = len(start_config)
        self._robot_joint_limits = self._config_validator._joint_limits
        
        self._tree = BidirectionalTree(self._start_config, self._goal_config)

        rospy.loginfo("[RRT] intialized with start config: {}, goal config: {}".format(self._start_config, self._goal_config))

    def plan(self, max_iterations=100):
        rospy.loginfo("[plan] started, running for up to {}".format(max_iterations))
        tree_type = 0
        for i in range(max_iterations):
            rospy.loginfo("[plan] iteration: {}, tree type: {}".format(
                i, self._tree.type_to_str(tree_type)))
            q_rand = self.sample_config(self._tree.get_goal_config(tree_type))
            rospy.logdebug("[plan] sampling config {}".format(q_rand))
            extend_result, q_new_index = self.extend(tree_type, q_rand)
            q_new = self._tree.get_all_nodes()[q_new_index]
            if not extend_result == "trapped":
                connect_result = self.connect((tree_type + 1) % 2, q_new)
                if connect_result == "reached":
                    edge_between_nodes = self._tree.get_edge_between_nodes()
                    edge_between_nodes[tree_type] = q_new_index
                    edge_between_nodes[(tree_type + 1) % 2] = len(self._tree.get_all_nodes()) - 1
                    self._tree.set_edge_between_nodes(edge_between_nodes[0], edge_between_nodes[1])
                    rospy.logdebug(
                        "[plan] edge connecting two trees (start: {}, goal: {})".format(
                            self._tree.get_edge_between_nodes()[BidirectionalTree.START_TREE],
                            self._tree.get_edge_between_nodes()[BidirectionalTree.GOAL_TREE])
                    )
                    return self.reconstruct_path()
            tree_type = (tree_type + 1) % 2
        return []
        
    def sample_config(self, goal=None):
        """ Samples a random configuration within joint_limits and, if a goal is specified, returns goal_config the 10% of the time. """
        config = [0] * self._robot_joint_count
        rand_val = random.randrange(0, 100)
        if rand_val >= 10 or goal is None:
            for i in range(self._robot_joint_count):
                joint_limit = self._robot_joint_limits[i]
                config[i] = random.uniform(joint_limit[0], joint_limit[1])
            return config
        else:
            return goal
            
    def extend(self, tree_type, q_rand):
        q_near_index = self._tree.get_nn_index(q_rand, tree_type)
        q_near = self._tree.get_all_nodes()[q_near_index]
        rospy.logdebug("[extend] q_near_index: {} in {}".format(
            q_near_index, self._tree.type_to_str(tree_type)))
        q_new = self.new_config(q_near, q_rand)
        if q_new is not None and self.is_valid_edge(q_new, q_near):
            rospy.logdebug("[extend] adding node {} to {}".format(
                q_new, self._tree.type_to_str(tree_type)))
            q_new_index = self._tree.add_node(q_new, tree_type)
            rospy.logdebug("[extend] adding edge (parent: {}, child: {}) to {}".format(
                q_near_index, q_new_index, self._tree.type_to_str(tree_type)))
            self._tree.add_edge(q_new_index, q_near_index, tree_type)

            if self.is_config_close(q_new, q_rand):
                return "reached", q_new_index
            else:
                return "advanced", q_new_index
        return "trapped", -1
    
    def connect(self, tree_type, q_rand, max_iterations=100):
        rospy.logdebug("[connect] trying to connect rand config to tree {}".format(
            self._tree.type_to_str(tree_type)))
        s, q_new_index = self.extend(tree_type, q_rand)
        i = 0
        while s is "advanced" and i < max_iterations:
            rospy.logdebug("[connect]  i: {}, s: {}".format(i, s))
            s, q_new_index = self.extend(tree_type, q_rand)
            i += 1

        return s

    def is_valid_edge(self, config_1_list, config_2_list):
        """ Checks if the path from config_1 to config_2 is valid (i.e. not in collision) """
        # Create Smoothed Path
        path = [config_1_list, config_2_list]
        path_coeffs = self.get_path_coeffs(path, 0.5)
        smooth_path = self.get_configs(path_coeffs, 0.01, 0.5)
        # Check that each config is valid
        for config in smooth_path:
            if not self._config_validator.is_valid(config, do_append_joints=True):
                return False
        return True

    def new_config(self, q_near, q_rand):
        q_new = []
        for i in range(self._robot_joint_count):
            if q_rand[i] - q_near[i] > 0:
                direction = 1
            else:
                direction = -1
            q_new.append(q_near[i] + 0.1 * direction)
        if self._config_validator.is_valid(q_new, do_append_joints=True):
            return q_new
        else:
            return None

    def is_config_close(self, first_config, second_config):
        """ Checks whether `first_config` and `second_config` are close. """
        for i in range(self._robot_joint_count):
            if abs(first_config[i] - second_config[i]) > 10 * math.pi / 180:
                return False

        errors = [abs(first_config[i] - second_config[i]) for i in range(self._robot_joint_count)]
        rospy.logdebug("[is_config_close] config {} is close enough to {} with error {}".
              format(first_config, second_config, errors))
        return True

    def is_goal(self, config, tree_type):
        """ Checks if the configuration is at the goal. """
        if tree_type == BidirectionalTree.GOAL_TREE:
            goal = self._goal_config
        elif tree_type == BidirectionalTree.START_TREE:
            goal = self._start_config
        else:
            rospy.logerror("[is_goal] invalid tree type {}!".format(tree_type))
            return False

        for i in range(self._robot_joint_count):
            if abs(goal[i] - config[i]) > 10 * math.pi / 180:
                return False

        errors = [abs(goal[i] - config[i]) for i in range(self._robot_joint_count)]
        rospy.logdebug("[is_goal] config {} is close enough to tree {}'s goal with error {}".
              format(config, tree_goal, errors))
        return True
        
    def reconstruct_path(self):
        """ Reconstructs path from start to goal by following parents. """
        tree_to_path_idxs = [[], []]
        for tree_type in [BidirectionalTree.START_TREE, BidirectionalTree.GOAL_TREE]:
            idx = self._tree.get_edge_between_nodes()[tree_type]
            node_to_parent = self._tree.get_edges(tree_type)
            while idx >= 0:
                parent_idx = -1
                if idx in node_to_parent:
                    parent_idx = node_to_parent[idx]

                rospy.logdebug("[reconstruct_path] at node {} with parent {} in {}".
                      format(idx, parent_idx, self._tree.type_to_str(tree_type)))
                tree_to_path_idxs[tree_type].append(idx)
                idx = parent_idx

        tree_to_path_idxs[BidirectionalTree.START_TREE].reverse()

        path_idxs = tree_to_path_idxs[BidirectionalTree.START_TREE] + tree_to_path_idxs[BidirectionalTree.GOAL_TREE]
        path = [self._tree.get_all_nodes()[idx] for idx in path_idxs]
        return path

    def shortcutting(self, path):
        rospy.loginfo("[shortcutting] started")
        i = 0
        while i < len(path):
            for j in range(len(path)-1, i+1, -1):
                if self.is_valid_edge(path[i], path[j]):
                    del path[i+1:j]
                    break
            i += 1
        rospy.loginfo("[shortcutting] {} nodes remaining in path".format(len(path)))

    def get_coeffs(self, q_0, qd_0, q_1, qd_1, t_1):
        th_3 = -qd_0 * t_1 - 2 * q_1 + 2 * q_0 + 2 * qd_0 * t_1 + qd_1 * t_1
        th_3 /= 9 * (t_1 ** 3)
        th_2 = q_1 - q_0 - qd_0 * t_1 - (t_1 ** 3) * th_3
        th_2 /= t_1 ** 2
        return [q_0, qd_0, th_2, th_3]

    def get_path_coeffs(self, path, duration):
        """ Fit a piecewise polynomial to the RRT's path """
        """ Input: path from the RRT (list of configurations), and total path duration """
        """ Output: 2D list of coefficients """
        """ Example: if the RRT's path has 10 points, then this will return a list of 9 sets of 24 coefficients """
        coeffs = []
        for i in range(len(path) - 1):
            ths = []
            for j in range(self._robot_joint_count):
                ths.extend(self.get_coeffs(path[i][j], 0, path[i + 1][j], 0, duration))
            coeffs.append(ths)
        return coeffs
        
    def get_values(self, t, th_0, th_1, th_2, th_3):
        q_t = th_0 + th_1 * t + th_2 * (t ** 2) + th_3 * (t ** 3)
        return q_t


    def get_configs(self, coeffs, dt, duration):
        """ Produce samples along the smoothed RRT path """
        """ Input: path coeffs, time step between each config, and total path duration """
        """ Output: list of configs """
        configs = []
        for i in range(len(coeffs)):
            for j in range(int(duration / dt)):
                current_config = []
                for k in range(self._robot_joint_count):
                    current_config.append(
                        self.get_values(j * dt, coeffs[i][k * 4], coeffs[i][k * 4 + 1], coeffs[i][k * 4 + 2],
                                        coeffs[i][k * 4 + 3]))
                    # Normalize the new angle
                    current_value_deg = current_config[k] * (180 / math.pi)

                    while current_value_deg < -180:
                        current_value_deg += 360
                        current_config[k] = current_value_deg * (math.pi / 180)

                    while current_value_deg > 180:
                        current_value_deg -= 360
                        current_config[k] = current_value_deg * (math.pi / 180)

                configs.append(current_config)
        return configs


class BidirectionalTree:
    START_TREE = 0  # Tree rooted at the start config
    GOAL_TREE = 1  # Tree rooted at the goal config

    START_NODE_INDEX = 0
    GOAL_NODE_INDEX = 1

    def __init__(self, start_config, goal_config):
        self._nodes = [start_config, goal_config]
        self._start_tree_indices = [0]
        self._goal_tree_indices = [1]
        self._tree_type_to_edges = [{}, {}]
        self._edge_between_nodes = [-1, -1]

    def type_to_str(self, tree_type):
        if tree_type == self.START_TREE:
            return "START_TREE"
        elif tree_type == self.GOAL_TREE:
            return "GOAL_TREE"

        return "UNKNOWN"

    def get_goal_config(self, tree_type):
        if tree_type == self.START_TREE:
            # Tree grows towards the goal config (at index 1)
            return self._nodes[1]
        elif tree_type == self.GOAL_TREE:
            # Tree grows towards the start config (at index 0)
            return self._nodes[0]

        rospy.logerror("Error: invalid tree type {}".format(tree_type))
        return []

    def get_all_nodes(self):
        """ Returns all nodes in both start and goal trees. """
        return self._nodes

    def get_nodes(self, tree_type):
        if tree_type == self.START_TREE:
            return self._start_tree_indices[:]
        elif tree_type == self.GOAL_TREE:
            return self._goal_tree_indices[:]

        rospy.logerror("Error: invalid tree type {}".format(tree_type))
        return []

    def get_node_indices(self, tree_type):
        if tree_type == self.START_TREE:
            return self._start_tree_indices
        elif tree_type == self.GOAL_TREE:
            return self._goal_tree_indices

        rospy.logerror("Error: invalid tree type {}".format(tree_type))
        return []

    def get_edges(self, tree_type):
        return self._tree_type_to_edges[tree_type]

    def get_edge_between_nodes(self):
        return self._edge_between_nodes

    def set_edge_between_nodes(self, first_idx, second_idx):
        self._edge_between_nodes = [first_idx, second_idx]

    def add_edge(self, q_new, q_near, tree_type):
        self._tree_type_to_edges[tree_type][q_new] = q_near

    def add_node(self, config, tree_type):
        self._nodes.append(config)
        index = len(self._nodes) - 1
        if tree_type == self.START_TREE:
            self._start_tree_indices.append(index)
        elif tree_type == self.GOAL_TREE:
            self._goal_tree_indices.append(index)
        else:
            rospy.logerror("Error: invalid tree type {}".format(tree_type))

        return index

    def get_nn_index(self, q, tree_type):
        """ Get index for nearest neighbor of q in specified tree type """
        indices = []
        if tree_type == self.START_TREE:
            indices = self._start_tree_indices
        elif tree_type == self.GOAL_TREE:
            indices = self._goal_tree_indices
        else:
            rospy.logerror("Error: invalid tree type {}".format(tree_type))
        
        min_distance = float('inf')
        min_idx = -1
        for i in indices:
            # Calculate distance between q and current node
            distance = [(self._nodes[i][j] - q[j]) ** 2 for j in range(len(self._nodes[i]))]
            if distance < min_distance:
                min_distance = distance
                min_idx = i

        return indices[min_idx]

    def get_nn(self, q, tree_type):
        index = self.get_nn_index(q, tree_type)
        return self._nodes[index]
