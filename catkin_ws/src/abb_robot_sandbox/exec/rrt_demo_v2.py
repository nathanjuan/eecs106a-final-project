#!/usr/bin/env python
import rospy
import os
import yaml
from copy import deepcopy
from profilehooks import profile
from rospkg import RosPack
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from abb_robot_sandbox.vis import Vis
from abb_robot_sandbox.col import CollisionBody
from abb_robot_sandbox.util import RobotKinematics
from abb_robot_sandbox.rrt_v2 import RRT
from sensor_msgs.msg import JointState

DEBUG_START_EE_POS = [1.185, 0.05, 1.57]
DEBUG_START_EE_ORIEN = [0, 0.7071, 0, 0.7071]
DEBUG_START_CONFIG = [0, 0, 0, 0, 0, 0]

DEBUG_GOAL_EE_POS = [-1.5961, 0.8398, 0.6118]
DEBUG_GOAL_EE_ORIEN = [-0.8805, -0.2162, -0.4096, -0.1006]
DEBUG_GOAL_CONFIG = [2.66, 0.70, 0, 0, 0, 0]

DEBUG_DASH_EE_POS = [0.366, 1.223, 1.272]
DEBUG_DASH_EE_ORIEN = [-0.475, 0.638, 0.362, 0.486]
DEBUG_DASH_CONFIG = [1.28, 0, 0.27, 0, 0, 0]

DEBUG_SHORTCUT_PATH = False

SETUP_SAVE_PATH_TO_FILE = True
SAVE_FILE = "output/path.yaml"

SETUP_LOAD_PATH_FROM_FILE = True
LOAD_FILE = "output/path.yaml"


class RRTDemo(object):
    def __init__(self):
        self._jnt_pub = rospy.Publisher("/in/joint_states", JointState, queue_size=1)
        self._vis = Vis()
        self._kin = RobotKinematics()
        self._server = InteractiveMarkerServer("rrt_demo_marker_server")
        self._menu_handler = MenuHandler()

        rp = RosPack()

        self._start_pos = DEBUG_START_EE_POS
        self._pos_at_dash_pickup = DEBUG_DASH_EE_POS
        self._goal_pos = DEBUG_GOAL_EE_POS

        self._start_orien = DEBUG_START_EE_ORIEN
        self._orien_at_dash_pickup = DEBUG_DASH_EE_ORIEN
        self._goal_orien = DEBUG_GOAL_EE_ORIEN

        self._shortcut_path = DEBUG_SHORTCUT_PATH

        # Get dashboard and robot collision spheres
        self._dashboard_cb = CollisionBody()
        self._robot_cb = CollisionBody()
        obstacle_cb = CollisionBody()
        path_to_package = rp.get_path('abb_robot_sandbox')
        self._dashboard_cb.load(os.path.join(path_to_package, 'config/dashboard_collision_spheres.yaml'))
        self._robot_cb.load(os.path.join(path_to_package, 'config/robot_collision_spheres.yaml'))
        obstacle_cb.load(os.path.join(path_to_package, 'config/obstacle_collision_spheres.yaml'))
        self._obstacle_cbs = [obstacle_cb]

        # If SETUP_SAVE_PATH_TO_FILE is True, setup file saving
        if SETUP_SAVE_PATH_TO_FILE:
            self._save_file = open(os.path.join(path_to_package, SAVE_FILE), "w")

        self._save_path_to_file = False

        # If SETUP_LOAD_PATH_FROM_FILE is True, setup loading from file
        if SETUP_LOAD_PATH_FROM_FILE:
            self._load_file = yaml.safe_load(open(os.path.join(path_to_package, LOAD_FILE), "r"))

        self._use_smooth_path = False

        # Create interactive markers
        start_marker = self._vis.visualize_vacuum_cups(
            self._start_pos, [1, 0, 0, 0], color=[1, 0, 0, 1], marker_id=0, interactive=True)
        dash_pickup_marker = self._vis.visualize_vacuum_cups(
            self._pos_at_dash_pickup, [1, 0, 0, 0], color=[0, 1, 0, 1], marker_id=1, interactive=True)
        final_goal_marker = self._vis.visualize_vacuum_cups(
            self._goal_pos, [1, 0, 0, 0], color=[1, 1, 0, 1], marker_id=2, interactive=True)
        self._create_menu([final_goal_marker, start_marker, dash_pickup_marker])

        self._path_to_display = []
        self._vis_tree_start_nodes = []
        self._vis_tree_goal_nodes = []
        self._vis_tree_start_edges = []
        self._vis_tree_goal_edges = []
        self._vis_tree_connector_edge = []

    def run(self):
        """ Runs the main visualization loop. """
        rate = rospy.Rate(10)
        count = 0
        while not rospy.is_shutdown():
            if count >= len(self._path_to_display):
                count = 0
            if len(self._path_to_display) > 0:
                joint_state = JointState()
                joint_state.position = self._path_to_display[count]
                self._jnt_pub.publish(joint_state)
                # self._vis.visualize_at_config_dashboard(
                #     self._path_to_display[count], marker_id=0)

            if len(self._obstacle_cbs) > 0:
                obstacle_spheres = self._obstacle_cbs[0].get_spheres('link_1')
                for i in range(len(obstacle_spheres)):
                    sphere = obstacle_spheres[i]
                    pos = [sphere[0], sphere[1], sphere[2]]
                    rad = sphere[3]
                    if pos is not None:
                        self._vis.visualize_sphere(
                            pos, rad / 2, rgba=[1, 0, 0, 0.5], marker_ns="obs_spheres", marker_id=i)

            # Tree Visualization
            for i, node in enumerate(self._vis_tree_start_nodes):
                self._vis.visualize_sphere(
                    node, 0.05, [1, 0, 0, 1], marker_ns="vis_start_tree_nodes",
                    marker_id=i)
            for i, node in enumerate(self._vis_tree_goal_nodes):
                self._vis.visualize_sphere(
                    node, 0.05, [1, 1, 0, 1], marker_ns="vis_goal_tree_nodes",
                    marker_id=i)
            for i, points in enumerate(self._vis_tree_start_edges):
                self._vis.visualize_arrow(
                    points[0], points[1], 0.02, [1, 0, 0, 1],
                    marker_ns="vis_start_tree_edges", marker_id=i)
            for i, points in enumerate(self._vis_tree_goal_edges):
                self._vis.visualize_arrow(
                    points[0], points[1], 0.02, [1, 1, 0, 1],
                    marker_ns="vis_goal_tree_edges", marker_id=i)
            if self._vis_tree_connector_edge:
                self._vis.visualize_arrow(self._vis_tree_connector_edge[0], self._vis_tree_connector_edge[1], 0.02,
                                          [0, 1, 0, 1], marker_ns="vis_connect_tree_edges", marker_id=0)
            self._vis.visualize_text([self._start_pos[0], self._start_pos[1], self._start_pos[2] + 0.5], "Start", 0.2,
                                     marker_ns="start_pos_text", marker_id=0)
            self._vis.visualize_text([self._goal_pos[0], self._goal_pos[1], self._goal_pos[2] + 0.5], "Goal", 0.2,
                                     marker_ns="goal_pos_text", marker_id=0)
            self._vis.visualize_text(
                [self._pos_at_dash_pickup[0], self._pos_at_dash_pickup[1], self._pos_at_dash_pickup[2] + 0.5],
                "Dash Pickup", 0.2, marker_ns="dash_pickup_pos_text", marker_id=0)

            count += 1
            rate.sleep()

    def _run_demo(self, feedback):
        """ Runs the RRT to create a path between the start to goal while going through the dash pickup spot. """
        # Begin path to dashboard from starting position
        to_dash_smooth_path, to_dash_path = self._run_rrt(
            self._start_pos, self._pos_at_dash_pickup,
            self._start_orien, self._orien_at_dash_pickup, seed=42)

        # Begin path to goal from dashboard position
        to_goal_smooth_path, to_goal_path = self._run_rrt(
            self._pos_at_dash_pickup, self._goal_pos,
            self._orien_at_dash_pickup, self._goal_orien, seed=42)

        # If file saving is setup and enabled with the checkbox, save the configs to the file.
        if self._save_path_to_file:
            print("Saving path to file!")
            path = deepcopy(to_dash_path)
            # path.extend(to_goal_path)
            print(self._save_file)
            self._save_file.write("configs:\n")
            for i, config in enumerate(path):
                self._save_file.write("  - node: " + str(i) + "\n")
                self._save_file.write("    config: {}, {}, {}, {}, {}, {}\n"
                                      .format(config[0], config[1], config[2], config[3], config[4], config[5]))

        # Combine the different paths
        self._path_to_display = []
        self._path_to_display.extend(to_dash_smooth_path)
        self._path_to_display.extend(to_goal_smooth_path)
        rospy.loginfo("Final smoothed path has {} waypoints...".format(
            len(self._path_to_display)))

    def _run_rrt(self, start_pos, goal_pos, start_orien, goal_orien, seed=None):
        """ Runs the actual RRT with a specified start and goal position and orientation """
        # Check for kinematic feasibility of start and end pos
        start_config = self._kin.ikpy(start_pos, start_orien)
        goal_config = self._kin.ikpy(goal_pos, goal_orien)
        if start_config is None:
            rospy.logerr("IK failed when converting to config from {}!".
                         format(start_pos))
            return []
        if goal_config is None:
            rospy.logerr("IK failed when converting to config from {}!".
                         format(goal_pos))
            return []

        rrt = RRT(start_config, goal_config, self._robot_cb, self._obstacle_cbs, seed)
        # Plan with RRT (returns reconstructed path)
        path = rrt.plan(max_iterations=1000)
        if path == "failure":
            rospy.logerr("Error planning with given configs")
            print("Final Tree: " + str(rrt.tree.get_all_nodes()))
            print("Final Tree 0 Edges: " + str(rrt.tree.get_edges(0)))
            print("Final Tree 1 Edges: " + str(rrt.tree.get_edges(1)))
            return []
        # Find shortcuts in path and add start config to path
        if self._shortcut_path:
            path = rrt.shortcutting(path)
        else:
            rospy.logwarn("NOT shortcutting path")

        print("Path with shortcutting: " + str(path))
        # Do polynomial smoothing on path
        coeffs = rrt.get_path_coeffs(path, 0.5)
        smooth_path = rrt.get_configs(coeffs, 0.01, 0.5)
        return smooth_path, path

    @profile
    def _vis_tree(self, feedback):
        """ Visualizes the tree of the RRT between the start and goal nodes. """
        # TODO: Specifying orientation for IK seems to have no effect
        start_config = self._kin.ikpy(self._start_pos, self._start_orien)
        goal_config = self._kin.ikpy(self._goal_pos, self._goal_orien)

        # Check for kinematic feasibility of start and dash_pickup pos
        if start_config is None:
            rospy.logerr("IK failed when converting to config from {} {}!".format(self._start_pos, self._start_orien))
            return
        if goal_config is None:
            rospy.logerr("IK failed when converting to config from {} {}!".format(self._goal_pos, self._goal_orien))
            return

        rospy.loginfo("Start EE pos: {}, orien: {}".format(
            self._start_pos, self._start_orien))
        rospy.loginfo("Start config (IK): {}".format(start_config))
        rospy.loginfo("Goal EE pos: {}, orien: {}".format(
            self._goal_pos, self._goal_orien))
        rospy.loginfo("Goal config (IK): {}".format(goal_config))
        rrt = RRT(
            start_config, goal_config, self._robot_cb, self._obstacle_cbs, 42)

        path = rrt.plan(max_iterations=1000)
        if len(path) > 0:
            if self._shortcut_path:
                rospy.loginfo("Shortcutting path...")
                path = rrt.shortcutting(path)
            else:
                rospy.logwarn("NOT shortcutting path")

            # Do polynomial smoothing on path
            coeffs = rrt.get_path_coeffs(path, 0.5)
            smooth_path = rrt.get_configs(coeffs, 0.01, 0.5)
            self._path_to_display = deepcopy(smooth_path)
        else:
            rospy.logerr("Failed to find a path!")

        # rospy.loginfo("-- RRT NODES:")
        # for i, config in enumerate(rrt.tree._nodes):
        #     rospy.loginfo("  i: {}, node: {}".format(i, config))
        rospy.loginfo("-- START TREE indices: {}".format(
            rrt.tree.get_node_indices(rrt.tree.START_TREE)))
        rospy.loginfo("-- START TREE edges: {}".format(
            rrt.tree.get_edges(rrt.tree.START_TREE)))
        rospy.loginfo("-- GOAL TREE indices: {}".format(
            rrt.tree.get_node_indices(rrt.tree.GOAL_TREE)))

        start_configs = rrt.tree.get_nodes(rrt.tree.START_TREE)
        self._vis_tree_start_nodes = []
        for config in start_configs:
            self._vis_tree_start_nodes.append(self._kin.fk_pos(config))

        goal_configs = rrt.tree.get_nodes(rrt.tree.GOAL_TREE)
        self._vis_tree_goal_nodes = []
        for config in goal_configs:
            self._vis_tree_goal_nodes.append(self._kin.fk_pos(config))

        start_edges = rrt.tree.get_edges(rrt.tree.START_TREE)
        self._vis_tree_start_edges = []
        for first_idx, second_idx in start_edges.items():
            self._vis_tree_start_edges.append(
                (self._kin.fk_pos(rrt.tree.get_all_nodes()[first_idx]),
                 self._kin.fk_pos(rrt.tree.get_all_nodes()[second_idx])))

        goal_edges = rrt.tree.get_edges(rrt.tree.GOAL_TREE)
        self._vis_tree_goal_edges = []
        for first_idx, second_idx in goal_edges.items():
            self._vis_tree_goal_edges.append(
                (self._kin.fk_pos(rrt.tree.get_all_nodes()[first_idx]),
                 self._kin.fk_pos(rrt.tree.get_all_nodes()[second_idx])))
        all_configs = rrt.tree.get_all_nodes()
        path_between_trees = rrt.tree.get_edge_between_nodes()
        self._vis_tree_connector_edge = [self._kin.fk_pos(all_configs[path_between_trees[0]]),
                                         self._kin.fk_pos(all_configs[path_between_trees[1]])]

    def _display_path_from_file(self, feedback):
        if SETUP_LOAD_PATH_FROM_FILE:
            rrt = RRT([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], self._robot_cb)
            path = []
            for config in self._load_file["configs"]:
                # Add config to path after mapping each parsed string in the config to a double
                path.append(list(map(float, config["config"].split(", "))))
            if self._use_smooth_path:
                self._path_to_display = rrt.get_configs(rrt.get_path_coeffs(path, 0.5), 0.01, 0.5)
            else:
                self._path_to_display = path
        else:
            rospy.logerr("Can't display path from file because file is not setup!")

    def _create_menu(self, int_markers):
        """ Creates a right click menu with options for running the RRT, visualizing the RRT's tree, and toggling
        shortcutting. """
        self._menu_handler.insert("Run", callback=self._run_demo)
        self._menu_handler.insert("Visualize Tree", callback=self._vis_tree)
        load_path_submenu = self._menu_handler.insert("Load path from file")
        self._menu_handler.insert("Display loaded path", parent=load_path_submenu, callback=self._display_path_from_file)
        self._menu_handler.setCheckState(self._menu_handler.insert("Use smooth path", parent=load_path_submenu, callback=self._toggle_use_smooth_path), MenuHandler.UNCHECKED)
        self._menu_handler.setCheckState(self._menu_handler.insert("Shortcutting", callback=self._toggle_shortcutting),
                                         MenuHandler.CHECKED if DEBUG_SHORTCUT_PATH else MenuHandler.UNCHECKED)
        self._menu_handler.setCheckState(self._menu_handler.insert("Save RRT path to file", callback=self._toggle_save_path_to_file), MenuHandler.UNCHECKED)
        for int_marker in int_markers:
            self._server.insert(int_marker, self._process_feedback)
            self._menu_handler.apply(self._server, int_marker.name)

        self._server.applyChanges()

    def _process_feedback(self, feedback):
        """ When an interactive marker is moved, this function will replace the old position of the marker with a new one """
        p = [feedback.pose.position.x, feedback.pose.position.y,
             feedback.pose.position.z]
        o = [feedback.pose.orientation.x, feedback.pose.orientation.y,
             feedback.pose.orientation.z, feedback.pose.orientation.w]

        if feedback.marker_name == "vacuum_cups_0":  # Start
            self._start_pos = p
            self._start_orien = o
            print("Start:", p, o)
        elif feedback.marker_name == "vacuum_cups_1":  # Dash Pickup
            self._pos_at_dash_pickup = p
            self._orien_at_dash_pickup = o
        elif feedback.marker_name == "vacuum_cups_2":  # Goal
            self._goal_pos = p
            self._goal_orien = o
            print("Goal:", p, o)

    def _toggle_shortcutting(self, feedback):
        """ Turns on or off the shortcutting for vis_tree or run_rrt """
        handle = feedback.menu_entry_id
        state = self._menu_handler.getCheckState(handle)
        if state == MenuHandler.CHECKED:
            self._menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self._shortcut_path = False
        else:
            self._menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            self._shortcut_path = True
        self._menu_handler.reApply(self._server)
        self._server.applyChanges()

    def _toggle_save_path_to_file(self, feedback):
        handle = feedback.menu_entry_id
        state = self._menu_handler.getCheckState(handle)
        if state == MenuHandler.UNCHECKED:
            if SETUP_SAVE_PATH_TO_FILE:
                self._save_path_to_file = True
                self._menu_handler.setCheckState(handle, MenuHandler.CHECKED)
                rospy.loginfo("Saving RRT path to file enabled.")
            else:
                rospy.logwarn("Can't enable saving RRT path to file because save file is not specified!")
        else:
            self._save_path_to_file = False
            self._menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            rospy.loginfo("Saving RRT path to file disabled.")
        self._menu_handler.reApply(self._server)
        self._server.applyChanges()

    def _toggle_use_smooth_path(self, feedback):
        handle = feedback.menu_entry_id
        state = self._menu_handler.getCheckState(handle)
        if state == MenuHandler.UNCHECKED:
            self._use_smooth_path = True
            self._menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            rospy.loginfo("Using smooth path when displaying the path loaded from the specified file.")
        else:
            self._use_smooth_path = False
            self._menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            rospy.loginfo("Using normal path when displaying the path loaded from the specified file.")
        self._menu_handler.reApply(self._server)
        self._server.applyChanges()


if __name__ == '__main__':
    rospy.init_node('rrt_demo')

    demo = RRTDemo()
    demo.run()
