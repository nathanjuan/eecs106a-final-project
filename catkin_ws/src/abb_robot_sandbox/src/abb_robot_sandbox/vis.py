import kdl_parser_py.urdf  # sudo apt install ros-kinetic-kdl-parser-py
from PyKDL import *
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import *
from abb_robot_sandbox.util import RobotKinematics
import rospy


class Vis(object):
    def __init__(self, robot_base_frame_id, robot_tip_frame_id, robot_link_names):
        self.robot_base_frame_id = robot_base_frame_id
        self.robot_tip_frame_id = robot_tip_frame_id
        self.robot_link_names = robot_link_names
        self._vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)
        self._vis_array_pub = rospy.Publisher('visualization_marker_array',
                                              MarkerArray, queue_size=10)

        self._tree = None
        self._frame_to_fk_solver = {}
        # self._link_name_to_mesh = {
        #     'base_link': 'package://abb_irb4600_support/meshes/irb4600/visual/base_link.dae',
        #     'link_1': 'package://abb_irb4600_support/meshes/irb4600/visual/link_1.dae',
        #     'link_2': 'package://abb_irb4600_support/meshes/irb4600_60_205/visual/link_2.dae',
        #     'link_3': 'package://abb_irb4600_support/meshes/irb4600_60_205/visual/link_3.dae',
        #     'link_4': 'package://abb_irb4600_support/meshes/irb4600_60_205/visual/link_4.dae',
        #     'link_5': 'package://abb_irb4600_support/meshes/irb4600_60_205/visual/link_5.dae',
        #     'link_6': 'package://abb_irb4600_support/meshes/irb4600_60_205/visual/link_6.dae'
        # }
        # self._link_name_to_mesh = {
        #     'iiwa_link_0': 'package://iiwa_description/meshes/iiwa14/visual/link_0.stl',
        #     'iiwa_link_1': 'package://iiwa_description/meshes/iiwa14/visual/link_1.stl',
        #     'iiwa_link_2': 'package://iiwa_description/meshes/iiwa14/visual/link_2.stl',
        #     'iiwa_link_3': 'package://iiwa_description/meshes/iiwa14/visual/link_3.stl',
        #     'iiwa_link_4': 'package://iiwa_description/meshes/iiwa14/visual/link_4.stl',
        #     'iiwa_link_5': 'package://iiwa_description/meshes/iiwa14/visual/link_5.stl',
        #     'iiwa_link_6': 'package://iiwa_description/meshes/iiwa14/visual/link_6.stl',
        #     'iiwa_link_7': 'package://iiwa_description/meshes/iiwa14/visual/link_7.stl'
        # }
        self._link_name_to_mesh = {
            'base_link': 'package://ur_description/meshes/ur5/visual/base.dae',
            'shoulder_link': 'package://ur_description/meshes/ur5/visual/shoulder.dae',
            'upper_arm_link': 'package://ur_description/meshes/ur5/visual/upperarm.dae',
            'forearm_link': 'package://ur_description/meshes/ur5/visual/forearm.dae',
            'wrist_1_link': 'package://ur_description/meshes/ur5/visual/wrist1.dae',
            'wrist_2_link': 'package://ur_description/meshes/ur5/visual/wrist2.dae',
            'wrist_3_link': 'package://ur_description/meshes/ur5/visual/wrist3.dae'
        }

        try:
            ok, self._tree = kdl_parser_py.urdf.treeFromParam('robot_description')
            if ok:
                rospy.loginfo('Ok! Parsed tree with {} joints and {} segments'.
                              format(self._tree.getNrOfJoints(),
                                     self._tree.getNrOfSegments()))
                self._chain = self._tree.getChain(robot_base_frame_id, robot_tip_frame_id)


                for i in range(len(self.robot_link_names)):
                    base_to_link_chain = self._tree.getChain(robot_base_frame_id, robot_link_names[i])
                    self._frame_to_fk_solver[robot_link_names[i]] = \
                        ChainFkSolverPos_recursive(base_to_link_chain)
            else:
                rospy.logerr('Not ok!')
        except Exception as err:
            rospy.logerr('[Vis] Error: {}'.format(err))

        self._kinematics = RobotKinematics(robot_tip_frame_id)

    def visualize_sphere(self, pos, radius, rgba=[1, 0, 0, 1],
                         marker_ns='sphere_vis', marker_id=0, interactive=False):
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.robot_base_frame_id
        msg.ns = marker_ns
        msg.id = marker_id
        msg.lifetime = rospy.Duration(0.25)
        msg.type = Marker.SPHERE
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.scale.x = radius * 2
        msg.scale.y = radius * 2
        msg.scale.z = radius * 2
        msg.color.r = rgba[0]
        msg.color.g = rgba[1]
        msg.color.b = rgba[2]
        msg.color.a = rgba[3]
        if not interactive:
            self._vis_pub.publish(msg)
        else:
            return self.make_marker_interactive(msg)

    def visualize_cube(self, pos, scale, rgba=[1, 0, 0, 1],
                         marker_ns='cube_vis', marker_id=0, interactive=False):
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.robot_base_frame_id
        msg.ns = marker_ns
        msg.id = marker_id
        msg.type = Marker.CUBE
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.scale.x = scale[0]
        msg.scale.y = scale[1]
        msg.scale.z = scale[2]
        msg.color.r = rgba[0]
        msg.color.g = rgba[1]
        msg.color.b = rgba[2]
        msg.color.a = rgba[3]
        if not interactive:
            self._vis_pub.publish(msg)
        else:
            return self.make_marker_interactive(msg)

    def visualize_at_config(self, link_names, config=[0.] * 7, rgba=None,
                            marker_id=0, marker_ns_prefix='iiwa14_vis'):
        msg = MarkerArray()

        base_pose = Pose()
        base_pose.orientation.w = 1.0
        base_marker = self.get_mesh_marker(
            self.robot_base_frame_id, base_pose, rgba, marker_ns_prefix, marker_id)
        msg.markers.append(base_marker)

        for i in range(len(link_names)):
            q = JntArray(i + 1)
            for j in range(i + 1):
                q[j] = config[j]

            link_name = link_names[i]
            base_to_link = Frame()
            self._frame_to_fk_solver[link_name].JntToCart(q, base_to_link)

            link_pose = Pose()
            link_pose.position.x = base_to_link.p.x()
            link_pose.position.y = base_to_link.p.y()
            link_pose.position.z = base_to_link.p.z()

            rot = base_to_link.M.GetQuaternion()

            link_pose.orientation.x = rot[0]
            link_pose.orientation.y = rot[1]
            link_pose.orientation.z = rot[2]
            link_pose.orientation.w = rot[3]

            link_marker = self.get_mesh_marker(
                link_name, link_pose, rgba, marker_ns_prefix, marker_id)
            msg.markers.append(link_marker)

        self._vis_array_pub.publish(msg)

    def visualize_at_config_dashboard(self, config=[0.] * 7, rgba=None,
                                      marker_id=0, marker_ns_prefix='iiwa14_vis', interactive=False):
        """ Visualize the robot with the dashboard rigidly attached at it's end-effector. """
        self.visualize_at_config(config, rgba, marker_id, marker_ns_prefix)
        dashboard_pos, dashboard_orientation = self._kinematics.fk(config)
        if not rgba:
            self.visualize_dashboard(
                dashboard_pos, dashboard_orientation, [1, 0, 0, 0.5], marker_id,
                marker_ns_prefix, interactive)
        else:
            self.visualize_dashboard(
                dashboard_pos, dashboard_orientation, rgba, marker_id,
                marker_ns_prefix, interactive)

    def get_mesh_marker(self, link_name, link_pose, rgba=None,
                        marker_ns_prefix='iiwa14_vis', marker_id=0):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.robot_base_frame_id
        marker.ns = marker_ns_prefix + '/' + link_name
        marker.id = marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.pose = link_pose
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        if rgba:
            marker.color.r = rgba[0]
            marker.color.g = rgba[1]
            marker.color.b = rgba[2]
            marker.color.a = rgba[3]
            marker.mesh_use_embedded_materials = False
        else:
            marker.mesh_use_embedded_materials = True

        marker.mesh_resource = self._link_name_to_mesh[link_name]
        return marker

    def visualize_dashboard(self, pos, orientation, color=[1, 0, 0, 0.5],
                            marker_id=0, ns='dashboard', interactive=False):
        marker = Marker()
        pose = Pose()
        # Set Position
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        # Set Orientation
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        # Set size
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        # Set pose, ns, id and time to marker
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'iiwa_link_0'
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.pose = pose
        # Set color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.mesh_use_embedded_materials = False
        # Set dashboard dae as mesh resource
        marker.mesh_resource = 'package://abb_robot_sandbox/meshes/mini_dashboard.dae'
        if not interactive:
            self._vis_pub.publish(marker)
        else:
            return self.make_marker_interactive(marker)

    def visualize_vacuum_cups(self, pos, orientation, color=[1, 0, 0, 1],
                            marker_id=0, ns='vacuum_cups', interactive=False):
        marker = Marker()
        pose = Pose()
        # Set Position
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        # Set Orientation
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        # Set size
        marker.scale.x = 2
        marker.scale.y = 2
        marker.scale.z = 2
        # Set pose, ns, id and time to marker
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'base_link'
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.pose = pose
        # Set color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.mesh_use_embedded_materials = False
        # Set vacuum cups dae as mesh resource
        marker.mesh_resource = 'package://abb_robot_sandbox/meshes/VacuumCupsRotated.dae'
        if not interactive:
            self._vis_pub.publish(marker)
        else:
            return self.make_marker_interactive(marker)

    def make_marker_interactive(self, marker):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = marker.ns + "_" + str(marker.id)

        marker_control = InteractiveMarkerControl()
        marker_control.interaction_mode = InteractiveMarkerControl.BUTTON
        marker_control.always_visible = True
        marker_control.markers.append(marker)

        int_marker.controls.append(marker_control)

        rotate_control = InteractiveMarkerControl()
        rotate_control.orientation.x = 1
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 0
        rotate_control.orientation.w = 1
        rotate_control.name = "move_x"
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control)
        rotate_control = InteractiveMarkerControl()
        rotate_control.orientation.x = 1
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 0
        rotate_control.orientation.w = 1
        rotate_control.name = "rotate_x"
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotate_control)

        rotate_control = InteractiveMarkerControl()
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 1
        rotate_control.orientation.z = 0
        rotate_control.orientation.w = 1
        rotate_control.name = "move_y"
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control)
        rotate_control = InteractiveMarkerControl()
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 1
        rotate_control.orientation.z = 0
        rotate_control.orientation.w = 1
        rotate_control.name = "rotate_y"
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotate_control)

        rotate_control = InteractiveMarkerControl()
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 1
        rotate_control.orientation.w = 1
        rotate_control.name = "move_z"
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control)
        rotate_control = InteractiveMarkerControl()
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 1
        rotate_control.orientation.w = 1
        rotate_control.name = "rotate_z"
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotate_control)

        int_marker.pose.position.x = marker.pose.position.x
        int_marker.pose.position.y = marker.pose.position.y
        int_marker.pose.position.z = marker.pose.position.z

        return int_marker

    def visualize_arrow(self, start_pos, end_pos, width, rgba=[1, 0, 0, 1], marker_ns='arrow_vis', marker_id=0):
        point1 = Point()
        point1.x = start_pos[0]
        point1.y = start_pos[1]
        point1.z = start_pos[2]

        point2 = Point()
        point2.x = end_pos[0]
        point2.y = end_pos[1]
        point2.z = end_pos[2]


        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'iiwa_link_0'
        msg.ns = marker_ns
        msg.id = marker_id
        msg.type = Marker.ARROW
        msg.points = [point1, point2]
        msg.scale.x = width
        msg.scale.y = width
        msg.scale.z = 0.0001
        msg.color.r = rgba[0]
        msg.color.g = rgba[1]
        msg.color.b = rgba[2]
        msg.color.a = rgba[3]

        self._vis_pub.publish(msg)

    def visualize_text(self, pos, text, height, rgba=[1, 0, 0, 1], marker_ns="text_vis", marker_id=0):
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'iiwa_link_0'
        msg.ns = marker_ns
        msg.id = marker_id
        msg.type = Marker.TEXT_VIEW_FACING
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        msg.pose = pose
        msg.text = text
        msg.scale.z = height
        msg.color.r = rgba[0]
        msg.color.g = rgba[1]
        msg.color.b = rgba[2]
        msg.color.a = rgba[3]

        self._vis_pub.publish(msg)

    def visualize_orien_indicator(self, pos, size, marker_ns="orientation_indicator"):
        width = 0.175 * size

        base_point = Point()
        base_point.x = pos[0]
        base_point.y = pos[1]
        base_point.z = pos[2]

        for i in range(3):
            end_point = Point()
            if i == 0:
                end_point.x = pos[0] + size
                end_point.y = pos[1]
                end_point.z = pos[2]
            elif i == 1:
                end_point.x = pos[0]
                end_point.y = pos[1] + size
                end_point.z = pos[2]
            else:
                end_point.x = pos[0]
                end_point.y = pos[1]
                end_point.z = pos[2] + size

            msg = Marker()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "iiwa_link_0"
            msg.ns = marker_ns
            msg.id = i
            msg.type = Marker.ARROW
            msg.points = [base_point, end_point]
            msg.scale.x = width
            msg.scale.y = width * 1.3
            msg.scale.z = 0
            msg.color.a = 1
            if i == 0:
                msg.color.r = 1
                msg.color.g = 0
                msg.color.b = 0
            elif i == 1:
                msg.color.r = 0
                msg.color.g = 1
                msg.color.b = 0
            else:
                msg.color.r = 0
                msg.color.g = 0
                msg.color.b = 1

            self._vis_pub.publish(msg)
