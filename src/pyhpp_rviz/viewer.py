import threading

import numpy as np
import pinocchio as pin
import pyhpp.core as core
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from hpp_gepetto_viewer.msg import HppVectorConfiguration, PathInfo, PinocchioJoint
from nav_msgs.msg import Path
from pinocchio.visualize import BaseVisualizer
from pyhpp import tools
from pyhpp.manipulation import Device, modelsInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher

from .publisher.Navigation import NavigationPublisher
from .publisher.RobotDescriptionPublisher import RobotDescriptionPublisher
from .publisher.StaticTfPublisher import StaticTFPublisher
from .publisher.TransformStampedPublisher import TransformStampedPublisher


class RVizVisualizer(BaseVisualizer):
    """Pinocchio RViz2 visualizer (ROS 2)"""

    def __init__(self):
        self.robot: Device = None
        self.model: pin.Model = pin.Model()
        self.data: pin.Data = pin.Data()

        self.fixed_frame = "world"

        self.robot_description_publisher: RobotDescriptionPublisher = None
        self.static_tf_publisher: StaticTFPublisher = None
        self.navigation_publisher: NavigationPublisher = None
        self.transform_stamped_publisher: TransformStampedPublisher = None
        self._joints_node: Node = None
        self._path_node: Node = None

        self.pinocchioJoint_pub: Publisher = None
        self.path_info_pub: Publisher = None

        self._executor = None
        self._spin_thread = None

        self.current_path = None
        self.last_vector_configuration = None

        self.path_info_pub = None
        self.pinocchioJoint_pub = None

        self.description_publishers = {}

        self.graph = None
        self.problem = None
        self._graph_thread = None
        self._react_graph_viewer_port = 6789
        self._react_graph_viewer_host = "localhost"
        self._web_socket_bridge_port = 8765
        self._web_socket_bridge_host = "localhost"

    # ====================== Init ======================

    def initViewer(self, robot: Device = None):
        self._init_model(robot)
        self._init_ros_nodes()
        self._init_executor()
        self._publish_robot_descriptions(robot.modelsInfo())

    def _init_model(self, robot: Device):
        self.model = robot.model()
        self.data = self.model.createData()
        self.robot = robot

    def _init_ros_nodes(self):

        if not rclpy.ok():
            rclpy.init()

        self.robot_description_publisher = RobotDescriptionPublisher()
        self.static_tf_publisher = StaticTFPublisher()
        self.navigation_publisher = NavigationPublisher()
        self.transform_stamped_publisher = TransformStampedPublisher()

        self._joints_node = Node("hpp_pinocchio_joint_publisher")
        self._path_node = Node("hpp_pinocchio_path_publisher")
        self._waypoint_node = Node("hpp_waypoint_publisher")

        # Subscriptions
        self._joints_node.create_subscription(
            PinocchioJoint, "/hpp/pinocchio_joints", self._on_joint_receive, 10
        )

        self._path_node.create_subscription(
            PathInfo, "/hpp/trajectory_time", self._on_trajectory_time, 10
        )

        self._path_node.create_subscription(
            PathInfo, "/hpp/target_frame", self._on_target_frame, 10
        )

        # Publishers
        self.pinocchioJoint_pub = self._joints_node.create_publisher(
            HppVectorConfiguration, "/hpp/scene_objects", 10
        )
        self.path_info_pub = self._path_node.create_publisher(
            PathInfo, "/hpp/pathInfo", 10
        )
        self.waypoint_pub = self._waypoint_node.create_publisher(
            PoseStamped, "/hpp_waypoint_server/waypoint", 10
        )

    def _init_executor(self):
        self._executor = MultiThreadedExecutor()
        for node in [
            self.robot_description_publisher,
            self.static_tf_publisher,
            self.navigation_publisher,
            self.transform_stamped_publisher,
            self._joints_node,
            self._path_node,
            self._waypoint_node,
        ]:
            self._executor.add_node(node)

        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def _publish_robot_descriptions(self, models: list[modelsInfo]):
        if models is None:
            return
        for m in models:
            m.urdfPath = m.urdfPath or ""
            m.prefix = m.prefix or ""
            with open(tools.xacro.retrieve_resource(m.urdfPath)) as f:
                robot_desc = f.read()
            if not robot_desc:
                raise ValueError(
                    f"URDF file at {m.urdfPath} is empty or could not be read."
                )
            self.robot_description_publisher.publish(m.prefix, robot_desc)

        self._publish_root_static_transforms(models)

    # ====================== RVIZ SETUP ======================
    def _publish_root_static_transforms(self, models: list[modelsInfo]):
        for m in models:
            ns = m.prefix.strip("/") if m.prefix else ""

            root_frame = self._get_first_body_frame_for_namespace(ns)
            if not root_frame:
                continue

            pose: pin.SE3 = m.pose
            self.static_tf_publisher.publish(
                parent_frame=self.fixed_frame,
                child_frame=root_frame,
                xyz=pose.translation,
                quat_xyzw=pin.Quaternion(pose.rotation).coeffs(),
            )

    def _get_first_body_frame_for_namespace(self, namespace: str) -> str | None:
        prefix = f"{namespace}/" if namespace else ""
        for frame in self.model.frames:
            if frame.type != pin.FrameType.BODY:
                continue
            if frame.name == "universe":
                continue
            if prefix and not frame.name.startswith(prefix):
                continue
            if not prefix and "/" in frame.name:
                continue
            return frame.name
        return None

    # ====================== Display ======================
    def __call__(self, q):
        self.display(q)

    def display(self, q=None):
        if q is None:
            return
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        if self.transform_stamped_publisher is not None:
            self._publish_scene(q)
        self.last_vector_configuration = q

    # ====================== Publication ======================

    def _publish_scene(self, q):
        q_vec = np.asarray(q).reshape(-1)
        now = self.transform_stamped_publisher.get_clock().now().to_msg()

        transforms = []
        for frame_id in range(self.model.nframes):
            frame = self.model.frames[frame_id]
            if frame.name in ("universe",):
                continue

            oMf = self.data.oMf[frame_id]
            quat = pin.Quaternion(oMf.rotation)

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.fixed_frame
            t.child_frame_id = frame.name
            t.transform.translation.x = float(oMf.translation[0])
            t.transform.translation.y = float(oMf.translation[1])
            t.transform.translation.z = float(oMf.translation[2])
            t.transform.rotation.x = float(quat.x)
            t.transform.rotation.y = float(quat.y)
            t.transform.rotation.z = float(quat.z)
            t.transform.rotation.w = float(quat.w)
            transforms.append(t)

        self.transform_stamped_publisher.broadcaster.sendTransform(transforms)
        self._publish_joint_states(q_vec, now)

    def _publish_joint_states(self, q_vec, now):
        """Publish the joint states of the robot as HppVectorConfiguration on /hpp/scene_objects."""
        array_msg = HppVectorConfiguration()

        for joint_id in range(1, self.model.njoints):
            joint: pin.JointModel = self.model.joints[joint_id]
            name = self.model.names[joint_id]

            jtype = "JOINT" if joint.nq == 1 else "FREE_FLYER"
            idx_q = joint.idx_q
            min_val = float(self.model.lowerPositionLimit[idx_q])
            max_val = float(self.model.upperPositionLimit[idx_q])

            msg = PinocchioJoint()
            msg.name = name
            msg.type = jtype
            msg.min = min_val
            msg.max = max_val

            if jtype == "JOINT":
                msg.values = [float(q_vec[idx_q])]
            elif jtype == "FREE_FLYER":
                vals = q_vec[idx_q : idx_q + 7]
                msg.values = vals.tolist()

            array_msg.joints.append(msg)

        array_msg.hpp_vector = q_vec.tolist()
        self.pinocchioJoint_pub.publish(array_msg)

    def _on_joint_receive(self, msg: PinocchioJoint):
        if self.last_vector_configuration is None:
            return

        j: pin.JointModel = self.model.joints[self.model.getJointId(msg.name)]
        if j is None:
            print(f"Received joint name '{msg.name}' not found in model.")
            return

        q = self.last_vector_configuration.copy()

        if msg.type == "FREE_FLYER":
            vals = [float(v) for v in msg.values[:7]]
            quat = np.array(vals[3:7])
            norm = np.linalg.norm(quat)
            if norm > 1e-9:
                quat /= norm
            vals[3:7] = quat.tolist()
            q[j.idx_q : j.idx_q + 7] = vals

        elif msg.type == "JOINT":
            q[j.idx_q] = msg.values[0]

        self.display(q)

    # ====================== Path ======================

    def loadPath(self, path: core.bindings.Path):
        self.current_path = path
        msg = PathInfo()
        msg.path_length = float(path.length())
        msg.frame_names = [frame.name for frame in self.model.frames]
        self.path_info_pub.publish(msg)

    def _on_trajectory_time(self, msg: PathInfo):
        if self.current_path is None:
            return
        t = np.clip(msg.current_time, 0.0, self.current_path.length())
        self.display(self.current_path.eval(t)[0])

    def _on_target_frame(self, msg: PathInfo):
        if self.current_path is None:
            return
        self.displayPath(
            self.current_path, topic_name="hpp_path", target_frame=msg.target_frame
        )

    def displayPath(
        self,
        path: core.bindings.Path,
        dt: float = 0.05,
        topic_name: str = "hpp_path",
        origin: str = "world",
        target_frame: str | None = None,
    ):
        if target_frame is None or target_frame == "":
            return
        frame_names = [f.name for f in self.model.frames]
        if target_frame not in frame_names:
            print(
                f"⚠️ Frame '{target_frame}' not found in model.\n Available frames: {frame_names}"
            )
            return

        now = self.navigation_publisher.get_clock().now().to_msg()
        msg = Path()
        msg.header.frame_id = origin
        msg.header.stamp = now

        t = 0.0
        while t <= path.length() + 1e-6:
            q_vec = np.asarray(path.eval(t)[0]).reshape(-1)
            pin.forwardKinematics(self.model, self.data, q_vec)
            pin.updateFramePlacements(self.model, self.data)

            oMf = self.data.oMf[self.model.getFrameId(target_frame)]
            quat = pin.Quaternion(oMf.rotation)

            pose = PoseStamped()
            pose.header.frame_id = origin
            pose.header.stamp = now
            pose.pose.position.x = oMf.translation[0]
            pose.pose.position.y = oMf.translation[1]
            pose.pose.position.z = oMf.translation[2]
            pose.pose.orientation.x = quat.x
            pose.pose.orientation.y = quat.y
            pose.pose.orientation.z = quat.z
            pose.pose.orientation.w = quat.w
            msg.poses.append(pose)
            t += dt

        self.navigation_publisher.publish(path_msg=msg, topic_name=topic_name)

    def printActualRvizVectorConfiguration(self, with_names: bool = True):
        if self.last_vector_configuration is None:
            print("No configuration available.")
            return

        q_map = {}
        for joint_id in range(1, self.model.njoints):
            name = self.model.names[joint_id]
            joint: pin.JointModel = self.model.joints[joint_id]
            if joint.nq == 1:
                q_map[joint.idx_q] = name
            elif joint.nq == 7:
                labels = ["x", "y", "z", "qx", "qy", "qz", "qw"]
                for i, label in enumerate(labels):
                    q_map[joint.idx_q + i] = f"{label} {name} (free-flyer)"

        print("[")
        for idx, value in enumerate(self.last_vector_configuration):
            suffix = f"  # {q_map[idx]}" if with_names and idx in q_map else ""
            print(f"\t{value},{suffix}")
        print("]")

    # ====================== Waypoints ======================

    def addWaypoint(self, xyz: list[float], quat_xyzw: list[float]):
        pose = PoseStamped()
        pose.header.frame_id = self.fixed_frame
        pose.header.stamp = self._waypoint_node.get_clock().now().to_msg()
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation.x = quat_xyzw[0]
        pose.pose.orientation.y = quat_xyzw[1]
        pose.pose.orientation.z = quat_xyzw[2]
        pose.pose.orientation.w = quat_xyzw[3]
        self.waypoint_pub.publish(pose)

    def addWaypointFromFrame(self, target_frame: str):
        if target_frame is None or target_frame == "":
            return
        frame_names = [f.name for f in self.model.frames]
        if target_frame not in frame_names:
            print(
                f"⚠️ Frame '{target_frame}' not found in model.\n Available frames: {frame_names}"
            )
            return

        frame_id = self.model.getFrameId(target_frame)
        oMf = self.data.oMf[frame_id]
        quat = pin.Quaternion(oMf.rotation)
        self.addWaypoint(oMf.translation.tolist(), quat.coeffs().tolist())

    # ====================== Graph Viewer ======================
    def setProblem(self, problem):
        """Set problem for graph viewer integration.

        Args:
            problem: PyWProblem from pyhpp.manipulation
        """
        self.problem = problem
        self._publish_viewer_snapshot(self.graph, self.problem)

    def setGraph(self, graph):
        """Set constraint graph for graph viewer integration.

        Args:
            graph: PyWGraph from pyhpp.manipulation
        """
        self.graph = graph
        self._publish_viewer_snapshot(self.graph, None)

    def _publish_viewer_snapshot(self, graph=None, problem=None):
        """Send the current graph/problem snapshot to the React app if available."""
        if self._graph_thread is None or not self._graph_thread.is_alive():
            print("Graph viewer thread not running, cannot publish viewer snapshot.")
            return
        self._graph_thread.send_viewer_snapshot(graph, problem)

    def launch_graph_viewer(self):
        """Launch hpp-plot graph viewer in separate thread."""
        if self.graph is None:
            print("No constraint graph set. Use viewer.setGraph(graph)")
            return
        if self.problem is None:
            print("No problem set. Use viewer.setProblem(problem)")
            return

        try:
            if self._graph_thread is not None and self._graph_thread.is_alive():
                print("Graph viewer thread is already running")
                return
            from pyhpp_plot import GraphViewerThread

            thread = GraphViewerThread(
                self.graph,
                self.problem,
                self._on_config_generated,
                react_port=self._react_graph_viewer_port,
                react_host=self._react_graph_viewer_host,
                ws_port=self._web_socket_bridge_port,
                ws_host=self._web_socket_bridge_host,
                start_qt_viewer=False,
            )
            self._graph_thread = thread
            thread.start()
        except Exception as exc:
            print(f"Failed to launch graph viewer: {exc}")
            pass

    def _on_config_generated(self, config, label):
        """Called from graph viewer thread when config is generated."""
        self.last_vector_configuration = config
        self.display(config)

    def sendConfigToGraphViewer(self, config=None):
        if config is not None:
            self._graph_thread.sendConfig(config)
        elif self.last_vector_configuration is not None:
            self._graph_thread.sendConfig(self.last_vector_configuration)
        else:
            print("No configuration available to send to graph viewer.")

    # ====================== Méthodes abstraites ======================

    def captureImage(self, w=None, h=None):
        pass

    def disableCameraControl(self):
        pass

    def enableCameraControl(self):
        pass

    def drawFrameVelocities(self, *args, **kwargs):
        pass

    def setBackgroundColor(self, *args, **kwargs):
        pass

    def setCameraPose(self, pose):
        pass

    def setCameraPosition(self, position):
        pass

    def setCameraTarget(self, target):
        pass

    def setCameraZoom(self, zoom):
        pass

    def displayCollisions(self, visibility: bool):
        pass

    def displayVisuals(self, visibility: bool):
        pass
