import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tier4_external_api_msgs.srv import Engage


class NistNode(Node):
    def __init__(self):
        super().__init__('autoware_nist_node')

        # current position of the vehicle
        self._current_position_x = None

        # Callback groups
        callback_group1 = MutuallyExclusiveCallbackGroup()
        callback_group2 = MutuallyExclusiveCallbackGroup()

        # Subscribe to the goal topic
        # self.goal_subscriber = self.create_subscription(
        #     PoseStamped,
        #     '/move_base_simple/goal',
        #     self.goal_callback,
        #     10
        # )

        # Subscribe to the /initialpose topic
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_cb,
            10,
            callback_group=callback_group1
        )

        # Create a publisher for the PoseWithCovarianceStamped messages
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.publisher_initial_pose = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic='/initialpose',
            qos_profile=qos_profile)

        # Create a publisher for the PoseStamped messages
        self.publisher_goal_pose = self.create_publisher(
            PoseStamped,
            '/planning/mission_planning/goal',
            qos_profile=qos_profile
        )


        # # Create a publisher for the planned trajectory
        # self.trajectory_publisher = self.create_publisher(
        #     Trajectory,
        #     '/planned_trajectory',
        #     qos_profile=qos_profile
        # )

        # # Create a publisher for the vehicle control commands
        # self.control_publisher = self.create_publisher(
        #     VehicleControlCommand,
        #     '/vehicle_control_cmd',
        #     10
        # )

        self.current_goal = None
        self._timer = self.create_timer(2, self.run, callback_group=callback_group2)
        self._follower_initialized = False
        self._follower_goal_set = False

    def engage(self, engage):
        client = self.create_client(Engage, '/api/autoware/set/engage')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Engage service not available, waiting again...')
            
        request = Engage.Request()
        request.engage = engage
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Engage service call successful')
        else:
            self.get_logger().info('Engage service call failed')
        
        
    def run(self):
        '''
        Main loop of the node
        '''
        if not self._follower_initialized:
            self.initialize()
            self._follower_initialized = True
        
        if not self._follower_goal_set:
            if self._current_position_x is not None:
                self.go_to_goal()
                self._follower_goal_set = True
                self.engage(True)

    def initialpose_cb(self, msg):
        '''
        Callback function for the /initialpose topic

        Args:
            msg (PoseWithCovarianceStamped): The message received from the /initialpose topic
        '''
        # self.get_logger().info('Received initial pose message')
        self._current_position_x = msg.pose.pose.position.x
        # self.get_logger().info(f'Current position: {self._current_position_x}')

    # def goal_callback(self, goal):
    #     # This function is called when a new goal is received
    #     self.current_goal = goal

    # def navigate_to_goal(self):

    #     # Create a single waypoint in the trajectory
    #     waypoint = PoseStamped()
    #     waypoint.pose.position.x = 195.0
    #     waypoint.pose.position.y = -133.34
    #     waypoint.pose.position.z = 0.0
    #     waypoint.pose.orientation.x = 0.0
    #     waypoint.pose.orientation.y = 0.0
    #     waypoint.pose.orientation.z = 0.0
    #     waypoint.pose.orientation.w = 1.0

    #     # Add the waypoint to the trajectory
    #     trajectory.points.append(waypoint)

    #     # Publish the planned trajectory
    #     self.trajectory_publisher.publish(trajectory)

    #     # Simulate vehicle control commands
    #     control_command = VehicleControlCommand()
    #     control_command.velocity = 10.0  # Example velocity, adjust as needed
    #     control_command.acceleration = 0.0
    #     control_command.jerk = 0.0

    #     self.control_publisher.publish(control_command)

    def go_to_goal(self):
        '''
        Publish a PoseStamped message to the /planning/mission_planning/goal topic
        '''
        # Create a PoseStamped message
        pose = PoseStamped()

        # Set the pose values
        pose.header.frame_id = 'map'
        pose.pose.position.x = 290.48
        pose.pose.position.y = -133.49
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        # Publish the PoseStamped message
        self.publisher_goal_pose.publish(pose)
        self.get_logger().info('Published mission plan for the follower vehicle')


    def initialize(self):
        '''
        Initialize the ego vehicle's position in the map
        '''

        # Create a PoseWithCovarianceStamped message
        pose = PoseWithCovarianceStamped()

        # Set the pose values
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = 161.0
        pose.pose.pose.position.y = -133.0
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = 0.0
        pose.pose.pose.orientation.w = 1.0

        # Set the covariance values
        pose.pose.covariance = [0.0] * 36  # Fill covariance matrix with zeros
        pose.pose.covariance[0] = 0.25  # x
        pose.pose.covariance[7] = 0.25  # y
        pose.pose.covariance[35] = 0.06853891945200942  # yaw
        # Publish the PoseWithCovarianceStamped message
        self.publisher_initial_pose.publish(pose)
        self.get_logger().info('Initialized follower vehicle')

        


def main(args=None):
    '''
    Main function showing a multi-threaded executor.
    '''
    rclpy.init(args=args)
    nist_node = NistNode()
    executor = MultiThreadedExecutor()
    executor.add_node(nist_node)
    try:
        nist_node.get_logger().info('Beginning demo, end with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        nist_node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    nist_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
