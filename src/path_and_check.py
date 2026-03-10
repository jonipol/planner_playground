import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import IsPathValid 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from rclpy.executors import MultiThreadedExecutor

class SimpleNode(Node):
  def __init__(self):
    super().__init__('path_and_check')
    start = PoseStamped()
    start.header.frame_id = 'map'

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    
    # Longer path, slightly clipping towards the end
    # start.pose = Pose(position=Point(x=1.9678, y=-0.0649, z=0.0000), orientation=Quaternion(x=0.0000, y=0.0000, z=0.2154, w=0.9765))
    # goal.pose = Pose(position=Point(x=-0.5405, y=4.2112, z=0.0000), orientation=Quaternion(x=0.0000, y=0.0000, z=0.1885, w=0.9821))
    
    # Tiny move, very invalid
    start.pose = Pose(position=Point(x=2.5007, y=1.0792, z=0.0000), orientation=Quaternion(x=0.0000, y=0.0000, z=0.4328, w=0.9015))
    goal.pose = Pose(position=Point(x=2.7434, y=1.0518, z=0.0000), orientation=Quaternion(x=0.0000, y=0.0000, z=0.2210, w=0.9753))

    action_req = ComputePathToPose.Goal()
    action_req.start = start
    action_req.goal = goal
    action_req.use_start = True

    self.valid_client = self.create_client(IsPathValid, 'is_path_valid')
    self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
    self.action_client.wait_for_server()
    self.future = self.action_client.send_goal_async(action_req)
    self.future.add_done_callback(self.goal_response_callback)


  def goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().info('Goal rejected')
      return

    self.get_logger().info('Goal accepted')
    self.result_future = goal_handle.get_result_async()
    self.result_future.add_done_callback(self.get_result_callback)

  def get_result_callback(self, future):
    result = future.result().result
    self.get_logger().info(f'Path found')
    self.valid_client.wait_for_service()
    valid_req = IsPathValid.Request()
    valid_req.path = result.path
    valid_req.max_cost = 254
    resp = self.valid_client.call(valid_req)
    if resp.is_valid:
      self.get_logger().info('Path is valid')
    else:
      self.get_logger().warning('Path is invalid')
      self.get_logger().info(f'Response: {resp}')
    rclpy.shutdown()


def main(args=None):
  rclpy.init(args=args)
  node = SimpleNode()
  executor = MultiThreadedExecutor()
  executor.add_node(node)
  executor.spin()

if __name__ == '__main__':
  main()