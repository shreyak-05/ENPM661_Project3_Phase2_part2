import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from .submodules.astar_planner import plan_path

class RobotAStarPlannerNode(Node):
    def __init__(self):
        super().__init__('robot_a_star_planner')
        self.get_logger().info(f'READY AStar Planner')
        start = self.declare_parameter(
          'start_position', [0.0, 0.0, 0.0]).get_parameter_value().double_array_value
        end = self.declare_parameter(
          'end_position', [0.0, 0.0, 0.0]).get_parameter_value().double_array_value        
        robot_radius = self.declare_parameter(
          'robot_radius', 0.0).get_parameter_value().double_value
        clearance = self.declare_parameter(
          'clearance', 0.0).get_parameter_value().double_value
        delta_time = self.declare_parameter(
          'delta_time', 0.0).get_parameter_value().double_value
        goal_threshold = self.declare_parameter(
          'goal_threshold', 0.0).get_parameter_value().double_value
        wheel_radius = self.declare_parameter(
          'wheel_radius', 0.0).get_parameter_value().double_value
        wheel_distance = self.declare_parameter(
          'wheel_distance', 0.0).get_parameter_value().double_value
        rpms = self.declare_parameter(
          'rpms', [0.0, 0.0]).get_parameter_value().double_array_value

        #Convert Units m to cm
        int_start = [int(element*100) for element in start]
        int_end = [int(element*100) for element in end]
        robot_radius = robot_radius*100
        clearance = clearance*100
        goal_threshold = goal_threshold*100
        wheel_radius = wheel_radius*100
        wheel_distance = wheel_distance*100  
       # Fix: Convert numeric values to strings for logging
        self.get_logger().info(f"Robot radius: {robot_radius}")
        self.get_logger().info(f"Clearance: {clearance}")
        self.get_logger().info(f"Delta time: {delta_time}")
        self.get_logger().info(f"Goal threshold: {goal_threshold}")
        self.get_logger().info(f"Wheel radius: {wheel_radius}")
        self.get_logger().info(f"Wheel distance: {wheel_distance}")
        self.get_logger().info(f"RPMs: {rpms}")

        if len(int_end) == 2:
            int_end = [int_end[0], int_end[1], 0]
        self.get_logger().info(f"Added default orientation to goal: {int_end}")
        
   
        path = plan_path(self,tuple(int_start),tuple(int_end),robot_radius,clearance,delta_time,goal_threshold,wheel_radius,wheel_distance,rpms[0],rpms[1])
        print(f'Path 1: {path}')
        if path == None:
            self.get_logger().info(f'Path 1 Not Found')
            return
        self.get_logger().info(f'Path 1 Planned')
        actual_stop_pos = int_start
        for move in path:
            actual_stop_pos[0] += move[0]
            actual_stop_pos[1] += move[1]
            actual_stop_pos[2] += move[2]
        actual_stop_pos = [ int(x) for x in actual_stop_pos ]
        
        self.state = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1/30, self.on_timer)
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)        
        self.planned_path = path
        self.delta_time = delta_time
        self.current_twist = Twist()
        self.current_twist.angular.x = 0.0
        self.current_twist.angular.y = 0.0
        self.current_twist.angular.z = 0.0
        self.current_twist.linear.x = 0.0
        self.current_twist.linear.y = 0.0
        self.current_twist.linear.z = 0.0
        self.start_time = -1.0
        self.wait_start = 0
        self.last_transform_time_dif = 0.0
        self.get_logger().info(f'READY AStar Node')

    def on_timer(self):
        from_frame_rel = 'IMUSensor_BP_C_0'
        to_frame_rel = 'map'
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return        

        sim_time = t.header.stamp.sec + t.header.stamp.nanosec*10**-9 
        if self.start_time == -1.0:
            self.get_logger().info(f'Starting Phase 1')
            self.start_time = sim_time    
        if self.state == 0:
            if len(self.planned_path)>0:      
                if (sim_time-self.start_time)%self.delta_time < self.last_transform_time_dif:
                    self.planned_path.pop(0)
            if len(self.planned_path)>0: 
                self.last_transform_time_dif = (sim_time-self.start_time)%self.delta_time
                self.publish_twist_cmd(self.planned_path)
            else:
                self.get_logger().info(f'First Target Reached, Waiting')
                self.current_twist.linear.x = 0.0
                self.current_twist.linear.y = 0.0
                self.current_twist.angular.z = 0.0
                self.publisher.publish(self.current_twist)
                self.wait_start = sim_time
                self.state = 1
        elif self.state == 1:
            self.current_twist.linear.x = 0.0
            self.current_twist.linear.y = 0.0
            self.current_twist.angular.z = 0.0
            self.publisher.publish(self.current_twist)

    def publish_twist_cmd(self, planned_path):
        self.current_twist.linear.x = float(planned_path[0][0]/self.delta_time)
        self.current_twist.linear.y = float(planned_path[0][1]/self.delta_time)
        self.current_twist.angular.z = float(planned_path[0][2]/self.delta_time)
        self.publisher.publish(self.current_twist)
    
    # def publish_twist_cmd(self, planned_path):
      # # Convert from cm to m for linear velocities (divide by 100)
      # # Path values are in cm, but ROS expects m/s
      # self.current_twist.linear.x = float(planned_path[0][0]/(self.delta_time * 100))
      # self.current_twist.linear.y = float(planned_path[0][1]/(self.delta_time * 100))
      
      # # Angular velocity stays in radians
      # self.current_twist.angular.z = float(planned_path[0][2]/self.delta_time)
      
      # # Add some logging to debug
      # self.get_logger().info(f"CMD: x={self.current_twist.linear.x:.3f} m/s, " + 
      #                       f"y={self.current_twist.linear.y:.3f} m/s, " + 
      #                       f"θ={self.current_twist.angular.z:.3f} rad/s")
      
      # # Apply reasonable limits to prevent extreme movements
      # max_linear = 0.5  # m/s
      # max_angular = 1.0  # rad/s
      
      # self.current_twist.linear.x = max(min(self.current_twist.linear.x, max_linear), -max_linear)
      # self.current_twist.linear.y = max(min(self.current_twist.linear.y, max_linear), -max_linear)
      # self.current_twist.angular.z = max(min(self.current_twist.angular.z, max_angular), -max_angular)
      
      # self.publisher.publish(self.current_twist)

def main():
    rclpy.init()    
    node = RobotAStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        
