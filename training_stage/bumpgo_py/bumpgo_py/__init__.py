import rclpy
from rclpy.node import Node

class TreeNode(Node):
  
  def __init__(self):
    super().__init__("bump_and_go")
    loop_rate = 0.1
    self.timer = self.create_timer(loop_rate, self.timer_callback)
  

  def timer_callback(self):
    ### Diseno del arbol de comportamiento

    #####################################
    print("tick")


def main(args=None):
  rclpy.init(args=args)

  tree_node = TreeNode()

  rclpy.spin(tree_node)

  tree_node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()