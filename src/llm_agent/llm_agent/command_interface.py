#!/usr/bin/env python3
"""
Command Interface Node - Simple CLI for sending commands to the LLM agent.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CommandInterface(Node):
    """Simple command-line interface for sending parking commands."""
    
    def __init__(self):
        super().__init__('command_interface')
        
        # Publisher for user commands
        self.command_pub = self.create_publisher(String, '/user_command', 10)
        
        # Subscribers for responses
        self.response_sub = self.create_subscription(
            String,
            '/llm_response',
            self.response_callback,
            10
        )
        self.status_sub = self.create_subscription(
            String,
            '/llm_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info('Command Interface started')
        self.get_logger().info('Enter parking commands (e.g., "Park the car"):')
        
        # Start input loop in timer
        self.create_timer(0.1, self.input_loop)
        self.waiting_for_input = True
    
    def input_loop(self):
        """Check for user input."""
        if not self.waiting_for_input:
            return
        
        try:
            import sys
            import select
            
            # Non-blocking input check
            if select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.readline().strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    self.get_logger().info('Exiting...')
                    raise SystemExit()
                
                if user_input:
                    msg = String()
                    msg.data = user_input
                    self.command_pub.publish(msg)
                    self.get_logger().info(f'Sent command: "{user_input}"')
        except Exception:
            pass
    
    def response_callback(self, msg: String):
        """Handle LLM responses."""
        self.get_logger().info(f'LLM Response: {msg.data}')
    
    def status_callback(self, msg: String):
        """Handle status updates."""
        self.get_logger().info(f'Status: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    
    node = CommandInterface()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
