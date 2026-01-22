#!/usr/bin/env python3
"""
LLM Agent Node for Ackermann Vehicle Parking
Uses Qwen 2.5 VL via Ollama to interpret natural language parking commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion
from vehicle_interfaces.action import ParkingCommand

import requests
import json
import base64
import re
from typing import Optional, Dict, Any
import threading


class LLMAgentNode(Node):
    """ROS 2 node that interfaces with local Qwen 2.5 VL for parking commands."""
    
    # Predefined parking spots in the world
    PARKING_SPOTS = {
        'perpendicular_1': {'x': 8.0, 'y': 4.0, 'yaw': 1.5708},
        'perpendicular_2': {'x': 8.0, 'y': -4.0, 'yaw': 1.5708},
        'parallel': {'x': -8.0, 'y': 3.0, 'yaw': 0.0},
    }
    
    SYSTEM_PROMPT = """You are a parking assistant. Output JSON only.
When user says "parallel" or "between cars", use parking_type="parallel" and target_spot="parallel".
When user says "perpendicular" or just "park", use parking_type="simple" and target_spot="perpendicular_1".

Respond ONLY with JSON: {"parking_type": "parallel", "target_spot": "parallel", "speed": 2.0, "confidence": 0.9}"""

    def __init__(self):
        super().__init__('llm_agent_node')
        
        # Parameters
        self.declare_parameter('ollama_url', 'http://127.0.0.1:11434')
        self.declare_parameter('model_name', 'qwen3-vl:2b')
        self.declare_parameter('use_vision', False)
        
        self.ollama_url = self.get_parameter('ollama_url').value
        # Use the model that's actually installed
        self.model_name = 'qwen:latest' 
        self.use_vision = self.get_parameter('use_vision').value
        
        # Callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/user_command',
            self.command_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Optional camera subscriber for vision-based parking
        self.current_image: Optional[bytes] = None
        if self.use_vision:
            self.image_sub = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10,
                callback_group=self.callback_group
            )
        
        # Publishers
        self.response_pub = self.create_publisher(String, '/llm_response', 10)
        self.status_pub = self.create_publisher(String, '/llm_status', 10)
        
        # Action client for parking
        self.parking_client = ActionClient(
            self,
            ParkingCommand,
            '/execute_parking',
            callback_group=self.callback_group
        )
        
        # State
        self.processing = False
        self.lock = threading.Lock()
        
        self.get_logger().info(f'LLM Agent initialized with model: {self.model_name}')
        self.get_logger().info(f'Ollama URL: {self.ollama_url}')
        self.publish_status('ready')
    
    def publish_status(self, status: str):
        """Publish current agent status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def image_callback(self, msg: Image):
        """Store the latest camera image for vision processing."""
        # Convert ROS Image to bytes (simplified - assumes RGB8 format)
        self.current_image = bytes(msg.data)
    
    def command_callback(self, msg: String):
        """Process incoming user commands."""
        command = msg.data.strip()
        
        if not command:
            return
        
        with self.lock:
            if self.processing:
                self.get_logger().warn('Already processing a command, please wait...')
                return
            self.processing = True
        
        self.get_logger().info(f'Received command: "{command}"')
        self.publish_status('processing')
        
        # Process in a separate thread to avoid blocking
        thread = threading.Thread(target=self._process_command, args=(command,))
        thread.start()
    
    def _process_command(self, command: str):
        """Process the command using the LLM."""
        try:
            # Query the LLM
            response = self._query_llm(command)
            
            if response:
                self.get_logger().info(f'LLM response: {response}')
                
                # Publish raw response
                resp_msg = String()
                resp_msg.data = json.dumps(response)
                self.response_pub.publish(resp_msg)
                
                # Execute parking command if valid (Threshold removed for user demo)
                if True:
                    self._execute_parking(response)
                else:
                    self.get_logger().warn(f'Low confidence ({response.get("confidence", 0)}), not executing')
                    self.publish_status('low_confidence')
            else:
                self.get_logger().error('Failed to get LLM response')
                self.publish_status('error')
        
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')
            self.publish_status('error')
        
        finally:
            with self.lock:
                self.processing = False
            self.publish_status('ready')
    
    def _query_llm(self, user_command: str) -> Optional[Dict[str, Any]]:
        """Query the local LLM via Ollama API."""
        try:
            # Use /api/generate which works with all Ollama models
            url = f'{self.ollama_url}/api/generate'
            
            payload = {
                'model': self.model_name,
                'prompt': f"{self.SYSTEM_PROMPT}\n\nUser command: {user_command}\n\nRespond with JSON only:",
                'stream': False,
                'format': 'json'
            }
            
            self.get_logger().info(f'Querying Ollama at {url} with model {self.model_name}...')
            
            response = requests.post(url, json=payload, timeout=300)
            response.raise_for_status()
            result = response.json()
            
            # Extract content from either Chat or Generate response
            content = result.get('message', {}).get('content', '') or result.get('response', '')
            
            try:
                return json.loads(content)
            except json.JSONDecodeError:
                json_match = re.search(r'\{[^{}]*\}', content)
                return json.loads(json_match.group()) if json_match else None
        
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f'Cannot connect to Ollama at {self.ollama_url}. Is Ollama running?')
            return None
        except requests.exceptions.Timeout:
            self.get_logger().error('Ollama request timed out')
            return None
        except Exception as e:
            self.get_logger().error(f'Error querying LLM: {e}')
            return None
    
    def _execute_parking(self, llm_response: Dict[str, Any]):
        """Send parking command to the parking controller."""
        parking_type = llm_response.get('parking_type', 'simple')
        target_spot = llm_response.get('target_spot', 'perpendicular_1')
        speed = float(llm_response.get('speed', 50.0))
        
        # Get target pose
        if target_spot in self.PARKING_SPOTS:
            spot = self.PARKING_SPOTS[target_spot]
            target_pose = Pose()
            target_pose.position = Point(x=spot['x'], y=spot['y'], z=0.0)
            
            # Convert yaw to quaternion (simplified - only yaw rotation)
            import math
            yaw = spot['yaw']
            target_pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(yaw / 2),
                w=math.cos(yaw / 2)
            )
        else:
            # Default pose
            target_pose = Pose()
            target_pose.position = Point(x=5.0, y=0.0, z=0.0)
            target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Wait for action server
        if not self.parking_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Parking action server not available')
            self.publish_status('parking_server_unavailable')
            return
        
        # Create and send goal
        goal = ParkingCommand.Goal()
        goal.parking_type = parking_type
        goal.target_pose = target_pose
        goal.speed = speed
        
        self.get_logger().info(f'Sending parking goal: type={parking_type}, spot={target_spot}')
        self.publish_status('executing_parking')
        
        # Send goal asynchronously
        future = self.parking_client.send_goal_async(
            goal,
            feedback_callback=self._parking_feedback_callback
        )
        future.add_done_callback(self._parking_goal_response_callback)
    
    def _parking_goal_response_callback(self, future):
        """Handle parking goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Parking goal rejected')
            self.publish_status('goal_rejected')
            return
        
        self.get_logger().info('Parking goal accepted')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._parking_result_callback)
    
    def _parking_feedback_callback(self, feedback_msg):
        """Handle parking feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Parking progress: {feedback.progress_percent:.1f}% - {feedback.current_phase}'
        )
    
    def _parking_result_callback(self, future):
        """Handle parking result."""
        result = future.result().result
        
        if result.success:
            self.get_logger().info(f'Parking completed: {result.message}')
            self.publish_status('parking_complete')
        else:
            self.get_logger().warn(f'Parking failed: {result.message}')
            self.publish_status('parking_failed')


def main(args=None):
    rclpy.init(args=args)
    
    node = LLMAgentNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
