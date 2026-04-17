#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import os
import time
import threading
import json
from typing import List, Dict, Any, Optional
from collections import deque
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion, Vector3
from std_msgs.msg import Int32, String
from aruco_mapper_msgs.msg import ArucoMapperState
from euro.yaml_loader import load_tasks_from_yaml
import math
from euro.pathfinder_controller import PathfinderClient
from euro.simple_map import Map
from euro.task_scheduler import TaskScheduler, Task
from euro.strategy_executor import robot_status, lidar_status, stm32_task, front_task, back_task,rasp_pi_task, pami_task, path_planning_task, task_tracking_task
from euro.shared_state import state  # Import module to access SHUTDOWN_REQUESTED and other globals

# Global variable to store team color
color = None
n=0
found_goal_batch=False
class StrategyManager(Node):
    """
    Simple strategy manager that loads and displays tasks from YAML
    """
    
    def __init__(self, node_name='strategy_manager'):
        super().__init__(node_name)
        
        # Task storage - simple list
        self.tasks = []
        
        # Team color from YAML
        self.color = None
        
        # Strategy name from YAML
        self.strategy_name = None
        
        # Device queues
        self.queues = {
            'STM': deque(),
            'ESP_front': deque(),
            'ESP_back': deque(),
            'RASP_PI': deque(),
            'PAMI': deque()
        }
        
        # Flag to pause task loading during pathfinding
        self.is_replanning = False
        # Flag to interrupt waiting loops (for enemy detection handling)
        self.interrupt_wait = False
        # Flags to track which wait loops were interrupted
        self.load_task_to_queues_ended = False
        self.stm32_task_wait_ended = False
        # Global goal position storage
        self.global_pos_x = None
        self.global_pos_y = None
        self.global_pos_angle = None
        self.latest_robot_pose = None
        self.latest_robot_pose_beacon = None
        self.latest_robot_pose_beacon_count = 0
        self.end_game=False
        # Track last pick_up/put_down command and device from task
        self.last_task_command = None
        self.last_task_device = None
        # Storage for prepared path commands
        self.prepared_commands = []
        self.prepared_goal_angle = None
        # Create Round timer instance
        from euro.strategy_executor import Round
        self.round = Round()
        
        self.create_subscription(
            Vector3,
            '/robot_pose_raw',
            self.robot_pose_callback,
            10
        )
        self.create_subscription(
            ArucoMapperState,
            '/aruco_mapper/state_robots',
            self.aruco_mapper_callback,
            10
        )

        # Beacon readiness status for dashboard: 0 = not ready, 1 = ready
        beacon_qos = QoSProfile(depth=1)
        beacon_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        beacon_qos.reliability = ReliabilityPolicy.RELIABLE
        self.beacon_status_pub = self.create_publisher(Int32, '/web/beacon_status', beacon_qos)
        self.task_scheduler_status_pub = self.create_publisher(String, '/web/task_scheduler_status', 10)

        self.beacon_status_value = 0
        self.publish_beacon_status(self.beacon_status_value)
        self._beacon_startup_publish_count = 0
        self._beacon_startup_timer = self.create_timer(0.5, self._publish_initial_beacon_status)
        self._task_scheduler_status_timer = self.create_timer(0.25, self._publish_task_scheduler_status)
        self.pathfinder = PathfinderClient()
        self.path_called = False
        self.remove_esp_waiting=False
        # New scheduler stack (map + task scheduler) exposed through strategy manager.
        # This allows executor and other modules to use `strategy.task_scheduler.*`.
        self.map = Map(round_timer=self.round)
        self.task_scheduler = TaskScheduler(
            map=self.map,
            pathfinder_client=self.pathfinder,
            ros_node=self,
        )
        self.get_logger().info('🗺️ TaskScheduler initialized and attached to strategy_manager node')

    def publish_beacon_status(self, value: int) -> None:
        self.beacon_status_value = int(value)
        msg = Int32()
        msg.data = self.beacon_status_value
        self.beacon_status_pub.publish(msg)
        self.get_logger().info(f"📡 Published beacon status: {msg.data}")

    def _publish_initial_beacon_status(self) -> None:
        # Republish initial 0 a few times so late-starting dashboard listeners get it.
        if self._beacon_startup_publish_count >= 5:
            if self._beacon_startup_timer is not None:
                self._beacon_startup_timer.cancel()
                self._beacon_startup_timer = None
            return

        self.publish_beacon_status(self.beacon_status_value)
        self._beacon_startup_publish_count += 1

    def _publish_task_scheduler_status(self) -> None:
        """Publish periodic scheduler status for realtime dashboard."""
        if not hasattr(self, 'task_scheduler') or self.task_scheduler is None:
            return

        try:
            snapshot = self.task_scheduler.get_status_snapshot(source='strategy_manager', max_queue_items=30)
            snapshot['scheduler_started'] = True
            snapshot['timestamp_ms'] = int(time.time() * 1000)
            snapshot['ros_queues'] = {
                'STM': int(len(self.queues.get('STM', []))),
                'ESP_front': int(len(self.queues.get('ESP_front', []))),
                'ESP_back': int(len(self.queues.get('ESP_back', []))),
                'RASP_PI': int(len(self.queues.get('RASP_PI', []))),
            }

            msg = String()
            msg.data = json.dumps(snapshot)
            self.task_scheduler_status_pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"task_scheduler status publish skipped: {e}")

    def robot_pose_callback(self, msg):
        self.latest_robot_pose = msg
    
    def aruco_mapper_callback(self, msg):
        """Callback for aruco mapper state - store entire message"""
        self.latest_robot_pose_beacon = msg
        #self.get_logger().info(f"aruco mapper callback")
        self.latest_robot_pose_beacon_count += 1
    def path(self, end_game=False, prepare=False):
        """Plan path from current position to stored goal.
        
        Args:
            end_game: Whether this is for end game return home
            prepare: If True, generate and store path without loading to queue.
                    If False, load previously prepared path to queue without generating new path.
        
        Returns:
            bool: Success status
        """
        global n 
        global found_goal_batch
        
        if prepare:
            # PREPARE MODE: Generate path and store it, don't load to queue yet
            self.get_logger().info(f"🗺️  Preparing path (searching only, not loading to queue)")
            
            # Use pathfinder's path method (it uses stored goal and latest robot pose)
            stm32_commands, success = self.pathfinder.path()
            
            if success:
                # Store commands and goal angle for later execution
                self.prepared_commands = stm32_commands
                self.prepared_goal_angle = self.pathfinder.goal_angle
                self.get_logger().info(f"✅ Path prepared! {len(stm32_commands)} waypoints found (not loaded to queue yet)")
            else:
                self.prepared_commands = []
                self.prepared_goal_angle = None
                self.get_logger().warning("⚠️ Path preparation failed")
            
            return success
        
        else:
            # EXECUTE MODE: Load prepared path to queue (or generate if none prepared)
            self.get_logger().info(f"🚀 Loading path to queue for execution")
            
            # Set flag to prevent task loading during replanning
            self.is_replanning = True
            
            
            # Check if we have a prepared path
            if self.prepared_commands:
                self.get_logger().info(f"✓ Using prepared path with {len(self.prepared_commands)} waypoints")
                stm32_commands = self.prepared_commands
                goal_angle = self.prepared_goal_angle
                success = True
                # Clear prepared commands after use
                self.prepared_commands = []
                self.prepared_goal_angle = None
            else:
                # No prepared path - generate one now (fallback)
                self.get_logger().warning("⚠️ No prepared path available - generating now as fallback")
                stm32_commands, success = self.pathfinder.path()
                goal_angle = self.pathfinder.goal_angle if success else None
            
            # Load commands to queue
            if success:
                # Clear the STM32 queue
                self.queues['STM'].clear()
                state.SYSTEMS_IDLE_COUNT=0
                # Process commands and add to queue
                last_cmd = None
                for cmd in stm32_commands:
                    values = cmd.replace('go_to(', '').replace(')', '').split(',')
                    x = float(values[0].strip())
                    y = float(values[1].strip())
                    speed = int(values[2].strip())
                    if cmd != last_cmd:
                        if self.last_task_device=="ESP_back":
                            self.get_logger().info(f"go_to_backwards: x={x}, y={y}")
                            self.queues['STM'].append({
                                'command': 'go_to_backwards',
                                'params': [x, y, 1100.0, {'from_pathfinding': True}],  # Mark as from pathfinding
                                'wait': False
                            })
                        else:
                            self.get_logger().info(f"go_to: x={x}, y={y}")
                            self.queues['STM'].append({
                                'command': 'go_to',
                                'params': [x, y, 1100.0, {'from_pathfinding': True}],  # Mark as from pathfinding
                                'wait': False
                            })

                    last_cmd = cmd
                    
                if success and goal_angle is not None and not end_game:
                    self.queues['STM'].append({
                        'command': 'orientate',
                        'params': [goal_angle, 0.0],  # List format: [angle, is_radian]
                    })
                    if self.last_task_command=='pick_up' and found_goal_batch:
                        if self.last_task_device == 'ESP_front' :
                            self.queues['STM'].append({
                                'command': 'go_to',
                                'params': [self.global_pos_x*1000, self.global_pos_y*1000, 1100.0, 1.0, {'from_pathfinding': True}],  # Mark as from pathfinding
                                'wait': False
                            })
                            self.get_logger().info(f"Added go_to command with global position: x={self.global_pos_x}, y={self.global_pos_y} (last command after pathfinding )")
                        elif self.last_task_device == 'ESP_back' :
                            self.queues['STM'].append({
                                'command': 'go_to_backwards',
                                'params': [self.global_pos_x*1000, self.global_pos_y*1000, 1100.0, 1.0, {'from_pathfinding': True}],  # Mark as from pathfinding
                                'wait': False
                            })
                            
                n=len((self.queues["STM"]))
                
                self.get_logger().info(f"✅ Path loaded to queue! {len(self.queues['STM'])} commands ready for execution")
                # Resume task execution
                state.replanning = False
                self.is_replanning = False
                
            return success
    def handle_global_goal(self, x, y, angle):
        global found_goal_batch
        target_x_mm = float(x)*1000 
        target_y_mm = float(y) *1000

        batch_info = None
        dropoff_id = ""
        if hasattr(self, 'map') and self.map is not None:
            if self.last_task_command == 'pick_up':
                batch_info = self.map.find_batch_by_position(
                    x=target_x_mm,
                    y=target_y_mm,
                    max_distance_mm=300.0,
                )
            elif self.last_task_command == 'put_down':
                dropoff_id = str(self.map.find_dropoff_id_by_position(
                    x=target_x_mm,
                    y=target_y_mm,
                    max_distance_mm=300.0,
                ) or '').lower()

        success = (batch_info is not None) if self.last_task_command == 'pick_up' else bool(dropoff_id)
        batch_known_id = str(batch_info.known_id).lower() if batch_info is not None else ""

        if self.last_task_command == 'pick_up':
            self.get_logger().info(
                f"global goal pickup lookup at ({target_x_mm:.1f},{target_y_mm:.1f}) -> "
                f"found={success}, id={batch_known_id or 'none'}"
            )
        elif self.last_task_command == 'put_down':
            self.get_logger().info(
                f"global goal dropoff lookup at ({target_x_mm:.1f},{target_y_mm:.1f}) -> "
                f"found={success}, id={dropoff_id or 'none'}"
            )
        else:
            self.get_logger().info(
                f"global goal lookup skipped for last_task_command={self.last_task_command} "
                f"at ({target_x_mm:.1f},{target_y_mm:.1f})"
            )

        # Track the execution intent in task_scheduler from global-goal context.
        # - pick_up  -> target_id = batch known_id returned by get_batch_color
        # - put_down -> target_id = shared state known_id_front/back based on last ESP device
        if hasattr(self, 'task_scheduler') and self.task_scheduler is not None:
            tracked_type = None
            tracked_target_id = ""

            if self.last_task_command == 'pick_up':
                tracked_type = 'pick_up'
                tracked_target_id = str(batch_known_id or '').lower()
            elif self.last_task_command == 'put_down':
                tracked_type = 'put_down'
                tracked_target_id = str(dropoff_id or '').lower()
            else:
                #remove tracked task if no longer relevant
                if hasattr(self, 'task_scheduler') and self.task_scheduler is not None:
                    self.task_scheduler.untrack_task()
            if tracked_type is not None and tracked_target_id:
                tracked_task = Task(
                    target_id=tracked_target_id,
                    type=tracked_type,
                    options={},
                    score=0.0,
                )
                self.task_scheduler.track_task(tracked_task)
                self.get_logger().info(
                    f"🧭 task_scheduler.track_task -> type={tracked_type}, target_id={tracked_target_id or 'unknown'}"
                )
            elif tracked_type is not None:
                self.get_logger().warning(
                    f"⚠️ Skipping task tracking update: no batch id resolved for last_task_command={self.last_task_command}"
                )
        
        if success and self.last_task_command =='pick_up' :
            found_goal_batch=True
            self.get_logger().info(
                f"Batch found for pick_up near global goal: id={batch_known_id}"
            )
        else:
            found_goal_batch=False
            self.get_logger().info(
                "No matching batch for pick_up near global goal (or last command is not pick_up)"
            )
    def quaternion_to_yaw(self, quat: Quaternion) -> float:
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat
    
    def load_task_to_queues(self, task_name: str, allow_during_emergency: bool = False):
        """Load a task's commands into device queues"""
        # Find the task
        task = None
        for t in self.tasks:
            if t['name'] == task_name:
                task = t
                break
        
        if not task:
            self.get_logger().error(f"Task '{task_name}' not found!")
            return False
        
        # Iterate through sequence and add to queues
        for idx, cmd in enumerate(task['sequence']):
            device = cmd.get('device')
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            cmd_enqueue_epoch = int(getattr(state, 'ESP_DISPATCH_EPOCH', 0))
            if not self.end_game:
                self.get_logger().info(f"⚙️  [{idx+1}/{len(task['sequence'])}] {device} -> {command}")
                state.SYSTEMS_IDLE_COUNT=0

                # Add command to device queue
                if device in self.queues:
                    if wait :
                        self.get_logger().info(f"      ⏳ Waiting for dependencies...")
                        if(device == "STM"):
                            while not state.ESP_FRONT_IDLE or not state.ESP_BACK_IDLE:
                                self.get_logger().info(f"Waiting for ESP systems to be idle... ESP_FRONT_IDLE={state.ESP_FRONT_IDLE}, ESP_BACK_IDLE={state.ESP_BACK_IDLE}")
                                if self.interrupt_wait:
                                    self.get_logger().warning("🛑 ESP wait interrupted")
                                    self.load_task_to_queues_ended = True
                                    return False
                                time.sleep(0.1)
                        if(device == "ESP_front" or device == "ESP_back"):
                            state.SYSTEMS_IDLE_COUNT=0
                            expected_stm_count = len(self.queues["STM"])
                            if state.IDLE_FOR_SYSTEMS and not state.IDLE:
                                state.SYSTEMS_IDLE_COUNT=1
                            
                            # Track if prepare command has been added
                            prepare_added = False
                            
                            while state.SYSTEMS_IDLE_COUNT < expected_stm_count and len((self.queues["STM"]))>0 :
                                if int(getattr(state, 'ESP_DISPATCH_EPOCH', 0)) != cmd_enqueue_epoch:
                                    self.get_logger().warning(
                                        f"🛑 ESP32 command wait canceled due to dispatch epoch change ({cmd_enqueue_epoch} -> {state.ESP_DISPATCH_EPOCH})"
                                    )
                                    return False

                                self.get_logger().debug(
                                    f"Waiting for STM commands to be done in load tasks to queues... STM queue size: {expected_stm_count}, SYSTEMS_IDLE_COUNT: {state.SYSTEMS_IDLE_COUNT}"
                                )
                                # Static flow safety: if emergency/steal starts while waiting,
                                # abort this task loading so ESP commands are not queued from stale flow.
                                if(self.remove_esp_waiting):
                                    emergency_active = bool(getattr(state, 'EMERGENCY_STOP', False))
                                    if allow_during_emergency and not emergency_active:
                                        # Generated flow may inherit a stale interrupt flag after emergency clears.
                                        self.get_logger().info(
                                            "🔄 Consuming stale ESP wait interrupt after emergency clear; continuing generated enqueue"
                                        )
                                        self.remove_esp_waiting = False
                                    else:
                                        self.get_logger().warning(
                                            f"🛑 ESP32 command wait interrupted (allow_during_emergency={allow_during_emergency}, emergency_active={int(emergency_active)})"
                                        )
                                        self.remove_esp_waiting = False
                                        return False

                                # Add prepare command when on second-to-last STM command
                                if not prepare_added and state.SYSTEMS_IDLE_COUNT == expected_stm_count-1:
                                    prepare_cmd = {
                                        'command': 'prepare',
                                        'params': {'action': command},
                                        'wait': False,
                                        '_dispatch_epoch': state.ESP_DISPATCH_EPOCH,
                                    }
                                    self.queues[device].append(prepare_cmd)
                                    self.get_logger().info(
                                        f"      🔧 Adding prepare command for {device} with action: {command} (STM at {expected_stm_count-1}/{expected_stm_count})"
                                    )
                                    prepare_added = True
                                
                                # self.get_logger().info(
                                #     f"Waiting for STM commands to be sent... STM queue size: {expected_stm_count}, SYSTEMS_IDLE_COUNT: {state.SYSTEMS_IDLE_COUNT}"
                                # )
                                time.sleep(0.1)

                            if int(getattr(state, 'ESP_DISPATCH_EPOCH', 0)) != cmd_enqueue_epoch:
                                self.get_logger().warning(
                                    f"🛑 ESP32 enqueue canceled due to dispatch epoch change ({cmd_enqueue_epoch} -> {state.ESP_DISPATCH_EPOCH})"
                                )
                                return False

                            if expected_stm_count > 0 and state.SYSTEMS_IDLE_COUNT < expected_stm_count:
                                self.get_logger().warning(
                                    f"🛑 ESP32 dependency wait ended early (STM target={expected_stm_count}, done={state.SYSTEMS_IDLE_COUNT})"
                                )
                                return False

                    if device =="SSTM" and command =="set_position":
                        self.queues["RASP_PI"].append(cmd)
                    else:
                        # Mark busy only when command is actually enqueued.
                        # This avoids stale busy flags when wait is interrupted.
                        if device == "ESP_front":
                            state.ESP_FRONT_IDLE = False
                        if device == "ESP_back":
                            state.ESP_BACK_IDLE = False
                        if device == "STM" and command != "set_position":
                            state.IDLE_FOR_SYSTEMS = False

                        if device =="RASP_PI" and command=="Wait for manual start":
                            state.WAITING_TIRETTE=True
                        if not self.end_game:
                            cmd_to_enqueue = dict(cmd)
                            if device in ("ESP_front", "ESP_back"):
                                if int(getattr(state, 'ESP_DISPATCH_EPOCH', 0)) != cmd_enqueue_epoch:
                                    self.get_logger().warning(
                                        f"🛑 Skipping stale {device} enqueue due to epoch mismatch ({cmd_enqueue_epoch} -> {state.ESP_DISPATCH_EPOCH})"
                                    )
                                    return False
                                cmd_to_enqueue['_dispatch_epoch'] = state.ESP_DISPATCH_EPOCH
                            self.queues[device].append(cmd_to_enqueue)
                    
                else:
                    self.get_logger().warn(f"      ⚠ Unknown device: {device}")
        
        return True
    def are_all_queues_empty(self) -> bool:
        """Check if all device queues are empty"""
        for queue in self.queues.values():
            if len(queue) > 0:
                return False
        return True
    
    def wait_for_all_queues_empty(self, ignore_rasp_pi: bool = False):
        """Wait until all device queues are empty (unless replanning or interrupted).

        Args:
            ignore_rasp_pi: When True, do not block on the RASP_PI queue.
                This is useful when called from a currently executing RASP_PI
                command (e.g. 'steal'), where the queue head is popped only
                after the handler returns.
        """

        while (
            (not (self.are_all_queues_empty() or (state.ESP_FRONT_IDLE is True and state.ESP_BACK_IDLE is True)))
            or (not state.IDLE)
            or state.WAITING_TIRETTE
            or len(self.queues["STM"]) != 0
            or ((not ignore_rasp_pi) and len(self.queues["RASP_PI"]) != 0)
            or self.end_game is True
        ):
            # Check if shutdown is requested
            self.get_logger().debug(
                f"Waiting for queues to empty... STM queue size: {len(self.queues['STM'])}, "
                f"RASP_PI queue size: {len(self.queues['RASP_PI'])}, ignore_rasp_pi={ignore_rasp_pi}, "
                f"ESP_FRONT_IDLE: {state.ESP_FRONT_IDLE}, ESP_BACK_IDLE: {state.ESP_BACK_IDLE}, "
                f"IDLE: {state.IDLE}, WAITING_TIRETTE: {state.WAITING_TIRETTE}, "
                f"replanning: {state.replanning}"
            )
            if state.SHUTDOWN_REQUESTED:
                self.get_logger().info("Shutdown requested, stopping queue wait")
                return
            
            # If replanning is happening, wait for it to finish
            if self.is_replanning:
                time.sleep(0.5)
                continue
            if not self.are_all_queues_empty():
                time.sleep(0.1)
            elif not state.IDLE:
                time.sleep(0.1)
    
    def display_queue_contents(self):
        """Display the full contents of all queues with parameters"""
        pass

    def handle_steal_command(self, params: Optional[List[Any]] = None, wait: bool = False) -> bool:
        """
        Handle YAML command `RASP_PI -> steal`.

        Behavior:
          - On each received steal command, generate ONE optimal runtime task set
            from TaskScheduler and execute it immediately.
          - Return True only if task generation and queue loading succeeded.
        """
        params = params or []
        self.get_logger().info(f"🏴‍☠️ Handling steal command (params={params})")

        def _enqueue_generated_tasks(generated_tasks: List[Dict], sequence_index: int) -> bool:
            ts = int(time.time() * 1000)
            for idx, generated_task in enumerate(generated_tasks):
                # Ensure unique name to avoid collisions in self.tasks lookup
                base_name = generated_task.get('name', f'steal_generated_{idx+1}')
                generated_task['name'] = f"{base_name}_{ts}_{sequence_index}_{idx+1}"

                # Track last actuator command info FIRST so global-goal lookup below
                # uses the correct command context (pick_up vs put_down).
                if generated_task.get('sequence') and len(generated_task['sequence']) > 0:
                    last_cmd = generated_task['sequence'][-1]
                    last_command = last_cmd.get('command')
                    last_device = last_cmd.get('device')
                    if last_command in ['pick_up', 'put_down'] and last_device in ['ESP_front', 'ESP_back']:
                        self.last_task_command = last_command
                        self.last_task_device = last_device

                explicit_tracking_type = ''
                explicit_tracking_target_id = ''
                tracking_info = generated_task.get('tracking')
                if isinstance(tracking_info, dict):
                    explicit_tracking_type = str(tracking_info.get('type', '')).strip().lower()
                    explicit_tracking_target_id = str(tracking_info.get('target_id', '')).strip().lower()

                    if explicit_tracking_type in ('pick_up', 'put_down') and explicit_tracking_target_id:
                        tracked_task = Task(
                            target_id=explicit_tracking_target_id,
                            type=explicit_tracking_type,
                            options={},
                            score=0.0,
                            task_name=str(generated_task.get('name', '') or ''),
                        )
                        self.task_scheduler.track_task(tracked_task)
                        self.get_logger().info(
                            f"🧭 task_scheduler.track_task (explicit) -> type={explicit_tracking_type}, target_id={explicit_tracking_target_id}"
                        )

                # Refresh global goal for dynamically generated tasks so emergency
                # replanning uses the current task goal instead of stale previous data.
                global_goal = generated_task.get('global_goal')
                if global_goal:
                    self.global_pos_x = global_goal.get('x')
                    self.global_pos_y = global_goal.get('y')
                    self.global_pos_angle = global_goal.get('theta')

                    if self.global_pos_x is not None and self.global_pos_y is not None and self.global_pos_angle is not None:
                        if explicit_tracking_type == 'put_down' and explicit_tracking_target_id:
                            # For generated put_down flows, rely on explicit target id
                            # instead of distance-based dropoff lookup.
                            pass
                        else:
                            self.handle_global_goal(
                                float(self.global_pos_x) / 1000.0,
                                float(self.global_pos_y) / 1000.0,
                                float(self.global_pos_angle)
                            )
                        self.global_pos_x = float(self.global_pos_x) / 1000.0
                        self.global_pos_y = float(self.global_pos_y) / 1000.0
                        self.global_pos_angle = float(self.global_pos_angle)
                        if explicit_tracking_type == 'put_down' and explicit_tracking_target_id:
                            self.get_logger().info(
                                f"🎯 Dynamic task global goal set: x={self.global_pos_x:.3f}, y={self.global_pos_y:.3f}, angle={self.global_pos_angle:.1f}° "
                                f"(explicit_dropoff_id={explicit_tracking_target_id})"
                            )
                        else:
                            self.get_logger().info(
                                f"🎯 Dynamic task global goal set: x={self.global_pos_x:.3f}, y={self.global_pos_y:.3f}, angle={self.global_pos_angle:.1f}°"
                            )

                self.tasks.append(generated_task)
                try:
                    success = self.load_task_to_queues(generated_task['name'], allow_during_emergency=True)
                    if not success:
                        self.get_logger().error(f"❌ Failed loading generated steal task: {generated_task['name']}")
                        return False

                    # Enforce task-by-task ordering for generated flows (especially return-home):
                    # do not enqueue the next generated task until the current one is fully done.
                    self.wait_for_all_queues_empty(ignore_rasp_pi=False)

                    if state.SHUTDOWN_REQUESTED or self.end_game:
                        self.get_logger().warning("⚠️ Stopping generated steal flow (shutdown/end-game)")
                        return False
                finally:
                    if generated_task in self.tasks:
                        self.tasks.remove(generated_task)
            return True

        generated_tasks = None
        generation_attempt = 0

        # First, wait for any ongoing actions to complete fully.
        # This ensures we start from a clean, idle state, mimicking the static task flow.
        self.wait_for_all_queues_empty(ignore_rasp_pi=True)

        while generated_tasks is None:
            if state.SHUTDOWN_REQUESTED or self.end_game or getattr(state, 'return_home_triggered', False):
                self.get_logger().warning("⚠️ Steal generation aborted (shutdown/end-game)")
                return False

            generation_attempt += 1

            start_pose = None
            if self.latest_robot_pose is not None:
                # TaskScheduler expects start_pose in mm/degrees, while /robot_pose_raw is m/rad.
                start_pose = {
                    'x': float(self.latest_robot_pose.x) * 1000.0,
                    'y': float(self.latest_robot_pose.y) * 1000.0,
                    'theta': math.degrees(float(self.latest_robot_pose.z)),
                }

            generated_tasks = self.task_scheduler.get_optimal_task(start_pose=start_pose)

            if generated_tasks is None:
                if generation_attempt == 1 or generation_attempt % 10 == 0:
                    self.get_logger().warning(
                        f"⚠️ No executable steal task yet (attempt {generation_attempt}) - retrying..."
                    )
                time.sleep(0.1)
                continue

        self.get_logger().info(f"✅ Generated {len(generated_tasks)} task(s) for steal command")

        # Wait for all previous commands to finish BEFORE enqueuing the new task.
        # This prevents race conditions where the steal loop tries to run a new
        # task before the previous one (e.g., put_down) has completed.
        self.wait_for_all_queues_empty(ignore_rasp_pi=False)

        contains_return_home_final = any(
            str(task.get('name', '')).startswith('return_home_bezier')
            for task in generated_tasks
        )

        if not _enqueue_generated_tasks(generated_tasks, 1):
            return False

        # keep compatibility with caller's wait flag
        if wait:
            self.wait_for_all_queues_empty(ignore_rasp_pi=False)

        if contains_return_home_final:
            self.end_game = True
            self.get_logger().info(
                "🏁 Final return-home bezier completed; stopping further task execution loop"
            )

        return True
    
    def interrupt_waiting(self):
        """Set flag to interrupt all waiting loops.
        This allows stopping wait_for_all_queues_empty() and other blocking operations
        to handle emergency situations like enemy detection."""
        self.interrupt_wait = True
        self.get_logger().info("🛑 Wait interrupt triggered - will exit waiting loops")
    
    def clear_interrupt(self):
        """Clear the interrupt flag after handling emergency."""
        self.interrupt_wait = False
        self.load_task_to_queues_ended = False
        self.stm32_task_wait_ended = False
        self.get_logger().info("✅ Wait interrupt cleared")

def reset_systems(pneumatic_controller,front_controller,back_controller,stm32_controller=None,strategy=None,immediate=False,xd=True):
    """Reset all systems - can queue or execute immediately"""
    if strategy and not immediate:
        # Add RESET command to RASP_PI queue for background execution
        strategy.queues['RASP_PI'].append({
            'command': 'RESET',
            'params': {},
            'wait': False
        })
    else:
        # Execute reset commands immediately (for shutdown or when strategy not available)
        if stm32_controller:
            if xd:
                stm32_controller.stop(timeout=2.0)
                time.sleep(0.5)
                stm32_controller.reset(timeout=2.0)
        
        # Reset pneumatic and ESP controllers
        pneumatic_controller.front(20)
        pneumatic_controller.back(20)
        front_controller.kabech(0)
        back_controller.kabech(0)
        front_controller.dawer(0)
        back_controller.dawer(0)
        front_controller.n20(0)
        back_controller.n20(20)
        # Allow time for messages to be sent
        time.sleep(0.2)
    
def main(args=None):
    import sys

    def parse_bool(value):
        if value is None:
            return False
        return str(value).strip().lower() in ("1", "true", "yes", "y", "on")

    rclpy.init(args=args)
    
    # Import controllers
    from euro.stm32_controller import STM32Controller
    from euro.lidar import Lidar
    from euro.pathfinder_controller import PathfinderClient
    from euro.pneumatic_controller import PneumaticController
    from euro.front_systems_controller import FrontSystemsController
    from euro.back_systems_controller import BackSystemsController
    from euro.rpi_controller import RPiController
    # Initialize controllers
    
    lidar = Lidar()
    # Create strategy manager
    strategy = StrategyManager()
   
    front= FrontSystemsController()
    back= BackSystemsController()
    pneumatic= PneumaticController()
    rpi= RPiController()
    # Create executor for ROS2 nodes
    executor = MultiThreadedExecutor()
    executor.add_node(lidar)
    executor.add_node(strategy)
    executor.add_node(front)
    executor.add_node(back)
    executor.add_node(pneumatic)
    executor.add_node(rpi)
    executor.add_node(strategy.pathfinder)
    # Reset STM32 and wait for reconnection
    strategy.get_logger().info("Resetting STM32...")
    rpi.reset_stm32()
    strategy.get_logger().info("Resetting pneumatic, ESP back, and ESP front...")
    rpi.reset_all_esps()
    # Wait longer for STM32 to boot and connect to micro-ROS agent
    strategy.get_logger().info("Waiting for STM32 to boot and connect to micro-ROS agent...")
    
    stm32 = STM32Controller('rpi_stm32')
    executor.add_node(stm32)
    
    # Wait for STM32 service to become available
    strategy.get_logger().info("Waiting for STM32 service to become available...")
    if stm32.navi_client.wait_for_service(timeout_sec=20.0):
        strategy.get_logger().info("✅ STM32 service is ready!")
    else:
        strategy.get_logger().error("❌ STM32 service not available after 20 seconds - robot may not respond to commands")
    
     # Start background threads for each device
    strategy.get_logger().info("Starting background threads...")
    # Reset ArUco stability in background thread (after executor starts spinning)
    def reset_aruco_stability():
        time.sleep(0.5)  # Wait for executor to start
        #strategy.pathfinder.reset_stability()
        strategy.pathfinder.clear_map()
    
    reset_thread = threading.Thread(
        target=reset_aruco_stability,
        daemon=True
    )
    reset_thread.start()
    
    status_thread = threading.Thread(
        target=robot_status,
        args=(strategy, stm32, lidar),
        daemon=True
    )
    status_thread.start()
    
    lidar_thread = threading.Thread(
        target=lidar_status,
        args=(strategy, stm32, lidar, strategy.round),
        daemon=True
    )
    lidar_thread.start()
    
    path_planning_thread = threading.Thread(
        target=path_planning_task,
        args=(strategy, lidar, strategy.round, pneumatic, front, back,stm32),
        daemon=True
    )
    path_planning_thread.start()

    task_tracking_thread = threading.Thread(
        target=task_tracking_task,
        args=(strategy, stm32, lidar),
        daemon=True
    )
    task_tracking_thread.start()
    
    stm32_thread = threading.Thread(
        target=stm32_task,
        args=(strategy, stm32,lidar),
        daemon=True
    )
    stm32_thread.start()
    
    front_thread = threading.Thread(
        target=front_task,
        args=(strategy, front,pneumatic),  
        daemon=True
    )
    front_thread.start()
    
    back_thread = threading.Thread(
        target=back_task,
        args=(strategy, back,pneumatic), 
        daemon=True
    )
    back_thread.start()
    
    # Get tasks file path and optional PAMI publish toggle before starting rasp_pi_thread
    # Usage:
    #   ros2 run euro strategy_manager <tasks_file> <publish_pami>
    # Example:
    #   ros2 run euro strategy_manager /path/to/strategy.yaml true
    publish_pami = False

    # Check if a YAML file path was provided as argument
    if len(sys.argv) > 1:
        tasks_file = sys.argv[1]
    else:
        # Default to tasks.yaml in package config directory
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share_dir = get_package_share_directory('euro')
            tasks_file = os.path.join(package_share_dir, 'config', 'tasks.yaml')
        except:
            package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            tasks_file = os.path.join(package_dir, 'config', 'tasks.yaml')

    # Optional second positional argument: publish PAMI strategies or not
    if len(sys.argv) > 2:
        publish_pami = parse_bool(sys.argv[2])

    strategy.get_logger().info(f"PAMI publish mode: {'ENABLED' if publish_pami else 'DISABLED'}")
    
    # Load tasks
    try:
        strategy.tasks, strategy.color, strategy.strategy_name = load_tasks_from_yaml(tasks_file)
        
        # Send team color to hybrid_astar immediately after loading from YAML
        color_to_send = strategy.color if strategy.color else 'yellow'
        success, message = strategy.pathfinder.set_team_color(color_to_send)
        if success:
            strategy.get_logger().info(f"🎨 Team color loaded and sent to hybrid_astar: {message}")
        else:
            strategy.get_logger().warning(f"⚠️ Failed to send team color to hybrid_astar: {message}")
        
        # Load and publish PAMI strategies
        pami_folder = os.path.join(os.path.dirname(tasks_file), '..', 'pami_strategies')
        pami_folder = os.path.abspath(pami_folder)
        
        if publish_pami and os.path.exists(pami_folder):
            from euro.pami_controller import PamiController
            pami_ctrl = PamiController(
                strategy.strategy_name,
                pami_folder,
                'pami_controller_internal',
                team_color=strategy.color,
                target_order=1,
                auto_publish=False,
            )
        elif publish_pami and not os.path.exists(pami_folder):
            strategy.get_logger().warning(f"PAMI folder not found, disabling PAMI publish: {pami_folder}")
            pami_ctrl = None
        else:
            pami_ctrl = None
        
        # Start rasp_pi_thread with pami_ctrl
        rasp_pi_thread = threading.Thread(
            target=rasp_pi_task,
            args=(strategy, strategy.round, rpi, stm32, front, back, pneumatic, pami_ctrl),
            daemon=True
        )
        rasp_pi_thread.start()

        # Start dedicated PAMI thread so publish/start do not block strategy flow
        pami_thread = threading.Thread(
            target=pami_task,
            args=(strategy, pami_ctrl),
            daemon=True
        )
        pami_thread.start()

        # Queue PAMI strategy publish at startup (non-blocking from main flow)
        if pami_ctrl:
            strategy.queues['PAMI'].append({'command': 'publish_strategies'})
            strategy.get_logger().info("📤 Queued PAMI strategy publish at startup")
        
        # Spin executor briefly to initialize publishers before sending reset
        for _ in range(5):
            executor.spin_once(timeout_sec=0.01)

        # Ensure startup reset/clear completes before the first update_map.
        # Without this wait, update_map can run first and then get wiped by the
        # delayed reset thread, which breaks the expected first-update flow.
        if reset_thread.is_alive():
            strategy.get_logger().info("Waiting for reset/clear map startup sequence to finish...")
            reset_thread.join(timeout=5.0)
            if reset_thread.is_alive():
                strategy.get_logger().warning("Startup reset thread did not finish before update_map timeout")
        
        #reset_systems(pneumatic,front,back,None,strategy,True,False)
        #time.sleep(1)
        #strategy.pathfinder.update_map()
        # Execute tasks sequentially
        # Start a thread to execute tasks one by one
        def execute_tasks_sequentially():
            # Execute all static tasks first
            for idx, task in enumerate(strategy.tasks):
                # Check if shutdown was requested
                if state.SHUTDOWN_REQUESTED:
                    strategy.get_logger().info("Shutdown requested, stopping task execution")
                    return
                # Check last command in task sequence
                if task.get('sequence') and len(task['sequence']) > 0:
                    last_cmd = task['sequence'][-1]
                    last_command = last_cmd.get('command')
                    last_device = last_cmd.get('device')
                    
                    if last_command in ['pick_up', 'put_down'] and last_device in ['ESP_front', 'ESP_back']:
                        strategy.get_logger().info(f"📋 Last command in task '{task['name']}': {last_command} on device {last_device}")
                        # Store in strategy attributes
                        strategy.last_task_command = last_command
                        strategy.last_task_device = last_device
                
                # Extract and store global goal
                global_goal = task['global_goal']
                if global_goal:
                    strategy.global_pos_x = global_goal.get('x')
                    strategy.global_pos_y = global_goal.get('y')
                    # Support both 'angle' and 'theta' keys
                    strategy.global_pos_angle = global_goal.get('theta') 
                    strategy.get_logger().info(f"DEBUG: Raw global_goal dict = {global_goal}")
                    strategy.get_logger().info(f"DEBUG: After extraction - x={strategy.global_pos_x}, y={strategy.global_pos_y}, angle={strategy.global_pos_angle}")

                    if strategy.global_pos_x != None and strategy.global_pos_y != None and strategy.global_pos_angle != None :
                        strategy.get_logger().info(f"x= {strategy.global_pos_x} ")
                        strategy.get_logger().info(f"y= {strategy.global_pos_y} ")
                        strategy.get_logger().info(f"angle= {strategy.global_pos_angle} ")
                        strategy.get_logger().info(f"Global goal found for task '{task['name']}'")
                        strategy.handle_global_goal(float(strategy.global_pos_x)/1000, float(strategy.global_pos_y)/1000, float(strategy.global_pos_angle))
                        strategy.get_logger().info("global pose set goal")
                        strategy.global_pos_x =float( strategy.global_pos_x)/1000
                        strategy.global_pos_y =float( strategy.global_pos_y)/1000
                        strategy.global_pos_angle =float( strategy.global_pos_angle)
                        #strategy.pathfinder.set_goal(float(strategy.global_pos_x)/1000, float(strategy.global_pos_y)/1000,float(strategy.global_pos_angle))
                else:
                    strategy.get_logger().warning(f"⚠️  No global goal found for task: {task['name']}")

                # Do not enqueue static YAML tasks while steal loop is active/running.
                # This keeps queue ownership with dynamically generated TaskScheduler tasks.
                while (
                    (state.STEAL_LOOP_ACTIVE or state.STEAL_LOOP_RUNNING)
                    and not state.SHUTDOWN_REQUESTED
                    and not strategy.end_game
                ):
                    # strategy.get_logger().warning(
                    #     f"⏸️ Static task enqueue paused for '{task['name']}' (steal loop active/running)"
                    # )
                    time.sleep(0.05)

                if state.SHUTDOWN_REQUESTED or strategy.end_game:
                    strategy.get_logger().info("Stopping static task execution loop (shutdown/end-game)")
                    return
                
                # Load task into queues
                strategy.get_logger().warning(f"  loading  tasks {task['name']} into queues...")
                loaded = strategy.load_task_to_queues(task['name'], allow_during_emergency=False)
                if not loaded:
                    strategy.get_logger().warning(
                        f"⏭️ Skipping static task '{task['name']}' enqueue (interrupted by emergency/steal)"
                    )
                    # In steal-only emergency mode, do not proceed to following static tasks
                    # after an interrupted enqueue; ownership transfers to generated flow.
                    if (
                        state.replanning
                        or state.STEAL_LOOP_ACTIVE
                        or state.STEAL_LOOP_RUNNING
                        or strategy.remove_esp_waiting
                        or str(getattr(state, 'last_stm_command', '') or '') == 'emergency'
                    ):
                        strategy.get_logger().warning(
                            "🛑 Static task execution halted after interruption; waiting for emergency/steal recovery"
                        )
                        return
                    continue
                
                # Wait for all commands to complete
                strategy.wait_for_all_queues_empty()
        # Start task execution in a background thread
        task_executor_thread = threading.Thread(
            target=execute_tasks_sequentially,
            daemon=True
        )
        task_executor_thread.start()
        
    except Exception as e:
        strategy.get_logger().error(f"Failed: {str(e)}")
        return
    
   
    
    # Keep node alive with 'q' key shutdown
    import sys
    import select
    import termios
    import tty
    
    # Check if stdin is a TTY before configuring terminal settings
    is_tty = sys.stdin.isatty()
    old_settings = None
    
    if is_tty:
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        # Set terminal to raw mode to read single characters
        tty.setcbreak(sys.stdin.fileno())
    
    try:
        shutdown_requested = False
        while rclpy.ok() and not shutdown_requested:
            # Spin once to process callbacks
            executor.spin_once(timeout_sec=0.1)
            # Check if 'q' was pressed
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'q':
                    shutdown_requested = True
                    
                    # Stop STM32 and reset all systems immediately (not queued)
                    try:
                        strategy.get_logger().info("Stopping and resetting all systems...")
                        reset_systems(pneumatic,front,back,stm32,strategy,immediate=True)
                        time.sleep(0.5)
                    except Exception as e:
                        strategy.get_logger().error(f"Failed to stop/reset systems: {e}")
                    
                
                    
                    break
    
    except KeyboardInterrupt:
        pass
    finally:
        # Signal threads to stop first
        strategy.get_logger().info("Signaling threads to stop...")
        state.SHUTDOWN_REQUESTED = True
        
        # Give threads time to react to shutdown flag
        time.sleep(0.3)
        
        # Wait for threads to finish (with longer timeout)
        threads = [
            ('Status Monitor', status_thread),
            ('Task Tracking', task_tracking_thread),
            ('STM32 Task', stm32_thread),
            ('Front Task', front_thread),
            ('Back Task', back_thread),
            ('RaspPi Task', rasp_pi_thread),
            ('PAMI Task', pami_thread),
            ('Task Executor', task_executor_thread)
        ]
        
        for name, thread in threads:
            if thread.is_alive():
                strategy.get_logger().info(f"Waiting for {name} thread to stop...")
                thread.join(timeout=5.0)
                if thread.is_alive():
                    strategy.get_logger().warning(f"Thread {name} did not stop in time (still blocked)")
                else:
                    strategy.get_logger().info(f"✓ {name} thread stopped")
        
        # Restore terminal settings if they were changed
        if old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except:
                pass  # Terminal may already be closed
        
        # Cleanup nodes
        try:
            # Clean up GPIO resources after threads are stopped
            rpi.cleanup()
            
            # Destroy all nodes
            strategy.destroy_node()
            stm32.destroy_node()
            lidar.destroy_node()
            front.destroy_node()
            back.destroy_node()
            pneumatic.destroy_node()
            rpi.destroy_node()
            executor.shutdown()
        except Exception as e:
            strategy.get_logger().warning(f"Cleanup warning: {e}")
        
        # Only shutdown if context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()