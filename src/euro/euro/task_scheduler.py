import math
from simple_map import Map, Pose2D, Speed2D, EnemyRelativeObstacle, BatchInfo
from typing import Any, List, Dict, Optional
from dataclasses import dataclass


@dataclass
class Task:
    """Represents a task for the robot to perform."""
    target_id: str
    type: str
    options: Dict[str, str] = None
    score: float = 0.0

class TaskScheduler:
    """Simple task scheduler that generates tasks based on the current map state."""
    
    def __init__(self, map: Map, pathfinder_client=None):
        self.map = map
        self.pathfinder = pathfinder_client
        self.pickup_queue: List[Task] = []
        self.dropoff_queue: List[Task] = []
        self.return_home_flag = False
        self.tracked_task: Task = None
        self.tracked_task_invalid = False

    def pop_pickup_task(self) -> Optional[Task]:
        """Pop the next pickup task from the queue."""
        if self.pickup_queue:
            task = self.pickup_queue.pop(0)
            self.track_task(task)
            return task
        return None
    
    def pop_dropoff_task(self) -> Optional[Task]:
        """Pop the next dropoff task from the queue."""
        if self.dropoff_queue:
            task = self.dropoff_queue.pop(0)
            self.track_task(task)
            return task
        return None
    
    def check_return_home_flag(self) -> bool:
        """Check if the return home flag is set."""
        return self.return_home_flag
    
    def track_task(self, task: Task) -> None:
        """Track the specified task."""
        self.tracked_task = task
        self.tracked_task_invalid = False

    def untrack_task(self) -> None:
        """Untrack the currently tracked task."""
        self.tracked_task = None
    
    def calculate_score(self, task: Task) -> float:
        # TODO: Implement a more sophisticated scoring mechanism based on the current map state, task type, and other heuristics.
        """Calculate a score for the given task based on the current map state."""
        # Placeholder scoring logic - can be expanded with more complex heuristics
        if task.type == "pickup":
            batch = self.map.batches.get(task.target_id)
            if batch:
                distance = ((self.map.robot_position.x - batch.x) ** 2 + (self.map.robot_position.y - batch.y) ** 2) ** 0.5
                return 1 / (distance + 1e-5)  # Higher score for closer batches
        elif task.type == "dropoff":
            dropoff = self.map.drop_offs.get(task.target_id)
            if dropoff:
                distance = ((self.map.robot_position.x - dropoff["x"]) ** 2 + (self.map.robot_position.y - dropoff["y"]) ** 2) ** 0.5
                return 1 / (distance + 1e-5)  # Higher score for closer dropoffs
        return 0.0
    
    def generate_tasks(self) -> None:
        """Generate tasks based on the current map state."""
        #TODO: Implement task generation logic based on the current map state, such as creating pickup tasks for detected batches and dropoff tasks for known dropoff locations.
        pass
    
    def check_tracked_task_validity(self) -> Optional[str]:
        """Check the validity of the currently tracked task."""
        #TODO: Implement logic to check if the currently tracked task is still valid based on the current map state. Return a string indicating the reason for invalidity if it is no longer valid, or None if it is still valid.
        self.tracked_task_invalid = False

    def cancel_tracked_task(self) -> bool:
        return self.tracked_task_invalid
    
    def build_pickup_task(
        self,
        batch_known_id: str,
        pickup_face: str,
        pose_selector: int,
        start_pose: Optional[Dict[str, float]] = None,
        approach_offset_mm: float = 100.0,
    ) -> Optional[Dict[str, Any]]:
        """
        Build a pickup task for a specific batch with explicit face and pose selection.
        Includes path planning from current or provided start pose.
        
        Args:
            batch_known_id: Batch ID key from pickup_poses (e.g., 'S1', 'S3')
            pickup_face: 'front' or 'back' for ESP device selection
            pose_selector: 0 for lowest X pickup pose, 1 for highest X pickup pose
            start_pose: Optional start pose dict {'x', 'y', 'theta'}
                        x/y in mm, theta in degrees
            approach_offset_mm: Distance before pickup pose for path target (default 100mm)
        
        Returns:
            Task dict compatible with strategy_manager queue format, or None if invalid inputs
        """
        # Pathfinder is required for path-based task generation
        if self.pathfinder is None:
            return None

        normalized_id = batch_known_id.upper()

        # Validate batch exists in pickup_poses
        if normalized_id not in self.map.pickup_poses:
            return None
        
        # Validate pickup_face
        if pickup_face not in ['front', 'back']:
            return None
        
        # Get poses for this batch
        poses = self.map.pickup_poses[normalized_id]
        if not poses:
            return None
        
        # Validate pose_selector
        if pose_selector not in [0, 1]:
            return None
        
        # If only one pose, use it regardless of selector
        if len(poses) == 1:
            selected_pose = poses[0]
        else:
            # Sort by x coordinate to determine lowest (0) vs highest (1)
            sorted_poses = sorted(poses, key=lambda p: p['x'])
            if pose_selector >= len(sorted_poses):
                return None
            selected_pose = sorted_poses[pose_selector]
        
        # Extract position and theta from selected pose
        pickup_x = selected_pose['x']  # in mm
        pickup_y = selected_pose['y']  # in mm
        pickup_thetas = selected_pose['theta']  # list of angles in degrees
        
        # Choose first theta from the list (or single value)
        if isinstance(pickup_thetas, list) and len(pickup_thetas) > 0:
            pickup_theta = pickup_thetas[0]
        else:
            pickup_theta = pickup_thetas if isinstance(pickup_thetas, (int, float)) else 0
        
        # Determine ESP device and STM command type
        esp_device = 'ESP_front' if pickup_face == 'front' else 'ESP_back'
        stm_command = 'go_to' if pickup_face == 'front' else 'go_to_backwards'

        # Pre-approach point: plan to a point before final pickup pose, then do final precise move.
        # Uses pickup orientation to compute a point behind the pickup pose.
        offset = max(0.0, float(approach_offset_mm))
        theta_rad = math.radians(float(pickup_theta))
        approach_x = pickup_x - (offset * math.cos(theta_rad))
        approach_y = pickup_y - (offset * math.sin(theta_rad))

        # Plan path to pre-approach target
        self.pathfinder.set_goal(
            approach_x / 1000.0,
            approach_y / 1000.0,
            theta_rad
        )

        if start_pose is not None:
            if not all(k in start_pose for k in ('x', 'y', 'theta')):
                return None
            stm32_commands, path_success = self.pathfinder.path(
                start_x=float(start_pose['x']) / 1000.0,
                start_y=float(start_pose['y']) / 1000.0,
                start_theta=math.radians(float(start_pose['theta']))
            )
        else:
            stm32_commands, path_success = self.pathfinder.path()

        if not path_success:
            return None
        
        # Build task sequence compatible with strategy_executor
        task_sequence: List[Dict[str, Any]] = []

        # Add waypoints from pathfinder
        last_cmd = None
        for cmd in stm32_commands:
            values = cmd.replace('go_to(', '').replace(')', '').split(',')
            x = float(values[0].strip())
            y = float(values[1].strip())
            if cmd != last_cmd:
                task_sequence.append({
                    'device': 'STM',
                    'command': stm_command,
                    'params': [x, y, 1100.0,0.0, {'from_pathfinding': True, 'from_pickup_task': True}],
                    'wait': False
                })
            last_cmd = cmd
        task_sequence.append({
            'device': 'STM',
            'command': stm_command,
            'params': [pickup_x, pickup_y, 1100.0, 1.0, {'from_pathfinding': True, 'from_pickup_task': True}],
            'wait': False
        })
        task_sequence.append({
            'device': esp_device,
            'command': 'pick_up',
            'params': {},
            'wait': True
        })
        
        # Build and return task dict
        task = {
            'name': f'pickup_{normalized_id}_{pickup_face}',
            'sequence': task_sequence,
            'global_goal': {
                'x': pickup_x,
                'y': pickup_y,
                'theta': pickup_theta
            }
        }
        return task
    
    def loop(self) -> None:
        """Main loop to update tasks based on the current map state."""
        self.generate_tasks()
        self.check_tracked_task_validity()
