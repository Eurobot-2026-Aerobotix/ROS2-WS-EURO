from simple_map import Map, Pose2D, Speed2D, EnemyRelativeObstacle, BatchInfo
from typing import List, Dict, Optional
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
    
    def __init__(self, map: Map):
        self.map = map
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
    
    def loop(self) -> None:
        """Main loop to update tasks based on the current map state."""
        self.generate_tasks()
        self.check_tracked_task_validity()
