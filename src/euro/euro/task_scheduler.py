from simple_map import Map, Pose2D, Speed2D, EnemyRelativeObstacle, BatchInfo
from typing import List, Dict, Optional
from dataclasses import dataclass


@dataclass
class Task:
    """Represents a task for the robot to perform."""
    target_id: str
    type: str
    options: Dict[str, str] = {}
    score: float = 0.0

class TaskScheduler:
    """Simple task scheduler that generates tasks based on the current map state."""
    must_dropoff_time = 12.0
    must_go_home_time = 7.0 # time at 1m distance from nest
    def __init__(self, map: Map):
        self.map = map
        self.task_queue: List[Task] = []
        self.tracked_task: Task = None
        self.tracked_task_invalid = False
        self.predicted_enemy_goal : str = "unknown"

    def pop_task(self) -> Optional[Task]:
        """Pop the next task from the queue."""
        if self.task_queue:
            task = self.task_queue.pop(0)
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
    
    def check_side(self, task: Task) -> None:
        task.options["side"] = "back"
        pass
    
    def predict_enemy_goal(self) -> None:
        """Predict the goal position of the enemy based on their current speed and position."""
        pass

    def generate_tasks(self) -> None:
        """Generate tasks based on the current map state."""
        #TODO: Implement task generation logic based on the current map state, such as creating pickup tasks for detected batches and dropoff tasks for known dropoff locations.
        queue = []
        if self.map.robot_front_empty or self.map.robot_back_empty:
            for batch_id, batch in self.map.batches.items():
                if batch.color != "unknown":
                    task = Task(target_id=batch_id, type="pickup")
                    task.score = self.calculate_score(task)
                    if batch_id in ["s3", "s5", "f3", "f5", "f8"]:
                        self.check_side(task)
                    queue.append(task)
        if not self.map.robot_front_empty or not self.map.robot_back_empty:
            for dropoff_id, dropoff in self.map.drop_offs.items():
                if dropoff["status"] == "empty":
                    task = Task(target_id=dropoff_id, type="dropoff")
                    task.score = self.calculate_score(task)
                    if dropoff_id in ["f3", "f5", "f8"]:
                        self.check_side(task)
                    queue.append(task)
        # sorting tasks
        queue.sort(key=lambda t: t.score, reverse=True)
        if self.map.time_left < self.must_dropoff_time:
            # If time is running out, prioritize dropoff tasks
            queue = [t for t in queue if t.type == "dropoff"] + [t for t in queue if t.type != "dropoff"]
        if (self.map.time_left < max(self.must_go_home_time * self.map.distance_from(self.map.robot_position, self.map.nest_position), 5)) and len([t for t in queue if t.type == "dropoff"]) == 0:
            task = Task(target_id="nest", type="return_home")
            task.options["drop"] = "none"
            if not self.map.robot_front_empty:
                task.options["drop"] = "front"
            elif not self.map.robot_back_empty:
                task.options["drop"] = "back"
            queue.insert(0, task)  # Prioritize return home task
            
        self.task_queue = queue
    
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
