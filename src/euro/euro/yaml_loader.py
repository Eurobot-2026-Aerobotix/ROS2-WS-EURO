#!/usr/bin/env python3

import yaml
import os
from typing import List, Dict, Any

def load_tasks_from_yaml(yaml_file: str) -> tuple[List[Dict[str, Any]], str]:
    """
    Load tasks from YAML file into a simple list
    
    Args:
        yaml_file: Path to YAML file containing tasks
        
    Returns:
        Tuple of (list of tasks, color string). Each task is a dictionary with name, description, dependencies, and sequence
    """
    if not os.path.exists(yaml_file):
        raise FileNotFoundError(f"YAML file not found: {yaml_file}")
    
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    
    if 'tasks' not in data:
        raise ValueError("No 'tasks' key found in YAML file")
    
    # Extract color field (defaults to 'yellow' if not specified)
    color = data.get('color', 'yellow')
    
    # Check if tasks is already a list (new format) or dict (old format)
    tasks_data = data['tasks']
    
    tasks = []
    
    if isinstance(tasks_data, list):
        # New format: tasks is a list
        for idx, task_data in enumerate(tasks_data, 1):
            task = {
                'name': f"Task_{idx}",  # Generate name if not provided
                'description': task_data.get('description', ''),
                'global_goal': task_data.get('global_goal', {}),
                'sequence': task_data.get('sequence', [])
            }
            tasks.append(task)
    else:
        # Old format: tasks is a dict
        for task_name, task_data in tasks_data.items():
            task = {
                'name': task_name,
                'description': task_data.get('description', ''),
                'global_goal': task_data.get('global_goal', {}),
                'sequence': task_data.get('sequence', [])
            }
            tasks.append(task)
    
    return tasks, color


def _convert_params_to_dict(device: str, command: str, params: list) -> dict:
    """
    Convert params list to dict based on device and command type
    
    Args:
        device: Device name (STM, ESP_front, etc.)
        command: Command name
        params: List of parameters
        
    Returns:
        Dictionary of parameters
    """
    # Handle STM32/STM commands
    if device.upper() in ['STM32', 'STM']:
        if command == 'set_position' and len(params) >= 3:
            return {
                'x': float(params[0]),
                'y': float(params[1]),
                'angle': float(params[2])
            }
        elif command == 'go_to' and len(params) >= 3:
            return {
                'x': float(params[0]),
                'y': float(params[1]),
                'speed': float(params[2])
            }
        elif command == 'go_to_backwards' and len(params) >= 3:
            return {
                'x': float(params[0]),
                'y': float(params[1]),
                'speed': float(params[2])
            }
        elif command == 'bezier' and len(params) >= 7:
            return {
                'x1': float(params[0]),
                'y1': float(params[1]),
                'x2': float(params[2]),
                'y2': float(params[3]),
                'x3': float(params[4]),
                'y3': float(params[5]),
                'speed': float(params[6])
            }
        elif command == 'orientate' and len(params) >= 1:
            return {
                'angle': float(params[0]),
                'is_radian': 0.0 if len(params) < 2 else float(params[1])
            }
        elif command == 'move_forward' and len(params) >= 2:
            return {
                'distance': float(params[0]),
                'speed': float(params[1]),
                'is_smooth': 0.0 if len(params) < 3 else float(params[2])
            }
        elif command == 'rotate' and len(params) >= 1:
            return {
                'angle': float(params[0]),
                'is_radian': 0.0 if len(params) < 2 else float(params[1]),
                'is_smooth': 0.0 if len(params) < 3 else float(params[2])
            }
    
    # Handle ESP commands (typically no params or simple params)
    if 'ESP' in device.upper():
        if len(params) == 0:
            return {}
    
    # Default: create generic numbered params
    return {f'param_{i}': param for i, param in enumerate(params)}
