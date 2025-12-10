#!/usr/bin/env python3

import yaml
import os
from typing import List, Dict, Any

def load_tasks_from_yaml(yaml_file: str) -> List[Dict[str, Any]]:
    """
    Load tasks from YAML file into a simple list
    
    Args:
        yaml_file: Path to YAML file containing tasks
        
    Returns:
        List of tasks, each task is a dictionary with name, description, dependencies, and sequence
    """
    if not os.path.exists(yaml_file):
        raise FileNotFoundError(f"YAML file not found: {yaml_file}")
    
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    
    if 'tasks' not in data:
        raise ValueError("No 'tasks' key found in YAML file")
    
    # Convert tasks dict to list of tasks
    tasks = []
    for task_name, task_data in data['tasks'].items():
        task = {
            'name': task_name,
            'description': task_data.get('description', ''),
            'dependencies': task_data.get('dependencies', []),
            'sequence': task_data.get('sequence', [])
        }
        tasks.append(task)
    
    return tasks
