#!/usr/bin/env python3
"""
Strategy Executor - Device task executors for the strategy manager
Handles periodic execution of commands from device queues
"""

import rclpy
import time
import threading
import math
from geometry_msgs.msg import Point

class Round:
    """Timer class to track game round time"""
    
    def __init__(self):
        self._start_time = None
    
    def start_game(self):
        """Start the game timer"""
        self._start_time = time.time()

    def has_started(self):
        """Return whether the game timer has started."""
        return self._start_time is not None
    
    def time(self):
        """Return elapsed seconds since start_game() was called"""
        if self._start_time is None:
            return 0.0
        return time.time() - self._start_time

# Global state variables
IDLE = True
IDLE_FOR_SYSTEMS=True
PREVIOUS_IDLE_FOR_SYSTEMS=True
SYSTEMS_IDLE_COUNT=0
EMERGENCY_STOP = False
ESP_FRONT_IDLE=True
ESP_BACK_IDLE=True
color_pt_front=None
color_pt_back=None
color_pt=None
SHUTDOWN_REQUESTED = False  # Global shutdown flag
WAITING_TIRETTE=False
replanning=False
last_stm_command=""
return_home_triggered = False  # Flag to ensure return home logic only triggers once
known_id_front=None
known_id_back=None
STEAL_LOOP_ACTIVE = False
STEAL_LOOP_RUNNING = False
STEAL_REPEAT_COUNT = 30

def path_planning_task(strategy, lidar, round_timer, pneumatic_controller=None, front_controller=None, back_controller=None,stm32_controller=None):
    """Dedicated thread for handling return home at 90s"""
    global SHUTDOWN_REQUESTED
    global last_stm_command
    global IDLE
    global IDLE_FOR_SYSTEMS
    global SYSTEMS_IDLE_COUNT
    global replanning
    global return_home_triggered
    global STEAL_LOOP_ACTIVE
    global STEAL_LOOP_RUNNING
    global STEAL_REPEAT_COUNT
    strategy.get_logger().info("🏠 Return home monitor thread started")
    return_home_triggered = False
    reset_stm_triggered=False
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Check if only 10 seconds remain (90s elapsed out of 100s)
        if round_timer.time()>=100.0 and not reset_stm_triggered:
            strategy.get_logger().info("⏰ Game time reached 100s -    resetting stm")
            stm32_controller.reset()
            reset_stm_triggered=True
            strategy.end_game = True
        if round_timer.time() >= 190.0 and round_timer.time() <= 100.0 and not return_home_triggered:
            return_home_triggered = True
            strategy.end_game = True
            strategy.get_logger().warning("⏰ Only 10 seconds left! Returning home to (250, 225)...")
            
            # Reset systems without stopping STM32 (like at start)
            if pneumatic_controller and front_controller and back_controller:
                strategy.get_logger().info("🔄 Resetting pneumatic and ESP systems for return home...")
                pneumatic_controller.front(16)
                pneumatic_controller.back(16)
                front_controller.kabech(0)
                back_controller.kabech(0)
                front_controller.dawer(0)
                back_controller.dawer(0)
                front_controller.n20(0)
                back_controller.n20(0)
                time.sleep(0.2)
            
            # Clear all device queues
            strategy.queues['ESP_front'].clear()
            strategy.queues['ESP_back'].clear()
            strategy.queues['RASP_PI'].clear()
            strategy.queues['STM'].clear()
            strategy.get_logger().info("🗑️  Cleared all task queues for return home")
            
            # Set goal to home position (250mm, 225mm)
            strategy.pathfinder.set_goal(0.25, 0.225, 0.0)  # Convert mm to meters
            strategy.get_logger().info("🏠 Home goal set: (250mm, 225mm)")
            
            # Generate return home path with retry loop
            path_found = False
            attempt_count = 0
            
            while not path_found and not SHUTDOWN_REQUESTED and (round_timer.time() < 100.0):
                attempt_count += 1
                strategy.get_logger().info(f"🏠 Return home path attempt #{attempt_count}")
                
                # Re-set home goal before each attempt
                strategy.pathfinder.set_goal(0.25, 0.225, 0.0)
                path_found = strategy.path(end_game=True)
                
                if path_found:
                    strategy.get_logger().info(f"✅ Return home path found on attempt #{attempt_count}")
                    # Set flags for execution
                    last_stm_command = "replanned"
                    SYSTEMS_IDLE_COUNT = 0
                    IDLE = True
                    IDLE_FOR_SYSTEMS = True
                    lidar.clear_on = True
                    lidar.clear_goto_target()
                    lidar.generate_path = 0
                    replanning = False
                else:
                    strategy.get_logger().warning(f"⚠️ Return home path attempt #{attempt_count} failed - retrying...")
                    time.sleep(0.5)

        if STEAL_LOOP_ACTIVE and not STEAL_LOOP_RUNNING:
            STEAL_LOOP_RUNNING = True
            try:
                repeat_count = max(1, int(STEAL_REPEAT_COUNT))
            except (TypeError, ValueError):
                repeat_count = 1

            try:
                strategy.get_logger().info(
                    f"🏴‍☠️ Path-planning steal loop started ({repeat_count} time(s))"
                )
                for attempt in range(1, repeat_count + 1):
                    if not rclpy.ok() or SHUTDOWN_REQUESTED or strategy.end_game or return_home_triggered or not STEAL_LOOP_ACTIVE:
                        strategy.get_logger().warning(
                            "⚠️ Stopping path-planning steal loop due to shutdown/end-game"
                        )
                        break

                    # Gate steal attempts until robot is idle and either:
                    # - emergency is cleared, or
                    # - lidar requested a replan (generate_path=1).
                    # This avoids deadlocks when emergency stays latched for the
                    # previous blocked target.
                    wait_gate_count = 0
                    while rclpy.ok() and not SHUTDOWN_REQUESTED and not strategy.end_game and STEAL_LOOP_ACTIVE:
                        path_replan_requested = bool(getattr(lidar, 'generate_path', 0) == 1)
                        ready_for_steal = bool(IDLE) and (not EMERGENCY_STOP or path_replan_requested)
                        if ready_for_steal:
                            break

                        wait_gate_count += 1
                        if wait_gate_count == 1 or wait_gate_count % 20 == 0:
                            strategy.get_logger().info(
                                f"⏳ Steal loop waiting (EMERGENCY_STOP={int(bool(EMERGENCY_STOP))}, IDLE={IDLE}, generate_path={int(path_replan_requested)})"
                            )
                        time.sleep(0.05)

                    # If lidar explicitly asks for replanning while emergency is still
                    # active, clear stale target so emergency can be reevaluated against
                    # the new generated steal path.
                    if bool(IDLE) and EMERGENCY_STOP and bool(getattr(lidar, 'generate_path', 0) == 1):
                        strategy.get_logger().warning(
                            "🧭 Steal loop proceeding on generate_path=1 while emergency active; clearing stale go-to target"
                        )
                        lidar.clear_goto_target()
                        lidar.generate_path = 0

                    if not rclpy.ok() or SHUTDOWN_REQUESTED or strategy.end_game or return_home_triggered or not STEAL_LOOP_ACTIVE:
                        strategy.get_logger().warning(
                            "⚠️ Stopping path-planning steal loop before attempt due to shutdown/end-game"
                        )
                        break

                    # Ensure emergency-replanning flag does not block generated-flow waits.
                    replanning = False

                    strategy.get_logger().info(
                        f"🏴‍☠️ Path-planning steal attempt {attempt}/{repeat_count}"
                    )
                    success = strategy.handle_steal_command(params=[repeat_count, 0], wait=True)
                    if not success:
                        strategy.get_logger().warning(
                            f"⚠️ Path-planning steal attempt {attempt}/{repeat_count} failed"
                        )
                        break

                    if strategy.end_game:
                        strategy.get_logger().info(
                            f"🏁 End-game reached during path-planning steal loop at attempt {attempt}/{repeat_count}"
                        )
                        break

                strategy.get_logger().info(
                    f"✅ Path-planning steal loop finished ({repeat_count} time(s))"
                )
            finally:
                STEAL_LOOP_ACTIVE = False
                STEAL_LOOP_RUNNING = False
                STEAL_REPEAT_COUNT = 30
        
        time.sleep(0.05)  # 50ms for quicker IDLE updates

def prepare_path(strategy, lidar):
    """Deprecated: prepared-path flow disabled. Emergency handling is steal-only."""
    strategy.get_logger().info("ℹ️ prepare_path() skipped (steal-only mode enabled)")
    return False

def execute_prepared_path(strategy, lidar):
    """Deprecated: prepared-path flow disabled. Emergency handling is steal-only."""
    strategy.get_logger().info("ℹ️ execute_prepared_path() skipped (steal-only mode enabled)")
    return False

def robot_status(strategy, stm32, lidar):
    """Runs every 100ms to check STM32 movement status and lidar obstacles"""
    global IDLE
    global EMERGENCY_STOP
    global SHUTDOWN_REQUESTED
    first_emergency = True
    global IDLE_FOR_SYSTEMS
    global PREVIOUS_IDLE_FOR_SYSTEMS
    global SYSTEMS_IDLE_COUNT
    global last_stm_command
    
    strategy.get_logger().info("🤖 Robot status monitoring started")
    status_loop_count = 0
    stopped_by_failsafe=False
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        status_loop_count += 1
        
        # Only check if robot is idle when we think it's busy
        if not IDLE and not stopped_by_failsafe :
            if not IDLE_FOR_SYSTEMS and last_stm_command != "emergency":
                success, is_completed, _ = stm32.IDLE_FOR_SYSTEMS(1.0)
                #strategy.get_logger().info(f"idle for systems  : {is_completed}")
            else:
                success, is_completed, _ = stm32.IDLE(1.0)
                #strategy.get_logger().info(f"idle : {is_completed}")
            if is_completed and last_stm_command == "navigation":
                _ ,stopped_by_failsafe=stm32.fail_safe(1.0)    
                strategy.get_logger().info(f"failsafe result : {stopped_by_failsafe}")
            if not success:
                strategy.get_logger().warning("⚠️ Failed to check robot status")
            elif stopped_by_failsafe and last_stm_command == "navigation":
                strategy.get_logger().info("✅ Robot stopped by failsafe not updating idle to be stopped ")
            else:
                if not IDLE_FOR_SYSTEMS and not EMERGENCY_STOP:
                    PREVIOUS_IDLE_FOR_SYSTEMS=IDLE_FOR_SYSTEMS
                    IDLE_FOR_SYSTEMS = is_completed
                    
                    # Log state changes for IDLE_FOR_SYSTEMS
                    if IDLE_FOR_SYSTEMS != PREVIOUS_IDLE_FOR_SYSTEMS:
                        strategy.get_logger().info(f"🔄 IDLE_FOR_SYSTEMS changed: {PREVIOUS_IDLE_FOR_SYSTEMS} → {IDLE_FOR_SYSTEMS}")
                else:
                    old_idle = IDLE
                    IDLE = is_completed
                    
                    # Log state changes for IDLE
                    if IDLE != old_idle:
                        strategy.get_logger().info(f"🔄 IDLE changed: {old_idle} → {IDLE} (last_stm_command={last_stm_command})")   
            if  IDLE_FOR_SYSTEMS ==True and PREVIOUS_IDLE_FOR_SYSTEMS==False and last_stm_command!="emergency" :
                SYSTEMS_IDLE_COUNT+=1
            PREVIOUS_IDLE_FOR_SYSTEMS=IDLE_FOR_SYSTEMS
        #else:
            # Log periodically when robot is idle to confirm the thread is running
            # if status_loop_count % 100 == 0:  # Every 10 seconds
            #     strategy.get_logger().info(f"✓ Robot status monitor: IDLE={IDLE}, EMERGENCY_STOP={EMERGENCY_STOP}")        
        time.sleep(0.02)  # 20ms to reduce command dispatch latency

def lidar_status(strategy, stm32, lidar, round_timer):
    """Runs every 100ms to check lidar status and handle emergency stops"""
    global EMERGENCY_STOP
    global SHUTDOWN_REQUESTED
    first_emergency = True
    
    strategy.get_logger().info("🔍 Lidar status monitoring started")
    lidar_loop_count = 0
    
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        lidar_loop_count += 1
        
        # Update lidar with current round time
        elapsed_s = round_timer.time()
        lidar.set_round_time(elapsed_s)
        
        # Check emergency stop from lidar
        old_emergency = EMERGENCY_STOP
        EMERGENCY_STOP = lidar.getstatus()
        
        # Log when emergency state changes
        if EMERGENCY_STOP != old_emergency:
            if EMERGENCY_STOP:
                strategy.get_logger().warning(f"🚨 EMERGENCY_STOP activated by lidar! (first_emergency={first_emergency})")
            else:
                strategy.get_logger().info(f"✅ EMERGENCY_STOP cleared by lidar (first_emergency={first_emergency})")
        
        # Handle emergency stop
        if EMERGENCY_STOP and first_emergency:
            strategy.get_logger().warning("🚨 Emergency stop activated!")
            #success, response = stm32.stop()
            first_emergency = False
        
        # Reset flag when emergency is cleared
        if not EMERGENCY_STOP:
            if not first_emergency:
                strategy.get_logger().info("🔄 Emergency flag reset - ready for next emergency")
            first_emergency = True
        
        # Periodic status check
        #if lidar_loop_count % 100 == 0:  # Every 10 seconds
            #strategy.get_logger().info(f"✓ Lidar monitor: EMERGENCY_STOP={EMERGENCY_STOP}, round_time={round_timer.time():.1f}s")
        
        time.sleep(0.01)  # 10ms

def to_float_list(params):
    """Convert all params to float"""
    try:
        return [float(p) for p in params]
    except (TypeError, ValueError) as e:
        raise ValueError(f"Invalid STM32 params: {params}") from e

def handle_stm32_command(command, params, stm32, lidar, strategy):
    """Execute STM32 command based on type
    
    Args:
        command: Command name string
        params: List of command parameters
        stm32: STM32 controller instance
        lidar: Lidar instance
        strategy: Strategy manager instance
    """
    global IDLE
    global IDLE_FOR_SYSTEMS
    
    # Check if this is a path-generated command (marked in params)
    from_pathfinding = False
    if isinstance(params, list) and len(params) > 0 and isinstance(params[-1], dict):
        # Last param is metadata dict
        metadata = params.pop()
        from_pathfinding = metadata.get('from_pathfinding', False)
    
    if command == 'move_forward':
        stm32.move_forward(params[0], params[1], params[2] if len(params) > 2 else 0.0)
        IDLE = False
        IDLE_FOR_SYSTEMS = False
    elif command == 'rotate':
        stm32.rotate(params[0], params[1] if len(params) > 1 else 0.0)
        IDLE = False
        IDLE_FOR_SYSTEMS = False
    elif command == 'go_to':
        lidar.set_goto_target(params[0]/1000, params[1]/1000, from_pathfinding=from_pathfinding)  # Convert mm to meters for lidar
        IDLE_FOR_SYSTEMS = False
        
        if len(params) > 3:
            stm32.go_to(params[0], params[1], params[2], params[3])
        else:
            stm32.go_to(params[0], params[1], params[2])
        IDLE = False
        
    elif command == 'orientate':
        stm32.orientate(params[0], params[1] if len(params) > 1 else 0.0)
        IDLE = False
        IDLE_FOR_SYSTEMS = False
    elif command == 'bezier':
        if len(params) > 9:
            stm32.bezier_curve(params[0], params[1], params[2], params[3], params[4], params[5], params[8], params[9])
        else:
            stm32.bezier_curve(params[0], params[1], params[2], params[3], params[4], params[5], params[8])
        IDLE = False
        IDLE_FOR_SYSTEMS = False
    
    elif command == 'go_to_backwards':
        # Publish target position for collision detection
        lidar.set_goto_target(params[0]/1000, params[1]/1000, from_pathfinding=from_pathfinding)  # Convert mm to meters for lidar
        
        if len(params) > 3:
            stm32.go_to_backwards(params[0], params[1], params[2], params[3])
        else:
            stm32.go_to_backwards(params[0], params[1], params[2])
        IDLE = False
        IDLE_FOR_SYSTEMS = False
    elif command == 'bezier_backwards':
        if len(params) > 9:
            stm32.bezier_curve_backwards(params[0], params[1], params[2], params[3], params[4], params[5], params[8], params[9])
        else:
            stm32.bezier_curve_backwards(params[0], params[1], params[2], params[3], params[4], params[5], params[8])
        IDLE = False
        IDLE_FOR_SYSTEMS = False
    elif command == 'set_position':
        stm32.set_position(params[0], params[1], params[2])
    else:
        strategy.get_logger().warn(f"Unknown STM32 command: {command}")
    strategy.command_waiting = False

def correct_bezier(strategy, stm32, params):
    """Correct robot position using beacon after bezier curve execution
    
    Args:
        strategy: Strategy manager instance with beacon data
        stm32: STM32 controller instance
        params: Command parameters with fallback positions at indices 6 and 7
    """
    global SHUTDOWN_REQUESTED
    
    # Wait for at least 2 position updates from beacon
    available_aruco=False 
    while strategy.latest_robot_pose_beacon_count <2 or not available_aruco and not SHUTDOWN_REQUESTED:
        available_aruco=False
        if strategy.latest_robot_pose_beacon and len(strategy.latest_robot_pose_beacon.tracks) > 0:
            for track in strategy.latest_robot_pose_beacon.tracks:
                if track.aruco_id == 211:
                    available_aruco=True
                elif track.aruco_id == 212:
                    available_aruco=True
        
        time.sleep(0.1)
        strategy.get_logger().info(f"Waiting for  positions from beacon to fix the")
    
    stm_msg = 0.0
    pos = None
    
    strategy.get_logger().info(f"len(strategy.latest_robot_pose_beacon.tracks): {len(strategy.latest_robot_pose_beacon.tracks)}")
    
    # Search for ArUco markers 10 or 6 in beacon tracks
    if strategy.latest_robot_pose_beacon and len(strategy.latest_robot_pose_beacon.tracks) > 0:
        for track in strategy.latest_robot_pose_beacon.tracks:
            if track.aruco_id == 211:
                stm_msg = 1.0
                pos = track.map_pos
                strategy.get_logger().info(f"track.pos front: {track.map_pos}")
                break
            elif track.aruco_id == 212:
                stm_msg = 2.0
                pos = track.map_pos
                strategy.get_logger().info(f"track.pos back : {track.map_pos}")
                break
    # Apply position correction
    if pos is not None:
        strategy.get_logger().info(f"Received pos from beacon: x={pos.x}, y={pos.y}")
        stm32.fix_position(stm_msg, pos.x*1000, pos.y*1000)
    else:
        strategy.get_logger().info(f"Using fallback position: x={params[6]}, y={params[7]}")
        stm32.fix_position(stm_msg, params[6], params[7])

def stm32_task(strategy, stm32,lidar):
    """Runs every 100ms to process STM32 queue"""
    global IDLE
    global SHUTDOWN_REQUESTED
    global IDLE_FOR_SYSTEMS
    global EMERGENCY_STOP
    global last_stm_command
    global replanning
    global STEAL_LOOP_ACTIVE
    global STEAL_LOOP_RUNNING
    global STEAL_REPEAT_COUNT
    # Import Vector3 for publishing target position
    from geometry_msgs.msg import Vector3
    
    # Variables to save last command for resuming after emergency
    last_command = None
    last_params = None
    global SYSTEMS_IDLE_COUNT
    was_in_emergency = False
    global WAITING_TIRETTE
    strategy.get_logger().info("🚀 STM32 task started")
    
    main_loop_count = 0
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        main_loop_count += 1
        
        # Log main loop status every 50 iterations (5 seconds)
        if main_loop_count % 50 == 0:
            strategy.get_logger().info(f"🔁 STM32 main loop #{main_loop_count}: queue_size={len(strategy.queues['STM'])}, IDLE={IDLE}, EMERGENCY_STOP={EMERGENCY_STOP}")
        
        # Check if there are commands in the queue and robot is idle.
        # In steal-only emergency flow, do not dispatch commands while emergency is active.
        if len(strategy.queues['STM']) > 0 and IDLE and not EMERGENCY_STOP:
            cmd = strategy.queues['STM'][0]  # Peek at first command
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            # Extract metadata dict if present (before converting to floats)
            metadata = None
            if isinstance(params, list) and len(params) > 0 and isinstance(params[-1], dict):
                metadata = params.pop()
            
            params = to_float_list(params)
            
            # Re-add metadata after float conversion
            if metadata is not None:
                params.append(metadata)
            #strategy.get_logger().info(f"goto : x: {params[0]} y : {params[1]}")
            # Save command for potential resume after emergency
            last_command = command
            last_params = params
            last_stm_command=""
            
            # Execute command using handler function
            strategy.get_logger().info(f"🎯 Executing STM command: {command} with params: {params[:3] if len(params) > 3 else params}")
            while(command =="set_position" and WAITING_TIRETTE==True and not SHUTDOWN_REQUESTED):
                time.sleep(0.01)
            handle_stm32_command(command, params, stm32, lidar, strategy)
            last_stm_command="navigation"
            
            strategy.get_logger().info(f"⏳ Entering wait loop - IDLE={IDLE}, EMERGENCY_STOP={EMERGENCY_STOP}, last_stm_command={last_stm_command}")
            
            # Wait for command completion with emergency handling
            loop_iteration = 0
            while ((not IDLE and not SHUTDOWN_REQUESTED) or (EMERGENCY_STOP) or last_stm_command=="emergency") and last_stm_command != "replanned":
                loop_iteration += 1
                
                # Check for interrupt from strategy manager
                if strategy.interrupt_wait:
                    strategy.get_logger().warning("🛑 STM32 command wait interrupted - exiting loop")
                    strategy.stm32_task_wait_ended = True
                    break
                
                # Log state every 20 iterations (2 seconds)
                if loop_iteration % 20 == 0:
                    strategy.get_logger().info(f"🔄 Wait loop iteration {loop_iteration}: IDLE={IDLE}, EMERGENCY_STOP={EMERGENCY_STOP}, was_in_emergency={was_in_emergency}, replanning={replanning}, last_stm_command={last_stm_command}")
                
                # Check for emergency stop
                if EMERGENCY_STOP and not was_in_emergency and not replanning :
                    strategy.get_logger().warning(f"🚨 Emergency detected during movement - stopping STM32 (IDLE={IDLE}, IDLE_FOR_SYSTEMS={IDLE_FOR_SYSTEMS})")
                    stm32.stop()
                    last_stm_command="emergency"
                    IDLE=False
                    IDLE_FOR_SYSTEMS=True
                    replanning = True

                    # Emergency mode is steal-only: drop static queued motion/actuator work.
                    strategy.queues['STM'].clear()
                    strategy.queues['ESP_front'].clear()
                    strategy.queues['ESP_back'].clear()
                    strategy.get_logger().warning("🧹 Cleared STM/ESP queues for steal-only emergency recovery")

                    # Arm steal loop immediately from emergency stop.
                    if not STEAL_LOOP_ACTIVE and not STEAL_LOOP_RUNNING:
                        STEAL_REPEAT_COUNT = 30
                        STEAL_LOOP_ACTIVE = True
                        strategy.get_logger().warning(
                            "🏴‍☠️ Emergency-triggered steal loop armed (STEAL_LOOP_ACTIVE=True, STEAL_REPEAT_COUNT=30)"
                        )

                    was_in_emergency = True
                    strategy.get_logger().info("⏸️  Waiting for STM32 to report IDLE after emergency stop...")
                    
                    idle_wait_count = 0
                    while(not IDLE and not SHUTDOWN_REQUESTED):
                        idle_wait_count += 1
                        if idle_wait_count % 10 == 0:  # Log every 0.5 seconds
                            strategy.get_logger().info(f"⏸️  Still waiting for IDLE... (count={idle_wait_count}, IDLE={IDLE})")
                        time.sleep(0.05)  # Wait until STM32 reports idle after stop
                    
                    strategy.get_logger().info(f"✓ STM32 now IDLE after emergency stop (waited {idle_wait_count * 0.05:.2f}s)")
                    
                    # Keep lidar collision watch active until path is clear.
                    lidar.start_enemy_collision_check()  # Start waiting period before rechecking obstacle
                    strategy.get_logger().info("🔍 Started enemy collision check - waiting for obstacle to clear")
                
                # Check if emergency cleared
                elif not EMERGENCY_STOP and was_in_emergency  :
                    strategy.get_logger().info("✅ Emergency cleared - switching to steal-generated execution")
                    was_in_emergency = False
                    # Never resume pre-emergency static command in steal-only mode.
                    last_stm_command = "replanned"
                
                time.sleep(0.1)
            
            # Exited wait loop
            strategy.get_logger().info(f"🏁 Exited wait loop - IDLE={IDLE}, EMERGENCY_STOP={EMERGENCY_STOP}, was_in_emergency={was_in_emergency}, last_stm_command={last_stm_command}")
            
            if  last_stm_command!="navigation": 
                strategy.get_logger().info(f"⚠️ Command ended with non-navigation state: idle={IDLE}, EMERGENCY_STOP={EMERGENCY_STOP}, last_stm_command='{last_stm_command}'")
                # Reset emergency flag when command completes normally
                was_in_emergency = False
                
                
                
                # Remove command from queue after execution
            if last_stm_command=="navigation":
                strategy.get_logger().info(f"✓ Navigation completed successfully for command: {command}")
                # Correct position using beacon after bezier curves
                if command == 'bezier' or command == 'bezier_backwards':
                    strategy.get_logger().info("📍 Correcting bezier position")
                    
                    strategy.latest_robot_pose_beacon_count = 0
                    #correct_bezier(strategy, stm32, params)
                lidar.clear_goto_target()
                strategy.pathfinder.clear_goal()
                # Check if queue not empty before popping (race condition with queue.clear())
                if len(strategy.queues['STM']) > 0:
                    strategy.queues['STM'].popleft()
                    strategy.get_logger().info(f"🗑️  Removed command from queue - {len(strategy.queues['STM'])} commands remaining")
            elif last_stm_command=="replanned":
                # After replanning, don't pop - let outer loop execute new commands
                strategy.get_logger().info(f"✓ Replanning complete - new commands ready to execute ({len(strategy.queues['STM'])} commands in queue)")
                # Don't pop from queue - commands were just added and need to be executed
                # Reset replanning flag now that we're back in main loop
                replanning = False
        else:
            # Log why we're not processing commands (every 50 iterations to avoid spam)
            if main_loop_count % 50 == 0:
                if len(strategy.queues['STM']) == 0:
                    strategy.get_logger().info("⏸️  STM queue empty - waiting for commands")
                elif not IDLE:
                    strategy.get_logger().info(f"⏸️  Robot not IDLE - waiting (IDLE={IDLE}, IDLE_FOR_SYSTEMS={IDLE_FOR_SYSTEMS}, EMERGENCY_STOP={EMERGENCY_STOP})")
        
        time.sleep(0.1)  # 100ms
def handle_color_pattner(strategy,front):
    global known_id_back
    global known_id_front
    """Handle color pattern retrieval"""
    x = strategy.latest_robot_pose.x
    y = strategy.latest_robot_pose.y
    z = strategy.latest_robot_pose.z
    # Create Point for current position
    pos = Point()
    pos.x = float(x)
    pos.y = float(y)
    #strategy.get_logger().info(f"robot position : x={x} , y={y}")
    pos.z = 0.0
    known_id = ''

    color_pattern, success, direction, batch_center, batch_known_id = strategy.pathfinder.get_batch_color(known_id, pos)
    # Default to "yyyy" if no pattern received or failed
    if not color_pattern or not success or len(color_pattern) != 4:
        color_pattern = "yyyy"
        strategy.get_logger().info(f"⚠️  No batch found or failed - defaulting to pattern: {color_pattern}")
    else:
        strategy.get_logger().info(f"pattern received from beacon: {color_pattern} ")
    known_idd=""
    # Delete the batch after successfully retrieving its color pattern
    if success and batch_known_id:
        deleted = strategy.pathfinder.delete_batch(batch_known_id)
        if deleted:
            strategy.get_logger().info(f"🗑️  Batch '{batch_known_id}' deleted from map after pickup")

            # Keep SimpleMap in sync and prevent the deleted batch from being
            # re-introduced by background get_active_batches refresh.
            scheduler = getattr(strategy, 'task_scheduler', None)
            simple_map = getattr(scheduler, 'map', None) if scheduler is not None else None
            if simple_map is not None and hasattr(simple_map, 'remove_batch'):
                try:
                    simple_map.remove_batch(batch_known_id, persistently=True)
                    strategy.get_logger().info(
                        f"🧠 SimpleMap synced: removed batch '{batch_known_id}' (persistent delete)"
                    )
                except Exception as e:
                    strategy.get_logger().warning(
                        f"⚠️  Failed to sync deleted batch '{batch_known_id}' into SimpleMap: {e}"
                    )
        else:
            strategy.get_logger().warning(f"⚠️  Failed to delete batch '{batch_known_id}' from map")
    if success and batch_known_id:
        if front:
            known_id_front = batch_known_id
            known_idd=batch_known_id
        else:
            known_id_back = batch_known_id
            known_idd=batch_known_id
    if success and batch_center:
        theta = math.degrees(z)  # Convert radians to degrees
        
        # Determine if we should reverse based on direction and robot orientation
        should_reverse = False
        
        if direction == "vertical":
            # For horizontal batches: reverse if (front AND theta == -180) OR (back AND theta == 0)
            if (front and abs(abs(theta) - (180)) < 5) or (not front and abs(theta - 0) < 5):
                strategy.get_logger().info(f"front: {front} theta : {theta} vertical")
                should_reverse = False
            else:
                should_reverse=True
        elif direction == "horizontal":
            # For vertical batches: reverse if (front AND theta == -90) OR (back AND theta == 90)
            if (front and abs(theta - (-90)) < 5) or (not front and abs(theta - 90) < 5):
                should_reverse = True
                strategy.get_logger().info(f"front: {front} theta : {theta} horizontal")
            else: 
                should_reverse=False
        if should_reverse:
            color_pattern = color_pattern[::-1]
            strategy.get_logger().info(f"🔄 Reversed color pattern - direction: {direction}, front: {front}, theta: {theta:.1f}° known_id reversed:{known_idd} ")
        else: 
            strategy.get_logger().info(f"not Reversed color pattern - direction: {direction}, front: {front}, theta: {theta:.1f}° known_id reversed:{known_idd} ")
    else:
        strategy.get_logger().warning("Failed to get batch center from service")
    strategy.get_logger().info(f"color received from beacon{color_pattern}")

    return color_pattern
def convert_color_pattern(color_pattern, robot_color):
    """
    Convert color pattern from 'y'/'b' format to binary string.
    
    Args:
        color_pattern (str): Pattern string with 'y' (yellow) and 'b' (blue), e.g., "yybb", "ybyb"
        robot_color (str): The robot's color, either 'y' (yellow) or 'b' (blue)
    
    Returns:
        str: Binary string where robot's color (my color) is '0' and opposite color (not my color) is '1'
        
    Examples:
        convert_color_pattern("yybb", "y") -> "0011"  - first two are my color (0), last two are not (1)
        convert_color_pattern("yybb", "b") -> "1100" - first two are not my color (1), last two are (0)
        convert_color_pattern("ybyb", "y") -> "0101"  - y=0, b=1, y=0, b=1
        convert_color_pattern("ybyb", "b") -> "1010" - y=1, b=0, y=1, b=0
    """
    if not color_pattern:
        return "0"
    
    binary_str = ""
    for char in color_pattern.lower():
        if char == robot_color.lower():
            binary_str += "0"  # My color = 0
        else:
            binary_str += "1"  # Not my color = 1
    
    return binary_str


def stana(controller):
    """Wait for controller to be idle, with shutdown check"""
    global SHUTDOWN_REQUESTED
    waiting_started = False
    # Use shorter timeout to allow quicker shutdown response

    idle = controller.IDLE(timeout=5.0)
    if not waiting_started and idle:
        idle=False
    if not idle and waiting_started==False:
        waiting_started = True
        controller.get_logger().info("⏳ Waiting for controller to be idle...")
    while not idle and not SHUTDOWN_REQUESTED:
        idle = controller.IDLE(timeout=5.0)
        if not waiting_started and idle:
            idle=False
        if not idle and waiting_started==False:
            waiting_started = True
            controller.get_logger().info("⏳ Waiting for controller to be idle...")
        time.sleep(0.1)
    
def talaa(strategy, front_controller=None,front=True):
    """Request color pattern from pathfinding service"""
    x = strategy.global_pos_x
    y = strategy.global_pos_y
    global color_pt
    strategy.get_logger().info(f"📞 talaa() called - about to request color pattern from pathfinder")
    # Create Point for current position
    # pos = Point()
    # pos.x = float(x)
    # pos.y = float(y)
    # pos.z = 0.0
    
    known_id = ''

    color_pattern=handle_color_pattner(strategy,front)
    success=True
    if success:
        # Convert global color ('yellow' or 'blue') to single letter ('y' or 'b')
        # Access color from strategy object instead of global variable
        if  strategy.color:
            robot_color = 'y' if strategy.color.lower() == 'yellow' else 'b'
        else:
            robot_color = 'y'  # Default to 'y' if color not set
        color_pt = convert_color_pattern(color_pattern, robot_color)
        color_to_N20=int(str(color_pt).replace('0', '3').replace('1', '4'))
        print(f"Converted color pattern: {color_pt}, N20 command: {color_to_N20}")
        front_controller.n20(color_to_N20)
        return color_pt
    else:
        strategy.get_logger().warning("⚠️ Failed to get color pattern")
        return None

#1 louta wl 2 fel wost wl 3 lfouk
def _update_robot_holder_status(strategy, front: bool, empty: bool) -> None:
    """Sync holder state into `strategy.task_scheduler.map` (new architecture)."""
    scheduler = getattr(strategy, 'task_scheduler', None)
    if scheduler is None:
        if hasattr(strategy, 'get_logger'):
            strategy.get_logger().warning("⚠️ task_scheduler is missing on strategy")
        return

    map_obj = getattr(scheduler, 'map', None)
    if map_obj is None:
        if hasattr(strategy, 'get_logger'):
            strategy.get_logger().warning("⚠️ strategy.task_scheduler.map is missing")
        return

    if front:
        map_obj.update_robot_front_status(bool(empty))
    else:
        map_obj.update_robot_back_status(bool(empty))


def _update_robot_color_in_map(strategy) -> None:
    """Sync `strategy.color` into `strategy.task_scheduler.map` at game start."""
    scheduler = getattr(strategy, 'task_scheduler', None)
    map_obj = getattr(scheduler, 'map', None) if scheduler is not None else None
    if map_obj is None:
        if hasattr(strategy, 'get_logger'):
            strategy.get_logger().warning("⚠️ strategy.task_scheduler.map is missing (color not synced)")
        return

    color = getattr(strategy, 'color', None)
    if not color:
        if hasattr(strategy, 'get_logger'):
            strategy.get_logger().warning("⚠️ strategy.color is not set (color not synced)")
        return

    map_obj.set_robot_color(str(color).strip().lower())


def pick_up(controller,pneumatic,strategy,front):
    global color_pt_front
    global color_pt_back
    global ESP_FRONT_IDLE
    global ESP_BACK_IDLE
    global SHUTDOWN_REQUESTED
    
    strategy.get_logger().info(f"🚀 pick_up() called for {'FRONT' if front else 'BACK'}")
    
    if SHUTDOWN_REQUESTED:
        strategy.get_logger().warning("⚠️ pick_up() aborted - SHUTDOWN_REQUESTED")
        return
    if EMERGENCY_STOP:
        strategy.get_logger().warning("⚠️ pick_up() deferred - EMERGENCY_STOP is active")
        return
    #stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    
    if front:
        pneumatic.front(0)
    else: pneumatic.back(0)
    controller.kabech(1)
    time.sleep(0.8)
    controller.n20(1111)
    stana(controller)
    time.sleep(1)
    if SHUTDOWN_REQUESTED:
        return
    
    
    time.sleep(0.0)
    strategy.get_logger().info(f"🔍 Requesting NEW color pattern for {'FRONT' if front else 'BACK'} at position ({strategy.latest_robot_pose.x:.3f}, {strategy.latest_robot_pose.y:.3f})")
    color_pattern = talaa(strategy, controller,front)
    if front:
        color_pt_front = color_pattern
        ESP_FRONT_IDLE=True
        _update_robot_holder_status(strategy, front=True, empty=False)
        strategy.get_logger().info(f"📦 Front picked up - NEW Color pattern: {color_pt_front}")
    else:
        color_pt_back = color_pattern
        ESP_BACK_IDLE=True
        _update_robot_holder_status(strategy, front=False, empty=False)
        strategy.get_logger().info(f"📦 Back picked up - NEW Color pattern: {color_pt_back}")
    stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    controller.kabech(0)
def put_down(controller,pneumatic,strategy,front,params=None):
    global ESP_FRONT_IDLE
    global ESP_BACK_IDLE
    global color_pt_front
    global color_pt_back
    global SHUTDOWN_REQUESTED
    
    if SHUTDOWN_REQUESTED:
        return
    
    params = params if isinstance(params, dict) else {}
    insert_offset_mm = float(params.get('putdown_insert_distance_mm', 0.0) or 0.0)

    color_pattern = color_pt_front if front else color_pt_back
    strategy.get_logger().info(f"color_pattern {color_pattern}")
    color_to_N20=int(str(color_pattern).replace('1', '4').replace('0','1'))
    strategy.get_logger().info(f"TO n20 command  {color_to_N20}")
    time.sleep(0.3)
    #controller.n20(color_to_N20)
    #stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    
    if front:
        pneumatic.front(15)
        # time.sleep(0.5)
        # pneumatic.front(15)
    else: 
        pneumatic.back(15)
        # time.sleep(0.5)
        # pneumatic.back(15)
    # Sync placed batch into pathfinder obstacle map
    if hasattr(strategy, 'pathfinder') and strategy.pathfinder is not None:
        put_down_front = 1 if front else 0
        put_down_color = (strategy.color or 'yellow').strip().lower() if hasattr(strategy, 'color') else 'yellow'
        if put_down_color not in ('yellow', 'blue'): 
            put_down_color = 'yellow'

        use_pose = False
        x_mm = 0.0
        y_mm = 0.0
        theta_deg = 0.0
        if hasattr(strategy, 'latest_robot_pose') and strategy.latest_robot_pose is not None:
            try:
                x_mm = float(strategy.latest_robot_pose.x) * 1000.0
                y_mm = float(strategy.latest_robot_pose.y) * 1000.0
                theta_deg = math.degrees(float(strategy.latest_robot_pose.z))
                use_pose = True
            except Exception as e:
                strategy.get_logger().warning(f"⚠️ Could not use latest_robot_pose for put_down: {e}")

        success, message, obstacle_ids = strategy.pathfinder.put_down(
            front=put_down_front,
            color=put_down_color,
            use_pose=use_pose,
            x_mm=x_mm,
            y_mm=y_mm,
            theta_deg=theta_deg,
            insert_offset_mm=insert_offset_mm,
        )

        if success:
            strategy.get_logger().info(
                f"✅ Pathfinder put_down synced ({'front' if front else 'back'}, color={put_down_color}, use_pose={use_pose}) "
                f"-> {len(obstacle_ids)} obstacles"
            )
        else:
            strategy.get_logger().warning(
                f"⚠️ Pathfinder put_down failed ({'front' if front else 'back'}, color={put_down_color}): {message}"
            )
    if front:
        ESP_FRONT_IDLE=True
        _update_robot_holder_status(strategy, front=True, empty=True)
    else: ESP_BACK_IDLE=True
    if not front:
        _update_robot_holder_status(strategy, front=False, empty=True)
    
    if SHUTDOWN_REQUESTED:
        return
    
    controller.n20(0)
    
    stana(controller)
    if SHUTDOWN_REQUESTED:
        return
    time.sleep(0.5)
    if front:
        pneumatic.front(16)
    else: pneumatic.back(16)

    
def front_task(strategy, front_controller=None,pneumatic_controller=None):
    global color_pt_front
    global SHUTDOWN_REQUESTED
    """Runs every 100ms to process ESP32_1 queue"""
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue
        if len(strategy.queues['ESP_front']) > 0:
            cmd = strategy.queues['ESP_front'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)

            # Never run actuator actions while emergency is active.
            # Keep the command queued and retry when emergency clears.
            if EMERGENCY_STOP and command in ('pick_up', 'put_down'):
                time.sleep(0.1)
                continue

            # Safety ordering guard: actuator actions must only run after
            # STM commands are fully dispatched/executed for the current step.
            if command in ('pick_up', 'put_down') and (len(strategy.queues['STM']) > 0 or not IDLE_FOR_SYSTEMS):
                time.sleep(0.05)
                continue

            # Concurrency safety: queue may be cleared/replaced by another thread
            # (e.g., emergency handling) after we captured `cmd`.
            # Never execute if this command is no longer at queue head.
            if len(strategy.queues['ESP_front']) == 0 or strategy.queues['ESP_front'][0] is not cmd:
                time.sleep(0.01)
                continue
            
            if command == 'prepare':
                action = params.get('action')
                if action == 'pick_up':
                    front_controller.n20(2222)
                    front_controller.dawer(0)
                    strategy.get_logger().info(f"🔧 Front prepare for pick_up: n20(2222)")
                else:
                    cmd_to_n20=int(str(color_pt_front).replace('0','2').replace('1','4'))
                    front_controller.dawer(1)
                    front_controller.n20(cmd_to_n20)
            elif command =='put_down':
                put_down(front_controller,pneumatic_controller,strategy,True,params)
            elif command =='pick_up':
                pick_up(front_controller,pneumatic_controller,strategy,True)
            # Pop only if the same command is still at queue head
            if len(strategy.queues['ESP_front']) > 0 and strategy.queues['ESP_front'][0] is cmd:
                strategy.queues['ESP_front'].popleft()
        
        time.sleep(0.1)  # 100ms


def back_task(strategy, back_controller=None,pneumatic_controller=None):
    """Runs every 100ms to process ESP32_1 queue"""
    global color_pt_back
    global SHUTDOWN_REQUESTED
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue
        if len(strategy.queues['ESP_back']) > 0:
            cmd = strategy.queues['ESP_back'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)

            # Never run actuator actions while emergency is active.
            # Keep the command queued and retry when emergency clears.
            if EMERGENCY_STOP and command in ('pick_up', 'put_down'):
                time.sleep(0.1)
                continue

            # Safety ordering guard: actuator actions must only run after
            # STM commands are fully dispatched/executed for the current step.
            if command in ('pick_up', 'put_down') and (len(strategy.queues['STM']) > 0 or not IDLE_FOR_SYSTEMS):
                time.sleep(0.05)
                continue

            # Concurrency safety: queue may be cleared/replaced by another thread
            # (e.g., emergency handling) after we captured `cmd`.
            # Never execute if this command is no longer at queue head.
            if len(strategy.queues['ESP_back']) == 0 or strategy.queues['ESP_back'][0] is not cmd:
                time.sleep(0.01)
                continue
            
            if command == 'prepare':
                action = params.get('action')
                if action == 'pick_up':
                    back_controller.n20(2222)
                    back_controller.dawer(0)
                    strategy.get_logger().info(f"🔧 Back prepare for pick_up: n20(2222)")
                else:
                    cmd_to_n20=int(str(color_pt_back).replace('0','2').replace('1','4'))
                    back_controller.dawer(1)
                    back_controller.n20(cmd_to_n20)
            elif command =='put_down':
                put_down(back_controller,pneumatic_controller,strategy,False,params)
            elif command =='pick_up':
                pick_up(back_controller,pneumatic_controller,strategy,False)
            # Pop only if the same command is still at queue head
            if len(strategy.queues['ESP_back']) > 0 and strategy.queues['ESP_back'][0] is cmd:
                strategy.queues['ESP_back'].popleft()
        
        time.sleep(0.1)  # 100ms


def pami_task(strategy, pami_controller=None):
    """Runs every 20ms to process PAMI queue without blocking other executors."""
    global SHUTDOWN_REQUESTED

    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        if len(strategy.queues['PAMI']) > 0:
            cmd = strategy.queues['PAMI'][0]  # Peek
            command = cmd.get('command')

            if not pami_controller:
                strategy.get_logger().warning("PAMI controller not available - dropping PAMI command")
            elif command == 'publish_strategies':
                try:
                    strategy.get_logger().info("📤 Publishing PAMI strategies in background task...")
                    pami_controller.load_and_publish_pami_strategies()
                    strategy.get_logger().info("✅ PAMI strategies published")
                except Exception as e:
                    strategy.get_logger().error(f"❌ Error publishing PAMI strategies: {e}")
            elif command == 'start_pamis':
                try:
                    strategy.get_logger().info("🤖 Starting PAMIs in background task...")
                    pami_controller.start_pamis()
                    strategy.get_logger().info("✅ PAMIs started")
                except Exception as e:
                    strategy.get_logger().error(f"❌ Error starting PAMIs: {e}")
            else:
                strategy.get_logger().warning(f"Unknown PAMI command: {command}")

            if len(strategy.queues['PAMI']) > 0:
                strategy.queues['PAMI'].popleft()

            if len(strategy.queues['PAMI']) > 0:
                continue

        time.sleep(0.02)


def rasp_pi_task(strategy, round_timer, rpi_controller=None, stm32_controller=None, front_controller=None, back_controller=None, pneumatic_controller=None, pami_controller=None):
    """Runs every 100ms to process RASP_PI queue"""
    global SHUTDOWN_REQUESTED
    global WAITING_TIRETTE
    global IDLE
    global IDLE_FOR_SYSTEMS
    global return_home_triggered
    global STEAL_LOOP_ACTIVE
    global STEAL_REPEAT_COUNT
    while rclpy.ok() and not SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue
        if len(strategy.queues['RASP_PI']) > 0:
            cmd = strategy.queues['RASP_PI'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', [])
            wait = cmd.get('wait', False)
            
            # Execute RASP_PI commands
            if command == 'Wait for manual start':
                WAITING_TIRETTE=True
                if rpi_controller:
                    # Check if tirette is already inserted
                    tirette_status = rpi_controller.read_pin_17()
                    if tirette_status == 0:
                        strategy.get_logger().warning("⚠️ mafamech tirette ya maaahdi !")
                    result = rpi_controller.wait_start()
                    stm32_controller.reset()
                    WAITING_TIRETTE=False
                    strategy.get_logger().info(f"✅ Tirette tjebdet  STRATEGY GOOOOOOOOO !!!!")
                    
                    # Start game timer on pathfinder to begin periodic map updates
                    success, message = strategy.pathfinder.start_game()
                    if success:
                        strategy.get_logger().info(f"🎮 Pathfinder game timer started: {message}")
                    else:
                        strategy.get_logger().warning(f"⚠️ Failed to start pathfinder game timer: {message}")
                    
                    # Start timer and map update FIRST (critical for robot operation)
                    round_timer.start_game()
                    strategy.get_logger().info(f"⏱️  Game timer started!")
                    _update_robot_color_in_map(strategy)
                    strategy.get_logger().info(f"🎨 Robot color synced to map: {strategy.color}")
                    #strategy.get_logger().info("🗺️  Updating map...")
                    #strategy.pathfinder.update_map()
                    
                    # Enqueue PAMI start command to dedicated PAMI task
                    if pami_controller:
                        strategy.queues['PAMI'].append({'command': 'start_pamis'})
                        strategy.get_logger().info("🚀 Robot ready - queued PAMI start")
                else:
                    strategy.get_logger().warning("RPI controller not available - skipping wait_start")
            elif command == 'RESET':
                # Execute reset in background
                #if stm32_controller:
                    #stm32_controller.reset(timeout=2.0)
                if pneumatic_controller:
                    pneumatic_controller.front(16)
                    pneumatic_controller.back(16)
                if front_controller:
                    front_controller.kabech(0)
                    front_controller.dawer(0)
                    front_controller.n20(0)
                    stana(front_controller)
                if back_controller:
                    back_controller.kabech(0)
                    back_controller.dawer(0)
                    back_controller.n20(0)
                    stana(back_controller)
                # Allow time for messages to be sent
                time.sleep(0.1)
            elif command == 'set_position':
                strategy.get_logger().info("set position sent from the rasp queue")
                
                # Extract metadata dict if present (before converting to floats)
                if isinstance(params, list) and len(params) > 0 and isinstance(params[-1], dict):
                    params.pop()
                
                params = to_float_list(params)
                stm32_controller.set_position(params[0], params[1], params[2])
                IDLE_FOR_SYSTEMS=True
                IDLE=True

            elif command == 'wait_end_game':
                # Wait until game time reaches 1.25s + duration
                duration = float(params[0]) if params else 0
                target_time = 85 + duration
                return_home_triggered=True
                strategy.get_logger().info(f"⏳ Waiting until game time reaches {target_time}s (85s + {duration}s)")
                
                while rclpy.ok() and not SHUTDOWN_REQUESTED:
                    current_time = round_timer.time()
                    if current_time >= target_time:
                        strategy.get_logger().info(f"✅ Target time {target_time}s reached (current: {current_time}s)")
                        break
                    time.sleep(0.05)  # Check every 50ms

            elif command == 'steal':
                # Generate and queue dynamic steal tasks from YAML command
                # Params example: [3, 0] => min_members=3, device=auto
                try:
                    repeat_count = 1
                    if isinstance(params, list) and len(params) > 0:
                        try:
                            repeat_count = int(float(params[0]))
                        except (TypeError, ValueError):
                            strategy.get_logger().warning(
                                f"⚠️ Invalid steal repeat count '{params[0]}', defaulting to 1"
                            )
                            repeat_count = 1

                    if repeat_count <= 0:
                        strategy.get_logger().warning(
                            f"⚠️ Steal repeat count is {repeat_count}; skipping steal execution"
                        )
                    else:
                        STEAL_REPEAT_COUNT = repeat_count
                        STEAL_LOOP_ACTIVE = True
                        strategy.get_logger().info(
                            f"🏴‍☠️ Queued steal loop for path-planning task ({repeat_count} time(s))"
                        )
                except Exception as e:
                    strategy.get_logger().error(f"❌ Error handling RASP_PI steal command: {e}")

            else:
                strategy.get_logger().warn(f"Unknown RASP_PI command: {command}")
            
            # Remove command from queue after execution (if queue not cleared by another thread)
            if len(strategy.queues['RASP_PI']) > 0:
                strategy.queues['RASP_PI'].popleft()

            # If another command is already queued (e.g. set_position right after tirette),
            # process it immediately instead of waiting for the next polling tick.
            if len(strategy.queues['RASP_PI']) > 0:
                continue
            
        
        time.sleep(0.02)  # 20ms to reduce post-tirette startup delay