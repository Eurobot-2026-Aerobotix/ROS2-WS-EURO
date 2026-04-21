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

from euro.shared_state import state

def path_planning_task(strategy, lidar, round_timer, pneumatic_controller=None, front_controller=None, back_controller=None,stm32_controller=None):
    """Dedicated thread for handling return home at 90s"""
    strategy.get_logger().info("🏠 Return home monitor thread started")
    state.return_home_triggered = False
    reset_stm_triggered=False

    def _check_end_game_timeout() -> bool:
        nonlocal reset_stm_triggered
        if round_timer.time() >= 100.0 and not reset_stm_triggered:
            strategy.get_logger().info("⏰ Game time reached 100s - resetting stm")
            if stm32_controller is not None:
                try:
                    stm32_controller.reset()
                except Exception as e:
                    strategy.get_logger().warning(f"⚠️ Failed to reset STM at end-game timeout: {e}")
            else:
                strategy.get_logger().warning("⚠️ STM32 controller is None at end-game timeout")
            reset_stm_triggered = True
            strategy.end_game = True
            state.STEAL_LOOP_ACTIVE = False
            state.STEAL_LOOP_RUNNING = False
            state.STEAL_REPEAT_COUNT = 30
            # Prevent any new dispatch after timeout.
            try:
                strategy.queues['STM'].clear()
                strategy.queues['ESP_front'].clear()
                strategy.queues['ESP_back'].clear()
                strategy.queues['PAMI'].clear()
                strategy.queues['RASP_PI'].clear()
            except Exception:
                pass
            strategy.get_logger().warning("🛑 End-game timeout: cleared pending command queues")
        return bool(strategy.end_game)

    while rclpy.ok() and not state.SHUTDOWN_REQUESTED:
        _check_end_game_timeout()
        # Run dynamic steal flow whenever it is armed.
        # This allows task-tracking to arm dynamic generation even if
        # EMERGENCY_STOP is already cleared by lidar.
        if state.STEAL_LOOP_ACTIVE and not state.STEAL_LOOP_RUNNING and not strategy.end_game:
            state.STEAL_LOOP_RUNNING = True
            try:
                repeat_count = max(1, int(state.STEAL_REPEAT_COUNT))
            except (TypeError, ValueError):
                repeat_count = 1

            try:
                strategy.get_logger().info(
                    f"🏴‍☠️ Path-planning steal loop started ({repeat_count} time(s))"
                )
                for attempt in range(1, repeat_count + 1):
                    if _check_end_game_timeout():
                        strategy.get_logger().warning(
                            "⏰ Stopping path-planning steal loop due to 100s timeout"
                        )
                        break

                    if not rclpy.ok() or state.SHUTDOWN_REQUESTED or strategy.end_game or state.return_home_triggered or not state.STEAL_LOOP_ACTIVE:
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
                    while rclpy.ok() and not state.SHUTDOWN_REQUESTED and not strategy.end_game and state.STEAL_LOOP_ACTIVE:
                        if _check_end_game_timeout():
                            strategy.get_logger().warning(
                                "⏰ Steal loop gate stopped due to 100s timeout"
                            )
                            break

                        path_replan_requested = bool(getattr(lidar, 'generate_path', 0) == 1)
                        ready_for_steal = bool(state.IDLE) and (not state.EMERGENCY_STOP or path_replan_requested)
                        if ready_for_steal:
                            break

                        wait_gate_count += 1
                        if wait_gate_count == 1 or wait_gate_count % 20 == 0:
                            strategy.get_logger().info(
                                f"⏳ Steal loop waiting (EMERGENCY_STOP={int(bool(state.EMERGENCY_STOP))}, IDLE={state.IDLE}, generate_path={int(path_replan_requested)})"
                            )
                        time.sleep(0.05)

                    # If lidar explicitly asks for replanning while emergency is still
                    # active, clear stale target so emergency can be reevaluated against
                    # the new generated steal path.
                    if bool(state.IDLE) and state.EMERGENCY_STOP and bool(getattr(lidar, 'generate_path', 0) == 1):
                        strategy.get_logger().warning(
                            "🧭 Steal loop proceeding on generate_path=1 while emergency active; clearing stale go-to target"
                        )
                        lidar.clear_goto_target()
                        lidar.generate_path = 0

                    if not rclpy.ok() or state.SHUTDOWN_REQUESTED or strategy.end_game or state.return_home_triggered or not state.STEAL_LOOP_ACTIVE:
                        strategy.get_logger().warning(
                            "⚠️ Stopping path-planning steal loop before attempt due to shutdown/end-game"
                        )
                        break

                    # Ensure emergency-replanning flag does not block generated-flow waits.
                    state.replanning = False

                    strategy.get_logger().info(
                        f"🏴‍☠️ Path-planning steal attempt {attempt}/{repeat_count}"
                    )
                    success = strategy.handle_steal_command(params=[repeat_count, 0], wait=True)
                    # Add a small delay to allow other threads to update their state
                    time.sleep(0.2)

                    if _check_end_game_timeout():
                        strategy.get_logger().info(
                            f"🏁 End-game reached during path-planning steal loop at attempt {attempt}/{repeat_count}"
                        )
                        break

                    if not success:
                        strategy.get_logger().warning(
                            f"⚠️ Path-planning steal attempt {attempt}/{repeat_count} failed"
                        )
                        if state.EMERGENCY_STOP or state.replanning:
                            strategy.get_logger().warning(
                                "🔁 Keeping steal loop active after transient emergency enqueue failure"
                            )
                            continue
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
                state.STEAL_LOOP_ACTIVE = False
                state.STEAL_LOOP_RUNNING = False
                state.STEAL_REPEAT_COUNT = 30
        
        time.sleep(0.05)  # 50ms for quicker IDLE updates

def prepare_path(strategy, lidar):
    """Deprecated: prepared-path flow disabled. Emergency handling is steal-only."""
    strategy.get_logger().info("ℹ️ prepare_path() skipped (steal-only mode enabled)")
    return False

def execute_prepared_path(strategy, lidar):
    """Deprecated: prepared-path flow disabled. Emergency handling is steal-only."""
    strategy.get_logger().info("ℹ️ execute_prepared_path() skipped (steal-only mode enabled)")
    return False


def task_tracking_task(strategy, stm32, lidar=None):
    """
    Monitor TaskScheduler tracked-task cancellation and trigger emergency recovery.

    If `strategy.task_scheduler.cancel_tracked_task()` becomes True while the current
    execution context is a pick/put action, enforce the same steal-only emergency
    recovery flow used for lidar emergency handling.
    """
    strategy.get_logger().info("🧭 Task-tracking monitor thread started")

    while rclpy.ok() and not state.SHUTDOWN_REQUESTED:
        scheduler = getattr(strategy, 'task_scheduler', None)
        if scheduler is None:
            time.sleep(0.05)
            continue

        try:
            should_cancel = bool(scheduler.cancel_tracked_task())
        except Exception as e:
            strategy.get_logger().warning(f"⚠️ task_tracking: cancel_tracked_task() failed: {e}")
            time.sleep(0.05)
            continue

        if not should_cancel:
            time.sleep(0.05)
            continue

        # Ignore cancellation while executing generated post-putdown follow-up
        # tasks; those phases intentionally run after the initial put_down.
        tracked_task = getattr(scheduler, 'tracked_task', None)
        tracked_task_name = str(getattr(tracked_task, 'task_name', '') or '').lower()
        if tracked_task_name.startswith('putdown_insert_') or tracked_task_name.startswith('putdown_retreat_'):
            scheduler.tracked_task_invalid = False
            time.sleep(0.05)
            continue

        # React to tracked-task cancellation only in STM execution context:
        # - len(STM queue) > 1
        # - OR len(STM queue) == 1 and IDLE_FOR_SYSTEMS == False
        stm_queue_len = len(getattr(strategy, 'queues', {}).get('STM', []))
        in_stm_execution_window = (
            (stm_queue_len > 1)
            or (stm_queue_len == 1 and not bool(getattr(state, 'IDLE_FOR_SYSTEMS', True)))
        )
        if not in_stm_execution_window:
            scheduler.tracked_task_invalid = False
            time.sleep(0.05)
            continue

        in_pickput_context = str(getattr(strategy, 'last_task_command', '') or '').lower() in ('pick_up', 'put_down')
        if not in_pickput_context:
            # No active pickup/putdown context: consume stale invalid flag and continue.
            scheduler.tracked_task_invalid = False
            time.sleep(0.05)
            continue

        # Distance guard: do not cancel tracked task when robot is still far
        # from the current global goal (threshold in millimeters).
        max_cancel_distance_mm = 300.0
        latest_pose = getattr(strategy, 'latest_robot_pose', None)
        goal_x = getattr(strategy, 'global_pos_x', None)
        goal_y = getattr(strategy, 'global_pos_y', None)
        if latest_pose is not None and goal_x is not None and goal_y is not None:
            try:
                robot_x_m = float(getattr(latest_pose, 'x'))
                robot_y_m = float(getattr(latest_pose, 'y'))
                goal_x_m = float(goal_x)
                goal_y_m = float(goal_y)

                # Global goal can occasionally be in mm before normalization.
                if abs(goal_x_m) > 10.0 or abs(goal_y_m) > 10.0:
                    goal_x_m /= 1000.0
                    goal_y_m /= 1000.0

                goal_distance_mm = math.hypot(robot_x_m - goal_x_m, robot_y_m - goal_y_m) * 1000.0
                if goal_distance_mm < max_cancel_distance_mm:
                    strategy.get_logger().info(
                        f"🛡️ Task-tracking cancel ignored: distance to global goal is {goal_distance_mm:.1f}mm "
                        f"(>{max_cancel_distance_mm:.1f}mm)"
                    )
                    scheduler.tracked_task_invalid = False
                    time.sleep(0.05)
                    continue
            except Exception as e:
                strategy.get_logger().debug(
                    f"task_tracking: distance guard skipped (pose/goal parse failed): {e}"
                )

        if state.replanning:
            # Recovery already in progress; consume flag to avoid retrigger spam.
            scheduler.tracked_task_invalid = False
            time.sleep(0.05)
            continue

        strategy.get_logger().warning(
            f"🚨 Task-tracking cancellation detected during {strategy.last_task_command} - triggering emergency stop"
        )
        strategy.remove_esp_waiting = True
        stm32.soft_stop()
        state.last_stm_command = "emergency"
        state.IDLE = False
        state.IDLE_FOR_SYSTEMS = True
        state.replanning = True

        # Emergency mode is steal-only: drop static queued motion/actuator work.
        strategy.queues['STM'].clear()
        strategy.queues['ESP_front'].clear()
        strategy.queues['ESP_back'].clear()
        state.ESP_DISPATCH_EPOCH = int(getattr(state, 'ESP_DISPATCH_EPOCH', 0)) + 1
        state.ESP_FRONT_IDLE = True
        state.ESP_BACK_IDLE = True
        strategy.get_logger().warning(
            f"🧹 Cleared STM/ESP queues for task-tracking emergency recovery (epoch={state.ESP_DISPATCH_EPOCH})"
        )

        # Motion command is canceled: clear stale motion targets so lidar does not
        # keep EMERGENCY_STOP latched against an obsolete path.
        if lidar is not None:
            try:
                lidar.clear_goto_target()
            except Exception as e:
                strategy.get_logger().warning(f"⚠️ Failed to clear lidar go-to target: {e}")
        try:
            strategy.pathfinder.clear_goal()
        except Exception as e:
            strategy.get_logger().warning(f"⚠️ Failed to clear pathfinder goal: {e}")

        # Arm dynamic steal flow immediately (do not wait for this thread's
        # post-stop IDLE wait to complete). The path-planning thread will
        # gate actual attempts until motion state is safe.
        if not state.STEAL_LOOP_ACTIVE and not state.STEAL_LOOP_RUNNING:
            state.STEAL_REPEAT_COUNT = 3000
            state.STEAL_LOOP_RUNNING = False
            state.STEAL_LOOP_ACTIVE = True
            strategy.get_logger().warning(
                "🏴‍☠️ Task-tracking emergency armed steal loop (STEAL_LOOP_ACTIVE=True, STEAL_REPEAT_COUNT=30)"
            )

        if lidar is not None:
            try:
                lidar.start_enemy_collision_check()
            except Exception as e:
                strategy.get_logger().warning(f"⚠️ Failed to start lidar collision check: {e}")

        strategy.get_logger().info("⏸️  Waiting for STM32 to report IDLE after task-tracking emergency stop...")
        idle_wait_count = 0
        while (not state.IDLE and not state.SHUTDOWN_REQUESTED):
            idle_wait_count += 1
            if idle_wait_count % 10 == 0:
                strategy.get_logger().info(
                    f"⏸️  Still waiting for IDLE... (count={idle_wait_count}, IDLE={state.IDLE})"
                )
            time.sleep(0.05)

        strategy.get_logger().info(
            f"✓ STM32 now IDLE after task-tracking emergency stop (waited {idle_wait_count * 0.05:.2f}s)"
        )

        # If not already armed above (race/edge case), arm once here.
        if not state.STEAL_LOOP_ACTIVE and not state.STEAL_LOOP_RUNNING:
            state.STEAL_REPEAT_COUNT = 3000
            state.STEAL_LOOP_RUNNING = False
            state.STEAL_LOOP_ACTIVE = True
            strategy.get_logger().warning(
                "🏴‍☠️ Task-tracking emergency armed steal loop (STEAL_LOOP_ACTIVE=True, STEAL_REPEAT_COUNT=30)"
            )

        # Consume cancellation trigger so we only react once per invalidation event.
        scheduler.tracked_task_invalid = False

        time.sleep(0.05)

def robot_status(strategy, stm32, lidar):
    """Runs every 100ms to check STM32 movement status and lidar obstacles"""
    first_emergency = True
    
    strategy.get_logger().info("🤖 Robot status monitoring started")
    status_loop_count = 0
    stopped_by_failsafe=False
    while rclpy.ok() and not state.SHUTDOWN_REQUESTED:
        status_loop_count += 1
        
        # Only check if robot is idle when we think it's busy
        if not state.IDLE and not stopped_by_failsafe :
            if not state.IDLE_FOR_SYSTEMS and state.last_stm_command != "emergency":
                success, is_completed, _ = stm32.IDLE_FOR_SYSTEMS(1.0)
                #strategy.get_logger().info(f"idle for systems  : {is_completed}")
            else:
                success, is_completed, _ = stm32.IDLE(1.0)
                #strategy.get_logger().info(f"idle : {is_completed}")
            if is_completed and state.last_stm_command == "navigation":
                _ ,stopped_by_failsafe=stm32.fail_safe(1.0)    
                strategy.get_logger().info(f"failsafe result : {stopped_by_failsafe}")
            if not success:
                strategy.get_logger().warning("⚠️ Failed to check robot status")
            elif stopped_by_failsafe and state.last_stm_command == "navigation":
                strategy.get_logger().info("✅ Robot stopped by failsafe not updating idle to be stopped ")
            else:
                if not state.IDLE_FOR_SYSTEMS and not state.EMERGENCY_STOP:
                    state.PREVIOUS_IDLE_FOR_SYSTEMS=state.IDLE_FOR_SYSTEMS
                    state.IDLE_FOR_SYSTEMS = is_completed
                    
                    # Log state changes for IDLE_FOR_SYSTEMS
                    if state.IDLE_FOR_SYSTEMS != state.PREVIOUS_IDLE_FOR_SYSTEMS:
                        strategy.get_logger().info(f"🔄 IDLE_FOR_SYSTEMS changed: {state.PREVIOUS_IDLE_FOR_SYSTEMS} → {state.IDLE_FOR_SYSTEMS}")
                else:
                    old_idle = state.IDLE
                    state.IDLE = is_completed
                    
                    # Log state changes for IDLE
                    if state.IDLE != old_idle:
                        strategy.get_logger().info(f"🔄 IDLE changed: {old_idle} → {state.IDLE} (last_stm_command={state.last_stm_command})")   
            if  state.IDLE_FOR_SYSTEMS ==True and state.PREVIOUS_IDLE_FOR_SYSTEMS==False and state.last_stm_command!="emergency" :
                state.SYSTEMS_IDLE_COUNT+=1
            state.PREVIOUS_IDLE_FOR_SYSTEMS=state.IDLE_FOR_SYSTEMS
        #else:
            # Log periodically when robot is idle to confirm the thread is running
            # if status_loop_count % 100 == 0:  # Every 10 seconds
            #     strategy.get_logger().info(f"✓ Robot status monitor: IDLE={IDLE}, EMERGENCY_STOP={EMERGENCY_STOP}")        
        time.sleep(0.02)  # 20ms to reduce command dispatch latency

def lidar_status(strategy, stm32, lidar, round_timer):
    """Runs every 100ms to check lidar status and handle emergency stops"""
    first_emergency = True
    
    strategy.get_logger().info("🔍 Lidar status monitoring started")
    lidar_loop_count = 0
    
    while rclpy.ok() and not state.SHUTDOWN_REQUESTED:
        lidar_loop_count += 1
        
        # Update lidar with current round time
        elapsed_s = round_timer.time()
        lidar.set_round_time(elapsed_s)
        
        # Check emergency stop from lidar
        old_emergency = state.EMERGENCY_STOP
        state.EMERGENCY_STOP = lidar.getstatus()
        
        # Log when emergency state changes
        if state.EMERGENCY_STOP != old_emergency:
            if state.EMERGENCY_STOP:
                strategy.get_logger().warning(f"🚨 EMERGENCY_STOP activated by lidar! (first_emergency={first_emergency})")
            else:
                strategy.get_logger().info(f"✅ EMERGENCY_STOP cleared by lidar (first_emergency={first_emergency})")
        
        # Handle emergency stop
        if state.EMERGENCY_STOP and first_emergency:
            strategy.get_logger().warning("🚨 Emergency stop activated!")
            #success, response = stm32.stop()
            first_emergency = False
        
        # Reset flag when emergency is cleared
        if not state.EMERGENCY_STOP:
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
    
    # Check if this is a path-generated command (marked in params)
    from_pathfinding = False
    if isinstance(params, list) and len(params) > 0 and isinstance(params[-1], dict):
        # Last param is metadata dict
        metadata = params.pop()
        from_pathfinding = metadata.get('from_pathfinding', False)
    
    if command == 'move_forward':
        stm32.move_forward(params[0], params[1], params[2] if len(params) > 2 else 0.0)
        state.IDLE = False
        state.IDLE_FOR_SYSTEMS = False
    elif command == 'rotate':
        stm32.rotate(params[0], params[1] if len(params) > 1 else 0.0)
        state.IDLE = False
        state.IDLE_FOR_SYSTEMS = False
    elif command == 'go_to':
        lidar.set_goto_target(params[0]/1000, params[1]/1000, from_pathfinding=from_pathfinding)  # Convert mm to meters for lidar
        state.IDLE_FOR_SYSTEMS = False
        
        if len(params) > 3:
            stm32.go_to(params[0], params[1], params[2], params[3])
        else:
            stm32.go_to(params[0], params[1], params[2])
        state.IDLE = False
        
    elif command == 'orientate':
        stm32.orientate(params[0], params[1] if len(params) > 1 else 0.0)
        state.IDLE = False
        state.IDLE_FOR_SYSTEMS = False
    elif command == 'bezier':
        if len(params) > 9:
            stm32.bezier_curve(params[0], params[1], params[2], params[3], params[4], params[5], params[8], params[9])
        else:
            stm32.bezier_curve(params[0], params[1], params[2], params[3], params[4], params[5], params[8])
        state.IDLE = False
        state.IDLE_FOR_SYSTEMS = False
    
    elif command == 'go_to_backwards':
        # Publish target position for collision detection
        lidar.set_goto_target(params[0]/1000, params[1]/1000, from_pathfinding=from_pathfinding)  # Convert mm to meters for lidar
        
        if len(params) > 3:
            stm32.go_to_backwards(params[0], params[1], params[2], params[3])
        else:
            stm32.go_to_backwards(params[0], params[1], params[2])
        state.IDLE = False
        state.IDLE_FOR_SYSTEMS = False
    elif command == 'bezier_backwards':
        if len(params) > 9:
            stm32.bezier_curve_backwards(params[0], params[1], params[2], params[3], params[4], params[5], params[8], params[9])
        else:
            stm32.bezier_curve_backwards(params[0], params[1], params[2], params[3], params[4], params[5], params[8])
        state.IDLE = False
        state.IDLE_FOR_SYSTEMS = False
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
    
    # Wait for at least 2 position updates from beacon
    available_aruco=False 
    while strategy.latest_robot_pose_beacon_count <2 or not available_aruco and not state.SHUTDOWN_REQUESTED:
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
    # Import Vector3 for publishing target position
    from geometry_msgs.msg import Vector3
    # Variables to save last command for resuming after emergency
    last_command = None
    last_params = None
    was_in_emergency = False
    strategy.get_logger().info("🚀 STM32 task started")
    
    main_loop_count = 0
    while rclpy.ok() and not state.SHUTDOWN_REQUESTED:
        main_loop_count += 1
        
        # Log main loop status every 50 iterations (5 seconds)
        if main_loop_count % 50 == 0:
            strategy.get_logger().info(f"🔁 STM32 main loop #{main_loop_count}: queue_size={len(strategy.queues['STM'])}, IDLE={state.IDLE}, EMERGENCY_STOP={state.EMERGENCY_STOP}")
        
        # Check if there are commands in the queue and robot is idle.
        # In steal-only emergency flow, do not dispatch commands while emergency is active.
        if len(strategy.queues['STM']) > 0 and state.IDLE and not state.EMERGENCY_STOP and not strategy.end_game:
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
            state.last_stm_command=""
            
            # Execute command using handler function
            strategy.get_logger().info(f"🎯 Executing STM command: {command} with params: {params[:3] if len(params) > 3 else params}")
            while(command =="set_position" and state.WAITING_TIRETTE==True and not state.SHUTDOWN_REQUESTED):
                time.sleep(0.01)
            handle_stm32_command(command, params, stm32, lidar, strategy)
            state.last_stm_command="navigation"
            
            strategy.get_logger().info(f"⏳ Entering wait loop - IDLE={state.IDLE}, EMERGENCY_STOP={state.EMERGENCY_STOP}, last_stm_command={state.last_stm_command}")
            # Wait for command completion with emergency handling
            loop_iteration = 0
            while ((not state.IDLE and not state.SHUTDOWN_REQUESTED) or (state.EMERGENCY_STOP) or state.last_stm_command=="emergency") and state.last_stm_command != "replanned":
                loop_iteration += 1
                
                # Check for interrupt from strategy manager
                if strategy.interrupt_wait:
                    strategy.get_logger().warning("🛑 STM32 command wait interrupted - exiting loop")
                    strategy.stm32_task_wait_ended = True
                    break
                
                # Log state every 20 iterations (2 seconds)
                if loop_iteration % 20 == 0:
                    strategy.get_logger().info(f"🔄 Wait loop iteration {loop_iteration}: IDLE={state.IDLE}, EMERGENCY_STOP={state.EMERGENCY_STOP}, was_in_emergency={was_in_emergency}, replanning={state.replanning}, last_stm_command={state.last_stm_command}")

                # Emergency can be set by another thread (e.g. task_tracking_task),
                # in which case `was_in_emergency` in this STM thread remains False.
                # If robot is already idle and lidar emergency is clear, transition to
                # replanned state to avoid being stuck forever in the wait loop.
                if state.last_stm_command == "emergency" and (not state.EMERGENCY_STOP) and state.IDLE:
                    strategy.get_logger().info(
                        "✅ External emergency state resolved - switching STM wait loop to replanned"
                    )
                    state.last_stm_command = "replanned"
                    continue
                
                # Check for emergency stop
                if state.EMERGENCY_STOP and not was_in_emergency and not state.replanning :
                    strategy.get_logger().warning(f"🚨 Emergency detected during movement - stopping STM32 (IDLE={state.IDLE}, IDLE_FOR_SYSTEMS={state.IDLE_FOR_SYSTEMS})")
                    strategy.remove_esp_waiting=True
                    stm32.stop()
                    state.last_stm_command="emergency"
                    state.IDLE=False
                    state.IDLE_FOR_SYSTEMS=True
                    state.replanning = True

                    # Emergency mode is steal-only: drop static queued motion/actuator work.
                    strategy.queues['STM'].clear()
                    strategy.queues['ESP_front'].clear()
                    strategy.queues['ESP_back'].clear()
                    state.ESP_DISPATCH_EPOCH = int(getattr(state, 'ESP_DISPATCH_EPOCH', 0)) + 1
                    state.ESP_FRONT_IDLE = True
                    state.ESP_BACK_IDLE = True
                    strategy.get_logger().warning(
                        f"🧹 Cleared STM/ESP queues for steal-only emergency recovery (epoch={state.ESP_DISPATCH_EPOCH})"
                    )

                    # Motion command is canceled: clear stale motion targets so lidar does not
                    # keep EMERGENCY_STOP latched against an obsolete path.
                    if lidar is not None:
                        try:
                            lidar.clear_goto_target()
                        except Exception as e:
                            strategy.get_logger().warning(f"⚠️ Failed to clear lidar go-to target: {e}")
                    try:
                        strategy.pathfinder.clear_goal()
                    except Exception as e:
                        strategy.get_logger().warning(f"⚠️ Failed to clear pathfinder goal: {e}")

                    

                    was_in_emergency = True
                    strategy.get_logger().info("⏸️  Waiting for STM32 to report IDLE after emergency stop...")
                    lidar.start_enemy_collision_check()
                    idle_wait_count = 0
                    while(not state.IDLE and not state.SHUTDOWN_REQUESTED):
                        idle_wait_count += 1
                        if idle_wait_count % 10 == 0:  # Log every 0.5 seconds
                            strategy.get_logger().info(f"⏸️  Still waiting for IDLE... (count={idle_wait_count}, IDLE={state.IDLE})")
                        time.sleep(0.05)  # Wait until STM32 reports idle after stop
                    strategy.get_logger().info(f"✓ STM32 now IDLE after emergency stop (waited {idle_wait_count * 0.05:.2f}s)")
                    # Arm steal loop immediately from emergency stop.
                    if not state.STEAL_LOOP_ACTIVE and not state.STEAL_LOOP_RUNNING:
                        state.STEAL_REPEAT_COUNT = 3000
                        state.STEAL_LOOP_ACTIVE = True
                        strategy.get_logger().warning(
                            "🏴‍☠️ Emergency-triggered steal loop armed (STEAL_LOOP_ACTIVE=True, STEAL_REPEAT_COUNT=3000)"
                        )
                        
                    # Keep lidar collision watch active until path is clear.
                    # lidar.start_enemy_collision_check()  # Start waiting period before rechecking obstacle
                    # strategy.get_logger().info("🔍 Started enemy collision check - waiting for obstacle to clear")
                
                # Check if emergency cleared
                elif not state.EMERGENCY_STOP and was_in_emergency  :
                    strategy.get_logger().info("✅ Emergency cleared - switching to steal-generated execution")
                    was_in_emergency = False
                    # Never resume pre-emergency static command in steal-only mode.
                    state.last_stm_command = "replanned"
                
                time.sleep(0.1)
            
            # Exited wait loop
            strategy.get_logger().info(f"🏁 Exited wait loop - IDLE={state.IDLE}, EMERGENCY_STOP={state.EMERGENCY_STOP}, was_in_emergency={was_in_emergency}, last_stm_command={state.last_stm_command}")
            
            if  state.last_stm_command!="navigation": 
                strategy.get_logger().info(f"⚠️ Command ended with non-navigation state: idle={state.IDLE}, EMERGENCY_STOP={state.EMERGENCY_STOP}, last_stm_command='{state.last_stm_command}'")
                # Reset emergency flag when command completes normally
                was_in_emergency = False
                
                
                
                # Remove command from queue after execution
            if state.last_stm_command=="navigation":
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
            elif state.last_stm_command=="replanned":
                # After replanning, don't pop - let outer loop execute new commands
                strategy.get_logger().info(f"✓ Replanning complete - new commands ready to execute ({len(strategy.queues['STM'])} commands in queue)")
                # Don't pop from queue - commands were just added and need to be executed
                # Reset replanning flag now that we're back in main loop
                state.replanning = False
        else:
            # Log why we're not processing commands (every 50 iterations to avoid spam)
            if main_loop_count % 50 == 0:
                if len(strategy.queues['STM']) == 0:
                    strategy.get_logger().info("⏸️  STM queue empty - waiting for commands")
                elif not state.IDLE:
                    strategy.get_logger().info(f"⏸️  Robot not IDLE - waiting (IDLE={state.IDLE}, IDLE_FOR_SYSTEMS={state.IDLE_FOR_SYSTEMS}, EMERGENCY_STOP={state.EMERGENCY_STOP})")
                elif strategy.end_game:
                    strategy.get_logger().info("🏁 End-game active - STM dispatch blocked")
        
        time.sleep(0.1)  # 100ms
def handle_color_pattner(strategy,front):
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
            state.known_id_front = batch_known_id
            known_idd=batch_known_id
        else:
            state.known_id_back = batch_known_id
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
    waiting_started = False
    # Use shorter timeout to allow quicker shutdown response

    idle = controller.IDLE(timeout=5.0)
    if not waiting_started and idle:
        idle=False
    if not idle and waiting_started==False:
        waiting_started = True
        controller.get_logger().info("⏳ Waiting for controller to be idle...")
    while not idle and not state.SHUTDOWN_REQUESTED:
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
        state.color_pt = convert_color_pattern(color_pattern, robot_color)
        color_to_N20=int(str(state.color_pt).replace('0', '3').replace('1', '4'))
        print(f"Converted color pattern: {state.color_pt}, N20 command: {color_to_N20}")
        front_controller.n20(color_to_N20)
        return state.color_pt
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


def _should_abort_esp_action(strategy, front: bool, action_name: str) -> bool:
    """Abort ESP action immediately on shutdown or emergency and restore idle flags."""
    if not state.SHUTDOWN_REQUESTED and not state.EMERGENCY_STOP:
        return False

    if front:
        state.ESP_FRONT_IDLE = True
    else:
        state.ESP_BACK_IDLE = True

    strategy.get_logger().warning(
        f"⚠️ {action_name} aborted - SHUTDOWN_REQUESTED={int(bool(state.SHUTDOWN_REQUESTED))}, "
        f"EMERGENCY_STOP={int(bool(state.EMERGENCY_STOP))}"
    )
    return True


def _is_stale_esp_command(cmd) -> bool:
    if not isinstance(cmd, dict):
        return False
    cmd_epoch = int(cmd.get('_dispatch_epoch', getattr(state, 'ESP_DISPATCH_EPOCH', 0)))
    current_epoch = int(getattr(state, 'ESP_DISPATCH_EPOCH', 0))
    return cmd_epoch != current_epoch


def pick_up(controller,pneumatic,strategy,front):
    
    strategy.get_logger().info(f"🚀 pick_up() called for {'FRONT' if front else 'BACK'}")
    
    if _should_abort_esp_action(strategy, front, "pick_up()"):
        return
    #stana(controller)
    if _should_abort_esp_action(strategy, front, "pick_up()"):
        return
    
    if front:
        pneumatic.front(0)
    else: pneumatic.back(0)
    if _should_abort_esp_action(strategy, front, "pick_up()"):
        return

    controller.kabech(1)
    time.sleep(0.8)
    if _should_abort_esp_action(strategy, front, "pick_up()"):
        return

    controller.n20(1111)
    stana(controller)
    if _should_abort_esp_action(strategy, front, "pick_up()"):
        return

    time.sleep(0.2)
    if _should_abort_esp_action(strategy, front, "pick_up()"):
        return
    
    
    time.sleep(0.0)
    strategy.get_logger().info(f"🔍 Requesting NEW color pattern for {'FRONT' if front else 'BACK'} at position ({strategy.latest_robot_pose.x:.3f}, {strategy.latest_robot_pose.y:.3f})")
    color_pattern = talaa(strategy, controller,front)
    if _should_abort_esp_action(strategy, front, "pick_up()"):
        return

    if front:
        state.color_pt_front = color_pattern
        state.ESP_FRONT_IDLE=True
        _update_robot_holder_status(strategy, front=True, empty=False)
        strategy.get_logger().info(f"📦 Front picked up - NEW Color pattern: {state.color_pt_front}")
    else:
        state.color_pt_back = color_pattern
        state.ESP_BACK_IDLE=True
        _update_robot_holder_status(strategy, front=False, empty=False)
        strategy.get_logger().info(f"📦 Back picked up - NEW Color pattern: {state.color_pt_back}")
    stana(controller)
    if _should_abort_esp_action(strategy, front, "pick_up()"):
        return

    controller.kabech(0)
def put_down(controller,pneumatic,strategy,front,params=None):
    
    if state.SHUTDOWN_REQUESTED:
        return
    
    params = params if isinstance(params, dict) else {}
    use_putdown_angle = bool(params.get('use_putdown_angle', False))
    putdown_theta_deg_param = params.get('putdown_theta_deg', None)
    insert_offset_mm = float(
        params.get('insert_distance_mm', params.get('putdown_insert_distance_mm', 0.0)) or 0.0
    )

    color_pattern = state.color_pt_front if front else state.color_pt_back
    strategy.get_logger().info(f"color_pattern {color_pattern}")
    color_to_N20=int(str(color_pattern).replace('1', '4').replace('0','1'))
    strategy.get_logger().info(f"TO n20 command  {color_to_N20}")
    time.sleep(0.3)
    #controller.n20(color_to_N20)
    #stana(controller)
    if state.SHUTDOWN_REQUESTED:
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

        if use_putdown_angle:
            try:
                desired_theta_deg = float(putdown_theta_deg_param)
                if use_pose:
                    theta_deg = desired_theta_deg
                    strategy.get_logger().info(
                        f"🧭 put_down using TaskScheduler angle: {theta_deg:.1f}°"
                    )
                else:
                    strategy.get_logger().warning(
                        "⚠️ put_down angle requested but pose unavailable; cannot apply requested angle"
                    )
            except (TypeError, ValueError):
                strategy.get_logger().warning(
                    f"⚠️ Invalid put_down angle in params: {putdown_theta_deg_param}"
                )

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
        strategy.get_logger().info("Setting ESP_FRONT_IDLE to True in put_down")
        state.ESP_FRONT_IDLE=True
        _update_robot_holder_status(strategy, front=True, empty=True)
    else: 
        strategy.get_logger().info("Setting ESP_BACK_IDLE to True in put_down")
        state.ESP_BACK_IDLE=True
    if not front:
        _update_robot_holder_status(strategy, front=False, empty=True)
    
    if state.SHUTDOWN_REQUESTED:
        return
    
    controller.n20(0)
    
    stana(controller)
    if state.SHUTDOWN_REQUESTED:
        return
    time.sleep(0.5)
    if front:
        pneumatic.front(16)
    else: pneumatic.back(16)

    
def front_task(strategy, front_controller=None,pneumatic_controller=None):
    """Runs every 100ms to process ESP32_1 queue"""
    while rclpy.ok() and not state.SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue
        if len(strategy.queues['ESP_front']) > 0:
            cmd = strategy.queues['ESP_front'][0]  # Peek at first command

            if _is_stale_esp_command(cmd):
                if len(strategy.queues['ESP_front']) > 0 and strategy.queues['ESP_front'][0] is cmd:
                    strategy.queues['ESP_front'].popleft()
                state.ESP_FRONT_IDLE = True
                strategy.get_logger().warning("🗑️ Dropped stale ESP_front command from old epoch")
                time.sleep(0.05)
                continue

            if state.EMERGENCY_STOP:
                time.sleep(0.05)
                continue
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            if command == 'prepare':
                action = params.get('action')
                if action == 'pick_up':
                    front_controller.n20(2222)
                    front_controller.dawer(0)
                    strategy.get_logger().info(f"🔧 Front prepare for pick_up: n20(2222)")
                else:
                    cmd_to_n20=int(str(state.color_pt_front).replace('0','2').replace('1','4'))
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
    while rclpy.ok() and not state.SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue
        if len(strategy.queues['ESP_back']) > 0:
            cmd = strategy.queues['ESP_back'][0]  # Peek at first command

            if _is_stale_esp_command(cmd):
                if len(strategy.queues['ESP_back']) > 0 and strategy.queues['ESP_back'][0] is cmd:
                    strategy.queues['ESP_back'].popleft()
                state.ESP_BACK_IDLE = True
                strategy.get_logger().warning("🗑️ Dropped stale ESP_back command from old epoch")
                time.sleep(0.05)
                continue

            if state.EMERGENCY_STOP:
                time.sleep(0.05)
                continue
            
            command = cmd.get('command')
            params = cmd.get('params', {})
            wait = cmd.get('wait', False)
            
            if command == 'prepare':
                action = params.get('action')
                if action == 'pick_up':
                    back_controller.n20(2222)
                    back_controller.dawer(0)
                    strategy.get_logger().info(f"🔧 Back prepare for pick_up: n20(2222)")
                else:
                    cmd_to_n20=int(str(state.color_pt_back).replace('0','2').replace('1','4'))
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

    while rclpy.ok() and not state.SHUTDOWN_REQUESTED:
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
    while rclpy.ok() and not state.SHUTDOWN_REQUESTED:
        # Check if there are commands in the queue
        if len(strategy.queues['RASP_PI']) > 0:
            cmd = strategy.queues['RASP_PI'][0]  # Peek at first command
            
            command = cmd.get('command')
            params = cmd.get('params', [])
            wait = cmd.get('wait', False)
            
            # Execute RASP_PI commands
            if command == 'Wait for manual start':
                state.WAITING_TIRETTE=True
                if rpi_controller:
                    # Check if tirette is already inserted
                    tirette_status = rpi_controller.read_pin_17()
                    if tirette_status == 0:
                        strategy.get_logger().warning("⚠️ mafamech tirette ya maaahdi !")
                    while(tirette_status == 0):  
                        time.sleep(0.05)
                        tirette_status = rpi_controller.read_pin_17()
                    result = rpi_controller.wait_start()
                    stm32_controller.reset()
                    state.WAITING_TIRETTE=False
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
                    strategy.get_logger().info("🗺️  Updating map...")
                    strategy.pathfinder.update_map()
                    
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
                state.IDLE_FOR_SYSTEMS=True
                state.IDLE=True

            elif command == 'wait_end_game':
                # Wait until game time reaches 1.25s + duration
                duration = float(params[0]) if params else 0
                target_time = 85 + duration
                state.return_home_triggered=True
                strategy.get_logger().info(f"⏳ Waiting until game time reaches {target_time}s (85s + {duration}s)")
                
                while rclpy.ok() and not state.SHUTDOWN_REQUESTED:
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
                        state.STEAL_REPEAT_COUNT = repeat_count
                        state.STEAL_LOOP_ACTIVE = True
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