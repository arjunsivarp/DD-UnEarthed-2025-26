#!/usr/bin/env python3

import hub
import sys
import time

import color, motor, motor_pair, runloop
from hub import light_matrix, button, motion_sensor, light, port

# ----------------------------
# CONFIG / CONSTANTS
# ----------------------------
DEBUG = True                # master switch for logs
LOG_LEVEL = 2            # 0=ERROR, 1=INFO, 2=DEBUG (most verbose)
WHEEL_CIRCUMFERENCE = 19.2 # 19.2 cm
TURN_SPEED = 220            # tank turn speed
DRIVE_SPEED = 400        # default forward speed
REVERSE_SPEED = 400        # default reverse speed
STALL_WINDOW = 250        # ms window to test if encoder is changing
STALL_MIN_DELTA = 2        # min encoder delta (degrees) considered "moving"
TURN_TIMEOUT_PER_DEG = 12  # ms per deg; used to bound turn timeouts
DRIVE_TIMEOUT_PER_CM = 250  # ms per cm; used to bound drive timeouts

# Global Variables
#------------------------------
current_heading = 0

# UTILITY COMMAND FUNCTIONS
#----------------------------------------

# initialize motor and reset yaw
def do_init():
    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    i = 0
    while (hub.motion_sensor.stable() == False):
        i = i + 1
        # Use time.sleep_ms
        # to ensure it is synchronized
        time.sleep_ms(1)
        #hub.light_matrix.write(str(i))
        if i >= 100:
            break


# Return true if LEFT button is pressed
def is_left_button_pressed():
    return button.pressed(button.LEFT) > 0


# Return true if RIGHT button is pressed
def is_right_button_pressed():
    return button.pressed(button.RIGHT) > 0

def get_time_taken_in_seconds(start_time, end_time):
    return int(time.ticks_diff(end_time, start_time)/1000)

# ----------------------------
# LIGHTWEIGHT LOGGER
# ----------------------------
_log_counter = 0

def _ticks_ms():
    # Minimal ticks impl (no blocking)
    try:
        import time
        return time.ticks_ms()
    except:
        # Fallback: monotonically increasing (not real time)
        global _log_counter
        _log_counter += 10
        return _log_counter

def _lvl_ok(lvl):
    return DEBUG and lvl <= LOG_LEVEL

def log_info(msg, **fields):
    if _lvl_ok(1):
        if fields:
            print("[INFO " + str(_ticks_ms()) + "] " + str(msg) + " :: " + str(fields))
        else:
            print("[INFO " + str(_ticks_ms()) + "] " + str(msg))

def log_debug(msg, **fields):
    if _lvl_ok(2):
        if fields:
            print("[DBG " + str(_ticks_ms()) + "] " + str(msg) + " :: " + str(fields))
        else:
            print("[DBG " + str(_ticks_ms()) + "] " + str(msg))

def log_error(msg, **fields):
    if _lvl_ok(0):
        if fields:
            print("[ERR " + str(_ticks_ms()) + "] " + str(msg) + " :: " + str(fields))
        else:
            print("[ERR " + str(_ticks_ms()) + "] " + str(msg))


# ----------------------------
# UTILS / SENSORS
# ----------------------------
def degrees_for_distance(distance_cm):
    return int((distance_cm / WHEEL_CIRCUMFERENCE) * 360)

def _drive_deg_avg():
    try:
        a = motor.relative_position(port.A)
    except:
        a = 0
    try:
        b = motor.relative_position(port.B)
    except:
        b = a# if only A exists, fall back
    return (a + b) / 2.0

def _motor_deg(p):
    try:
        return motor.relative_position(p)
    except:
        return 0

async def _estimate_deg_per_sec(sample_ms=500):
    start_deg = _abs_motor_A()
    start_t = _ticks_ms()
    await _sleep(sample_ms)
    end_deg = _abs_motor_A()
    end_t = _ticks_ms()
    dt = max(1, end_t - start_t)# ms
    ddeg = abs(end_deg - start_deg)
    rate = (ddeg * 1000.0) / dt# deg/s
    log_info("estimate_deg_per_sec", rate_deg_s=rate, window_ms=dt)
    return rate


async def _compute_drive_timeout_ms(distance_cm, safety_mult=1.8, min_ms=1500):
    # Sample current encoder speed
    rate_deg_s = await _estimate_deg_per_sec(400)# ~0.4s sample
    if rate_deg_s < 1.0:
        # Fallback if the robot is barely moving during sample
        fallback = int(max(min_ms, distance_cm * 300))# 300 ms/cm conservative
        log_error("drive_timeout:fallback", rate_deg_s=rate_deg_s, timeout_ms=fallback)
        return fallback

    target_deg = degrees_for_distance(distance_cm)
    # time (s) = degrees / (deg/s); convert to ms, add safety multiplier
    t_ms = int((target_deg / rate_deg_s) * 1000.0 * safety_mult)
    t_ms = max(t_ms, min_ms)
    log_info("drive_timeout:computed", target_deg=target_deg, rate_deg_s=rate_deg_s, timeout_ms=t_ms)
    return t_ms

def _angle_error(current_deg, target_deg):
    # Returns signed error in [-180, +180] so the robot takes the shortest turn
    e = current_deg - target_deg
    while e > 180:
        e -= 360
    while e < -180:
        e += 360
    return e

def _abs_motor_A():
    return abs(_motor_deg(port.A))

def _abs_motor_B():
    return abs(_motor_deg(port.B))

def _raw_yaw():
    # Typical SPIKE transform; adjust if your runtime differs
    return motion_sensor.tilt_angles()[0] * -0.1

def _state_snapshot(label):
    snap = {
        "label": label,
        "yaw": _raw_yaw(),
        "mA": _motor_deg(port.A),
        "mB": _motor_deg(port.B),
    }
    log_debug("STATE", **snap)
    return snap

async def _sleep(ms):
    await runloop.sleep_ms(ms)

def _safe_stop(hold=True):
    # Use BRAKE by default to reduce coasting overshoot
    stop_mode = motor.BRAKE
    try:
        motor_pair.stop(motor_pair.PAIR_1, stop=stop_mode)
    except:
        pass


# ----------------------------
# FOLLOW-FOR (DISTANCE PREDICATE)
# ----------------------------
def follow_for_distance(initial_position=0, distance_to_cover=0):
    current_position = _abs_motor_A()
    distance_covered = current_position - initial_position
    if distance_covered < 0:
        distance_covered = -distance_covered
    return distance_covered < abs(distance_to_cover)

def follow_for_distance_raw(initial_deg=0, target_deg=0):
    # Measure raw delta from the starting snapshot; no abs() per-sample!
    current = _drive_deg_avg()
    delta = current - initial_deg
    if delta < 0:
        delta = -delta
    return delta < (target_deg if target_deg >= 0 else -target_deg)


# ----------------------------
# PID GYRO FOLLOW (ASYNC)
# ----------------------------
async def follow_gyro_angle(kp, ki, kd, speed, target_angle, sleep_time, follow_for, timeout_ms=None, **kwargs):
    log_info("follow_gyro_angle:start", kp=kp, ki=ki, kd=kd, speed=speed, target=target_angle, sleep=sleep_time)
    start = _ticks_ms()
    integral = 0.0
    last_error = 0.0

    # Stall detection vars
    last_check = _ticks_ms()
    last_enc = _abs_motor_A()

    while follow_for(**kwargs):
        # Timeout guard
        if timeout_ms is not None and (_ticks_ms() - start) > timeout_ms:
            log_error("follow_gyro_angle:timeout", timeout_ms=timeout_ms)
            break

        current_angle = _raw_yaw()
        #error = current_angle - target_angle
        error = _angle_error(current_angle, target_angle)
        integral += error
        derivative = error - last_error
        last_error = error
        steering_value = int((error * kp) + (integral * ki) + (derivative * kd))

        motor_pair.move(motor_pair.PAIR_1, steering_value, velocity=int(speed))

        # Stall check
        now = _ticks_ms()
        if (now - last_check) >= STALL_WINDOW:
            enc = _abs_motor_A()
            delta = abs(enc - last_enc)
            log_debug("drive:stall_check", encA=enc, delta=delta, window_ms=(now - last_check))
            if delta < STALL_MIN_DELTA and abs(speed) > 0:
                log_error("drive:possible_stall", delta=delta)
            last_check = now
            last_enc = enc

        if sleep_time:
            await _sleep(sleep_time)

    _safe_stop(hold=True)
    log_info("follow_gyro_angle:end", elapsed_ms=_ticks_ms() - start)

# ----------------------------
# TURNING
# ----------------------------
async def _pivot_turn_abs(angle_deg, tank_speed=220):
    """Positive angle = RIGHT, Negative = LEFT. Turns relative to current yaw."""
    if angle_deg == 0:
        log_info("turn:skip_zero")
        return

    start_yaw = _raw_yaw()# capture starting yaw
    target = abs(angle_deg)
    direction = "RIGHT" if angle_deg > 0 else "LEFT"

    left = tank_speed if angle_deg > 0 else -tank_speed
    right = -tank_speed if angle_deg > 0 else tank_speed

    motor_pair.move_tank(motor_pair.PAIR_1, left, right)
    log_info("turn:start", direction=direction, target_deg=target)

    while True:
        delta = abs(_raw_yaw() - start_yaw)
        if delta >= target:
            break
        await _sleep(10)

    _safe_stop(hold=True)
    log_info("turn:done", actual_delta=abs(_raw_yaw() - start_yaw))


# ----------------------------
# HIGH-LEVEL ACTIONS
# ----------------------------
async def runFwd(distance_cm, speed=DRIVE_SPEED, kp=1.2, kd=0.25, ki=0.0):
    log_info("runFwd:start", cm=distance_cm, speed=speed)
    pre = _state_snapshot("runFwd:pre")

    init_deg = _drive_deg_avg()
    target_deg = degrees_for_distance(distance_cm)
    log_debug("drive:target_deg", target_deg=target_deg)

    # NEW: hold the current heading, not 0
    hold_heading = _raw_yaw()
    log_debug("drive:hold_heading", heading=hold_heading)

    # timeout policy (keep yours; adaptive is best if you added it)
    timeout_ms = await _compute_drive_timeout_ms(distance_cm) if '_compute_drive_timeout_ms' in globals() else max(1500, int(distance_cm * DRIVE_TIMEOUT_PER_CM))

    await follow_gyro_angle(kp=kp, ki=ki, kd=kd,
                            speed=-abs(speed),        # keep your forward sign fix
                            target_angle=hold_heading,# <<— lock to current heading
                            sleep_time=10,
                            follow_for=follow_for_distance_raw,
                            timeout_ms=timeout_ms,
                            initial_deg=init_deg,
                            target_deg=target_deg)

    post = _state_snapshot("runFwd:post")
    delta_deg = (_drive_deg_avg() - init_deg)
    if delta_deg < 0:
        delta_deg = -delta_deg
    measured_cm = (delta_deg / 360.0) * WHEEL_CIRCUMFERENCE
    log_info("runFwd:summary", requested_cm=distance_cm, measured_cm=measured_cm, delta_deg=delta_deg)


async def runRev(distance_cm, speed=REVERSE_SPEED, kp=-1.2, kd=-0.25, ki=0.0):
    log_info("runRev:start", cm=distance_cm, speed=speed)
    pre = _state_snapshot("runRev:pre")

    init_deg = _drive_deg_avg()
    target_deg = degrees_for_distance(distance_cm)
    log_debug("drive:target_deg", target_deg=target_deg)

    # NEW: hold the current heading, not 0
    hold_heading = _raw_yaw()
    log_debug("drive:hold_heading", heading=hold_heading)

    timeout_ms = await _compute_drive_timeout_ms(distance_cm) if '_compute_drive_timeout_ms' in globals() else max(1500, int(distance_cm * DRIVE_TIMEOUT_PER_CM))

    await follow_gyro_angle(kp=kp, ki=ki, kd=kd,
                            speed=abs(speed),            # keep your reverse sign fix
                            target_angle=hold_heading,# <<— lock to current heading
                            sleep_time=10,
                            follow_for=follow_for_distance_raw,
                            timeout_ms=timeout_ms,
                            initial_deg=init_deg,
                            target_deg=target_deg)

    post = _state_snapshot("runRev:post")
    delta_deg = (_drive_deg_avg() - init_deg)
    if delta_deg < 0:
        delta_deg = -delta_deg
    measured_cm = (delta_deg / 360.0) * WHEEL_CIRCUMFERENCE
    log_info("runRev:summary", requested_cm=distance_cm, measured_cm=measured_cm, delta_deg=delta_deg)


async def turnRight(angle_deg, tank_speed=TURN_SPEED):
    log_info("turnRight:requested", deg=angle_deg, speed=tank_speed)
    await _pivot_turn_abs(abs(angle_deg), tank_speed=tank_speed)
    log_info("turnRight:done")

async def turnLeft(angle_deg, tank_speed=TURN_SPEED):
    log_info("turnLeft:requested", deg=angle_deg, speed=tank_speed)
    await _pivot_turn_abs(-abs(angle_deg), tank_speed=tank_speed)
    log_info("turnLeft:done")

# ----------------------------
# TEST SEQUENCES
# ----------------------------
async def runTest():
    log_info("==== TEST START ====")
    await runFwd(20, 500)
    await turnRight(50)
    await turnLeft(30)
    await runRev(20, 500)
    log_info("==== TEST END ====")
 

# END UTILITY FUNCTIONS
#----------------------------------------

#----------------------------------------
# TEST FUNCTIONS
#----------------------------------------
async def run94():
    # Keeps your original entrypoint name, calls the same sequence
    await runTest()

async def run95():
    print("RUN 95 - Test 1 - run for 30 cm")
    await runFwd(30,500)
    print("RUN 95 - Test 2 - run for 30 cm with Target angle 90")

            
async def run96():
    print("RUN 96 - Test 1 - run for 30 cm backwards")
    await runRev(30,500)
    print("RUN 96 - Test 2 - run for 30 cm backwards with Target angle 90")


async def run97():
    print("RUN 97 - Test 1 - Turn right 90 deg")
    await turnRight(90)
    # turn to align to a mission
    #await pivot_gyro_turn_abs(left_speed=300, right_speed=-300, angle=90, stop=True)
    #await turnRight(8)

async def run98():
    print("RUN 98 - Test 1 - Turn Left 90 deg")
    await turnLeft(90)  

async def run99():
    print("RUN 99 - Test 1 - run for 200 cm")
    await runFwd(200,500)

#----------------------------------------
# RUN 1
# Run 1 - M11 (Angler Artifacts) and M12 (Salvage Operation); M15
#----------------------------------------
# run 1 program
async def run1():

    print("RUN 1")
    await runFwd(48,500)

    await motor.run_for_degrees(port.F, 300, -900)
    #await runloop.sleep_ms(500)

    await runRev(5,100)

    motor.run_for_degrees(port.F, 250, 900)

    await runFwd(12,500)

    for i in range(10):
        await motor.run_for_degrees(port.D, 100, -900)
        await motor.run_for_degrees(port.D, 500, -900)

    await runRev(12,100)
    await runRev(55,500)

# END RUN 1
#----------------------------------------

#----------------------------------------
# RUN 2 - Surface Brushing
# Run 2 - M1 (Surface Brushing)
#----------------------------------------
# run 2 program
async def run2():
    print("RUN 2")
    await runFwd(62.5,500)
    await runRev(15,500)
    await runFwd(9,500)
    await runRev(5,500)

    #bring the sliding arm closer to squeeze the brush
    motor.reset_relative_position(port.D, 0)
    #light_matrix.write(str(motor.relative_position(port.D)) )
    await motor.run_to_relative_position(port.D,-375,200)
    #light_matrix.write(str(motor.relative_position(port.D)) )

    await runRev(10,100)
    #await turnRight(45)
    #await runFwd(10,500)
    #await turnLeft(45)
    await runRev(45,500)


# END RUN 2
#----------------------------------------

#----------------------------------------
# RUN 3
# Run 3 - M2 (Map Reveal)
#----------------------------------------
# run 3 program
async def run3():
    print("RUN 3")
    angleToTract = 0
    await runFwd(70,500)
    await turnLeft(41,100)
    angleToTract = angleToTract + (-40)
    await runFwd(12,100,angleToTract)
    await runRev(3,100, angleToTract)

    print ("here1 " + str(angleToTract))
    #lift arm up
    #await motor.run_for_degrees(port.F, 220, 100)
    motor.reset_relative_position(port.F, 0)
    await motor.run_to_relative_position(port.F,220,200)

    print ("here2 " + str(angleToTract))
    await runRev(15,100, angleToTract)
    print ("here3 " + str(angleToTract))
    angleToTract = angleToTract + (-120)
    print ("here4 " + str(angleToTract))
    await turnLeft(140,100)
    await runFwd(60,500,angleToTract)

# END RUN 3
#----------------------------------------


#----------------------------------------
# RUN 4
# Run 4 - M3 (Mineshaft Explorer) and M4 (Careful recovery); M15
#----------------------------------------
# run 4 program
async def run4():
    print("RUN 4")


# END RUN 4
#----------------------------------------

#----------------------------------------
# RUN 5
# Run 5 - M5 (Who Lived Here) and M6 (Forge) ; M7 (Heavy Lifting) , M8 (Silo)
#----------------------------------------
# run 5 program
async def run5():

    print("RUN 5")
    await runFwd(35,200)
    for i in range(4):
        await motor.run_for_degrees(port.F, 120, 500)
        await motor.run_for_degrees(port.F, -120, 200)

    await runRev(35,200)


# END RUN 5
#----------------------------------------

#----------------------------------------
# RUN 6
# Run 6 - M9 (Whats on Sale) and M10 (Tip the Scales); M15
#----------------------------------------
# run 6 program
async def run6():
    print("RUN 6")

    await runFwd(50,200)
    await turnLeft(43)
    await runFwd(10,200)
    await turnLeft(8,200)
    await runRev(60, 1000)
    await runloop.sleep_ms(500)
    await runFwd(10,200)
    await turnRight(10)
    await runRev(25)
    #await turnLeft(90)
    await runRev(30)

# END RUN 6
#----------------------------------------

#----------------------------------------
# RUN 7
# Run 7 -M13 (Statue rebuild) and M14 (Forum)
#----------------------------------------
# run 7 program
async def run7():
    print("RUN 7")
    motor.run_for_degrees(port.F, 120, 100)
#    await runloop.sleep_ms(500)
    motor.run_for_degrees(port.F, 120, -100)

# END RUN 7
#----------------------------------------

# MAIN EXECUTE FUNCTION
#----------------------------------------

async def execute(run_numbers=None):

    runs_to_execute = list()

    if isinstance(run_numbers, int):
        run_numbers = [run_numbers]

    # If run_numbers are not provided execute all runs
    runs_to_execute = run_numbers if run_numbers else [1, 2, 3, 4, 5, 6, 7, 94, 95, 96, 97, 98, 99]

    start_times = [time.ticks_ms() for _ in runs_to_execute]
    end_times = [time.ticks_ms() for _ in runs_to_execute]

    run_functions_map = {
                            1: run1, # or run1a
                            2: run2,
                            3: run3,
                            4: run4,
                            5: run5,
                            6: run6,
                            7: run7,
                            94: run94, #Test Movements
                            95: run95, #Go Forward
                            96: run96,  #Go Reverse
                            97: run97,  # Turn Right
                            98: run98,  # Turn Left
                            99: run99   # Run from one end to another.
                        }
    print("######## Start - Daring Unearthed Runs ########")

    # Initialization
    # Define motor pair for robot movements
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)

    do_init()
    #light_matrix.write("1")
    light.color(light.POWER, color.RED)

    for i, run_number in enumerate(runs_to_execute):

        # waiting for left button to be pressed to start the run
        await runloop.until(is_left_button_pressed)
        print("Starting Run: " + str(run_number))

        light.color(light.POWER, color.GREEN)
        light_matrix.write(str(run_number))

        start_times[i] = time.ticks_ms()
        do_init()

        runloop.run(run_functions_map[run_number]())
        end_times[i] = time.ticks_ms()
        light.color(light.POWER, color.YELLOW)

        if i > 0:
            print("Transition time: " + str(get_time_taken_in_seconds(end_times[i - 1], start_times[i])) + " s")
        print("Run " + str(run_number) + " time " + str(get_time_taken_in_seconds(start_times[i], end_times[i])) + " s")
        print("---------------------------------------------------------------------------")

    # Print execution times
    print("---------------------------------------------------------------------------")
    print("SUMMARY:")
    total_runs_time = 0
    total_transitions_time = 0
    total_time = 0

    for i, run_number in enumerate(runs_to_execute):
        if i > 0:
            transition_time = get_time_taken_in_seconds(end_times[i - 1], start_times[i])
            print("Transition time: " + str(transition_time) + " s")
            total_transitions_time += transition_time
            total_time += transition_time

        run_time = get_time_taken_in_seconds(start_times[i], end_times[i])
        print("Run " + str(run_number) + " time " + str(run_time) + " s")
        total_runs_time += run_time
        total_time += run_time

    print("***************************************************************************")

    print("TOTAL RUN TIME = " + str(total_runs_time) + " s")
    print("TOTAL TRANSITIONS TIME = " + str(total_transitions_time) + " s")
    print("TOTAL TIME = " + str(total_transitions_time + total_runs_time) + " s")

    print("***************************************************************************")


# END MAIN EXECUTE FUNCTION
#----------------------------------------
##### Testing Runs ##########
# Go Forward- Run - 95
# Go Reverse- Run - 96
# Turn Right- Run - 97
# Turn Left- Run - 98
# Run from one end to another. - Run - 99
#----------------------------------------
####### Final Mission Runs ###########
# Run 1 - M11 (Angler Artifacts) and M12 (Salvage Operation); M15
# Run 2 - M1 (Surface Brushing)  
# Run 3 - M2 (Map Reveal)
# Run 4 - M3 (Mineshaft Explorer) and M4 (Careful recovery); M15
# Run 5 - M5 (Who Lived Here) and M6 (Forge) ; M7 (Heavy Lifting) , M8 (Silo)
# Run 6 - M9 (Whats on Sale) and M10 (Tip the Scales); M15
# Run 7 - M13 (Statue rebuild) and M14 (Forum)

############### Chained Runs ###############

# SLOT 0 - All Runs#
#runloop.run(execute([1, 2, 3, 4, 5, 6]))

# SLOT 1 - Run 1 Onwards
runloop.run(execute([1]))

# SLOT 2 - Run 2 Onwards
# runloop.run(execute([2]))

# SLOT 3 - Run 3 Onwards
# runloop.run(execute([3]))

# SLOT 4 - Run 4 Onwards
# runloop.run(execute([4]))

# SLOT 5 - Run 5
# runloop.run(execute([5]))

# SLOT 6 - Run 6
# runloop.run(execute([6]))

# SLOT 19 - Run 99
# runloop.run(execute([99]))

# SLOT Trials
#runloop.run(execute([0]))

