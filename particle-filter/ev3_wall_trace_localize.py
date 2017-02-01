
# Credits: Some code snippets shamelessly lifted from class exercises in Udacity CS373 (Artificial Intelligence for Robotics)

import time
from math import *
import random


saved_measurements = []
saved_steps = []
saved_real_starts = []
saved_real_ends = []

# Sonar measurement data from real robot runs
#
# Data Set #1:
saved_steps.append(25)
saved_measurements.append([[25.9, 74.8], [27.1, 57.2], [26.7, 39.1], [25.7, 20.9], [18.9, 200.0], [20.4, 200.0], [32.6, 200.0], [77.3, 200.0], [37.6, 200.0], [44.5, 98.4], [33.6, 70.2], [27.8, 53.2], [23.7, 34.5], [30.3, 76.2], [27.3, 59.7], [23.5, 41.0], [19.8, 22.7], [19.1, 200.0], [18.9, 198.3], [19.1, 179.4], [32.6, 200.0], [121.0, 145.9], [37.4, 200.0], [43.5, 149.5], [32.2, 128.2], [24.7, 128.2]])
saved_real_starts.append([171.4, 313.0, 0.0])
saved_real_ends.append([251.0, 111.0, 5.11])


# Data Set #2:
saved_steps.append(27)
saved_measurements.append([[25.9, 74.8], [26.7, 56.7], [26.7, 38.4], [25.9, 20.4], [18.4, 200.0], [20.7, 200.0], [32.6, 200.0], [77.4, 200.0], [38.3, 200.0], [27.1, 81.2], [25.1, 65.3], [24.3, 47.4], [22.7, 29.0], [25.9, 75.4], [25.9, 58.2], [25.1, 39.9], [24.2, 21.1], [19.0, 200.0], [20.9, 192.7], [29.0, 175.2], [190.2, 157.3], [32.6, 200.0], [38.4, 200.0], [29.0, 133.0], [26.3, 115.2], [22.8, 98.6], [19.5, 79.6], [17.0, 62.2]])
saved_real_starts.append([171.4, 313.0, 0.0])
saved_real_ends.append([257.0, 67.0, 4.83])



#################
# USER SETTINGS #
#################

run_steps = 25 # only applies to real robot
single_particle_mode = False # debugging mode with a single particle, uses known starting point (avoids running full particle filter)
real_robot_mode = False # run on real robot rather than simulating with logged robot data
active_set = 1 # active data set (see above) for simulation mode
motion_noise_on = True # motion noise
grab_target = True # go to target on real robot runs (must stay in place when gathering data for subsequent simulation)
known_starting_orientation = False # simulate compass
pf_number_particles = 500 # number of particles in particle filter
sensor_noise_left = 10.0 # left sensor noise
sensor_noise_front = 15.0 # front sensor noise (front sensor tends to be much further from wall on average)
base_steering_noise = 0.03 # steering noise (orientation noise)
base_distance_noise = 5.0 # distance noise (position noise)
base_turning_noise = 0.05 # turning noise
robust_likelihood_constant = 0.000001 # avoids being overly aggressive on particle killing


target_position = (188, 290)

Kp = 0.05 # wall follower PD controller Kp constant
Kd = 0.1 # wall follower PD controller Kd constant
base_power = 30.0 # base wheel power / 10 (degrees/s)
power_multiplier = 0.20 # constrains max/min wheel power levels
target_wall_dist = 25.4 # (cm)
wheel_diameter = 6.6 # (cm)
wheel_circumference = pi * wheel_diameter # (cm)
dist_between_wheels = 11.4 # (cm)

sonar_max_distance = 200.0 # readings beyond this distance are unreliable (cm)
sonar_max_angle = 25.0 * pi / 180.0 # sonar cone: +/- 25 (deg)

dist_front_sensor_to_center = 10.0 # offset from robot center (midpoint between wheels) to front sensor (cm)
dist_left_sensor_to_center = 10.0 # offset from robot center (midpoint between wheels) to left sensor (cm)
left_sensor_orientation_offset = pi / 2.0 # left sensor offset from front (rad)

world_size_x = 361 # (cm)
world_size_y = 349 # (cm)

if motion_noise_on:
    steering_noise = base_steering_noise
    distance_noise = base_distance_noise
    turning_noise = base_turning_noise
else:
    steering_noise = 0.0
    distance_noise = 0.0
    turning_noise = 0.0

if real_robot_mode:
    from ev3.lego import Motor
    from ev3.lego import UltrasonicSensor
    from ev3.lego import ColorSensor
    from ev3.ev3dev import Tone
    a = Motor(port=Motor.PORT.A) # left large motor
    d = Motor(port=Motor.PORT.D) # right large motor
    a.reset()
    d.reset()
    a.position_mode=Motor.POSITION_MODE.RELATIVE
    d.position_mode=Motor.POSITION_MODE.RELATIVE
    sonar_left = UltrasonicSensor(port=2)
    sonar_front = UltrasonicSensor(port=3)
    color_left = ColorSensor(port=1)
    color_right = ColorSensor(port=4)
    sound = Tone()


# Occupancy grid - checks for available positions
class Grid:

    def __init__(self, rows, cols):
        self.obstacles = []
        self.grid = []
        for row in xrange(rows): self.grid += [[0]*cols]

    def add_obstacle(self, row_start, row_end, col_start, col_end):
        for row in xrange(row_start, row_end + 1):
            for col in xrange(col_start, col_end + 1):
                self.grid[row][col] = 1

    def is_available(self, x, y):

        if (0 <= x < world_size_x) and (0 <= y < world_size_y):
            if self.grid[y][x] == 0:
                return True

        return False

    def print_grid(self):
        rows = len(self.grid)
        cols = len(self.grid[0])
        print "[ ",
        for row in reversed(xrange(rows)):
            if row >= 0: print "\n  ",
            print "[ ",
            for col in xrange(cols):
                if col > 0: print ",",
                print str(self.grid[row][col]),
            print "]",
        print "]"


# Robot simulator (particle)
class robot_sim:

    def __init__(self):
        if single_particle_mode: # known initial position for single particle mode
            self.x = saved_real_starts[active_set - 1][0]
            self.y = saved_real_starts[active_set - 1][1]
            self.orientation = saved_real_starts[active_set - 1][2]
        else: # random initial position
            self.x = random.random() * (world_size_x - 1)
            self.y = random.random() * (world_size_y - 1)
            if known_starting_orientation: # known initial orientation only
                self.orientation = saved_real_starts[active_set - 1][2]
            else:
                self.orientation = random.choice([0.0, pi / 2, pi, 3 * pi / 2]) # initial random choice of orientation for particle filter

        self.sensor_noise_left = 0.0
        self.sensor_noise_front = 0.0
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.turning_noise = 0.0

    def set(self, new_x, new_y, new_orientation):

        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_sensor_noise_left, new_sensor_noise_front, new_steering_noise, new_distance_noise, new_turning_noise):

        self.sensor_noise_left = float(new_sensor_noise_left)
        self.sensor_noise_front = float(new_sensor_noise_front)
        self.steering_noise = float(new_steering_noise)
        self.distance_noise = float(new_distance_noise)
        self.turning_noise = float(new_turning_noise)

    def find_closest_wall(self, walls, sensor):
    # Find closest wall to specified sensor, within sonar cone

        orientation_offset = 0.0
        sensor_dist_offset = 0.0

        if sensor == 'left':
            orientation_offset = left_sensor_orientation_offset
            sensor_dist_offset = dist_left_sensor_to_center

        elif sensor == 'front':
            sensor_dist_offset = dist_front_sensor_to_center

        close_walls = []

        for wall in walls:

            on_line_segment = False

            x1 = wall[0]
            y1 = wall[1]
            x2 = wall[2]
            y2 = wall[3]

            # angle between sensor and wall (rad)
            angle_to_wall = acos((cos(self.orientation + orientation_offset) * (y1 - y2) + sin(self.orientation + orientation_offset) * (x2 - x1)) / distance_between((x1,y1), (x2,y2)))

            # check that we don't exceed the sonar cone
            if abs(angle_to_wall) > sonar_max_angle:
                continue

            # accommodate differences between sensor mount positions and robot center (mid-point between wheels)
            sensor_x = self.x + sensor_dist_offset * cos(self.orientation + orientation_offset)
            sensor_y = self.y + sensor_dist_offset * sin(self.orientation + orientation_offset)

            # forward vector from sensor to wall (cm)
            dist_to_wall = ((y2 - y1) * (x1 - sensor_x) - (x2 - x1) * (y1 - sensor_y)) / ((y2 - y1) * cos(self.orientation + orientation_offset) - (x2 - x1) * sin(self.orientation + orientation_offset))

            # must be *forward* vector only
            if dist_to_wall < 0:
                continue

            # if distance is beyond sonar range, ignore this wall
            if dist_to_wall > sonar_max_distance:
                continue

            # intercept point on wall based on following forward vector from sensor
            x_intercept_point = sensor_x + dist_to_wall * cos(self.orientation + orientation_offset)
            y_intercept_point = sensor_y + dist_to_wall * sin(self.orientation + orientation_offset)

            # check that intercept point is within the endpoints of the wall
            if (x1 - x2) == 0:
                if (y1 <= y_intercept_point <= y2) or (y2 <= y_intercept_point <= y1):
                    on_line_segment = True

            elif (y1 - y2) == 0:
                if (x1 <= x_intercept_point <= x2) or (x2 <= x_intercept_point <= x1):
                    on_line_segment = True

            elif ((x1 <= x_intercept_point <= x2) or (x2 <= x_intercept_point <= x1)) and ((y1 <= y_intercept_point <= y2) or (y2 <= y_intercept_point <= y1)):
                on_line_segment = True

            if not on_line_segment:
                continue

            # everything looks good, add wall as a candidate
            close_walls.append(dist_to_wall)

            if single_particle_mode:
                print 'Sim - Found valid wall: ', wall
                print 'Sim - Angle to wall: ', angle_to_wall
                print 'Sim - Distance to wall: ', dist_to_wall
                print 'Sim - Wall intercept point: ', x_intercept_point, y_intercept_point

        if not close_walls: # return max sonar distance if no walls (avoids high measurement errors for walls further than max sonar distance)
            if single_particle_mode:
                print 'Sim - Sensor dist, ', sensor, ':', sonar_max_distance
            return sonar_max_distance
        else:
            # choose the closest viable wall
            if single_particle_mode:
                print 'Sim - Sensor dist, ', sensor, ':', min(close_walls)
            return min(close_walls)


    def move_time(self, pwr_l, pwr_r, duration):
    # Move in a straight line or arc, for specified time in ms, depending on left/right motor power levels (differential drive)

        result = robot_sim()
        result.sensor_noise_left = self.sensor_noise_left
        result.sensor_noise_front = self.sensor_noise_front
        result.steering_noise = self.steering_noise
        result.distance_noise = self.distance_noise
        result.turning_noise = self.turning_noise

        velocity_left = pwr_l * 10 * wheel_circumference / 360.0 # (cm/s)
        velocity_right = pwr_r * 10 * wheel_circumference / 360.0 # (cm/s)

        x = 0.0
        y = 0.0
        orientation = 0.0

        if velocity_left == velocity_right: # going straight
            # print 'Sim - going straight...'
            orientation = (self.orientation + random.gauss(0.0, self.steering_noise)) % (2 * pi) # add steering noise to orient
            dist = velocity_right * duration / 1000.0 # distance
            x = self.x + dist * cos(orientation) + random.gauss(0.0, self.distance_noise) # add distance noise to x
            y = self.y + dist * sin(orientation) + random.gauss(0.0, self.distance_noise) # add distance noise to y

        else:
            # print 'Sim - slight turn...'
            R = dist_between_wheels * (velocity_left + velocity_right) / (2 * (velocity_right - velocity_left)) # radius of arc
            beta = (velocity_right - velocity_left) * (duration / 1000.0) / dist_between_wheels # angle moved in arc
            orientation = (self.orientation + beta + random.gauss(0.0, self.steering_noise)) % (2 * pi) # add steering noise to orient
            cx = self.x - sin(self.orientation) * R # circle center x position
            cy = self.y + cos(self.orientation) * R # circle center y position
            x = cx + sin(self.orientation + beta) * R + random.gauss(0.0, self.distance_noise) # add distance noise to x
            y = cy - cos(self.orientation + beta) * R + random.gauss(0.0, self.distance_noise) # add distance noise to y

        result.set(x, y, orientation)

        if single_particle_mode:
            print 'Sim - straight or slight turn...'
            print 'Sim - new x: ', x
            print 'Sim - new y: ', y
            print 'Sim - new orientation (rad): ', orientation
            print 'Sim - new orientation (deg): ', orientation * 180 / pi

        return result

    def turn_in_place(self, turn_angle):
    # Turn in place (velocity_right = -velocity_left) by specified degrees

        x = 0.0
        y = 0.0
        orientation = 0.0
        result = robot_sim()
        result.sensor_noise_left = self.sensor_noise_left
        result.sensor_noise_front = self.sensor_noise_front
        result.steering_noise = self.steering_noise
        result.distance_noise = self.distance_noise
        result.turning_noise = self.turning_noise

        orientation = (self.orientation + (turn_angle * pi / 180) + random.gauss(0.0, self.turning_noise)) % (2 * pi)
        x = self.x
        y = self.y
        result.set(x, y, orientation)

        if single_particle_mode:
            print 'Sim - turn in place...'
            print 'Sim - new x: ', x
            print 'Sim - new y: ', y
            print 'Sim - new orientation (rad): ', orientation
            print 'Sim - new orientation (deg): ', orientation * 180 / pi

        return result


    def measurement_prob(self, measurements):
    # Calculate measurement probability

        predicted_measurements = []
        predicted_measurements.append(self.find_closest_wall(walls, 'left'))
        predicted_measurements.append(self.find_closest_wall(walls, 'front'))

        # compute left sensor gaussian error - use sensor_noise_left

        error_sense_dist_left = abs(measurements[0] - predicted_measurements[0])

        if single_particle_mode:
            print 'Sim - Sensor difference, left: ', error_sense_dist_left

        error_left = (exp(- (error_sense_dist_left ** 2) / (self.sensor_noise_left ** 2) / 2.0) /
                      sqrt(2.0 * pi * (self.sensor_noise_left ** 2)))

        # compute front sensor gaussian error - use sensor_noise_front

        error_sense_dist_front = abs(measurements[1] - predicted_measurements[1])

        if single_particle_mode:
            print 'Sim - Sensor difference, front: ', error_sense_dist_front

        error_front = (exp(- (error_sense_dist_front ** 2) / (self.sensor_noise_front ** 2) / 2.0) /
                      sqrt(2.0 * pi * (self.sensor_noise_front ** 2)))

        total_error = error_left * error_front + robust_likelihood_constant

        if not mygrid.is_available(int(self.x), int(self.y)): # set gaussian error to 0 for particles that are out of bounds
            total_error = 0.0

        if single_particle_mode:
            print 'Sim - gaussian error left: ', error_left
            print 'Sim - gaussian error front: ', error_front
            print 'Sim - gaussian error total: ', total_error

        return total_error

    def __repr__(self):
    # Format particle print for visualization

        return '%.6s %.6s' % (str(self.x), str(self.y))


if real_robot_mode:
    # Real robot (Lego Mindstorm EV3)
    class robot_real:


        def sense_front(self):
        # Get front sensor distance in cm (lowest of 3 readings to reduce error)

            reading1 = sonar_front.dist_cm
            reading2 = sonar_front.dist_cm
            reading3 = sonar_front.dist_cm
            result = min(reading1, reading2, reading3) / 10.0
            if result < sonar_max_distance:
                return result
            else:
                return sonar_max_distance

        def sense_left(self):
        # Get left sensor distance in cm (lowest of 3 readings to reduce error)

            reading1 = sonar_left.dist_cm
            reading2 = sonar_left.dist_cm
            reading3 = sonar_left.dist_cm
            result = min(reading1, reading2, reading3) / 10.0
            if result < sonar_max_distance:
                return result
            else:
                return sonar_max_distance

        def move_time(self, pwr_l, pwr_r, duration):
        # Move in a straight line or arc, for specified time in ms, depending on left/right motor power levels (differential drive)

            a.run_time_limited(time_sp=duration, speed_sp=pwr_l * 10, regulation_mode=True, stop_mode=Motor.STOP_MODE.BRAKE)
            d.run_time_limited(time_sp=duration, speed_sp=pwr_r * 10, regulation_mode=True, stop_mode=Motor.STOP_MODE.BRAKE)
            time.sleep(duration / 1000.0)

        def move_distance(self, wheel_distance, pwr=30):
        # Move in a straight line for specified distance in cm (only used for final move towards target)

            a.reset()
            d.reset()
            wheel_angle = (wheel_distance / wheel_circumference) * 360.0
            a.run_position_limited(position_sp=wheel_angle, speed_sp=pwr * 10, stop_mode=Motor.STOP_MODE.BRAKE)
            d.run_position_limited(position_sp=wheel_angle, speed_sp=pwr * 10, stop_mode=Motor.STOP_MODE.BRAKE)
            time.sleep(wheel_angle / pwr / 10)

        def turn_in_place(self, turn_angle, pwr=15.0):
        # Turn in place (velocity_right = -velocity_left) by specified degrees

            a.reset()
            d.reset()
            wheel_distance = (turn_angle / 360.0) * pi * dist_between_wheels
            wheel_angle = (wheel_distance / wheel_circumference) * 360.0
            a.run_position_limited(position_sp=-wheel_angle, speed_sp=pwr * 10, stop_mode=Motor.STOP_MODE.BRAKE)
            d.run_position_limited(position_sp=wheel_angle, speed_sp=pwr * 10, stop_mode=Motor.STOP_MODE.BRAKE)
            time.sleep(abs(wheel_angle/pwr/10))

        def stop(self):
        # Stop robot (both motors)

            a.stop()
            d.stop()

        def spiral_search(self, step_count):
        # Perform a spiral search for the target once close by (using 2 color sensors)

            target_found = False

            turn_time = 350
            move_time = 50
            for i in range(step_count):
                a.run_time_limited(time_sp=move_time, speed_sp=200, regulation_mode=True, stop_mode=Motor.STOP_MODE.BRAKE)
                d.run_time_limited(time_sp=move_time, speed_sp=200, regulation_mode=True, stop_mode=Motor.STOP_MODE.BRAKE)
                time.sleep(move_time / 1000.0)
                if (color_left.color == 6) or (color_right.color == 6):
                    target_found = True
                    break
                d.run_time_limited(time_sp=turn_time, speed_sp=150, regulation_mode=True, stop_mode=Motor.STOP_MODE.BRAKE)
                time.sleep(turn_time / 1000.0)
                if (color_left.color == 6) or (color_right.color == 6):
                    target_found = True
                    break
                move_time += 5

            self.stop()
            return target_found


# Particle filter for localization
class particle_filter:

    def __init__(self):
    # Initialize particle filter

        self.p = []
        self.w = []

        if single_particle_mode:
            self.count = 1
        else:
            self.count = pf_number_particles

        for i in range(self.count):
            r = robot_sim()
            while not (mygrid.is_available(int(r.x), int(r.y))): # re-create initial particle if it lands on an unavailable spot
                r = robot_sim()
            r.set_noise(sensor_noise_left, sensor_noise_front, steering_noise, distance_noise, turning_noise)
            self.p.append(r)

    def measurement_update(self, new_measurements):
    # Particle filter measurement update

        temp_w = []

        for i in range(self.count):
            temp_w.append(self.p[i].measurement_prob(new_measurements))

        self.w = temp_w

    def motion_update(self, new_motion):
    # Particle filter motion update

        p2 = []
        for i in range(self.count):
            motion_command = new_motion[0]
            if motion_command == 'turn_in_place':
                turn_angle = new_motion[1]
                p2.append(self.p[i].turn_in_place(turn_angle))

            if motion_command == 'move_time':
                power_left = new_motion[1]
                power_right = new_motion[2]
                duration = new_motion[3]
                p2.append(self.p[i].move_time(power_left, power_right, duration))

        self.p = p2

    def resample(self):
    # Particle filter re-sampling

        w_mean = sum(self.w)/len(self.w)
        w_variance = map(lambda x: (x - w_mean)**2, self.w)
        w_mean_var = sum(w_variance)/len(w_variance)
        w_stdev = sqrt(w_mean_var)

        print 'PF - max w: ', max(self.w)
        print 'PF - mean w: ', w_mean
        print 'PF - standard deviation w: ', w_stdev

        p3 = []
        index = int(random.random() * self.count)
        beta = 0.0
        mw = max(self.w)
        for i in range(self.count):
            beta += random.random() * 2.0 * mw
            while beta > self.w[index]:
                beta -= self.w[index]
                index = (index + 1) % self.count
            p3.append(self.p[index])
        self.p = p3

    def get_position(self):
    # Particle filter position estimation

        x = 0.0
        y = 0.0
        orientation = 0.0
        for i in range(len(self.p)):
            x += self.p[i].x
            y += self.p[i].y
            orientation += (((self.p[i].orientation - self.p[0].orientation + pi) % (2.0 * pi))
                            + self.p[0].orientation - pi)
        return [x / len(self.p), y / len(self.p), orientation / len(self.p)]


def power_limit(pwr):
    # Constrain power levels on wheel motors (avoid power too low, power too high, excessive power ratios)

    pwr_min = base_power - base_power * power_multiplier
    pwr_max = base_power + base_power * power_multiplier
    if pwr < pwr_min:
        return pwr_min
    if pwr > pwr_max:
        return pwr_max
    return pwr

def distance_between(point1, point2):
    # Computes distance between point1 and point2. Points are (x, y) pairs

    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def angle_trunc(a):
    # This maps all angles to a domain of [-pi, pi]

    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    # Returns the angle, in radians, between the target and hunter positions

    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading



if real_robot_mode:
    ev3 = robot_real() # initialize a real robot

mygrid = Grid(world_size_y, world_size_x) # initialize grid with all obstacles in room
# format of obstacle is (y1, y2, x1, x2)
mygrid.add_obstacle(130, 348, 0, 106) # bed
mygrid.add_obstacle(279, 348, 255, 360) # dresser
mygrid.add_obstacle(0, 164, 283, 360) # table

walls = []
# format of wall is [x1, y1, x2, y2]
walls.append([0, 0, 0, 130])
walls.append([0, 130, 106, 130])
walls.append([106, 130, 106, 348])
walls.append([106, 348, 255, 348])
walls.append([255, 348, 255, 279])
walls.append([255, 279, 360, 279])
walls.append([360, 279, 360, 164])
walls.append([360, 164, 283, 164])
walls.append([283, 164, 283, 0])
walls.append([283, 0, 0, 0])


if real_robot_mode:
    measurement_history = [] # store all new measurements
else:
    measurement_history = saved_measurements[active_set - 1] # load measurements from past data set
    run_steps = saved_steps[active_set - 1]


# Initialize particle filter
pf = particle_filter()

estimated_position = [] # estimated position of robot

pid_lastError = 0
pid_derivative = 0


for i in range(run_steps + 1): # additional loop for first sense (before any moves)

    # Sense

    if real_robot_mode:
        sonar_l = ev3.sense_left() # get left sonar distance
        sonar_f = ev3.sense_front() # get front sonar distance
        measurements = [sonar_l, sonar_f]
        measurement_history.append(measurements)
    else:
        # Read from measurement history
        measurements = measurement_history[i]
        sonar_l = measurements[0]
        sonar_f = measurements[1]

    print ''
    print 'Robot - Sensor dist, left: ', sonar_l
    print 'Robot - Sensor dist, front: ', sonar_f

    # Particle Filter measurement update

    print 'PF - measurement update...'
    pf.measurement_update(measurements)

    # Particle Filter re-sample

    print 'PF - resampling...'
    pf.resample()

    # Particle Filter estimate position

    print 'PF - estimated position: '
    estimated_position = pf.get_position()
    print estimated_position

    print ''
    print 'PF: current particle set: '
    print pf.p

    # Move

    if i < run_steps: # don't move on last loop iteration

        if sonar_f < (target_wall_dist * 1.5): # turn in place (90 deg right) if front distance too low
            if real_robot_mode:
                ev3.turn_in_place(-90.0)
            motion = ['turn_in_place', -90.0]
            print 'PF - update motion...'
            pf.motion_update(motion)

        elif sonar_l > (target_wall_dist * 1.5): # turn in place (45 deg left) and step forward if left distance too high (this can occur twice in a row on a 90 deg left corner)
            if real_robot_mode:
                ev3.turn_in_place(45.0)
            motion = ['turn_in_place', 45.0]
            print 'PF - update motion...'
            pf.motion_update(motion)
            if real_robot_mode:
                ev3.move_time(base_power, base_power, 1500)
            motion = ['move_time', base_power, base_power, 1500]
            print 'PF - update motion...'
            pf.motion_update(motion)

        else: # wall follow using PD controller
            pid_error = sonar_l - target_wall_dist
            # print 'Robot - PD error: ', pid_error
            pid_derivative = pid_error - pid_lastError
            delta = Kp * pid_error + Kd * pid_derivative
            # print 'Robot - PD delta total: ', delta
            # print 'Robot - PD delta P component: ', Kp * pid_error
            # print 'Robot - PD delta D component: ', Kd * pid_derivative
            power_left = base_power - delta
            power_right = base_power + delta

            power_left = power_limit(power_left) # new left power
            power_right = power_limit(power_right) # new right power

            # print 'Robot - new left power: ', power_left
            # print 'Robot - new right power: ', power_right

            if real_robot_mode:
                ev3.move_time(power_left, power_right, 1000)

            motion = ['move_time', power_left, power_right, 1000]
            print 'PF - update motion...'
            pf.motion_update(motion)

            pid_lastError = pid_error


# print measurement history on real run
if real_robot_mode:
    print ''
    print 'Measurement history: '
    print measurement_history

# compare estimated final position with real final position (for simulated runs)
if not real_robot_mode:
    print ''
    print 'Final estimated position: ', estimated_position
    print 'Final real position: ', saved_real_ends[active_set - 1]
    deltas = [abs(estimated_position[0] - saved_real_ends[active_set - 1][0]), abs(estimated_position[1] - saved_real_ends[active_set - 1][1]), angle_trunc(abs(estimated_position[2] - saved_real_ends[active_set - 1][2]))]
    print 'Final position deltas: ', deltas

# print final particle set (used for particle visualization)
print ''
print 'PF: final particle set: '
print pf.p


if real_robot_mode and grab_target:

# Calculate distance/heading to target, move towards it, run spiral search - note that there is no collision avoidance

    distance_to_target = distance_between((estimated_position[0],estimated_position[1]), target_position)
    print 'Robot - Distance to target: ', distance_to_target
    heading_to_target = get_heading((estimated_position[0], estimated_position[1]), target_position)
    print 'Robot - Heading to target: ', heading_to_target
    turn_angle_deg = angle_trunc(heading_to_target - estimated_position[2]) * 180/pi
    print 'Robot - Turn angle towards target: ', turn_angle_deg
    print 'Robot - Turning towards target'
    ev3.turn_in_place(turn_angle_deg)
    print 'Robot - Driving towards target'
    ev3.move_distance(distance_to_target)

    found = ev3.spiral_search(100)
    if found:
        print 'Robot - Found target !'
        sound.play(1000, 3000)
    else:
        print 'Robot - Target not found'


