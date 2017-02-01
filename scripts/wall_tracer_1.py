import time
from ev3.lego import Motor
from ev3.lego import UltrasonicSensor
a = Motor(port=Motor.PORT.A)
d = Motor(port=Motor.PORT.D)
a.reset()
d.reset()
a.position_mode=Motor.POSITION_MODE.RELATIVE
d.position_mode=Motor.POSITION_MODE.RELATIVE

Kp = 0.03
Kd = 0.02
base_power = 30
power_multiplier = 0.20
target_wall_dist = 100

sonar_left = UltrasonicSensor(port=2)
sonar_front = UltrasonicSensor(port=3)


def sense_front():
    reading1 = sonar_front.dist_in
    reading2 = sonar_front.dist_in
    return min(reading1, reading2)

def sense_left():
    reading1 = sonar_left.dist_in
    reading2 = sonar_left.dist_in
    return min(reading1, reading2)

def move_time(pwr_l, pwr_r, duration):
    a.run_time_limited(time_sp=duration, speed_sp=pwr_l * 10, regulation_mode=True, stop_mode=Motor.STOP_MODE.BRAKE)
    d.run_time_limited(time_sp=duration, speed_sp=pwr_r * 10, regulation_mode=True, stop_mode=Motor.STOP_MODE.BRAKE)
    time.sleep(duration / 1000.0)

def turn_in_place(turn_angle, pwr=150.0):
    a.reset()
    d.reset()
    wheel_angle = 1.667 * turn_angle
    print 'Turning degrees: ', turn_angle
    a.run_position_limited(position_sp=wheel_angle, speed_sp=pwr, stop_mode=Motor.STOP_MODE.BRAKE)
    d.run_position_limited(position_sp=-wheel_angle, speed_sp=pwr, stop_mode=Motor.STOP_MODE.BRAKE)
    time.sleep(abs(wheel_angle/pwr))

def stop():
    a.stop()
    d.stop()

def power_limit(pwr):
    pwr_min = base_power - base_power * power_multiplier
    pwr_max = base_power + base_power * power_multiplier
    if pwr < pwr_min:
        return pwr_min
    if pwr > pwr_max:
        return pwr_max
    return pwr

start_time = time.time()
end_time = time.time() + 65

lastError = 0
derivative = 0

while True:

    print ''
    sonar_f = sense_front()
    sonar_l = sense_left()

    print 'Front distance: ', sonar_f
    print 'Left distance: ', sonar_l
    
    if sonar_f < (target_wall_dist * 1.5):
        turn_in_place(90)
        continue    

    if sonar_l > (target_wall_dist * 1.5):
        turn_in_place(-45)
        move_time(base_power, base_power, 1500)
        continue

    error = sonar_l - target_wall_dist
    print 'Error: ', error
    derivative = error - lastError
    delta = Kp * error + Kd * derivative
    print 'Delta total: ', delta
    print 'Delta P component: ', Kp * error
    print 'Delta D component: ', Kd * derivative
    power_left = base_power - delta
    power_right = base_power + delta

    power_left = power_limit(power_left)
    power_right = power_limit(power_right)

    print 'New Left Power: ', power_left
    print 'New Right Power: ', power_right

    move_time(power_left, power_right, 1000)

    lastError = error

    if time.time() > end_time:
        break
