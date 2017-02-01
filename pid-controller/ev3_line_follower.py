#!/usr/bin/python
"""
PID implementation:
P -> proportional
I -> integral
D -> derivative

+----------+--------+---------+--------+-----------+
|Increasing|Rise    |Overshoot|Setting |Error at   |
|constant  |time    |         |time    |equilibrium|
+----------+--------+---------+--------+-----------+
|kp        |Decrease|Increase |Small   |Decrease   |
|          |        |         |change  |           |
+----------+--------+---------+--------+-----------+
|ki        |Decrease|Increase |Increase|Eliminate  |
+----------+--------+---------+--------+-----------+
|kd        |Small   |Decrease |Decrease|None       |
|          |change  |         |        |           |
+----------+--------+---------+--------+-----------+
"""
import time
from ev3.lego import Motor
from ev3.lego import ColorSensor, UltrasonicSensor
from ev3.ev3dev import Tone

# the maximum and minium amount of power before an exception is thrown by ev3
MOTOR_MAX_POWER = 100.0
MOTOR_MIN_POWER = -100.0
NUMBER_OF_LAPS = 4
# set reverse to 1 if your bot has the color sensor up front, -1 back
REVERSE = -1

value_on_red = 100.0
value_not_on_red = 35.0
target = ((value_on_red - value_not_on_red) / 2) + value_not_on_red
tp = 70 # target power (speed)

kp = 0.65 # how fast we try to correct the 'out of target'
ki = 0.05 # smooth the correction by storing historical errors
kd = 0.09 # controls the overshooting

print "target power: " + str(tp)
print "proportional constant: " + str(kp)
print "proportional range: +" + str(target) + " / -" + str(target)

color_sensor = ColorSensor()
ultra_sensor = UltrasonicSensor()
motor_left = Motor(port=Motor.PORT.A)
motor_right = Motor(port=Motor.PORT.D)
tone = Tone()

def avg(lst):
    if len(lst) == 0:
        return 0
    return reduce(lambda x, y: x + y, lst) / len(lst)

def stop():
    motor_left.stop()
    motor_right.stop()

def victory():
    motor_left.run_forever(-100)
    motor_right.run_forever(-100)
    time.sleep(3)
    motor_right.run_forever(100)
    tone.play(1300, 800)
    time.sleep(0.9)
    tone.play(1300, 500)
    time.sleep(0.6)
    tone.play(1300, 200)
    time.sleep(0.4)
    tone.play(1300, 700)
    time.sleep(0.4)
    tone.play(1550, 700)
    time.sleep(0.4)
    tone.play(1800, 700)
    time.sleep(0.4)
    tone.play(1900, 1000)
    time.sleep(0.3)
    stop()

def defeat():
    stop()
    tone.play(350, 100)
    time.sleep(0.5)
    tone.play(250, 300)
    time.sleep(0.5)
    tone.play(100, 1000)

def clamp(n, minn = MOTOR_MIN_POWER, maxn = MOTOR_MAX_POWER):
    return max(min(maxn, n), minn)

def follow_line():
    integral = 0.0
    power_to_left = []
    power_to_right = []
    amount_of_red_sensed = []
    laps_time = []
    last_error = 0.0
    lap_counter = 0
    run_time = time.time()
    start_time = time.time()
    for i in range(4):
        tone.play(1000, 300*i)
        time.sleep(1)
    while True:
        try:
            color_read = color_sensor.rgb
        except:
            color_read = 100
        amount_of_red = color_read[0]
        print "We got the following rgb values: " + str(color_read)
        amount_of_red_sensed.append(amount_of_red)
        error = target - amount_of_red
        print "The current error is: " + str(error)
        integral = (2.0/3.0 * integral) + error # slightly forget past errors
        print "We have an integral of: " + str(integral)
        derivative = error - last_error
        print "We have a derivative of: " + str(derivative)
        last_error = error
        turn = (kp * error) + (ki * integral) + (kd * derivative)
        print "We are turning: " + str(turn)
        if ultra_sensor.dist_in < 25 and (time.time() - start_time) > 10:
            laps_time.append(time.time() - start_time)
            start_time = time.time()
            lap_counter += 1
            for i in range(lap_counter):
                tone.play(1000 + (i * 100), 500)
                time.sleep(0.2)
        if lap_counter == NUMBER_OF_LAPS:
            print "Awesome! We finish the race!"
            victory()
            break
        power_left = clamp(tp + turn)
        power_right = clamp(tp - turn)
        print "Passing power left: " + str(power_left)
        print "Passing power right: " + str(power_right)
        power_to_left.append(power_left)
        power_to_right.append(power_right)
        motor_left.run_forever(REVERSE * power_left)
        motor_right.run_forever(REVERSE * power_right)
        if color_sensor.rgb == (0, 0, 0):
            print "Bot has been picked up. Stopping..."
            defeat()
            break
    print ""
    print "Some fun facts: "
    print "Average power on left: " + str(avg(power_to_left))
    print "Average power on right: " + str(avg(power_to_right))
    print "Average red sensed: " + str(avg(amount_of_red_sensed))
    print ""
    print "Max and min values on left: " + str(max(power_to_left)) + " - " + str(min(power_to_left))
    print "Max and min values on right: " + str(max(power_to_right)) + " - " + str(min(power_to_right))
    print "Max and min values of red: " + str(max(amount_of_red_sensed)) + " - " + str(min(amount_of_red_sensed))
    print ""
    for i in range(len(laps_time)):
        print "The time of Lap " + str(i + 1) + " was " + str(laps_time[i]) + " seconds"
    print "Average time of Lap: " + str(avg(laps_time))
    print "Max and min time lap: " + str(max(laps_time)) + " - " + str(min(laps_time))
    print ""
    print "Total run time: " + str(time.time() - run_time)
    
    with open('power-left.txt', 'w') as out_file:
        out_file.write('\n'.join(map(str, power_to_left)))
        out_file.write('\n')
    with open('power-right.txt', 'w') as out_file:
        out_file.write('\n'.join(map(str, power_to_right)))
        out_file.write('\n')
    with open('red-sensed.txt', 'w') as out_file:
        out_file.write('\n'.join(map(str, amount_of_red_sensed)))
        out_file.write('\n')
    with open('laps-time.txt', 'w') as out_file:
        out_file.write('\n'.join(map(str, laps_time)))
        out_file.write('\n')

follow_line()
