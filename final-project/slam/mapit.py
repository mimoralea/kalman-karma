#!/usr/bin/python

import math
import time
import sys

# Globals
pi2 = 2*math.pi

# Simulation classes in order to run the program without the EV3 hardware.
simulation = False

class Sim_Motor:

   def __init__(self, port):
      self.port = port
      self.running = False
      self.position = 0
      self.speed = 0

   def stop(self):
      self.running = False

   def run_position_limited(self, position_sp, speed_sp):
      self.running = True
      self.speed = speed_sp
      self.position = position_sp
      self.speed = 0
      self.running = False

   def read_value(self, value_name):
      return self.position

class Sim_Sensor:

   def __init__(self, port):
      self.port = port

   def read_value(self, value_name):
      return 0

# Global varibales for the motors and sensors
if not simulation:
   from ev3.lego import Motor
   from ev3.lego import Msensor
   sonar_sensor = Msensor(port=2)
   infrared_sensor = Msensor(port=3)
   color_sensor = Msensor(port=4)
   forward_motor = Motor(port=Motor.PORT.C)
   turn_motor = Motor(port=Motor.PORT.B)
   turret_motor = Motor(port=Motor.PORT.A)
else:
   forward_motor = Sim_Motor(port="C")
   turn_motor = Sim_Motor(port="B")
   turret_motor = Sim_Motor(port="A")
   sonar_sensor = Sim_Sensor(port=2)
   infrared_sensor = Sim_Sensor(port=3)
   color_sensor = Sim_Sensor(port=4)

# Config values
forward_speed = 100
turn_speed = 100
turret_speed = 100
turn_max_clicks = 350
turret_max_clicks = 2540
turret_rad2clicks = turret_max_clicks/pi2
turret_clicks2rad = pi2/turret_max_clicks
turret_min_clicks = 5
turn_min_clicks = 10
turn_rad2clicks = turn_max_clicks/0.6435
turn_clicks2rad = 0.6435/turn_max_clicks
turn_max_rad = turn_max_clicks * turn_clicks2rad
turret_max_rad = turret_max_clicks * turret_clicks2rad
forward_cm2clicks = 37.037
forward_clicks2cm = 1.0/37.037
forward_min_cm = 1.0
forward_min_clicks = forward_min_cm * forward_cm2clicks
length = 21.0 / 2.0
close_value = 5.0
max_sonar = 1000
max_turn_radius = 28.01

class Robot:

   def __init__(self):
      self.x = 0.0
      self.y = 0.0
      self.heading = 0.0
      self.steering = 0.0
      self.turret_heading = 0.0
      self.turret_goal = 0
      self.sonar_readings = []
      self.turret_circle = pi2
      self.tx = 0.0
      self.ty = 0.0
      self.motion_action = 0
      self.sense_action = 0
      self.forward_position = int(forward_motor.read_value("position"))

   def set_position(self,x, y):
      self.x = x
      self.y = y

   def set_heading(self,heading):
      self.heading = heading

   def update_steering(self):
      current_motor_position = int(turn_motor.read_value("position"))
      self.steering = current_motor_position * turn_clicks2rad

   def update_turret_heading(self):
      current_motor_position = int(turret_motor.read_value("position"))
      self.turret_heading = current_motor_position * turret_clicks2rad

   def turn(self,delta):
      self.update_steering()
      steering_goal = self.steering + delta
      
      if steering_goal < -turn_max_rad:
         steering_goal = -turn_max_rad
      elif steering_goal > turn_max_rad:
         steering_goal = turn_max_rad
         
      new_motor_position = steering_goal * turn_rad2clicks

      if abs(self.steering*turn_rad2clicks - new_motor_position) < turn_min_clicks:
         return
      
      turn_motor.run_position_limited(position_sp=new_motor_position, speed_sp=turn_speed)

   def get_infrared(self):
      return int(infrared_sensor.read_value("value0"))

   def get_sonar(self):
      return int(sonar_sensor.read_value("value0"))

   def get_color(self):
      return int(color_sensor.read_value("value0"))

   def reset(self):
      forward_motor.stop()
      turn_motor.stop()
      turret_motor.stop()
      turn_motor.run_position_limited(position_sp=0, speed_sp=100)
      turret_motor.run_position_limited(position_sp=0, speed_sp=200)
      self.steering = 0.0
      self.turret_heading = 0.0
      self.turret_goal = 0

   def turn_turret(self,goal):
      self.update_turret_heading()
      self.turret_goal = goal

      if self.turret_goal < -turret_max_rad:
         self.turret_goal = -turret_max_rad
      elif self.turret_goal > turret_max_rad:
         self.turret_goal = turret_max_rad
      
      new_motor_position = self.turret_goal * turret_rad2clicks

      if abs(self.turret_heading*turret_rad2clicks - new_motor_position) < turret_min_clicks:
         self.turret_goal = self.turret_heading
         return

      turret_motor.run_position_limited(position_sp=new_motor_position, speed_sp=turret_speed)

   def sense(self):
      self.update_turret_heading()
      motor_state = int(turret_motor.read_value("run"))
      
      if (motor_state == 1):
         reading = self.get_sonar()
         heading = self.turret_heading + self.heading

         if reading <= max_sonar:
            self.sonar_readings.append([reading, (self.turret_heading + self.heading) % pi2])
            
         return 1

      if self.sense_action == 0:
         self.sense_action = 1
         self.turn_turret(self.turret_circle)
         return 1

      if self.sense_action == 1:
         for i in range(len(self.sonar_readings)):
            print '%5d %6.2f' % (self.sonar_readings[i][0], self.sonar_readings[i][1])
         if self.turret_circle == 0:
            self.turret_circle = pi2
         else:
            self.turret_circle = 0
         self.sense_action = 0
         return 0

      return 1

   def think(self):
      self.tx = 20.0
      self.ty = 10.0

   def act(self):
      error_dist = math.sqrt((self.x - self.tx)**2 + (self.y - self.ty)**2)

      if error_dist < forward_min_cm:
         self.motion_action = 0 # reset for next movement
         return 0

      if self.motion_action == 0:
         self.motion_action = 1 # next action is turn

      if self.motion_action == 1:
         angle = math.atan2(self.ty - self.y, self.tx - self.x)
         angle_difference = (angle - self.heading) % pi2
         
         if angle_difference > math.pi:
            angle_difference = math.pi - angle_difference

         self.turn(angle_difference)
         self.motion_action = 2
         
         return 0

      if self.motion_action == 2:
         motor_state = int(turn_motor.read_value("run"))
      
         if (motor_state == 1):
            return 0

         forward_motor.run_forever(forward_speed, regulation_mode = True)
         self.motion_action = 3

      if self.motion_action == 3:
         current_motor_position = int(forward_motor.read_value("position"))
         distance = (current_motor_position - self.forward_position) * forward_clicks2cm
         front_wheel_x = self.x + length * math.cos(self.heading) 
         front_wheel_y = self.y + length * math.sin(self.heading) 
         back_wheel_x = self.x - length * math.cos(self.heading) 
         back_wheel_y = self.y - length * math.sin(self.heading) 
         back_wheel_x += distance * math.cos(self.heading)
         back_wheel_y += distance * math.sin(self.heading)
         self.update_steering()
         front_wheel_x += distance * math.cos(self.heading + self.steering)
         front_wheel_y += distance * math.sin(self.heading + self.steering)
         self.x = (front_wheel_x + back_wheel_x) / 2.0
         self.y = (front_wheel_y + back_wheel_y) / 2.0
         self.forward_position = current_motor_position

         # make sure the denominator for the tangent never ends up as zero
         if (abs(front_wheel_x - back_wheel_x) > 0.0001):
            self.heading = math.atan2(front_wheel_y - back_wheel_y, front_wheel_x - back_wheel_x)
         
         error_dist = math.sqrt((self.x - self.tx)**2 + (self.y - self.ty)**2)
         print self.x, self.y, self.heading, error_dist
         
         if error_dist < forward_min_cm:
            self.motion_action = 0 # reset for next movement
            print "Goal"
            self.reset()
            sys.exit(0)
            return 0

         motor_state = int(turn_motor.read_value("run"))

         if motor_state != 1:
            angle = math.atan2(self.ty - self.y, self.tx - self.x)
            angle_difference = (angle - self.heading) % pi2
            
            if angle_difference > math.pi:
               angle_difference = math.pi - angle_difference

            self.turn(angle_difference)
            
      return 1

# Main program

car = Robot()
close_flag = car.get_infrared()
time.sleep(10)

#while close_flag >= close_value:
#   close_flag = car.get_infrared()
for i in range(2):
   while close_flag >= close_value and car.sense():
      close_flag = car.get_infrared()
   #car.think()
   #while close_flag >= close_value and car.act():
   #   close_flag = car.get_infrared()

print "STOPPING"
car.reset()
