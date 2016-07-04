from ev3.lego import Motor

def run_motor(seconds):
   milliseconds = seconds * 1000
   print "start running motor"
   d=Motor(port=Motor.PORT.A)
   d.run_time_limited(time_sp=milliseconds, speed_sp=80)
   print "done running motor"

# Runs the port A motor for 1 second.
run_motor(1)


