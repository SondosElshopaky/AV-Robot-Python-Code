from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from spike.operator import equal_to, greater_than, greater_than_or_equal_to, less_than, less_than_or_equal_to, equal_to, not_equal_to

#objects
hub = PrimeHub()
dbase = MotorPair('C','D')
motor_a = Motor('A')
motor_b = Motor('B')
motor_c = Motor('C')
motor_d = Motor('D')

#defining a GYRO_SPIN (function)
def gyro_spin(angle):
    hub.motion_sensor.reset_yaw_angle() # Resetting Gyro Angle to '0'
    drift= 4
    ## adjustable drift variable
    ## if the angle is positive
    if(angle > 0):
        dbase.start_tank(10,-10)
        wait_until(hub.motion_sensor.get_yaw_angle, greater_than, angle-drift)#the "while" condition will execute the following until its condition is no longer met.
        ##if the angle is negative
    else:
        dbase.start_tank(-10,10)
        wait_until(hub.motion_sensor.get_yaw_angle, less_than, angle+drift)#the "while" condition will execute the following until its condition is no longer met.
    dbase.stop()
    reset()
    #hub.light_matrix.write(hub.motion_sensor.get_yaw_angle())
## returning angle value
    
    
    
#defining a PID_MOVE (function)
def pid_move(distance, power):
    hub.motion_sensor.reset_yaw_angle()
    ## reset yaw angle
    ## declaring variables
    kpb=1.9
    kp=-1.9
    ki=0.000001
    kd=1.2
    ## constants yet to be tuned
    integral=0
    derivative=0
    error= 0
    lasterror= 0
    ## errors and last errors
    pidangle=0
    p=0
    i=0
    d=0
    deg_to_go=0
    motor_d.set_degrees_counted(0)
    ## resetting degrees counted
    deg_to_go= (distance*360)/(3.14*6.24)
    ## calculating degrees
    if (power > 0):

        while motor_d.get_degrees_counted()< deg_to_go:
            ## while the destination is not yet reached
            ##PID Formula
            error=0-hub.motion_sensor.get_yaw_angle()
            ## error equals current position
            P= error*kp
            integral= integral + error
            i= integral*ki
            derivative= lasterror - error
            d= derivative*kd
            ## finalizing
            pidangle=int(p+i+d)
            dbase.start(pidangle, power)
            lasterror = error
            ## storing error as last error as we will go through the loop again
        dbase.stop()
    else:
        while motor_d.get_degrees_counted()< deg_to_go:
                ## while the destination is not yet reached
                ##PID Formula
                error=0-hub.motion_sensor.get_yaw_angle()
                ## error equals current position
                P= error*kpb
                integral= integral + error
                i= integral*ki
                derivative= lasterror - error
                d= derivative*kd
                ## finalizing
                pidangle=int(p+i+d)
                dbase.start(pidangle, power)
                lasterror = error
                ## storing error as last error as we will go through the loop again
        dbase.stop()

def reset( ):
    hub.motion_sensor.reset_yaw_angle()


#execution initialisation
dbase.set_default_speed(20)
dbase.set_stop_action('hold')
dbase.set_motor_rotation(19.6, 'cm')

#mission()
pid_move(53, 40) #exiting base
gyro_spin(90) # spinning in front of blue container
dbase.move(12,'cm',0,-20) #pushing blue container
pid_move(152, 40) #moving towards the helicopter 
gyro_spin(125) #spinning in front of helicopter
dbase.move(32,'cm',0,-30) #air dropp
dbase.move(5,'cm',0,30) #moving away from air drop 
gyro_spin(-38) #turning to square up
dbase.move(3,'seconds',0,-20) #squaring up on the wall

