import pygame
from dronekit import connect, VehicleMode, LocationGlobalRelative, Vehicle
import time
from pymavlink import mavutil
#import connection

pygame.init() #Initialize pygame
win=pygame.display.set_mode((500,500)) #Display 500x500 screen

vehicle = connect('udp:127.0.0.1:14450', wait_ready = True)

#connection.arm_and_takeoff(50)

def set_ned_velocity(duration,x,y,z):
    msg=vehicle.message_factory.set_postion_target_local_ned_encode(
        0, # time boot
        0,0, # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, #velocity relative to home location
        0b0000111111000111, #bitmask to enable velocities
        0,0,0,
        x,y,z, #velocities in x(north), y(east), z(down)
        0,0,0, #acceleration in x,y,z
        0,0 #yaw, yaw_rate
        )
    #Sending msg every 1 sec for total duration
    for i in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1
    else:
        is_relative=0
    
    msg=vehicle.message_factory.long_encode(
        0, #time boot
        0,0, #target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0,  #confirmation
        heading, #angle of yaw in degrees
        0,  #yaw speed in deg/s
        1,  #direction -> -1 = ccw, 1=cw
        is_relative,    #relative offset 1, absolute angle 0
        0,0,0   #param 5~7 not used
        )    
    #Send command to vehicle
    vehicle.send_mavlink(msg)

running = True
t=5 #seconds
speed= 15 #m/s

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running=False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                set_ned_velocity(t,speed,0,0)
            if event.key == pygame.K_a:
                set_ned_velocity(t,0,-speed,0)
            if event.key == pygame.K_s:
                set_ned_velocity(t,-speed,0,0)
            if event.key == pygame.K_d:
                set_ned_velocity(t,0,speed,0)
            if event.key == pygame.K_UP:
                set_ned_velocity(t,0,0,-speed)
            if event.key == pygame.K_DOWN:
                set_ned_velocity(t,0,0,speed)
            if event.key == pygame.K_RIGHT:
                condition_yaw(90,relative=True)
            if event.key == pygame.K_LEFT:
                condition_yaw(-90,relative=True)
        if event.type == pygame.KEYUP and event.key == (pygame.K_LEFT or pygame.K_RIGHT or pygame.K_UP or pygame.K_DOWN or pygame.K_a or pygame.K_w or pygame.K_d or pygame.K_s):
            set_ned_velocity(t,0,0,0)

pygame.display.update()