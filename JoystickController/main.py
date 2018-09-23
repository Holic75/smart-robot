# -*- coding: utf-8 -*-
import sys
import time
import serial
import io

import communicator
import pygame
import command_controller
import data_message


def joystickSpeed():
    pygame.joystick.init()
    if pygame.joystick.get_count():
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        ax_hor = joystick.get_axis(0)
        ax_ver = -joystick.get_axis(1)

        ax_hor = ax_hor**3
        ax_ver = ax_ver**3

        speed1 = (ax_ver - ax_hor)
        speed2 = (ax_ver + ax_hor)
        return int(100*speed1), int(100*speed2)
    else:
        return 0, 0

def joystickCamera():
    pygame.joystick.init()
    if pygame.joystick.get_count():
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        ax_hor = -joystick.get_axis(2)
        ax_ver = -joystick.get_axis(5)
        ax_hor = ax_hor*abs(ax_hor)
        ax_ver = ax_ver*abs(ax_ver)
        return int(45.0*ax_hor), int(45.0*ax_ver)
    else:
        return 0, 0  

ser = serial.Serial(
    
    port=str(sys.argv[1]),
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.1

)

arduino_link = communicator.Communicator(ser,255)


#message1 = MessageEncoder.changeStateMessage(0,-30,30,True)

pygame.init()



arduino_controller = command_controller.CommandController()

arduino_controller.setSpeedSource( joystickSpeed)
arduino_controller.setCameraAngeSource( joystickCamera)


while True:
    pygame.event.get()
    arduino_controller.update()
    arduino_link.runReceptionLoop()   
    arduino_link.sendMessage(arduino_controller.encode_for_tx(True))

    if arduino_link.isMessageReceived():
        data_from_arduino = data_message.DataMessage(arduino_link.getReceivedMessage())
        print('\r' + str(data_from_arduino), end ='', flush=True)
        arduino_link.reset()

    time.sleep(0.05)





