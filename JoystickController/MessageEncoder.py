import struct


def changeStateMessage(command, return_response):
# [1, int16 (Speed1), int16(Speed2), int16 (direction angle), int16(vision_angle), byte (request respose)]
# < - little endian, c - byte of length 1, h - short integer of length 2
    message = struct.pack('<Bhhhh?', 1, command.speed1_command, \
                                        command.speed2_command, \
                                        command.direction_command, \
                                        command.camera_angle_command, \
                                        return_response)
    return message

        
