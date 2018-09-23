import struct
import math


class DataMessage:

    def __init__(self, byte_string_message):
        (self.light_sensor_left,
         self.light_sensor_right,
         self.ultrasonic_distance,
         self.temperature,
         self.yaw,
         self.pitch,
         self.roll,
         self.enc_1,
         self.enc_2) = struct.unpack_from('<hhhffffll', byte_string_message, 1)


    def __str__(self):
        angle_scale = 180.0/math.pi
        return    ('ls = ({0}, {1}); ud = {2}; '
                   't = {3:.1f}; enc  = ({4}; {5}); '
                   'ypr = ({6:.1f}; {7:.1f}; {8:.1f}) ').format(self.light_sensor_left,\
                    self.light_sensor_right, self.ultrasonic_distance,\
                    self.temperature, self.enc_1, self.enc_2, angle_scale*self.yaw,\
                    angle_scale*self.pitch, angle_scale*self.roll)
             


