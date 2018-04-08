import struct


class DataMessage:

    def __init__(self, byte_string_message):
        (self.light_sensor_left,
         self.light_sensor_right,
         self.ultrasonic_distance,
         self.temperature,
         self.pos_angle,
         self.pos_x,
         self.pos_y,
         self.total_distance) = struct.unpack_from('<hhhfffff', byte_string_message, 1)


    def __str__(self):
        return    ('Light_sensor = ({0}, {1}); ultrasonic_distance = {2}; '
                   'temperature = {3:.1f}; position = ({4:.0f}; {5:.1f}, {6:.1f}); '
                   'distance_passed = {7:.1f}').format(self.light_sensor_left,\
                    self.light_sensor_right, self.ultrasonic_distance,\
                    self.temperature, 180*self.pos_angle, self.pos_x, self.pos_y,\
                    self.total_distance)
             


