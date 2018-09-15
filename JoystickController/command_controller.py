import struct


class CommandController:
    def __init__(self):
        self._speedFunction = None
        self._cameraAnglefunction = None
        self._directionFunction = None
        self.speed1_command = 0
        self.speed2_command = 0
        self.direction_command = 0
        self.camera_angle_command_ver = 0
        self.camera_angle_command_hor = 0

    def setSpeedSource(self, speed_function):
        self._speedFunction = speed_function        
    
    def setCameraAngeSource(self, camera_angle_function):
        self._cameraAngleFunction = camera_angle_function

    def setDirectionSource(self, direction_function):
        self._directionFunction = direction_function
        
    def update(self):
        if (not self._speedFunction is None):
            self.speed1_command, self.speed2_command = self._speedFunction()

        if (not self._directionFunction is None):
            self.direction_command = self._directionFunction()

        if (not self._cameraAngleFunction is None):
            self.camera_angle_command_hor, self.camera_angle_command_ver = self._cameraAngleFunction()


    def encode_for_tx(self, return_response):
    # [1, int16 (Speed1), int16(Speed2), int16 (direction angle), int16(vision_angle), byte (request respose)]
    # < - little endian, c - byte of length 1, h - short integer of length 2
        message = struct.pack('<Bhhhh?', 1, self.speed1_command, \
                                        self.speed2_command, \
                                        self.camera_angle_command_ver, \
                                        self.camera_angle_command_hor, \
                                        return_response)
        return message

        
