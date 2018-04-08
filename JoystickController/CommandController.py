

class CommandController:
    def __init__(self):
        self.m_speedFunction = None
        self.m_cameraAnglefunction = None
        self.m_directionFunction = None
        self.speed1_command = 0
        self.speed2_command = 0
        self.direction_command = 0
        self.camera_angle_command = 0

    def setSpeedSource(self, speed_function):
        self.m_speedFunction = speed_function        
    
    def setCameraAngeSource(self, camera_angle_function):
        self.m_cameraAngleFunction = camera_angle_function

    def setDirectionSource(self, direction_function):
        self.m_directionFunction = direction_function
        
    def update(self):
        if (not self.m_speedFunction is None):
            self.speed1_command, self.speed2_command = self.m_speedFunction()

        if (not self.m_directionFunction is None):
            self.direction_command = self.m_directionFunction()

        if (not self.m_cameraAngleFunction is None):
            self.camera_angle_command = self.m_cameraAngleFunction()

        
