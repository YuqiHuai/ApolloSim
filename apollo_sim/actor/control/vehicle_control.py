class VehicleControl(object):

    def __init__(self, throttle: float = 0.0, brake: float = 0.0, steering: float = 0.0):
        self.throttle = throttle
        self.brake = brake
        self.steering = steering

    def json_data(self):
        return {
            "throttle": self.throttle,
            "brake": self.brake,
            "steering": self.steering
        }
