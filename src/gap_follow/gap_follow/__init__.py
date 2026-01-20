self.prev_steering = 0.0
steering_angle = 0.7 * self.prev_steering + 0.3 * steering_angle
self.prev_steering = steering_angle
