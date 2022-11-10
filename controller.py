"""This file is the main controller file

Here, you will design the controller for your for the adaptive cruise control system.
"""

# NOTE: Very important that the class name remains the same
class Controller:
    def __init__(self, target_speed: float, distance_threshold: float):
        self.target_speed = target_speed
        self.distance_threshold = distance_threshold
        self.d_prev_error = 0
        self.v_prev_error = 0

    def run_step(self, obs) -> float:
        """This is the main run step of the controller.

        Here, you will have to read in the observatios `obs`, process it, and output an
        acceleration value. The acceleration value must be some value between -10.0 and 10.0.

        Note that the acceleration value is really some control input that is used
        internally to compute the throttle to the car.

        Below is some example code where the car just outputs the control value 10.0
        """

        ego_velocity = obs.velocity
        target_velocity = obs.target_velocity
        dist_to_lead = obs.distance_to_lead

        d_curr_error = dist_to_lead - (self.distance_threshold + 10)
        d_proportional = 0.3 * (d_curr_error)
        d_derivative = 0.1 * ((d_curr_error - self.d_prev_error)/0.1)
        self.d_prev_error = d_curr_error

        v_curr_error = target_velocity - ego_velocity
        v_proportional = 0.3 * v_curr_error
        v_derivative = 0.1 * ((v_curr_error - self.v_prev_error)/0.1)
        self.v_prev_error = v_curr_error

        output1 = v_proportional + v_derivative
        output2 = d_proportional + d_derivative
        output = min(output1, output2)
        # return 10
        if output < -10.0:
            return -10.0
        elif output > 10.0:
            return 10.0
        else:
            return output

