import rospy

from pid import PID
from lowpass import LowPassFilter

FILT_TAU_ACCEL = 0.1 # filter time constant for low pass filter long. acceleration
TS = 0.05 # cycle time for 20 Hz 

LONG_JERK_LIMIT = 3.0

MIN_NUM = float('-inf')
MAX_NUM = float('inf')

class LongController(object):
    def __init__(self,vehicle_mass,brake_deadband,decel_limit,accel_limit,wheel_radius):

        self.n_wheel_accel = 2 # 2 wheel drive
        self.n_wheel_deccel = 4 # 4 wheel brake
        
        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.max_force = vehicle_mass * accel_limit # max throttle force
        self.min_force = vehicle_mass * decel_limit # max brake force (negative!)
        self.force_brake_deadband = -1.0 * vehicle_mass * brake_deadband  

        # This is from the simulator!!!- perhaps not valid for real vehicle
        #
        # throttle at 0.25 pedel position
        # v_start: 3.12 km/h v_end: 20.50 km/h --> delta_v = 4.83 m/s
        # t_start: 50.85 s   t_end: 62.15 s -> delta_t = 11.3 s
        # --> accel: 0.43 m/s^2
        #
        # throttle at 0.50 pedel position
        # v_start: 3.72 km/h v_end: 21.00 km/h --> delta_v = 4.80 m/s
        # t_start: 696.17 s   t_end: 698.87 s -> delta_t = 2.7 s
        # --> accel: 1.78 m/s^2
        #
        # throttle at 0.40 pedel position
        # v_start: 3.24 km/h v_end: 20.00 km/h --> delta_v = 4.66 m/s
        # t_start: 94.71 s   t_end: 100.61 s -> delta_t = 2.7 s
        # --> accel: 0.79 m/s^2

        # acceleration is not linear
        # guess for vehicle 
        # throttle 1.0 --> accel 3 m/s^2

        self.max_accel = 3.0 # 
        self.ref_force_throttle = vehicle_mass * self.max_accel # force at 100% throttle position

        
        # init low pass filter for acceleration
        self.accel_LowPassFilter = LowPassFilter(FILT_TAU_ACCEL,TS)
        self.last_spd = 0.0
        
        self.last_target_accel = 0.0
        self.last_expected_spd = 0.0
        
        self.last_brake_actv = False # brake active
        self.last_force = 0.0
        
        # init PID
        self.accel_PID = PID(kp=2.0*vehicle_mass*wheel_radius, ki=0.015*vehicle_mass*wheel_radius, kd=0.2*vehicle_mass*wheel_radius)
        
    def control(self,target_spd,current_spd,delta_t):
        
        # calulate filter acceleration of vehicle
        accel_raw = (current_spd - self.last_spd) / delta_t
        accel_filt = self.accel_LowPassFilter.filt(accel_raw)
        
        # calculate speed error
        spd_err = target_spd - current_spd
        # calulate target acceleration from speed error
        # could be tuned
        if target_spd > 0.05:
            k_accel = 0.2
        else:
            k_accel = 1.0
        if (abs(spd_err)<1.0):
            target_accel = k_accel * spd_err
        else:
            target_accel = k_accel * spd_err * spd_err * spd_err
        
        
        # limit jerk
        accel_change_limit = LONG_JERK_LIMIT * delta_t
        target_accel = max(min(target_accel, self.last_target_accel+accel_change_limit), self.last_target_accel-accel_change_limit)
        
        # check for min and max allowed acceleration
        target_accel = max( min(target_accel, self.accel_limit), self.decel_limit)
        trq = 0.0
        force_feedForward = 0.0
        force_PID = 0.0

        # calculate an expected speed from last target values
        # this can be used in PID to large errors, when target_spd jumps
        # it's smooth and near the expected behavior
        expected_spd = self.last_expected_spd + target_accel*delta_t
        expected_spd = max(min(target_spd, expected_spd),0.0)
        if target_spd > expected_spd:
            expected_spd = max(current_spd, expected_spd, self.last_expected_spd+0.1*delta_t)
        elif target_spd < expected_spd:
            expected_spd = min(current_spd, expected_spd, self.last_expected_spd-0.1*delta_t)
        # assert valid range
        expected_spd = max(min(target_spd, expected_spd),0.0)
        
        if target_spd > 0.05 or current_spd > 1.0:
            # overall resistance (tuned in simulator)
            force_resistance = 4*current_spd*current_spd + 40*current_spd + 40

            # calculate force from target_accel using mass
            # F = m * a
            force_feedForward = target_accel * self.vehicle_mass + force_resistance
            
            # use PID to get better control performance
            expected_err = expected_spd - current_spd
            force_PID = self.accel_PID.step(expected_err, delta_t, mn=MIN_NUM, mx=MAX_NUM)
                            
            # calulate overall torque
            force = force_feedForward + force_PID
            
        else: # hold vehicle 
            force = min(self.last_force, self.min_force / 2)
            self.accel_PID.freeze()
        
        # calulate throttle
        # published throttle is a percentage [0 ... 1]!!!
        if force > 0.:
            throttle = force / self.ref_force_throttle * 1.0
            brake = 0.0
            self.last_brake_actv = False
        elif self.last_brake_actv:
            throttle = 0.0
            brake = -force * self.wheel_radius / self.n_wheel_accel
            self.last_brake_actv = True
        elif force < self.force_brake_deadband or (force < 0. and target_spd < 1.0):
            throttle = 0.0
            brake = -force * self.wheel_radius / self.n_wheel_accel
            self.last_brake_actv = True
        else:
            throttle = 0.0
            brake = 0.0
            self.last_brake_actv = False
            
        # output for debug
        #rospy.logwarn("accel_raw: %f" % accel_raw + "; accel_filt: %f" % accel_filt + "; target_accel: %f" % target_accel + "; current_spd: %f" % current_spd)
        #rospy.logwarn("target_spd: {0:.2f} km/h, current_spd: {1:.2f} km/h, expected_spd: {2:.2f} km/h, target_accel: {3:.2f} m/ss, accel: {4:.2f} m/ss, throttle: {5:.2f}, brake: {6:.2f} Nm, force_PID: {7:.2f} N".format(target_spd * 3.6, current_spd * 3.6, expected_spd*3.6, target_accel, accel_filt, throttle, brake, force_PID))

        
            
        # write values for next iteration
        self.last_spd = current_spd
        self.last_target_accel = target_accel
        self.last_expected_spd = expected_spd
        self.last_force = force
        
        
        
        return throttle, brake
        
    def reset(self,current_spd):
        self.accel_PID.reset()
        self.last_spd = current_spd
        self.last_target_accel = 0.0
        self.last_expected_spd = current_spd
        self.last_force = 0.0

