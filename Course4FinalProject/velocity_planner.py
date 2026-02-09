#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Author: Ryan De Iaco
# Additional Comments: Carlos Wang
# Date: October 29, 2018

import numpy as np
from math import sin, cos, pi, sqrt

class VelocityPlanner:
    def __init__(self, time_gap, a_max, slow_speed, stop_line_buffer):
        self._time_gap         = time_gap
        self._a_max            = a_max
        self._slow_speed       = slow_speed
        self._stop_line_buffer = stop_line_buffer
        self._prev_trajectory  = [[0.0, 0.0, 0.0]]

    def get_open_loop_speed(self, timestep):
        if len(self._prev_trajectory) == 1:
            return self._prev_trajectory[0][2] 
        
        if timestep < 1e-4:
            return self._prev_trajectory[0][2]

        for i in range(len(self._prev_trajectory)-1):
            distance_step = np.linalg.norm(np.subtract(self._prev_trajectory[i+1][0:2], 
                                                       self._prev_trajectory[i][0:2]))
            velocity = self._prev_trajectory[i][2]
            
            # FIX: Avoid division by zero
            time_delta = distance_step / (velocity + 1e-6)
           
            if time_delta > timestep:
                v1 = self._prev_trajectory[i][2]
                v2 = self._prev_trajectory[i+1][2]
                v_delta = v2 - v1
                interpolation_ratio = timestep / time_delta
                return v1 + interpolation_ratio * v_delta

            else:
                timestep -= time_delta

        return self._prev_trajectory[-1][2]

    def compute_velocity_profile(self, path, desired_speed, ego_state, 
                                 closed_loop_speed, decelerate_to_stop, 
                                 lead_car_state, follow_lead_vehicle):
        profile = []
        start_speed = ego_state[3]
        if decelerate_to_stop:
            profile = self.decelerate_profile(path, start_speed)
        elif follow_lead_vehicle:
            profile = self.follow_profile(path, start_speed, desired_speed, 
                                          lead_car_state)
        else:
            profile = self.nominal_profile(path, start_speed, desired_speed)

        if len(profile) > 1:
            interpolated_state = [(profile[1][0] - profile[0][0]) * 0.1 + profile[0][0], 
                                  (profile[1][1] - profile[0][1]) * 0.1 + profile[0][1], 
                                  (profile[1][2] - profile[0][2]) * 0.1 + profile[0][2]]
            del profile[0]
            profile.insert(0, interpolated_state)

        self._prev_trajectory = profile
        return profile

    def decelerate_profile(self, path, start_speed): 
        profile          = []
        slow_speed       = self._slow_speed
        stop_line_buffer = self._stop_line_buffer

        decel_distance = calc_distance(start_speed, slow_speed, -self._a_max)
        brake_distance = calc_distance(slow_speed, 0, -self._a_max)

        path_length = 0.0
        for i in range(len(path[0])-1):
            path_length += np.linalg.norm([path[0][i+1] - path[0][i], 
                                           path[1][i+1] - path[1][i]])

        stop_index = len(path[0]) - 1
        temp_dist = 0.0
        while (stop_index > 0) and (temp_dist < stop_line_buffer):
            temp_dist += np.linalg.norm([path[0][stop_index] - path[0][stop_index-1], 
                                         path[1][stop_index] - path[1][stop_index-1]])
            stop_index -= 1

        if brake_distance + decel_distance + stop_line_buffer > path_length:
            speeds = []
            vf = 0.0
            for i in reversed(range(stop_index, len(path[0]))):
                speeds.insert(0, 0.0)
            for i in reversed(range(stop_index)):
                dist = np.linalg.norm([path[0][i+1] - path[0][i], 
                                       path[1][i+1] - path[1][i]])
                vi = calc_final_speed(vf, -self._a_max, dist)
                if vi > start_speed:
                    vi = start_speed
                speeds.insert(0, vi)
                vf = vi
            for i in range(len(speeds)):
                profile.append([path[0][i], path[1][i], speeds[i]])
        else:
            brake_index = stop_index 
            temp_dist = 0.0
            while (brake_index > 0) and (temp_dist < brake_distance):
                temp_dist += np.linalg.norm([path[0][brake_index] - path[0][brake_index-1], 
                                             path[1][brake_index] - path[1][brake_index-1]])
                brake_index -= 1

            decel_index = 0
            temp_dist = 0.0
            while (decel_index < brake_index) and (temp_dist < decel_distance):
                temp_dist += np.linalg.norm([path[0][decel_index+1] - path[0][decel_index], 
                                             path[1][decel_index+1] - path[1][decel_index]])
                decel_index += 1

            vi = start_speed
            for i in range(decel_index): 
                dist = np.linalg.norm([path[0][i+1] - path[0][i], 
                                       path[1][i+1] - path[1][i]])
                vf = calc_final_speed(vi, -self._a_max, dist)
                if vf < slow_speed:
                    vf = slow_speed
                profile.append([path[0][i], path[1][i], vi])
                vi = vf

            for i in range(decel_index, brake_index):
                profile.append([path[0][i], path[1][i], vi])
                
            for i in range(brake_index, stop_index):
                dist = np.linalg.norm([path[0][i+1] - path[0][i], 
                                       path[1][i+1] - path[1][i]])
                vf = calc_final_speed(vi, -self._a_max, dist)
                profile.append([path[0][i], path[1][i], vi])
                vi = vf

            for i in range(stop_index, len(path[0])):
                profile.append([path[0][i], path[1][i], 0.0])

        return profile

    def follow_profile(self, path, start_speed, desired_speed, lead_car_state):
        profile = []
        min_index = len(path[0]) - 1
        min_dist = float('Inf')
        for i in range(len(path)):
            dist = np.linalg.norm([path[0][i] - lead_car_state[0], 
                                   path[1][i] - lead_car_state[1]])
            if dist < min_dist:
                min_dist = dist
                min_index = i

        desired_speed = min(lead_car_state[2], desired_speed)
        ramp_end_index = min_index
        distance = min_dist
        distance_gap = desired_speed * self._time_gap
        while (ramp_end_index > 0) and (distance > distance_gap):
            distance += np.linalg.norm([path[0][ramp_end_index] - path[0][ramp_end_index-1], 
                                        path[1][ramp_end_index] - path[1][ramp_end_index-1]])
            ramp_end_index -= 1

        if desired_speed < start_speed:
            decel_distance = calc_distance(start_speed, desired_speed, -self._a_max)
        else:
            decel_distance = calc_distance(start_speed, desired_speed, self._a_max)

        vi = start_speed
        for i in range(ramp_end_index + 1):
            dist = np.linalg.norm([path[0][i+1] - path[0][i], 
                                   path[1][i+1] - path[1][i]])
            if desired_speed < start_speed:
                vf = calc_final_speed(vi, -self._a_max, dist)
            else:
                vf = calc_final_speed(vi, self._a_max, dist)

            profile.append([path[0][i], path[1][i], vi])
            vi = vf

        for i in range(ramp_end_index + 1, len(path[0])):
            profile.append([path[0][i], path[1][i], desired_speed])

        return profile

    def nominal_profile(self, path, start_speed, desired_speed):
        profile = []
        if desired_speed < start_speed:
            accel_distance = calc_distance(start_speed, desired_speed, -self._a_max)
        else:
            accel_distance = calc_distance(start_speed, desired_speed, self._a_max)

        ramp_end_index = 0
        distance = 0.0
        while (ramp_end_index < len(path[0])-1) and (distance < accel_distance):
            distance += np.linalg.norm([path[0][ramp_end_index+1] - path[0][ramp_end_index], 
                                        path[1][ramp_end_index+1] - path[1][ramp_end_index]])
            ramp_end_index += 1

        vi = start_speed
        for i in range(ramp_end_index):
            dist = np.linalg.norm([path[0][i+1] - path[0][i], 
                                   path[1][i+1] - path[1][i]])
            if desired_speed < start_speed:
                vf = calc_final_speed(vi, -self._a_max, dist)
                if vf < desired_speed:
                    vf = desired_speed
            else:
                vf = calc_final_speed(vi, self._a_max, dist)
                if vf > desired_speed:
                    vf = desired_speed

            profile.append([path[0][i], path[1][i], vi])
            vi = vf

        for i in range(ramp_end_index+1, len(path[0])):
            profile.append([path[0][i], path[1][i], desired_speed])

        return profile

def calc_distance(v_i, v_f, a):
    if abs(a) < 1e-5: return 0.0
    return (v_f**2 - v_i**2) / (2.0 * a)

def calc_final_speed(v_i, a, d):
    disc = v_i**2 + 2.0 * a * d
    if disc < 0: return 0.0
    return sqrt(disc)