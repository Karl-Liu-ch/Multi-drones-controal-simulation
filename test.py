from matplotlib import pyplot as plt
import numpy as np
from matplotlib import animation

class missile():
    def __init__(self, x, y, z, vx, vy, vz):
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.timestep = 1e-2
        self.movement = 0

        if np.sqrt(float(vx) ** 2.0 + float(vy) ** 2.0) == 0:
            self.alpha = None
        else:
            self.alpha = np.arcsin((float(vx) / np.sqrt(float(vx) ** 2.0 + float(vy) ** 2.0)))
        if np.sqrt(float(vx) ** 2.0 + float(vy) ** 2.0 + float(vz) ** 2.0) == 0:
            self.phi = None
        else:
            self.phi = np.arcsin((float(vz) / np.sqrt(float(vx) ** 2.0 + float(vy) ** 2.0 + float(vz) ** 2.0)))
        self.velocity = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

    def cal_position(self):
        self.x = self.x + self.vx * self.timestep
        self.y = self.x + self.vy * self.timestep
        self.z = self.x + self.vz * self.timestep
        self.movement = self.movement + np.sqrt((self.vx * self.timestep) ** 2+(self.vy * self.timestep) ** 2+(self.vz * self.timestep) ** 2)
        self.velocity = np.sqrt(self.vx ** 2 + self.vy ** 2 + self.vz ** 2)

    def update_v(self, vx, vy, vz):
        self.vx = vx
        self.vy = vy
        self.vz = vz

def Guidance_module(detect_p,detect_q,phi_missile,theta_missile,range):
    global K
    if range > 20:
        phi_missile = detect_q
        theta_missile = detect_p
    else:
        phi_missile = phi_missile + K * detect_q
        phi_missile = phi_missile - np.floor(phi_missile / (2 * np.pi)) * 2 * np.pi
        theta_missile = theta_missile + K * detect_p
        theta_missile = theta_missile - np.floor(theta_missile / (2 * np.pi)) * 2 * np.pi
    return phi_missile,theta_missile

def detection_module(range,delta_q,delta_p,epsilon):
    detect_range=range+epsilon
    detect_q=delta_q+epsilon
    detect_p=delta_p+epsilon
    return detect_range,detect_q,detect_p

def Missile_movement(x_missile,y_missile,z_missile,theta_missile,phi_missile,movement):
    global velocity_missile
    global delta_time

    V_missile_x = velocity_missile*np.cos(theta_missile)*np.cos(phi_missile)
    V_missile_y = velocity_missile*np.cos(theta_missile)*np.sin(phi_missile)
    V_missile_z = velocity_missile*np.sin(theta_missile)

    x_missile=x_missile+V_missile_x*delta_time
    y_missile=y_missile+V_missile_y*delta_time
    z_missile=z_missile+V_missile_z*delta_time

    movement = movement + np.sqrt((V_missile_x*delta_time)**2+(V_missile_y*delta_time)**2+(V_missile_z*delta_time)**2)
    return x_missile,y_missile,z_missile,V_missile_x,V_missile_y,V_missile_z,movement

def Target_movement(x_target,y_target,z_target,theta_target,phi_target):

    global velocity_target
    global delta_time

    V_target_x = velocity_target*np.cos(theta_target)*np.cos(phi_target)
    V_target_y = velocity_target*np.cos(theta_target)*np.sin(phi_target)
    V_target_z = velocity_target*np.sin(theta_target)

    x_target=x_target+V_target_x*delta_time
    y_target=y_target+V_target_y*delta_time
    z_target=z_target+V_target_z*delta_time

    return x_target, y_target, z_target,V_target_x,V_target_y,V_target_z

if __name__ == '__main__':
    pass