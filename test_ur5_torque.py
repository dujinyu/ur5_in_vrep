#!/usr/bin/env python
# -*- coding: utf-8 -*-

from vrep_remote_api import vrep

import numpy as np
import time
import csv
import matplotlib.pyplot as plt

from class_vrep import VREP

class UR5(VREP):
    """
        this class is for ur5 robot
    """
    def __init__(self, scene_path, ip="127.0.0.1", port=19997, synchronous=True):
        VREP.__init__(self, scene_path, server_ip=ip, server_port=port, synchronous=synchronous)
        self._base_name = "UR5"
        self._joint_names = [self._base_name + "_joint" + str(i+1) for i in range(6)]

        self._deg2rad = np.pi / 180

    def init_simulation(self):
        self.load_scene()
        self.start_simulation()
        self._get_handles()

    def end_simulation(self):
        self.stop_simulation()

    def move(self, vel=0.1):
        err_code = vrep.simxSetJointTargetVelocity(self.client_id, self._joint_handles[0], vel, self.one_shot)
        print("err_code:", err_code)
    
    def move_force_mode(self, force, vel=0.1, specified_joint=None):
        """
        Args:
            force: [f1, f2, ..., fn], 
        """
        if specified_joint:
            joint_handles = [None for _ in range(len(specified_joint))]
            for idx, joint in enumerate(specified_joint):
                joint_handles[idx] = self._joint_handles[joint-1]
        else:
            joint_handles = self._joint_handles

        for idx, joint_handle in enumerate(joint_handles):
            vrep.simxSetJointTargetVelocity(self.client_id, joint_handle, self.sign(force[idx]) * vel, self.one_shot)
            vrep.simxSetJointForce(self.client_id, joint_handle, np.abs(force[idx]), self.one_shot)
        
    def sign(self, data):
        if data >= 0:
            return 1
        else:
            return -1

    def move_position_mode(self, pos):
        pass

    def write2csv(self, filename, data, file_header=None):
        """
        Args:
            filename: string
            data: [d1, d2, ..., dn], M x N array or np.darray
        """
        file = open(filename, "w")
        writer = csv.writer(file)
        if file_header != None:
            writer.writerow(file_header)
        
        for d in data:
            writer.writerow(d)

        file.close()

    def read_from_csv(self, filename):
        """
        Args:
            filename: string
        return:
            data: data in csv file
        """
        data = []
        file = open(filename)
        reader = csv.reader(file)    # <_csv.reader object at 0x7fb7540a1668>
        # return list(reader)    # M x N matrix
        for line in reader:
            data.append(list(map(float, line)))

        return np.array(data)

    def get_state(self, specified_joint=None):
        """
        Args:
            specified_joint: [1, 5] represent joint 1 and 5
        return:
            state: [angle, ang_vel]
        """
        state = {}
        joint_position = np.zeros(len(self._joint_names))
        joint_velocity = np.zeros_like(joint_position)
        for idx, joint_handle in enumerate(self._joint_handles):
            # print(joint_handle)
            _, joint_position[idx] = vrep.simxGetJointPosition(self.client_id, joint_handle, self.streaming)
            _, joint_velocity[idx] = vrep.simxGetObjectFloatParameter(self.client_id, joint_handle, 
                                                                      vrep.sim_jointfloatparam_velocity, self.streaming)
        state["joint_position"] = joint_position
        state["joint_velocity"] = joint_velocity
        return state

    def get_action(self, specified_joint=None):
        """
        Args:
            specified_joint: [1, 5] represent joint 1 and 5
        return:
            action: [force]
        """
        if specified_joint:
            joint_handles = [None for _ in range(len(specified_joint))]
            for idx, joint in enumerate(specified_joint):
                joint_handles[idx] = self._joint_handles[joint-1]
        else:
            joint_handles = self._joint_handles

        action = {}
        joint_force = np.zeros(len(joint_handles))
        for idx, joint_handle in enumerate(joint_handles):
            _, joint_force[idx] = vrep.simxGetJointForce(self.client_id, joint_handle, self.streaming)

        action["force"] = joint_force
        return action
            

    def _get_handles(self):
        _, self._base_handle = vrep.simxGetObjectHandle(self.client_id, self._base_name, self.blocking)
        self._joint_handles = [None for _ in range(len(self._joint_names))]
        for idx, joint_name in enumerate(self._joint_names):
            _, self._joint_handles[idx] = vrep.simxGetObjectHandle(self.client_id, joint_name, self.blocking)

def record_data(ur5):
    ur5.write2csv("./ur5_state_1.csv", state_list)
    ur5.write2csv("./ur5_action_1.csv", action_list)

def run(ur5):
    ur5.init_simulation()
    t = 100
    state_list = []
    action_list = []
   
    # force_matrix = ur5.read_from_csv("./ur5_state.csv")
    # for force in force_matrix:
    #     ur5.move_force_mode(force)
    #     time.sleep(0.5)
    
    ur5.move()
    while t:
        # ur5.move()
        state = ur5.get_state()
        print(state)
        state_list.append(np.concatenate((state["joint_position"], state["joint_velocity"])))
        action = ur5.get_action()
        print(action)
        action_list.append(action["force"])
        time.sleep(0.5)
        t -= 1
    ur5.end_simulation()

    # record_data()

def mod(x):
    if x >= 0:
        return (x * (180 / np.pi)) % 360
    else:
        return -((abs(x) * (180 / np.pi)) % 360)

def plot_state(ur5):
    state_matrix = ur5.read_from_csv("./ur5_state.csv")

    x_axis =  np.linspace(0, 50, state_matrix.shape[0])
    fig1 = plt.figure()
    plt.title("joint angles")
    for i in range(6): 
        plt.plot(x_axis, list(map(mod, state_matrix[:, i])), label="joint_" + str(i+1))
        # plt.plot(x_axis, state_matrix[:, i], label="joint_" + str(i))
    plt.legend()

    fig2 = plt.figure()
    plt.title("joint velocity")
    for i in range(6, 12): 
        plt.plot(x_axis, state_matrix[:, i], label="joint_" + str(i-5))
        # plt.plot(x_axis, state_matrix[:, i], label="joint_" + str(i))
    plt.legend()

def plot_action(ur5):
    action_matrix = ur5.read_from_csv("./ur5_action.csv")

    x_axis = np.linspace(0, 50, action_matrix.shape[0])
    fig = plt.figure()
    plt.title("joint force")
    for i in range(6):
        plt.plot(x_axis, action_matrix[:, i], label="joint_" + str(i+1))
    plt.legend()

def main():
    # ur5 = UR5("./vrep_scene/ur5_torque.ttt")
    ur5 = UR5("./vrep_scene/ur5_test.ttt", synchronous=False)
    run(ur5)
    # plot_state(ur5)
    # plot_action(ur5)
    # plt.show()

if __name__ == "__main__":
    main() 


