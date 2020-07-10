#!/usr/bin/env python
# -*- coding: utf-8 -*-

from vrep_remote_api import vrep
import abc

class VREP(metaclass=abc.ABCMeta):
    """
        this class is for vrep
    """
    def __init__(self, scene_path, server_ip, server_port=19997, synchronous=True):
        self._server_ip = server_ip
        self._server_port = server_port
        self._scene_path = scene_path
        self._synchronous = synchronous

        self.client_id = None

        self.one_shot = vrep.simx_opmode_oneshot
        self.one_shot_wait = vrep.simx_opmode_oneshot_wait
        self.buffer = vrep.simx_opmode_buffer
        self.blocking = vrep.simx_opmode_blocking
        self.streaming = vrep.simx_opmode_streaming

        self._init_vrep()
        

    def _init_vrep(self):
        vrep.simxFinish(-1)
        delay = 5
        flag = False
        while delay:
            self.client_id = vrep.simxStart(self._server_ip, self._server_port,
                                             True, True, 5000, 5)
            if self.client_id != -1:
                print("Connected to V-REP successfully!!!")
                flag = True
                break
            else:
                print("Continue to connect to V-REP.")
            
            time.sleep(1)
            delay -= 1
        if flag:
            pass
        else:
            print("Can not connect to V-REP!")
    
    def close_vrep(self):
        """
            close vrep
        """
        print("Close V-REP!")
        vrep.simxFinish(self.client_id)

    def start_simulation(self):
        """
            start simulation
        """
        print("Start Simulation!")
        if self._synchronous:
            vrep.simxSynchronous(self.client_id, True)
            vrep.simxSynchronousTrigger(self.client_id)
        vrep.simxStartSimulation(self.client_id, self.blocking)

    def stop_simulation(self):
        """
            stop simulation
        """
        print("Stop Simulation!")
        vrep.simxStopSimulation(self.client_id, self.blocking)

    def load_scene(self, scene_path=None):
        """
            load scene
        """
        if scene_path:
            vrep.simxLoadScene(self.client_id, scene_path, 0, self.blocking)
        else:
            vrep.simxLoadScene(self.client_id, self._scene_path, 0, self.blocking)

    def close_scene(self):
        """
            close scene
        """
        self.stop_simulation()
        vrep.simxCloseScene(self.client_id, self.blocking)


