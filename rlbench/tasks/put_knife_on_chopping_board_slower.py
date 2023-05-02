from typing import List
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import ConditionSet, DetectedCondition, \
    NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task
import time

class PutKnifeOnChoppingBoardSlower(Task):

    def init_task(self) -> None:
        knife = Shape('knife')
        self._knife_block = Shape('knife_block')
        self._boundary = SpawnBoundary([Shape('boundary')])
        self.register_graspable_objects([knife])
        self.register_success_conditions([
            ConditionSet([DetectedCondition(knife, ProximitySensor('success')),
                          NothingGrasped(self.robot.gripper)],
                         order_matters=True)])
        
        self.interruption_delta = 50

        # # set control loop enabled to false for all joints
        # for joint in self.robot.arm.joints:
        #     joint.set_control_loop_enabled(False)


    def init_episode(self, index: int) -> List[str]:
        self.flag = False
        self.count = 0
        self._boundary.clear()
        self._boundary.sample(self._knife_block)
        return ['put the knife on the chopping board',
                'slide the knife out of the knife block and put it down on the '
                'chopping board',
                'place the knife on the chopping board',
                'pick up the knife and leave it on the chopping board',
                'move the knife from the holder to the chopping board']

    def variation_count(self) -> int:
        return 1

    def step(self):
        # get gripper state
        grasped = len(self.robot.gripper.get_grasped_objects()) > 0
        # self.count += 1
        # print(self.count)
        if grasped:

            self.count += 1
            if self.count == self.interruption_delta:
                print("Stop!!")
                time.sleep(2)
                print("Be careful and move more slowly!")
                time.sleep(1)
                self.flag = True
            
            if self.flag:
                time.sleep(0.015)


            # gripper_speed = self.robot.gripper.get_joint_target_velocities()
            # print("gripper speed: ", gripper_speed)
            # gripper_position = self.robot.gripper.get_joint_positions()
            # print("gripper position: ", gripper_position)

            # arm_target_speed = self.robot.arm.get_joint_target_velocities()  
            # arm_speed = self.robot.arm.get_joint_velocities()
            # print("arm target speed: ", arm_target_speed)
            # print("arm speed: ", arm_speed)  
            
            # target_speeds = [speed*0.8 for speed in arm_speed]
            # self.robot.arm.set_joint_target_velocities(target_speeds)
            # print("arm target speed: ", arm_target_speed)
            
            # reduce arm speed

            # if self.flag == False:
                # for joint in self.robot.arm.joints:
                #     joint.set_control_loop_enabled(False)
                # print("grasped knife, slowing down")
                # self.flag = True


            # time.sleep(0.04)