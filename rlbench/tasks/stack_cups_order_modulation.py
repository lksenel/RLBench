from typing import List
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.const import colors
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary

from pyrep.objects.dummy import Dummy


class StackCupsOrderModulation(Task):

    def init_task(self) -> None:
        success_sensor = ProximitySensor('success')
        self.cup1 = Shape('cup1')
        self.cup2 = Shape('cup2')
        self.cup3 = Shape('cup3')
        self.cup1_visual = Shape('cup1_visual')
        self.cup2_visual = Shape('cup2_visual')
        self.cup3_visaul = Shape('cup3_visual')

        self.waypoint0 = Dummy('waypoint0')
        self.waypoint1 = Dummy('waypoint1')
        self.waypoint2 = Dummy('waypoint2')
     
        # waypoints to drop cup 1 back on the table
        self.waypoint4 = Dummy('waypoint4')

        
        # waypoint 10 opens the gripper by default
        # self.waypoint5= Dummy('waypoint5')
        # self.waypoint5.set_pose(self.waypoint0.get_pose())
        

        # # waypoints to pick up cup 1 again and place it on top of the other two
        # self.waypoint16 = Dummy('waypoint16')
        # self.waypoint16.set_pose(self.waypoint0.get_pose())
        
        # # waypoint 17 closes the gripper by default
        # self.waypoint17 = Dummy('waypoint17')
        # self.waypoint17.set_pose(self.waypoint1.get_pose())

        # self.waypoint18 = Dummy('waypoint18')
        # self.waypoint18.set_pose(self.waypoint2.get_pose())


        self.boundary = SpawnBoundary([Shape('boundary')])

        self.register_graspable_objects([self.cup1, self.cup2, self.cup3])
        self.register_success_conditions([
            DetectedCondition(self.cup1, success_sensor),
            DetectedCondition(self.cup3, success_sensor),
            NothingGrasped(self.robot.gripper)
        ])

    def init_episode(self, index: int) -> List[str]:

        self.step_count = 0
        
        self.variation_index = index
        target_color_name, target_rgb = colors[index]

        random_idx = np.random.choice(len(colors))
        while random_idx == index:
            random_idx = np.random.choice(len(colors))
        _, other1_rgb = colors[random_idx]

        random_idx = np.random.choice(len(colors))
        while random_idx == index:
            random_idx = np.random.choice(len(colors))
        _, other2_rgb = colors[random_idx]

        self.cup2_visual.set_color(target_rgb)
        self.cup1_visual.set_color(other1_rgb)
        self.cup3_visaul.set_color(other2_rgb)

        self.boundary.clear()
        self.boundary.sample(self.cup2, min_distance=0.15,
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        self.boundary.sample(self.cup1, min_distance=0.15,
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        self.boundary.sample(self.cup3, min_distance=0.15,
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))


        return ['stack the other cups on top of the %s cup' % target_color_name,
                'place two of the cups onto the odd cup out',
                'put the remaining two cups on top of the %s cup'
                % target_color_name,
                'pick up and set the cups down into the %s cup'
                % target_color_name,
                'create a stack of cups with the %s cup as its base'
                % target_color_name,
                'keeping the %s cup on the table, stack the other two onto it'
                % target_color_name]

    def variation_count(self) -> int:
        return len(colors)

    def step(self):
        if self.step_count == 0:
            self.waypoint4.set_pose(self.waypoint2.get_pose())
        self.step_count += 1