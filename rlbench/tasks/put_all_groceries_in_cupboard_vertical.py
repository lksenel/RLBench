from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped, \
    DetectedSeveralCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
import time

# Change the order by changing the order of the objects in the list
GROCERY_NAMES = [
    # 'crackers',
    'sugar',
    # 'spam',
    'soup',
    # 'mustard',
    # 'chocolate jello',
    'strawberry jello',
]

UPRIGHT_OJECTS = [
    'soup',
    'spam',
    'mustard'
]



class PutAllGroceriesInCupboardVertical(Task):

    def init_task(self) -> None:
        self.groceries = [Shape(name.replace(' ', '_'))
                          for name in GROCERY_NAMES]
        self.grasp_points = [Dummy('%s_grasp_point' % name.replace(' ', '_'))
                             for name in GROCERY_NAMES]
        self.goals = [Dummy('goal_%s' % name.replace(' ', '_'))
                      for name in GROCERY_NAMES]
        self.drop_point = Dummy('drop_point')
        self.waypoint0 = Dummy('waypoint0')
        self.waypoint1 = Dummy('waypoint1')
        self.waypoint2 = Dummy('waypoint2')
        self.waypoint3 = Dummy('waypoint3')
        self.waypoint4 = Dummy('waypoint4')
        self.waypoint5 = Dummy('waypoint5')
        self.returning = False
        self.register_graspable_objects(self.groceries)
        self.boundary = SpawnBoundary([Shape('groceries_boundary')])

        self.register_waypoint_ability_start(0, self._move_to_next_target)  
        self.register_waypoint_ability_start(3, self._move_to_drop_zone)
        self.register_waypoints_should_repeat(self._repeat)
        self.groceries_to_place = len(GROCERY_NAMES)
        self.groceries_placed = 0

    def init_episode(self, index: int) -> List[str]:
        self.groceries_placed = 0
        self.boundary.clear()
        self.taken_back = False
        self.step_count = 0
        [self.boundary.sample(g, min_distance=0.25) for g in self.groceries]

        # drop point should be the same with the initial pick up point for the first upright object
        for name in GROCERY_NAMES:
            if name in UPRIGHT_OJECTS:
                self.drop_point.set_pose(Dummy('%s_grasp_point' % name.replace(' ', '_')).get_pose() + [0, 0, 0.05, 0, 0, 0, 0])
                # print("drop point pose: ", self.drop_point.get_pose())
                break
        
        # Comment this out for the actual demo
        # for i, name in enumerate(GROCERY_NAMES):
        #     if name in UPRIGHT_OJECTS:
        #         self.grasp_points[i] = Dummy('%s_grasp_point_upright' % name.replace(' ', '_'))

        self.register_success_conditions(
            [DetectedSeveralCondition(
                self.groceries[:self.groceries_to_place],
                ProximitySensor('success'), self.groceries_to_place),
             NothingGrasped(self.robot.gripper)])
        return ['put all of the groceries in the cupboard',
                'pick up all of the groceries and place them in the cupboard',
                'move the groceries to the shelves',
                'put the groceries on the table into the cupboard',
                'put away the groceries in the cupboard']

    def variation_count(self) -> int:
        return 1

    def boundary_root(self) -> Object:
        return Shape('boundary_root')

    def is_static_workspace(self) -> bool:
        return True

    def _move_to_next_target(self, _):
        if self.groceries_placed >= self.groceries_to_place:
            raise RuntimeError('Should not be here.')



        # if the last object that is places is an upright object, then we need to reposition it
        if not self.taken_back and self.groceries_placed > 0 and GROCERY_NAMES[self.groceries_placed-1] in UPRIGHT_OJECTS:
            
            # self.waypoint1.set_pose(Dummy('waypoint1').get_pose())
            self.waypoint1.set_pose(self.grasp_points[self.groceries_placed-1].get_pose())

            # waypoint 0 and waypoint 2 are both 12 cm above of the waypoint 1
            # self.waypoint0.set_pose(self.grasp_points[self.groceries_placed-1].get_pose() + [0, 0, 0.12, 0, 0, 0, 0])
            # self.waypoint2.set_pose(self.grasp_points[self.groceries_placed-1].get_pose() + [0, 0, 0.12, 0, 0, 0, 0])

            self.returning = True
            # update the grasp points for the upright objects
            for i, name in enumerate(GROCERY_NAMES):
                if name in UPRIGHT_OJECTS:
                    self.grasp_points[i] = Dummy('%s_grasp_point_upright' % name.replace(' ', '_'))
        
        else:
            self.waypoint1.set_pose(
                self.grasp_points[self.groceries_placed].get_pose())

            # waypoint 0 and waypoint 2 are both 12 cm above of the waypoint 1
            # self.waypoint0.set_pose(self.grasp_points[self.groceries_placed].get_pose() + [0, 0, 0.12, 0, 0, 0, 0])
            # self.waypoint2.set_pose(self.grasp_points[self.groceries_placed].get_pose() + [0, 0, 0.12, 0, 0, 0, 0])

    def _move_to_drop_zone(self, _):
        if self.returning:
            self.waypoint3.set_pose(self.drop_point.get_pose())
            # print(f"waypoint3 pose: {self.waypoint3.get_pose()}")
            self.waypoint4.set_pose(self.drop_point.get_pose()) 
            # raise the arm before ending the episode
            drop_pose = self.drop_point.get_pose()
            drop_pose[2] = drop_pose[2] + 0.25
            self.waypoint5.set_pose(drop_pose) 
            
            self.groceries_placed -= 2
            self.taken_back = True
            self.returning = False
        else:

            self.waypoint3.set_pose(
                self.goals[self.groceries_placed].get_pose())

            # waypoint 4 is 27 cm further in x axis (absolute position) than waypoint 3
            self.waypoint4.set_pose( self.goals[self.groceries_placed].get_pose() + [0.275, 0, 0, 0, 0, 0, 0])
            # waypoint 5 hs the same position as waypoint 3
            self.waypoint5.set_pose(self.goals[self.groceries_placed].get_pose())

    def _repeat(self):
        self.groceries_placed += 1
        return self.groceries_placed < self.groceries_to_place


