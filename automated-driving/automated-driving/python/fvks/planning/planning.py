import numpy as np
from math import isclose
from itertools import combinations
import xml.etree.ElementTree as et
from xml.dom import minidom
import pathlib
import datetime

import pyfvks
from fvks.scenario.commonroad.util import ExactValue, UncertainIntervalScalar
import fvks.scenario.commonroad.file_reader


class PlanningTask:
    def __init__(self, planning_problems=list()):
        if planning_problems and not all(isinstance(p, PlanningProblem) for p in planning_problems):
            raise Exception()
        self.planning_problems = planning_problems

    def add_planning_problem(self, planning_problem):
        if isinstance(planning_problem, PlanningProblem):
            self.planning_problems.append(planning_problem)
        else:
            raise Exception()

    def find_planning_problem_by_id(self, planning_problem_id):
        for p in self.planning_problems:
            if p.planning_problem_id == planning_problem_id:
                return p
        raise Exception()

    # ToDo test function
    def is_solution_valid(self, scenario):
        if not self.all_planning_problems_are_solved():
            return False
        elif(not self._all_goals_reached(scenario) or
             not self._all_solutions_are_collision_free(scenario)):
            return False
        return True

    def all_planning_problems_are_solved(self):
        for p in self.planning_problems:
            if not p.has_solution():
                print('Error: Not all planning problems are solved.')
                return False
        return True

    def _all_goals_reached(self, scenario):
        for p in self.planning_problems:
            if not p.goal_reached(p.car.trajectory,
                                     scenario):
                print('Goal is not reached for planning problem ', p.planning_problem_id)
                return False
        return True

    def _all_solutions_are_collision_free(self, scenario):
        # Test for collisions with the environment
        cc = scenario.create_collision_checker()
        for p in self.planning_problems:
            traj = p.car.create_collision_object()
            if cc.collide(traj):
                print('Solution trajectory of planning problem', p.planning_problem_id, 'collides with an obstacle.')
                return False
        # Check if ego vehicles collide
        for p_1, p_2 in combinations(self.planning_problems, 2):
            traj_1 = p_1.car.create_collision_object()
            traj_2 = p_2.car.create_collision_object()
            if traj_1.collide(traj_2):
                print('Ego vehicles', p_1.planning_problem_id, 'and', p_2.planning_problem_id, 'collide.')
                return False
        return True


class PlanningProblem:
    def __init__(self, planning_problem_id, initial_state, goal_states):
        self.planning_problem_id = planning_problem_id
        self.initial_state = initial_state
        self.goal = GoalRegion(goal_states)

        self.car = None
        self._has_solution = False

    def goal_reached(self, trajectory, scenario):
        for i, state in enumerate(trajectory.state_list):
            if self.goal.is_reached(state, scenario):
                return True, i
        return False, i

    def add_solution(self, trajectory, dt, length=4.0, width=1.8):
        if not isinstance(trajectory, fvks.scenario.Trajectory):
            raise Exception()
        self._has_solution = True
        self.car = fvks.scenario.SimulatedCar(trajectory, dt, self.planning_problem_id, length, width)

    def has_solution(self):
        return self._has_solution


class GoalRegion:
    def __init__(self, state_list):
        self.state_list = state_list

    def is_reached(self, state, scenario):
        is_reached_list = list()
        for goal_state in self.state_list:
            goal_state_fields = set(goal_state._fields)
            state_fields = set(state._fields)

            if not goal_state_fields.issubset(state_fields):
                is_reached_list.append(False)
                continue
            is_reached = True
            if hasattr(goal_state, 'position') and hasattr(state, 'position'):
                if type(goal_state.position) == list:
                    for pos in goal_state.position:
                        is_reached = is_reached and self._check_position(state.position, pos, scenario.lanelet_network)
                else:
                    is_reached = is_reached and self._check_position(state.position, goal_state.position,
                                                                     scenario.lanelet_network)
            if hasattr(goal_state, 'orientation') and hasattr(state, 'orientation'):
                is_reached = is_reached and self._check_value_exact_or_interval(state.orientation,
                                                                                goal_state.orientation)
            if hasattr(goal_state, 'time') and hasattr(state, 'time'):
                is_reached = is_reached and self._check_value_exact_or_interval(state.time, goal_state.time)
            if hasattr(goal_state, 'velocity') and hasattr(state, 'velocity'):
                is_reached = is_reached and self._check_value_exact_or_interval(state.velocity, goal_state.velocity)
            if hasattr(goal_state, 'acceleration') and hasattr(state, 'acceleration'):
                is_reached = is_reached and self._check_value_exact_or_interval(state.acceleration,
                                                                                goal_state.acceleration)
            is_reached_list.append(is_reached)
        return np.any(is_reached_list)

    @classmethod
    def _check_position(cls, position, desired_position, lanelet_network):
        if (type(desired_position) == pyfvks.collision.Point or
            type(desired_position) == pyfvks.collision.RectAABB or
            type(desired_position) == pyfvks.collision.RectOBB or
            type(desired_position) == pyfvks.collision.Circle or
            type(desired_position) == pyfvks.collision.Triangle or
            type(desired_position) == pyfvks.collision.ShapeGroup):
            point = pyfvks.collision.Point(position[0], position[1])
            is_reached = point.collide(desired_position)
        elif type(desired_position) == fvks.scenario.Lanelet:
            lanelet = lanelet_network.find_nearest_point_on_lanelet([position[0], position[1]])[2]
            is_reached = lanelet.lanelet_id == desired_position.lanelet_id
        else:
            raise Exception()
        return is_reached

    @classmethod
    def _check_value_exact_or_interval(cls, value, desired_value):
        if type(desired_value) == ExactValue:
            is_reached = isclose(value, desired_value.value)
        elif type(desired_value) == UncertainIntervalScalar:
            is_reached = desired_value.value_lo <= value <= desired_value.value_hi
        else:
            raise Exception()
        return is_reached


class CommonRoadSolutionFileWriter:

    def __init__(self, scenario, planning_task):
        self.scenario = scenario
        self.planning_task = planning_task
        self._root_node = et.Element('commonRoadSolution')

    def _write_header(self):
        self._root_node.set('timeStepSize', str(self.scenario.dt))
        self._root_node.set('commonRoadVersion', '2017a')
        self._root_node.set('benchmarkID', self.scenario.benchmark_id)
        self._root_node.set('date', datetime.datetime.today().strftime('%d-%b-%Y'))

    def _add_all_solutions_from_planning_task(self):
        for planning_problem in self.planning_task.planning_problems:
            self._root_node.append(self._create_solution_node(planning_problem))

    def _create_solution_node(self, planning_problem):
        solution_node = et.Element('planningProblemSolution')
        solution_node.set('id', str(planning_problem.planning_problem_id))
        if planning_problem.has_solution:
            solution_node.append(fvks.scenario.commonroad.file_writer.SimulatedCarXMLNode.
                                 create_node(planning_problem.car))
        return solution_node

    def _dump(self):
        rough_string = et.tostring(self._root_node, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")

    def write_to_file(self, filename):
        if pathlib.Path(filename).is_file():
            print("File already exist. Aborting.")
            return
        file = open(filename, "w")
        self._write_header()
        self._add_all_solutions_from_planning_task()
        file.write(self._dump())
        file.close()


class CommonRoadSolutionFileReader:

    def __init__(self, scenario_filename, solution_filename):
        self._scenario_filename = scenario_filename
        self._solution_filename = solution_filename
        self._tree = None
        self._time_step = None
        self._benchmark_id = None
        self._scenario = None
        self._planning_task = None

    def open(self):
        fr = fvks.scenario.commonroad.file_reader.CommonRoadFileReader(self._scenario_filename)
        self._scenario, self._planning_task = fr.open()
        self._parse_file()
        self._benchmark_id = self._get_benchmark_id()
        if not self._benchmark_id == self._scenario.benchmark_id:
            print('Benchmark ID of solution is different than in CommonRoad scenario', self._scenario_filename,
                  '. Aborting.')
            print(self._benchmark_id)
            print(self._scenario.benchmark_id)
            raise Exception()

        self._read_planning_task_solutions(self._scenario.dt)
        return self._scenario, self._planning_task

    def _parse_file(self):
        self._tree = et.parse(self._solution_filename)

    def _get_benchmark_id(self):
        return self._tree.getroot().get('benchmarkID')

    def _read_planning_task_solutions(self, time_step):
        for xml_node in self._tree.findall('planningProblemSolution'):
            planning_problem_id, car = self._read_planning_problem_solution(xml_node, time_step)
            planning_problem = self._planning_task.find_planning_problem_by_id(planning_problem_id)
            planning_problem.add_solution(car.trajectory, time_step, car.length, car.width)
        if not self._planning_task.all_planning_problems_are_solved():
            print('Not all planning problems are solved.')
            #raise Exception()

    def _read_planning_problem_solution(self, xml_node, time_step):
        planning_problem_id = int(xml_node.get('id'))
        xml_node_car = xml_node.find('obstacle')
        car = fvks.scenario.commonroad.file_reader.ObstacleFactory.create_from_xml_node(xml_node_car, time_step)
        return planning_problem_id, car

