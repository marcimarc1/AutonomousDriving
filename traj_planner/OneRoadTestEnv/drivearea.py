import os
import matplotlib.pyplot as plt
from fvks.scenario.commonroad.file_reader import CommonRoadFileReader
import pyfvks
import numpy as np
from fvks.planning.planning import PlanningProblem
from fvks.scenario.trajectory import StateTupleFactory
from fvks.scenario.lanelet import LaneletNetwork, Lanelet
from fvks.visualization.draw_dispatch import draw_object
from scipy.spatial import distance

class morphingcommonroad:
    '''generating obstacles out of lanelets  and getting initial trajectory'''

    def __init__(self, filename):
        self.scenario, self.planning_task = CommonRoadFileReader(filename).open()

    def get_time_by_dynamic_obstacles(self):
        dynamic = self.get_dynamic_obstacles()
        small = 0
        for o in dynamic:
            if len(o.trajectory.state_list) > small:
                small = len(o.trajectory.state_list)
        timesteps = small
        return timesteps


    def para_lanelet(self, stepsize, side, lanelet_id):
        '''
        parametrizes the lanelet
        side = 1 equals left, side = 2 equals right
        stepsize should be between 0 and 1'''
        # TODO: change function, to just interpolate necessary lanelets with the lanelet id function
        para = np.array([])
        if side == 2:
            lanelets = self.scenario.lanelet_network.lanelets[lanelet_id]
            for i in range(len(lanelets.right_vertices) - 1):
                para = np.append(lanelets.left_vertices[i, :] + stepsize * lanelets.right_vertices[i + 1, :])
        if side == 1:
            lanelets = self.scenario.lanelet_network.lanelets[lanelet_id]
            for i in range(len(lanelets.left_vertices) - 1):
                para = np.append(lanelets.left_vertices[i, :] + stepsize * lanelets.left_vertices[i + 1, :])

        return para

    def dist_to_laneletedges(self, posvehicle):
        '''generates compute_distance for the left and right border of the lanelet'''
        dright = np.array([])
        lanelet_id = self.get_lanelet_id(posvehicle)
        pararight = self.para_lanelet(0.1, 2, lanelet_id)
        for i in pararight:
            dright = np.append(np.linalg.norm(posvehicle - i))
        dright = np.min(dright)
        paraleft = self.para_lanelet(0.1, 1, lanelet_id)
        dleft = np.array([])
        for i in paraleft:
            dleft = np.append(np.linalg.norm(posvehicle - i))
        dleft = np.min(dleft)
        return dleft, dright


    def generating_driving_corridor(self, reduced = None,def_goal = None, direction = None):
        '''generates obstacles besides the driving corridor so that the constraints can apply
        needs goal and initial state
        :returns list of points for the boundary constraints of the optimization problem
        :returns left and right points of the driving corridor '''

        if reduced == None and def_goal == None:
            # for problem in self.planning_task:
            problem = self.planning_task.planning_problems

            initial_pos = problem[0].initial_state
            goal_state = problem[0].goal

            # print(self.scenario.lanelet_network)
            lanelet_net = LaneletNetwork()
            lanelet_net.add_lanelet_network(self.scenario.lanelet_network)
            seg = lanelet_net.find_nearest_point_on_lanelet(initial_pos.position)

            position = goal_state.state_list[0].position

            if type(position) == pyfvks.collision.RectOBB:
                seg_goal = lanelet_net.find_nearest_point_on_lanelet(position.center())
                goal_pos_lanelet = seg_goal[2].lanelet_id
            else:
                seg_goal = []
                seg_goal.append([0])
                seg_goal.append(0)
                goal_pos_lanelet = position[0].lanelet_id
                seg_goal.append(position[0])


            initial_pos_lanelet_id = seg[2].lanelet_id
            #goal_pos_lanelet = seg_goal[2].lanelet_id

            goal_pos_lanelet_adj_right = None
            goal_pos_lanelet_adj_left = None
            #laneletadjacents for the
            if seg_goal[2].adj_left_same_direction and not None:
                goal_pos_lanelet_adj_left = seg_goal[2].adj_left
                if goal_pos_lanelet_adj_left == None:
                    goal_pos_lanelet_adj_left = goal_pos_lanelet
            if seg_goal[2].adj_right_same_direction:
                goal_pos_lanelet_adj_right = seg_goal[2].adj_right
                if goal_pos_lanelet_adj_right == None:
                    goal_pos_lanelet = goal_pos_lanelet
            # finds the lanelet_ids from start to goal position
            # DONE: extend function, so that we can set the goal on an adjacent lanelet and on more than one successor

            lanelet_id_list = ([initial_pos_lanelet_id])
            for i in lanelet_net.lanelets:
                if lanelet_id_list[-1] == i.lanelet_id and not lanelet_id_list[
                                                                   -1] == goal_pos_lanelet:  # finds lane with the right lane id
                    lane = i
                    if len(lane.successor)==0:
                        break
                    lanelet_id_list.append(lane.successor[0])

                if goal_pos_lanelet_adj_left is not None or goal_pos_lanelet_adj_right is not None:
                    if lanelet_id_list[-1] == goal_pos_lanelet or lanelet_id_list[-1] == goal_pos_lanelet_adj_left or lanelet_id_list[-1] == goal_pos_lanelet_adj_right:
                        break
                #else lanelet_id_list[-1] == goal_pos_lanelet:
                #    break
                # for lanelets in lanelet_net.lanelets:
                #     if lanelets.lanelet_id == lane.successor:
                #         lanelet_id_list.append(lanelets.lanelet_id)
                #     if lanelets.lanelet_id == goal_pos_lanelet:
                #         break

            # generating new list of lanelets( from start to end
            ordered_lanelets = ([])
            for i in lanelet_id_list:
                for lane in lanelet_net.lanelets:
                    if lane.lanelet_id == i:
                        ordered_lanelets.append(lane)

            # add all adjacents on the left and the right to the list

            for adjacent in ordered_lanelets:
                if adjacent.adj_right is not None:
                    for i in lanelet_net.lanelets:
                        if i.lanelet_id == adjacent.adj_right and i not in ordered_lanelets:
                            ordered_lanelets.append(i)
                if adjacent.adj_left is not None:
                    for i in lanelet_net.lanelets:
                        if i.lanelet_id == adjacent.adj_left and i not in ordered_lanelets:
                            ordered_lanelets.append(i)

            # generating list of obstacles out of the lanelets
            pointsright = []
            pointsleft = []
            obstacleright = ([])
            obstacleleft = ([])
            for lanelets in ordered_lanelets:
                if lanelets.adj_right == None:
                    #for i in range(len(lanelets.right_vertices[:, 0]) - 1):
                    if pointsright == []:
                        pointsright.append(lanelets.right_vertices)
                    else:
                       pointsright[0] = np.append(pointsright[0], lanelets.right_vertices)
                    # obstacleright.append(self.obstacles_out_of_lanelets(lanelets.right_vertices[i,:] , lanelets.right_vertices[i+1,:],"right"))
                if lanelets.adj_left == None:
                    #for i in range(len(lanelets.left_vertices[:, 0]) - 1):
                    if pointsleft == []:
                        pointsleft.append(lanelets.left_vertices)
                    else:
                       pointsleft[0] = np.append(pointsleft[0], lanelets.right_vertices)
                    #  obstacleleft.append(self.obstacles_out_of_lanelets(lanelets.left_vertices[i,:], lanelets.left_vertices[i+1,:], "left"))

            # corridor_left = pyfvks.collision.ShapeGroup()
            # corridor_right = pyfvks.collision.ShapeGroup()
            # for obstacles in obstacleleft:
            #     corridor_left.add_shape(obstacles)
            # for obstacles in obstacleright:
            #     corridor_right.add_shape(obstacles)
            shaperight= np.shape(pointsright[0])
            shapeleft= np.shape(pointsleft[0])
            shape = np.shape(shapeleft)
            if np.shape(shaperight) == (1,):
                lright = int(len(pointsright[0])/2)
                pointsright[0] = np.reshape(pointsright[0], (lright, 2))
            if np.shape(shapeleft) == (1,):
                lleft = int(len(pointsleft[0]) / 2)
                pointsleft[0] = np.reshape(pointsleft[0], (lleft, 2))




        else:
            problem = self.planning_task.planning_problems

            initial_pos = problem[0].initial_state
            goal_state = def_goal

            # print(self.scenario.lanelet_network)
            lanelet_net = LaneletNetwork()
            lanelet_net.add_lanelet_network(self.scenario.lanelet_network)
            seg = lanelet_net.find_nearest_point_on_lanelet(initial_pos.position)

            position = goal_state

            goal_seg = lanelet_net.find_nearest_point_on_lanelet(position)
            goal_pos_lanelet_id =goal_seg[2].lanelet_id

            initial_pos_lanelet_id = seg[2].lanelet_id
            # goal_pos_lanelet = seg_goal[2].lanelet_id

            # goal_pos_lanelet_adj_right = None
            # goal_pos_lanelet_adj_left = None
            # # laneletadjacents for the
            # if seg_goal[2].adj_left_same_direction and not None:
            #     goal_pos_lanelet_adj_left = seg_goal[2].adj_left
            #     if goal_pos_lanelet_adj_left == None:
            #         goal_pos_lanelet_adj_left = goal_pos_lanelet
            # if seg_goal[2].adj_right_same_direction:
            #     goal_pos_lanelet_adj_right = seg_goal[2].adj_right
            #     if goal_pos_lanelet_adj_right == None:
            #         goal_pos_lanelet = goal_pos_lanelet
            # finds the lanelet_ids from start to goal position
            # DONE: extend function, so that we can set the goal on an adjacent lanelet and on more than one successor

            lanelet_id_list = [initial_pos_lanelet_id]
            for i in lanelet_net.lanelets:
                if goal_pos_lanelet_id in lanelet_id_list:
                    break
                if i.lanelet_id == initial_pos_lanelet_id:
                    if len(i.successor) is not 0:
                        lanelet_id_list.append(i.successor)

            if direction is not None and goal_pos_lanelet_id in lanelet_id_list:
                if direction == 'left':
                    d = seg[2].adj_left
                if direction == 'right':
                    d = seg[2].adj_right
                else:
                    return print('decide direction')


                lanelet_id_list.append(d)



            lanelet_id_list.append(goal_pos_lanelet_id)
            for i in lanelet_net.lanelets:
                if len(i.successor) is not 0:
                    lanelet_id_list.append(i.successor)
            lanelet_id_list = set(lanelet_id_list)


            # for i in lanelet_net.lanelets:
            #     if lanelet_id_list[-1] == i.lanelet_id and not lanelet_id_list[
            #                                                        -1] == goal_pos_lanelet:  # finds lane with the right lane id
            #         lane = i
            #         if len(lane.successor) == 0:
            #             break
            #         lanelet_id_list.append(lane.successor[0])
            #
            #     if goal_pos_lanelet_adj_left is not None or goal_pos_lanelet_adj_right is not None:
            #         if lanelet_id_list[-1] == goal_pos_lanelet or lanelet_id_list[-1] == goal_pos_lanelet_adj_left or \
            #                 lanelet_id_list[-1] == goal_pos_lanelet_adj_right:
            #             break
                # else lanelet_id_list[-1] == goal_pos_lanelet:
                #    break
                # for lanelets in lanelet_net.lanelets:
                #     if lanelets.lanelet_id == lane.successor:
                #         lanelet_id_list.append(lanelets.lanelet_id)
                #     if lanelets.lanelet_id == goal_pos_lanelet:
                #         break

            # generating new list of lanelets( from start to end
            ordered_lanelets = []
            for i in lanelet_id_list:
                for lane in lanelet_net.lanelets:
                    if lane.lanelet_id == i:
                        ordered_lanelets.append(lane)

            # add all adjacents on the left and the right to the list

            # for adjacent in ordered_lanelets:
            #     if adjacent.adj_right is not None:
            #         for i in lanelet_net.lanelets:
            #             if i.lanelet_id == adjacent.adj_right and i not in ordered_lanelets:
            #                 ordered_lanelets.append(i)
            #     if adjacent.adj_left is not None:
            #         for i in lanelet_net.lanelets:
            #             if i.lanelet_id == adjacent.adj_left and i not in ordered_lanelets:
            #                 ordered_lanelets.append(i)

            # generating list of obstacles out of the lanelets
            pointsright = []
            pointsleft = []
            obstacleright = ([])
            obstacleleft = ([])
            for lanelets in ordered_lanelets:
                if lanelets.adj_right  not in lanelet_id_list or lanelets.adj_right is None :
                    # for i in range(len(lanelets.right_vertices[:, 0]) - 1):
                    if pointsright == []:
                        pointsright.append(lanelets.right_vertices)
                    else:
                        pointsright[0] = np.append(pointsright[0], lanelets.right_vertices)
                    # obstacleright.append(self.obstacles_out_of_lanelets(lanelets.right_vertices[i,:] , lanelets.right_vertices[i+1,:],"right"))
                if lanelets.adj_left == None or lanelets.adj_left not in lanelet_id_list:
                    # for i in range(len(lanelets.left_vertices[:, 0]) - 1):
                    if pointsleft == []:
                        pointsleft.append(lanelets.left_vertices)
                    else:
                        pointsleft[0] = np.append(pointsleft[0], lanelets.right_vertices)
                    #  obstacleleft.append(self.obstacles_out_of_lanelets(lanelets.left_vertices[i,:], lanelets.left_vertices[i+1,:], "left"))

            # corridor_left = pyfvks.collision.ShapeGroup()
            # corridor_right = pyfvks.collision.ShapeGroup()
            # for obstacles in obstacleleft:
            #     corridor_left.add_shape(obstacles)
            # for obstacles in obstacleright:
            #     corridor_right.add_shape(obstacles)
            shaperight = np.shape(pointsright[0])
            shapeleft = np.shape(pointsleft[0])
            shape = np.shape(shapeleft)
            if np.shape(shaperight) == (1,):
                lright = int(len(pointsright[0]) / 2)
                pointsright[0] = np.reshape(pointsright[0], (lright, 2))
            if np.shape(shapeleft) == (1,):
                lleft = int(len(pointsleft[0]) / 2)
                pointsleft[0] = np.reshape(pointsleft[0], (lleft, 2))

        return pointsright[0], pointsleft[0]  # corridor_left, corridor_right,

    def obstacles_out_of_lanelets(self, p1, p2, side="right"):
        '''computes obstacles out of 2 points and the side of the lanelet.'''
        # TODO: Function is probably not needed
        # TODO: right center position of the obstacles
        # width/2, height/2, orientation, x-position , y-position
        width = np.abs(p1[0] - p2[0] + p1[1] - p2[1]) / 2  # np.linalg.norm([p1, p2])/2
        height = width / 5
        orientation = np.sin((p2[1] - p1[1]) / (width * 2))
        centerx = (p1[0] + p2[0]) / 2 + orientation * height / 2
        centery = (p1[1] + p2[
            1]) / 2  # + np.sqrt(np.abs(height**2-centerx**2))#evtl mit cos() machen damit die werte besser werden

        if side == 'right':
            centery += height * 2
            obstacle = pyfvks.collision.RectOBB(width, height, orientation, centerx, centery)
        if side == 'left':
            centery -= height * 2
            obstacle = pyfvks.collision.RectOBB(width, height, orientation, centerx, centery)
        return obstacle

    def get_planning_task(self):
        return self.planning_task

    def get_final_state(self):
        final_state = 0
        planning_task = self.get_planning_task()
        #a = planning_task.planning_problems[0].goal.state_list[0].position
        if type(planning_task.planning_problems[0].goal.state_list[0].position) == pyfvks.collision.RectOBB:
            final_state = planning_task.planning_problems[0].goal.state_list[0].position.center()

        else:
            final_state =0# planning_task.planning_problems[0].goal
        return final_state

    def get_dynamic_obstacles(self):
        dynamic_obstacles = []
        for o in self.scenario._obstacles:
            if o.obstacle_role.name == "dynamic":
                dynamic_obstacles.append(o)

        if dynamic_obstacles == []:
            print("No dynamic obstacles in this scenario")
        return dynamic_obstacles

    def dynamic_obstacles_boundary(self):
        ''' :return boundary list, each row are the boundarys of the dynamic obstacles at time x*dt'''
        #if the trajectory starts late and ends early there will be wrong solutions
        #for sake of simplicity just look at active cars
        dynamic_obstacles = self.get_dynamic_obstacles()
        rigth, left = self.generating_driving_corridor
        boundary_list = []
        # all trajectories should be equal in length if
        small = 0
        for o in dynamic_obstacles:
            if len(o.trajectory.state_list) > small:
                small = len(o.trajectory.state_list)

        for o in dynamic_obstacles:
            traj_orient = []
            trajectory_positions = []
            length = o.length
            width = o.width
            car = []
            show = o.trajectory.t0
            #trajectory starts at t0
            if o.active == True:
                for i in o.trajectory.state_list:
                    trajectory_positions.append(i.position)
                    traj_orient.append(i.orientation)
                    #Oriented rectangle with width / 2, height / 2, orientation, x - position, y - position
                    #obb = pyfvks.collision.RectOBB(length/2, width/2 , i.orientation, i.position[0], i.position[1])
                    points = np.array([[length/2, width/2], [length/2, -width/2],
                                      [ - length / 2, width / 2], [-length/2, -width/2]])
                    matrix = np.array([[np.cos(i.orientation), np.sin(i.orientation)],
                              [-np.sin(i.orientation), np.cos(i.orientation)]])
                    turnd_points = []
                    for p in points:
                        turnd_points.append(i.position + matrix @ p)
                    car.append(turnd_points)

                for i in range(small - len(o.trajectory.state_list)):
                #     x = car[1]+car[1]
                #     l = len(car)
                # #trajectory ends before timesteps end
                #     #if len(car) != small:
                #     car.append(x)#np.ones(np.shape(car)))
                    outofbounds = []
                    points = np.array([[length / 2, width / 2], [length / 2, -width / 2],
                                       [- length / 2, width / 2], [-length / 2, -width / 2]])
                    for p in points:
                        outofbounds.append(left[0] * 2 + p)
                    car.append(outofbounds)
                boundary_list.append(car)
            #trajectory starts somewhere not at t0
            # else:
            #     dt = self.scenario.dt
            #     time = dt * small
            #     for i in range(small - len(o.trajectory.state_list)):
            #         outofbounds = []
            #         points = np.array([[length / 2, width / 2], [length / 2, -width / 2],
            #                            [- length / 2, width / 2], [-length / 2, -width / 2]])
            #         for p in points:
            #             outofbounds.append(left[0]*2 +p)
            #         car.append(outofbounds)
            #     boundary_list.append(car)
            #     for i in o.trajectory.state_list:
            #
            #         trajectory_positions.append(i.position)
            #         traj_orient.append(i.orientation)
            #         #Oriented rectangle with width / 2, height / 2, orientation, x - position, y - position
            #         #obb = pyfvks.collision.RectOBB(length/2, width/2 , i.orientation, i.position[0], i.position[1])
            #         points = np.array([[length/2, width/2], [length/2, -width/2],
            #                           [ - length / 2, width / 2], [-length/2, -width/2]])
            #         matrix = np.array([[np.cos(i.orientation), np.sin(i.orientation)],
            #                   [-np.sin(i.orientation), np.cos(i.orientation)]])
            #         turnd_points = []
            #         for p in points:
            #             turnd_points.append(i.position + matrix @ p)
            #         car.append(turnd_points)
            #     boundary_list[-1].append(car)
        return boundary_list

    def dynamic_obstacles_boundary_reduced(self):
        ''' :return boundary list, each row are the boundarys of the dynamic obstacles at time x*dt
                    in a returned way, so that they look like the ego vehicle model
                    reduction to 3 points with radius
        '''
        #if the trajectory starts late and ends early there will be wrong solutions
        #for sake of simplicity just look at active cars
        dynamic_obstacles = self.get_dynamic_obstacles()
        rigth, left = self.generating_driving_corridor
        boundary_list = []
        # all trajectories should be equal in length if
        small = 0
        for o in dynamic_obstacles:
            if len(o.trajectory.state_list) > small:
                small = len(o.trajectory.state_list)

        for o in dynamic_obstacles:
            traj_orient = []
            trajectory_positions = []
            length = o.length
            width = o.width
            car = []
            show = o.trajectory.t0
            #trajectory starts at t0
            if o.active == True:
                for i in o.trajectory.state_list:
                    trajectory_positions.append(i.position)
                    traj_orient.append(i.orientation)
                    #Oriented rectangle with width / 2, height / 2, orientation, x - position, y - position
                    #obb = pyfvks.collision.RectOBB(length/2, width/2 , i.orientation, i.position[0], i.position[1])
                    points = np.array([[length/2, width/2], [length/2, -width/2],
                                      [ - length / 2, width / 2], [-length/2, -width/2]])
                    matrix = np.array([[np.cos(i.orientation), np.sin(i.orientation)],
                              [-np.sin(i.orientation), np.cos(i.orientation)]])
                    turnd_points = []
                    for p in points:
                        turnd_points.append(i.position + matrix @ p)
                    car.append(turnd_points)

                for i in range(small - len(o.trajectory.state_list)):
                #     x = car[1]+car[1]
                #     l = len(car)
                # #trajectory ends before timesteps end
                #     #if len(car) != small:
                #     car.append(x)#np.ones(np.shape(car)))
                    outofbounds = []
                    points = np.array([[length / 2, width / 2], [length / 2, -width / 2],
                                       [- length / 2, width / 2], [-length / 2, -width / 2]])
                    for p in points:
                        outofbounds.append(left[0] * 2 + p)
                    car.append(outofbounds)
                boundary_list.append(car)
            #trajectory starts somewhere not at t0
            # else:
            #     dt = self.scenario.dt
            #     time = dt * small
            #     for i in range(small - len(o.trajectory.state_list)):
            #         outofbounds = []
            #         points = np.array([[length / 2, width / 2], [length / 2, -width / 2],
            #                            [- length / 2, width / 2], [-length / 2, -width / 2]])
            #         for p in points:
            #             outofbounds.append(left[0]*2 +p)
            #         car.append(outofbounds)
            #     boundary_list.append(car)
            #     for i in o.trajectory.state_list:
            #
            #         trajectory_positions.append(i.position)
            #         traj_orient.append(i.orientation)
            #         #Oriented rectangle with width / 2, height / 2, orientation, x - position, y - position
            #         #obb = pyfvks.collision.RectOBB(length/2, width/2 , i.orientation, i.position[0], i.position[1])
            #         points = np.array([[length/2, width/2], [length/2, -width/2],
            #                           [ - length / 2, width / 2], [-length/2, -width/2]])
            #         matrix = np.array([[np.cos(i.orientation), np.sin(i.orientation)],
            #                   [-np.sin(i.orientation), np.cos(i.orientation)]])
            #         turnd_points = []
            #         for p in points:
            #             turnd_points.append(i.position + matrix @ p)
            #         car.append(turnd_points)
            #     boundary_list[-1].append(car)
        shape =  np.shape(boundary_list)
        reduced_boundary_list = np.zeros((shape[0],shape[1],3, 2))
        radius = np.zeros((shape[0],1))
        for i in range(shape[0]):
            for j in range(shape[1]):
                reduced_boundary_list[i][j][0][0] = (boundary_list[i][j][0][0]+boundary_list[i][j][1][0])/2
                reduced_boundary_list[i][j][0][1] = (boundary_list[i][j][0][1]+boundary_list[i][j][1][1])/2
                reduced_boundary_list[i][j][1][0] = (boundary_list[i][j][0][0]+boundary_list[i][j][3][0])/2
                reduced_boundary_list[i][j][1][1] = (boundary_list[i][j][0][1]+boundary_list[i][j][3][1])/2
                reduced_boundary_list[i][j][2][0] = (boundary_list[i][j][2][0]+boundary_list[i][j][3][0])/2
                reduced_boundary_list[i][j][2][1] = (boundary_list[i][j][2][1]+boundary_list[i][j][3][1])/2
                # if (boundary_list[i][j][0][0]+boundary_list[i][j][3][0]) >6:
            radius[i] = distance.euclidean(boundary_list[i][0][0],boundary_list[i][0][1])/2
                #     reduced_boundary_list[i][j] = np.append([(reduced_boundary_list[i][j][0][0]+reduced_boundary_list[i][j][1][0])/2, (reduced_boundary_list[i][j][0][1]+reduced_boundary_list[i][j][1][1])/2])
                #     reduced_boundary_list[i][j] = np.append([(reduced_boundary_list[i][j][1][0]+reduced_boundary_list[i][j][2][0])/2, (reduced_boundary_list[i][j][1][1]+reduced_boundary_list[i][j][2][1])/2])


        return reduced_boundary_list, radius


    def get_speed_limit(self):
        if self.scenario.lanelet_network.lanelets[0].speed_limit == None:
            speedlimit = 30
        else:
            speedlimit = self.scenario.lanelet_network.lanelets[0].speed_limit
        return speedlimit


def constraint_interpolation(vehposi, pointsleft, pointsright):  # use for upper and lower bound of the driving corridor
    '''returns the nearest points to the vehicle by linear interpolation
    :param points: 2 points (use method generating_driving_corridor
    :param vehpos: x position of vehicle'''
    # TODO: Right now it just works for the x position of the vehicle

    limitleft = []
    limitright = []
    for vehpos in vehposi:
        dist = []
        for i in pointsleft:
            dist.append(distance.euclidean(vehpos,i))


        for i in range(len(dist)):
            if dist[i]==min(dist):
                limitleft.append(pointsleft[i])
                break

        dist2=[]
        for i in pointsright:
            dist2.append(distance.euclidean(vehpos,i))


        for i in range(len(dist)):
            if dist[i]==min(dist):
                limitright.append(pointsright[i])
                break


    #y = np.interp(vehpos, xp, fp)


    #returns the left and right limit of the street, determined by the shortest distance of the points as the points
    return limitleft, limitright


def bounds(pointsleft, pointsright):
    ''':returns y coordinates to vehicle x-pos '''
    #leftbound = constraint_interpolation(x, left)
    #rightbound = constraint_interpolation(x, right)
    lower_x = min(min(pointsleft[:,0]), min(pointsright[:,0]))
    upper_x = max(max(pointsleft[:,0]), max(pointsright[:,0]))
    lower_y = min(min(pointsleft[:,1]), min(pointsright[:,1]))
    upper_y = max(max(pointsleft[:,1]), max(pointsright[:,1]))
    return lower_x, upper_x, lower_y, upper_y

def goal_bounds(area):
    ''':param area is the uncertainty area as RectOBB
    :return goal bounds x and y'''
    c = area.center()
    x = (c[0]-1,c[0]+1)
    y = (c[1]-1,c[1]+1)
    return x,y