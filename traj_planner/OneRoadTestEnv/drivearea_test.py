from traj_planner.BerthaTrajectoryPlanning.OneRoadTestEnv.drivearea import morphingcommonroad
from fvks.visualization.draw_dispatch import draw_object
import os
import matplotlib.pyplot as plt
cwd = os.getcwd() + '/scenarios/NGSIM/US101/NGSIM_US101_0.xml'
# cwd = cwd - "OneRoadTestEnv"
cwd = cwd.replace('OneRoadTestEnv/', '')
new_sim = morphingcommonroad(cwd)

pointsright, pointsleft = new_sim.generating_driving_corridor(reduced = 1, def_goal = [80,3], direction='right')

draw_object(new_sim.scenario)
plt.plot(pointsright[:,0],pointsright[:,1])
plt.plot(pointsleft[:,0],pointsleft[:,1])
plt.show()