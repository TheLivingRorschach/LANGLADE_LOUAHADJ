from math import sqrt
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraph
from hpp.corbaserver import Client
Client ().problem.resetProblem ()
from manipulation import robot, vf, ps, Ground, Box, Pokeball, PathPlayer, gripperName, ballName

vf.loadEnvironmentModel (Ground, 'ground')
vf.loadEnvironmentModel (Box, 'box')
vf.moveObstacle ('box/base_link_0', [0.3+0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_1', [0.3-0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_2', [0.3, 0.04, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_3', [0.3, -0.04, 0.04, 0, 0, 0, 1])

vf.loadObjectModel (Pokeball, 'pokeball')
robot.setJointBounds ('pokeball/root_joint', [-.4,.4,-.4,.4,-.1,1.,
                                              -1.0001, 1.0001,-1.0001, 1.0001,
                                              -1.0001, 1.0001,-1.0001, 1.0001,])

q1 = [0, -1.57, 1.57, 0, 0, 0, .3, 0, 0.025, 0, 0, 0, 1]

## Create graph
graph = ConstraintGraph (robot, 'graph')

graph.createNode (['grasp', 'placement','gripper-above-ball', 'ball-above-ground', 'grasp-placement'])

# Nodes going to and from grasp-placement
graph.edgde('grasp-placement', 'gripper-above-ball', 'move-gripper-up', 1, 'grasp-placement')
graph.edgde('grasp-placement', 'ball-above-ground', 'take-ball', 1, 'grasp-placement')
graph.edgde('grasp-placement', 'gripper-above-ball', 'grasp-ball', 1, 'gripper-above-ball')
graph.edgde('grasp-placement', 'ball-above-ground', 'put-ball-down', 1, 'ball-above-ground')

# Nodes gripper-above-all
graph.edgde('placement', 'gripper-above-ball', 'move-gripper-away', 1, 'grasp-placement')
graph.edgde('placement', 'gripper-above-ball', 'appraoch-ball', 1, 'grasp-placement')





ps.selectPathValidation ("Discretized", 0.01)
ps.selectPathProjector ("Progressive", 0.1)
graph.initialize ()

res, q_init, error = graph.applyNodeConstraints ('placement', q1)
q2 = q1 [::]
q2 [7] = .2

res, q_goal, error = graph.applyNodeConstraints ('placement', q2)

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# v = vf.createViewer ()
# pp = PathPlayer (v)
# v (q1)

