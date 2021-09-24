class MotionPlanner:
  def __init__ (self, robot, ps):
    self.robot = robot
    self.ps = ps

  def solveBiRRT (self, maxIter = float("inf")):
    self.ps.prepareSolveStepByStep ()
    finished = False

    # In the framework of the course, we restrict ourselves to 2 connected components.
    nbCC = self.ps.numberConnectedComponents ()
    if nbCC != 2:
      raise Exception ("There should be 2 connected components.")

    iter = 0
    while True:
      #### RRT begin
			qrand = self.robot.shootRandomConfig()
			newConfigs = list()
			q1 = self.ps.getNearestConfig(qrand,0)
			q2 = self.ps.getNearestConfig(qrand,1)
			validity, path_id, _ = self.ps.directPath(q1,q2,True)
			if not validity:
						qn = self.ps.configAtParam(path_id, self.ps.pathLength(path_id))
						newConfigs.append(qn)
						self.ps.addConfigToRoadmap(qn) # Add config to current roadmap		
        		self.ps.addEdgeToRoadmap(q1,qn, path_id, True)
			validity, path_id, _ = self.ps.directPath(q2,q1,True)
			if not validity:
						qn = self.ps.configAtParam(path_id, self.ps.pathLength(path_id))
						newConfigs.append(qn)
						self.ps.addConfigToRoadmap(qn) # Add config to current roadmap		
        		self.ps.addEdgeToRoadmap(q2,qn, path_id, True)
      ## Try connecting the new nodes together
			
      for i in range (len(newConfigs)): # len(2) normalement
						for j in range(...):
			
        qnear = self.ps.getNearestConfig(newConfigs,i)
				
      #### RRT end
      ## Check if the problem is solved.
      nbCC = self.ps.numberConnectedComponents ()
      if nbCC == 1:
        # Problem solved
        finished = True
        break
      iter = iter + 1
      if iter > maxIter:
        break
    if finished:
        self.ps.finishSolveStepByStep ()
        return self.ps.numberPaths () - 1

  def solvePRM (self):
    self.ps.prepareSolveStepByStep ()
    #### PRM begin
    #### PRM end
    self.ps.finishSolveStepByStep ()
