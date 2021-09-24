class MotionPlanner:
  def __init__ (self, robot, ps):
    self.robot = robot
    self.ps = ps

  def solveBiRRT (self, maxIter = float("inf")):
    print ("START")
    self.ps.prepareSolveStepByStep ()
    finished = False
    # In the framework of the course, we restrict ourselves to 2 connected components.
    nbCC = self.ps.numberConnectedComponents ()
    if nbCC != 2:
      raise Exception ("There should be 2 connected components.")
    iter = 0
    while True:
      print("test 1")
      #### RRT begin
      ## Try connecting the new nodes together
      Qrand = self.robot.shootRandomConfig()
      print(nbCC)
      for i in range(nbCC):

        Qnear,d=self.ps.getNearestConfig(Qrand,i)
        validity, path_id, _ = self.ps.directPath(Qnear, Qrand, True) # Testing for collisions
                
        Q_new = self.ps.configAtParam(path_id, self.ps.pathLength(path_id)) # Getting new config
        self.ps.addConfigToRoadmap(Q_new) # Add config to current roadmap		
        self.ps.addEdgeToRoadmap(Qnear,Q_new, path_id, True) # Adds an edge in current road map between current config. and nearest config.
        #print("test 2")
	

	
      #### RRT end



      ## Check if the problem is solved.
      nbCC = self.ps.numberConnectedComponents ()

      if nbCC == 1:
        # Problem solved
        print("Problem solved")
        finished = True
        break
      iter = iter + 1
      if iter > maxIter:
        print("Max iter")
        break
    if finished:
        self.ps.finishSolveStepByStep ()
        return self.ps.numberPaths () - 1

  def solvePRM (self):
    self.ps.prepareSolveStepByStep()
    #### PRM begin


    #### PRM end
    self.ps.finishSolveStepByStep()
