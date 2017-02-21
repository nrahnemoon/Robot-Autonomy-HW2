import numpy, operator
import random
import time
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        start_time = time.time()
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space



        currConfig = start_config;
        currIDf = ftree.GetRootId();
        currIDr = rtree.GetRootId();

        print "startConfig = [%.2f, %.2f]" %(start_config[0], start_config[1])
        print "goalConfig = [%.2f, %.2f]" %(goal_config[0], goal_config[1])
        #while (self.planning_env.Extend(currConfig, goal_config) == None):
        disc = True;
        while (disc):#self.planning_env.ComputeDistance(currConfig,goal_config) > epsilon):
            if(random.random() < .9):
                newCurrConfig = self.planning_env.GenerateRandomConfiguration();
            else:
                newCurrConfig = goal_config;
            #newCurrConfig = self.planning_env.GenerateRandomConfiguration();

            [nearID, nearConfig] = ftree.GetNearestVertex(newCurrConfig);
            print "newCurrConfig = [%.2f, %.2f]" %(newCurrConfig[0], newCurrConfig[1])
            print "nearID = %d, nearConfig = [%.2f, %.2f]" %(nearID, nearConfig[0], nearConfig[1])
            
            extension = self.planning_env.Extend(nearConfig, newCurrConfig)
            print extension

            if (extension != None):
                currConfig = extension
                currIDf = ftree.AddVertex(currConfig);
                ftree.AddEdge(nearID, currIDf);

                #plan.append(currConfig)

                print "currIDf = %d, currConfig = [%.2f, %.2f]" %(currIDf, currConfig[0], currConfig[1])
                self.planning_env.PlotEdge(nearConfig, currConfig);



            if(random.random() < .9):
                newCurrConfig = self.planning_env.GenerateRandomConfiguration();
            else:
                newCurrConfig = start_config;
            #newCurrConfig = self.planning_env.GenerateRandomConfiguration();

            [nearID, nearConfig] = rtree.GetNearestVertex(newCurrConfig);
            print "newCurrConfig = [%.2f, %.2f]" %(newCurrConfig[0], newCurrConfig[1])
            print "nearID = %d, nearConfig = [%.2f, %.2f]" %(nearID, nearConfig[0], nearConfig[1])
            
            extension = self.planning_env.Extend(nearConfig, newCurrConfig)
            print extension

            if (extension != None):
                currConfig = extension
                currIDr = rtree.AddVertex(currConfig);
                rtree.AddEdge(nearID, currIDr);

                #plan.append(currConfig)

                print "currIDr = %d, currConfig = [%.2f, %.2f]" %(currIDr, currConfig[0], currConfig[1])
                self.planning_env.PlotEdge(nearConfig, currConfig);
            #time.sleep(1)
            for i in range(len(ftree.vertices)):
                if (numpy.linalg.norm(numpy.array(currConfig) - numpy.array(ftree.vertices[i])) < 2):
                    [currIDf, newCurrConfig] = ftree.GetNearestVertex(ftree.vertices[i])
                    extension = self.planning_env.Extend(currConfig, newCurrConfig)
                    if (numpy.array_equal(extension,newCurrConfig)):
                        currIDr = rtree.AddVertex(currConfig);
                        rtree.AddEdge(currIDf, currIDr);
                        self.planning_env.PlotEdge(newCurrConfig, currConfig);
                        disc = False
                        break

        #goalID = ftree.AddVertex(goal_config);
        #ftree.AddEdge(currIDf, goalID);
        #self.planning_env.PlotEdge(currConfig, goal_config)

        #currConfig = goal_config
        #currIDf = goalID;
               
        plan.append(goal_config)
        plan.append(start_config)
        while 1:
            currIDr = rtree.edges[currIDr];
            currConfig = rtree.vertices[currIDr];
            if (currIDr == rtree.GetRootId()):
                break;
            else:
                plan.insert(1, currConfig);
        

        plan = list(reversed(plan))

        while 1:
            currIDf = ftree.edges[currIDf];
            currConfig = ftree.vertices[currIDf];
            if (currIDf == ftree.GetRootId()):
                break;
            else:
                plan.insert(1, currConfig);

        



        print("--- %s seconds ---" % (time.time() - start_time))
        import IPython
        IPython.embed()
        for config in plan:
            print "config = [%.2f, %.2f]" %(config[0], config[1])
        return plan
