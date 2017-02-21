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
        rplan = []
        fplan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space


        print "startConfig = [%.2f, %.2f]" %(start_config[0], start_config[1])
        print "goalConfig = [%.2f, %.2f]" %(goal_config[0], goal_config[1])
        disc = True;
        while (disc):
            if(random.random() < 0.9):
                desConfig = self.planning_env.GenerateRandomConfiguration();
            else:
                desConfig = goal_config;

            [nearID, nearConfig] = ftree.GetNearestVertex(desConfig);            
            extension = self.planning_env.Extend(nearConfig, desConfig)
            if (extension != None):
                extIDf = ftree.AddVertex(extension);
                ftree.AddEdge(nearID, extIDf);
                self.planning_env.PlotEdge(nearConfig, extension);
            
            #if(numpy.array_equal(extension, goal_config)):
            #    disc = False

            if(random.random() < 0.9):
                desConfig = self.planning_env.GenerateRandomConfiguration();
            else:
                desConfig = start_config;

            [nearID, nearConfig] = rtree.GetNearestVertex(desConfig);            
            extension = self.planning_env.Extend(nearConfig, desConfig)
            if (extension != None):
                extIDr = rtree.AddVertex(extension);
                rtree.AddEdge(nearID, extIDr);
                self.planning_env.PlotEdge(nearConfig, extension);
            
            #if(numpy.array_equal(extension, start_config)):
            #    disc = False

            for i in range(len(ftree.vertices)):
                if (numpy.linalg.norm(numpy.array(desConfig) - numpy.array(ftree.vertices[i])) < 2):
                    lastlink = self.planning_env.Extend(extension, ftree.vertices[i])
                    if (numpy.array_equal(lastlink,ftree.vertices[i])):
                        lastIDr = rtree.AddVertex(lastlink);
                        rtree.AddEdge(extIDr, lastIDr);
                        self.planning_env.PlotEdge(extension, lastlink);
                        disc = False
                        break

        fplan.insert(0,start_config)
        fplan.append(goal_config)
        while 1:
            extIDf = ftree.edges[extIDf];
            currConfig = ftree.vertices[extIDf];
            if (extIDf == ftree.GetRootId()):
                break;
            else:
                fplan.insert(1, currConfig);

        for config in fplan:
            print "fconfig = [%.2f, %.2f]" %(config[0], config[1])
        print("--- %s seconds ---" % (time.time() - start_time))
        
        rplan.insert(0,goal_config)
        rplan.append(start_config)
        while 1:
            extIDr = rtree.edges[extIDr];
            currConfig = rtree.vertices[extIDr];
            if (extIDr == rtree.GetRootId()):
                break;
            else:
                rplan.insert(1, currConfig);

        for config in rplan:
            print "fconfig = [%.2f, %.2f]" %(config[0], config[1])

        import IPython
        IPython.embed()

        return rplan
    '''
            if(random.random() < .9):
                desConfig = self.planning_env.GenerateRandomConfiguration();
            else:
                desConfig = start_config;
            #newCurrConfig = self.planning_env.GenerateRandomConfiguration();

            [nearID, nearConfig] = rtree.GetNearestVertex(desConfig);
            print "desConfig = [%.2f, %.2f]" %(desConfig[0], desConfig[1])
            print "nearID = %d, nearConfig = [%.2f, %.2f]" %(nearID, nearConfig[0], nearConfig[1])
            
            extension = self.planning_env.Extend(nearConfig, desConfig)
            print extension

            if (extension != None):
                desConfig = extension
                desIDr = rtree.AddVertex(desConfig);
                rtree.AddEdge(nearID, desIDr);

                #plan.append(currConfig)

                print "desIDr = %d, desConfig = [%.2f, %.2f]" %(desIDr, desConfig[0], desConfig[1])
                self.planning_env.PlotEdge(nearConfig, desConfig);
                #time.sleep(1)
                for i in range(len(ftree.vertices)):
                    if (numpy.linalg.norm(numpy.array(desConfig) - numpy.array(ftree.vertices[i])) < 2):
                        currIDf = i
                        #[desIDf, desConfigf] = ftree.GetNearestVertex(ftree.vertices[i])
                        #extension = self.planning_env.Extend(desConfig, desConfigf)
                        #if (numpy.array_equal(extension,desConfigf)):
                            #desIDr = rtree.AddVertex(desConfigf);
                            #rtree.AddEdge(desIDr, desIDf);
                            #self.planning_env.PlotEdge(desConfig, desConfigf);
                        disc = False
                        break
        #currConfig = goal_config
        #currIDf = goalID;
        rplan.append(goal_config)
        while 1:
            desIDr = rtree.edges[desIDr];
            desConfig = rtree.vertices[desIDr];
            if (desIDr == rtree.GetRootId()):
                break;
            else:
                rplan.insert(1,desConfig);
      

        #plan = list(reversed(plan))'''