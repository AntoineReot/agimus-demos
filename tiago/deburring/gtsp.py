from CORBA import Any, TC_long, TC_float
import numpy as np
import pickle, os
# np.set_printoptions(threshold=np.inf, precision=2)
from travelling_salesman import armPlanner, ClusterComputation, part_handles, q0, progressbar_iterable, robot, \
        InStatePlanner, c_lock_part, generate_valid_config,graph, loop_free, concatenate_paths, vf, ps
file = 'log_gtsp.txt'
def generateConfigs(handles, Nconfigs = 3, maxIterations = 100):
        """"Generates the initial set of configurations
        In :
             handles : list of handles,
             Nconfigs : maximal number of configs generated for each handle,
             maxIteration : maximal number of iterations to create the configurations for each handle.
        Out :
             home_configs_to_handles : dictionary associating a list of tuples (qphi, qhi,qhome) to each handle
             handles_to_configs : dictionary associating a list of handles to each home pose
             qhomes : list of home poses"""
        f=open(file, 'w')
        handles_to_configs = {}   #dictionary associating a list of tuples (qphi, qhi,qhome) to each handle
        home_configs_to_handles = {} #dictionary associating a list of handles to each home pose
        clusters_comp = ClusterComputation(armPlanner.cgraph, c_lock_part)
        nb_configs_found = 0
        handles.sort()
        qhomes=[q0]
        for hi in progressbar_iterable(handles, "Compute configs for each handle"):
                handles_to_configs[hi] = []
                cpi=clusters_comp.freeBaseConstraint(hi)
                cpi.setRightHandSideFromConfig(q0) #makes sure the object keeps the same position it had in configuration q0
                ci = clusters_comp.pregraspToGraspConstraint(hi)
                for j in range(maxIterations):
                        #generating configs for each handle
                        res, qphi = generate_valid_config(cpi, qguesses=[q0,], NrandomConfig=10)
                        if not res:
                                f.write("error {:d}th loop computing pregrasp for handle {}\n".format(j,hi))
                                continue
                        ci.setRightHandSideFromConfig(qphi) #the drill needs to have the same orientation it had in the pregrasp computed above
                        res, qhi = generate_valid_config(ci, qguesses=[qphi,], NrandomConfig=10)
                        if not res:
                                f.write("error {:d}th loop computing grasp for handle {}\n".format(j,hi))
                                continue
                        cedge = clusters_comp._cgraph.get(graph.edges["move_base"])
                        cc = cedge.targetConstraint().getConfigProjector()
                        cc.setRightHandSideFromConfig(qphi)
                        res, qhome = cc.apply (qphi)
                        r=robot.rankInConfiguration["tiago/root_joint"]
                        assert(qphi[r:r+4]==qhome[r:r+4])
                        home_configs_to_handles[tuple(qhome)]=[hi]
                        handles_to_configs[hi].append((qphi,qhi,qhome))
                        qhomes.append(qhome)
                        if len(handles_to_configs[hi]) >= Nconfigs:
                                break
                nb_configs_found+=len(handles_to_configs[hi])

        handles_to_configs_temp = {}  #dictionary with the same format as handles_to_configs, used to store configurations temporaly before concatenating them with those from handles_to_configs
        for hi in progressbar_iterable(handles, "Check if configs from other handles are valid"):
                #this loop goes through the configurations computed for handles != hi and checks whether these configurations are also valid for hi
                handles_to_configs_temp[hi] = []
                for (other_hi,configs) in handles_to_configs.items():
                        if other_hi != hi:
                                cpi=clusters_comp.fixedBaseConstraint(hi) #the base needs to be fixed now
                                ci = clusters_comp.pregraspToGraspConstraint(hi)
                                for j,config in enumerate(configs):        #config = (qphi,qhi,qhome)
                                        qhome = config[2]
                                        cpi.setRightHandSideFromConfig(qhome) #using constraints from an already existing configuration
                                        res, qphi = generate_valid_config(cpi, qguesses=[qhome,], NrandomConfig=10)
                                        if not res:
                                                f.write("error checking if {}th home config from {} is valid for {}\n".format(j,other_hi,hi))
                                                continue
                                        ci.setRightHandSideFromConfig(qphi) #the drill needs to have the same orientation it had in the pregrasp computed above
                                        res, qhi = generate_valid_config(ci, qguesses=[qphi,], NrandomConfig=10)
                                        if not res:
                                                f.write("error checking if {}th grasp config from {} is valid for {}\n".format(j,other_hi,hi))
                                                continue
                                        handles_to_configs_temp[hi].append((qphi,qhi,qhome))
                                        home_configs_to_handles[tuple(qhome)].append(hi)

        f.close()
        for (hi,configs) in handles_to_configs_temp.items():
                #adding the additional configs to the initial dictionary
                for config in configs:
                        handles_to_configs[hi].append(config)
        print("\n{} configs found".format(nb_configs_found))
        for hi in handles:
                print("{} of which are valid for handle {}".format(len(handles_to_configs[hi]), hi))
        return home_configs_to_handles, handles_to_configs, qhomes


def compute_base_matrix(qhomes):
        """"Computes the paths and distances between each home pose (arm retracted)
        In :
             qhomes : list of home poses
        Out :
             paths : paths matrix
             distances : corresponding cost matrix"""
        basePlanner = InStatePlanner ()
        basePlanner.setEdge("move_base")
        basePlanner.setReedsAndSheppSteeringMethod()
        basePlanner.plannerType="kPRM*"
        basePlanner.cproblem.setParameter('kPRM*/numberOfNodes', Any(TC_long,200))
        # if a roadmap has already been computed, read it
        roadmapFile = os.getcwd() + "/data/roadmap-base.bin"
        if os.path.exists(roadmapFile):
                print("Reading mobile base roadmap:", roadmapFile)
                basePlanner.readRoadmap(roadmapFile)
        print("Computing matrix for the base")
        paths, distances = basePlanner.computeMatrix(qhomes,False)
        print("done")
        # if no roadmap has already been computed, save this one
        if not os.path.exists(roadmapFile):
                basePlanner.writeRoadmap(roadmapFile)
        l = len(paths)
        for i in range(l):
                for j in range(i+1,l):
                        path = paths[i][j]
                        paths[j][i]=path.reverse() if path else None
        return paths, distances


def compute_arm_matrixes(home_configs_to_handles, handles_to_configs):
        """"Computes the paths and distances between each home pose and the handles it can reach
        In :
             home_configs_to_handles : dictionary associating a list of handles to each home pose
             handles_to_configs : dictionary associating a list of tuples (qphi, qhi,qhome) to each handle
        Out :
             home_to_handles_distances : dictionary associating a tuple (paths, costs) to each home
             elements : the list of nodes, which will be used as the rows and columns of the GTSP matrix"""
        armPlanner=InStatePlanner()
        armPlanner.setEdge(loop_free)
        home_to_handles_distances={}
        elements=[(q0,None)]
        for (tqhome,his) in home_configs_to_handles.items():
                lqhome=list(tqhome)
                qs=[lqhome]
                his.sort()
                for hi in his:
                        elements.append((tqhome,hi))
                        configs=handles_to_configs[hi]
                        for config in configs:
                                if config[2]==lqhome:
                                        qs.append(config[0])
                armPlanner.createEmptyRoadmap()
                armpaths, armdistances = armPlanner.computeMatrix(qs,False)
                l = len(armpaths)
                for i in range(l):
                        for j in range(i+1,l):
                                path = armpaths[i][j]
                                armpaths[j][i]=path.reverse() if path else None
                home_to_handles_distances[tqhome] = armpaths, armdistances
        return home_to_handles_distances, elements


def compute_cost_matrix(elements, qhomes, paths, distances, home_configs_to_handles, home_to_handles_distances, ponderator = 10):
        """"Computes the cost matrix of the GTSP and the associated paths
        In :
             elements : list of nodes
             qhomes : list of homes
             paths : paths between the home poses
             distances : distances between the home poses
             home_configs_to_handles : dictionary associating a list of handles to each home pose
             home_to_handles_distances : dictionary associating a tuple (paths, costs) to each home
             ponderator : ponderator to weigh the edges that require to move between home poses
        Out :
             pathsGTSP : the paths matrix between all the nodes
             distancesGTSP the corresponding costs matrix"""
        n=len(elements)
        distancesGTSP = np.zeros((n,n))
        pathsGTSP = [[None, ] * n for _ in range(n)]
        for i in range(n):
                for j in range(i+1,n):
                        if i==0:
                                q,h = elements[j]
                                index = qhomes.index(list(q))
                                q0_to_q = distances[i][index]
                                path1 = paths[i][index]

                                index = home_configs_to_handles[q].index(h)
                                q_to_h = home_to_handles_distances[q][1][0][index+1]
                                path2 = home_to_handles_distances[q][0][0][index+1]

                                value_to_compute = q0_to_q+q_to_h
                                distancesGTSP[i][j] = distancesGTSP[j][i] = value_to_compute
                                if path1 and path2:
                                        pathsGTSP[i][j] = concatenate_paths([path1,path2])
                                        pathsGTSP[j][i] = pathsGTSP[i][j].reverse()
                                else :
                                        pathsGTSP[i][j] = pathsGTSP[j][i] = None
                        else:
                                q1,h1=elements[i]
                                q2,h2=elements[j]
                                if q1==q2:
                                        index_1 = home_configs_to_handles[q1].index(h1)
                                        index_2 = home_configs_to_handles[q1].index(h2)
                                        h1_to_h2 = home_to_handles_distances[q1][1][index_1+1][index_2+1]
                                        path = home_to_handles_distances[q1][0][index_1+1][index_2+1]

                                        distancesGTSP[i][j] = distancesGTSP[j][i] = h1_to_h2
                                        if path:
                                                pathsGTSP[i][j] = path
                                                pathsGTSP[j][i] = path.reverse()

                                else:
                                        index_q1 = qhomes.index(list(q1))
                                        index_q2 = qhomes.index(list(q2))
                                        q1_to_q2 = distances[index_q1][index_q2]
                                        base_path = paths[index_q1][index_q2]

                                        index_h1 = home_configs_to_handles[q1].index(h1)
                                        index_h2 = home_configs_to_handles[q2].index(h2)
                                        h1_to_q1 = home_to_handles_distances[q1][1][index_h1+1][0]
                                        q2_to_h2 = home_to_handles_distances[q2][1][0][index_h2+1]
                                        path1 = home_to_handles_distances[q1][0][index_h1+1][0]
                                        path2 = home_to_handles_distances[q2][0][0][index_h2+1]

                                        distancesGTSP[i][j] = distancesGTSP[j][i] = h1_to_q1 + q1_to_q2*ponderator + q2_to_h2
                                        if path1 and base_path and path2:
                                                pathsGTSP[i][j] = concatenate_paths([path1,base_path,path2])
                                                pathsGTSP[j][i] = pathsGTSP[i][j].reverse()
                                        else:
                                                pathsGTSP[i][j] = pathsGTSP[j][i] = None
        return pathsGTSP, distancesGTSP

#to show home configs in gepetto-gui: (assuming v=vf.createViewer())
# for hi in part_handles:
#       for configs in handles_to_configs[hi]:
#               v(configs[1])
#               time.sleep(0.3)

holes=part_handles[:2]  #test avec 2 trous uniquement
homes_to_holes, handles_to_configs, homes_list = generateConfigs(holes, 4)
base_paths, base_distances = compute_base_matrix(homes_list)
homes_to_holes_distances,elts = compute_arm_matrixes(homes_to_holes, handles_to_configs)
pathsGTSP, distancesGTSP = compute_cost_matrix(elts, homes_list, base_paths, base_distances, homes_to_holes, homes_to_holes_distances, 10)

#show a path
#ps.client.basic.problem.addPath(pathsGTSP[1][2])
