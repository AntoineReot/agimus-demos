from CORBA import Any, TC_long, TC_float
import numpy as np
import pickle, os
import time
import csv
# np.set_printoptions(threshold=np.inf, precision=2)
from travelling_salesman import armPlanner, ClusterComputation, part_handles, q0, progressbar_iterable, robot, \
        InStatePlanner, c_lock_part, generate_valid_config,graph, loop_free, concatenate_paths, vf, ps
file = 'data/log_gtsp.txt'


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
        start_time = time.time()
        f=open(file, 'w')
        handles_to_configs = {}   #dictionary associating a list of tuples (qphi, qhi,qhome) to each handle
        home_configs_to_handles = {} #dictionary associating a list of handles to each home pose
        clusters_comp = ClusterComputation(armPlanner.cgraph, c_lock_part)
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
                        res, qhi = generate_valid_config(ci, qguesses=[qphi,], NrandomConfig=0)
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
                                        res, qhi = generate_valid_config(ci, qguesses=[qphi,], NrandomConfig=0)
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
        print("--- %s seconds to generate ---" % (time.time() - start_time))
        print("\n{} configs found".format(len(qhomes)-1))
        for hi in handles:
                print("{} of which are valid for handle {}".format(len(handles_to_configs[hi]), hi))
        return home_configs_to_handles, handles_to_configs, qhomes

def generateAdditionalConfigs(handles, home_configs_to_handles, handles_to_configs, qhomes, Nconfigs = 1, maxIterations = 100):
        """"Generates the initial set of configurations
        In :
             handles : list of handles,
             Nconfigs : maximal number of configs generated for each handle,
             maxIteration : maximal number of iterations to create the configurations for each handle.
        Out :
             home_configs_to_handles : dictionary associating a list of tuples (qphi, qhi,qhome) to each handle
             handles_to_configs : dictionary associating a list of handles to each home pose
             qhomes : list of home poses"""
        start_time = time.time()
        f=open(file, 'w')
        new_handles_to_configs = {}   #dictionary associating a list of tuples (qphi, qhi,qhome) to each handle
        new_home_configs_to_handles = {}
        clusters_comp = ClusterComputation(armPlanner.cgraph, c_lock_part)
        handles.sort()
        for hi in progressbar_iterable(handles, "Compute configs for each handle"):
                new_handles_to_configs[hi] = []
                cpi=clusters_comp.freeBaseConstraint(hi)
                cpi.setRightHandSideFromConfig(q0) #makes sure the object keeps the same position it had in configuration q0
                ci = clusters_comp.pregraspToGraspConstraint(hi)
                N_added = 0
                for j in range(maxIterations):
                        #generating configs for each handle
                        res, qphi = generate_valid_config(cpi, qguesses=[q0,], NrandomConfig=10)
                        if not res:
                                f.write("error {:d}th loop computing pregrasp for handle {}\n".format(j,hi))
                                continue
                        ci.setRightHandSideFromConfig(qphi) #the drill needs to have the same orientation it had in the pregrasp computed above
                        res, qhi = generate_valid_config(ci, qguesses=[qphi,], NrandomConfig=0)
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
                        new_home_configs_to_handles[tuple(qhome)]=[hi]
                        handles_to_configs[hi].append((qphi,qhi,qhome))
                        new_handles_to_configs[hi].append((qphi,qhi,qhome))
                        qhomes.append(qhome)
                        N_added += 1
                        if  N_added>= Nconfigs:
                                break

        handles_to_configs_temp = {}  #dictionary with the same format as handles_to_configs, used to store configurations temporaly before concatenating them with those from handles_to_configs
        for hi in progressbar_iterable(handles, "Check if configs from other handles are valid"):
                #this loop goes through the configurations computed for handles != hi and checks whether these configurations are also valid for hi
                handles_to_configs_temp[hi] = []
                for (other_hi,configs) in new_handles_to_configs.items():
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
                                        res, qhi = generate_valid_config(ci, qguesses=[qphi,], NrandomConfig=0)
                                        if not res:
                                                f.write("error checking if {}th grasp config from {} is valid for {}\n".format(j,other_hi,hi))
                                                continue
                                        handles_to_configs_temp[hi].append((qphi,qhi,qhome))
                                        home_configs_to_handles[tuple(qhome)].append(hi)
                                        new_home_configs_to_handles[tuple(qhome)].append(hi)

        f.close()
        for (hi,configs) in handles_to_configs_temp.items():
                #adding the additional configs to the initial dictionary
                for config in configs:
                        handles_to_configs[hi].append(config)
                        new_handles_to_configs[hi].append(config)
        print("--- %s seconds to generate ---" % (time.time() - start_time))
        print("\n{} configs added".format(len(qhomes)-1))
        for hi in handles:
                print("{} of which are valid for handle {}".format(len(handles_to_configs[hi]), hi))
        return home_configs_to_handles, handles_to_configs, qhomes, new_home_configs_to_handles, new_handles_to_configs,

def write_data(data, filename):
        with open(filename, 'wb') as f:
                pickle.dump(len(data), f)
                for value in data:
                        pickle.dump(value, f)

def read_data(filename):
        data = []
        with open(filename, 'rb') as f:
                for _ in range(pickle.load(f)):
                        data.append(pickle.load(f))
        return data


def compute_base_matrix(qhomes):
        """"Computes the paths and distances between each home pose (arm retracted)
        In :
             qhomes : list of home poses
        Out :
             paths : paths matrix
             distances : corresponding cost matrix"""
        start_time = time.time()
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
        print("--- %s seconds to compute base poses ---" % (time.time() - start_time))
        return paths, distances


def compute_arm_matrixes(home_configs_to_handles, handles_to_configs):
        """"Computes the paths and distances between each home pose and the handles it can reach
        In :
             home_configs_to_handles : dictionary associating a list of handles to each home pose
             handles_to_configs : dictionary associating a list of tuples (qphi, qhi,qhome) to each handle
        Out :
             home_to_handles_distances : dictionary associating a tuple (paths, costs) to each home
             nodes : the list of nodes, which will be used as the rows and columns of the GTSP matrix
             handle_indexes : dictionary associating the list of its indexes in the nodes list to each handle"""
        start_time = time.time()
        armPlanner=InStatePlanner()
        armPlanner.setEdge(loop_free)
        armPlanner.plannerType="kPRM*"
        armPlanner.cproblem.setParameter('kPRM*/numberOfNodes', Any(TC_long,200))
        home_to_handles_distances={}
        nodes=[(q0,None)]
        handle_indexes ={hi:[] for hi in handles_to_configs.keys()}
        for (tqhome,his) in home_configs_to_handles.items():
                lqhome=list(tqhome)
                q_pre_grasps=[lqhome]
                q_grasps = [lqhome]
                his.sort()
                for hi in his:
                        nodes.append((tqhome,hi))
                        handle_indexes[hi].append(len(nodes)-1)
                        configs=handles_to_configs[hi]
                        for config in configs:
                                if config[2]==lqhome:
                                        q_pre_grasps.append(config[0])
                                        q_grasps.append(config[1])
                armPlanner.createEmptyRoadmap()
                armpaths, armdistances = armPlanner.computeMatrix(q_pre_grasps,False)
                l = len(armpaths)
                handPlanner=InStatePlanner()
                handPlanner.createEmptyRoadmap()
                handPlanner.setEdge(loop_free)
                for i in range(l):
                        for j in range(i+1,l):
                                between_pre_grasps = armpaths[i][j]
                                if between_pre_grasps:
                                        if i==0:
                                                pre_to_grasp = handPlanner.computePath(q_pre_grasps[j], q_grasps[j], True)
                                                paths = [between_pre_grasps, pre_to_grasp]
                                        else:
                                                grasp_to_pre = handPlanner.computePath(q_grasps[i], q_pre_grasps[i], True)
                                                pre_to_grasp = handPlanner.computePath(q_pre_grasps[j], q_grasps[j], True)
                                                paths = [grasp_to_pre, between_pre_grasps, pre_to_grasp]
                                        armpaths[i][j] = concatenate_paths(paths)
                                        #print(armpaths[i][j].length()-between_pre_grasps.length(),(i,j))
                                #if armpaths[i][j]:
                                        armpaths[j][i]=armpaths[i][j].reverse()
                                else:
                                        armpaths[j][i]=None
                home_to_handles_distances[tqhome] = armpaths, armdistances
        print("--- %s seconds to compute generate arm poses ---" % (time.time() - start_time))
        return home_to_handles_distances, nodes, handle_indexes

def compute_additional_arm_matrixes(new_home_configs_to_handles, new_handles_to_configs, home_to_handles_distances, nodes, handle_indexes):
        """"Computes the paths and distances between each home pose and the handles it can reach
        In :
             home_configs_to_handles : dictionary associating a list of handles to each home pose
             handles_to_configs : dictionary associating a list of tuples (qphi, qhi,qhome) to each handle
        Out :
             home_to_handles_distances : dictionary associating a tuple (paths, costs) to each home
             nodes : the list of nodes, which will be used as the rows and columns of the GTSP matrix
             handle_indexes : dictionary associating the list of its indexes in the nodes list to each handle"""
        start_time = time.time()
        armPlanner=InStatePlanner()
        armPlanner.setEdge(loop_free)
        armPlanner.plannerType="kPRM*"
        armPlanner.cproblem.setParameter('kPRM*/numberOfNodes', Any(TC_long,200))
        #home_to_handles_distances={}
        #nodes=[(q0,None)]
        #handle_indexes ={hi:[] for hi in handles_to_configs.keys()}
        for (tqhome,his) in new_home_configs_to_handles.items():
                lqhome=list(tqhome)
                q_pre_grasps=[lqhome]
                q_grasps = [lqhome]
                his.sort()
                for hi in his:
                        nodes.append((tqhome,hi))
                        handle_indexes[hi].append(len(nodes)-1)
                        configs=new_handles_to_configs[hi]
                        for config in configs:
                                if config[2]==lqhome:
                                        q_pre_grasps.append(config[0])
                                        q_grasps.append(config[1])
                armPlanner.createEmptyRoadmap()
                armpaths, armdistances = armPlanner.computeMatrix(q_pre_grasps,False)
                l = len(armpaths)
                handPlanner=InStatePlanner()
                handPlanner.createEmptyRoadmap()
                handPlanner.setEdge(loop_free)
                for i in range(l):
                        for j in range(i+1,l):
                                between_pre_grasps = armpaths[i][j]
                                if between_pre_grasps:
                                        if i==0:
                                                pre_to_grasp = handPlanner.computePath(q_pre_grasps[j], q_grasps[j], True)
                                                paths = [between_pre_grasps, pre_to_grasp]
                                        else:
                                                grasp_to_pre = handPlanner.computePath(q_grasps[i], q_pre_grasps[i], True)
                                                pre_to_grasp = handPlanner.computePath(q_pre_grasps[j], q_grasps[j], True)
                                                paths = [grasp_to_pre, between_pre_grasps, pre_to_grasp]
                                        armpaths[i][j] = concatenate_paths(paths)
                                        #print(armpaths[i][j].length()-between_pre_grasps.length(),(i,j))
                                #if armpaths[i][j]:
                                        armpaths[j][i]=armpaths[i][j].reverse()
                                else:
                                        armpaths[j][i]=None
                home_to_handles_distances[tqhome] = armpaths, armdistances
        print("--- %s seconds to compute generate arm poses ---" % (time.time() - start_time))
        return home_to_handles_distances, nodes, handle_indexes

def compute_approximate_arm_matrixes(home_configs_to_handles, handles_to_configs):
        start_time = time.time()
        armPlanner=InStatePlanner()
        armPlanner.setEdge(loop_free)
        distance = armPlanner.cproblem.getDistance()
        home_to_handles_distances={}
        nodes=[(q0,None)]
        handle_indexes ={hi:[] for hi in handles_to_configs.keys()}
        for (tqhome,his) in home_configs_to_handles.items():
                lqhome=list(tqhome)
                q_pre_grasps=[lqhome]
                his.sort()
                for hi in his:
                        nodes.append((tqhome,hi))
                        handle_indexes[hi].append(len(nodes)-1)
                        configs=handles_to_configs[hi]
                        for config in configs:
                                if config[2]==lqhome:
                                        q_pre_grasps.append(config[0])
                l=len(q_pre_grasps)
                armdistances = np.zeros((l,l))
                for i in range(l):
                        for j in range(i+1, l):
                                armdistances[i][j] = armdistances[j][i] = distance.call(q_pre_grasps[i], q_pre_grasps[j])
                home_to_handles_distances[tqhome] = None, armdistances
        print("--- %s seconds to compute generate approx arm poses ---" % (time.time() - start_time))
        return home_to_handles_distances, nodes, handle_indexes


def compute_cost_matrix(nodes, qhomes, paths, distances, home_configs_to_handles, home_to_handles_distances, ponderator = 20):
        """"Computes the cost matrix of the GTSP and the associated paths
        In :
             nodes : list of nodes
             qhomes : list of homes
             paths : paths between the home poses
             distances : distances between the home poses
             home_configs_to_handles : dictionary associating a list of handles to each home pose
             home_to_handles_distances : dictionary associating a tuple (paths, costs) to each home
             ponderator : ponderator to weigh the edges that require to move between home poses
        Out :
             pathsGTSP : the paths matrix between all the nodes
             distancesGTSP the corresponding costs matrix"""
        start_time = time.time()
        n=len(nodes)
        r=robot.rankInConfiguration["tiago/root_joint"]
        distancesGTSP = np.zeros((n,n))
        pathsGTSP = np.full((n,n), None)
        for i in progressbar_iterable(range(n), "Compute final matrix"):
                for j in range(i+1,n):
                        if i==0:
                                q,h = nodes[j]
                                index = qhomes.index(list(q))
                                q0_to_q = distances[i][index]
                                path1 = paths[i][index]

                                index = home_configs_to_handles[q].index(h)
                                d_q = home_to_handles_distances[q]
                                q_to_h = d_q[1][0][index+1]
                                path2 = d_q[0][0][index+1]

                                distancesGTSP[i][j] = distancesGTSP[j][i] = q0_to_q+q_to_h
                                if path1 and path2:
                                         pathsGTSP[i][j] = concatenate_paths([path1,path2])
                                         pathsGTSP[j][i] = pathsGTSP[i][j]#.reverse()
                                else :
                                         pathsGTSP[i][j] = pathsGTSP[j][i] = None
                        else:
                                q1,h1=nodes[i]
                                q2,h2=nodes[j]
                                if q1[r:r+4]==q2[r:r+4]:
                                        h_q1 = home_configs_to_handles[q1]
                                        index_1 = h_q1.index(h1)
                                        index_2 = h_q1.index(h2)
                                        d_q1 = home_to_handles_distances[q1]
                                        h1_to_h2 = d_q1[1][index_1+1][index_2+1]
                                        path = d_q1[0][index_1+1][index_2+1]

                                        distancesGTSP[i][j] = distancesGTSP[j][i] = h1_to_h2
                                        if path:
                                                 pathsGTSP[i][j] = path
                                                 pathsGTSP[j][i] = path#.reverse()

                                else:
                                        index_q1 = qhomes.index(list(q1))
                                        index_q2 = qhomes.index(list(q2))
                                        q1_to_q2 = distances[index_q1][index_q2]
                                        base_path = paths[index_q1][index_q2]

                                        index_h1 = home_configs_to_handles[q1].index(h1)
                                        index_h2 = home_configs_to_handles[q2].index(h2)
                                        d_q1 = home_to_handles_distances[q1]
                                        d_q2 = home_to_handles_distances[q2]
                                        h1_to_q1 = d_q1[1][index_h1+1][0]
                                        q2_to_h2 = d_q2[1][0][index_h2+1]
                                        path1 = d_q1[0][index_h1+1][0]
                                        path2 = d_q2[0][0][index_h2+1]

                                        distancesGTSP[i][j] = distancesGTSP[j][i] = h1_to_q1 + q1_to_q2*ponderator + q2_to_h2
                                        if path1 and base_path and path2:
                                                 pathsGTSP[i][j] = concatenate_paths([path1,base_path,path2])
                                                 pathsGTSP[j][i] = pathsGTSP[i][j]#.reverse()
                                        else:
                                                 pathsGTSP[i][j] = pathsGTSP[j][i] = None
        print("--- %s seconds to compute cost matrix ---" % (time.time() - start_time))
        return pathsGTSP, distancesGTSP


def compute_approximate_cost_matrix(nodes, qhomes, distances, home_configs_to_handles, home_to_handles_distances, ponderator = 20):
        start_time = time.time()
        n=len(nodes)
        r=robot.rankInConfiguration["tiago/root_joint"]
        distancesGTSP = np.zeros((n,n))
        for i in progressbar_iterable(range(n), "Compute final matrix"):
                for j in range(i+1,n):
                        if i==0:
                                q,h = nodes[j]
                                index = qhomes.index(list(q))
                                q0_to_q = distances[i][index]

                                index = home_configs_to_handles[q].index(h)
                                q_to_h = home_to_handles_distances[q][1][0][index+1]

                                distancesGTSP[i][j] = distancesGTSP[j][i] = q0_to_q+q_to_h
                        else:
                                q1,h1=nodes[i]
                                q2,h2=nodes[j]
                                if q1[r:r+4]==q2[r:r+4]:
                                        index_1 = home_configs_to_handles[q1].index(h1)
                                        index_2 = home_configs_to_handles[q1].index(h2)
                                        h1_to_h2 = home_to_handles_distances[q1][1][index_1+1][index_2+1]

                                        distancesGTSP[i][j] = distancesGTSP[j][i] = h1_to_h2
                                else:
                                        index_q1 = qhomes.index(list(q1))
                                        index_q2 = qhomes.index(list(q2))
                                        q1_to_q2 = distances[index_q1][index_q2]

                                        index_h1 = home_configs_to_handles[q1].index(h1)
                                        index_h2 = home_configs_to_handles[q2].index(h2)
                                        h1_to_q1 = home_to_handles_distances[q1][1][index_h1+1][0]
                                        q2_to_h2 = home_to_handles_distances[q2][1][0][index_h2+1]
                                        distancesGTSP[i][j] = distancesGTSP[j][i] = h1_to_q1 + q1_to_q2*ponderator + q2_to_h2
        print("--- %s seconds to compute approx cost matrix ---" % (time.time() - start_time))
        return distancesGTSP


def generate_GLNS_file(distances, nodes, indexes, holes, precision):
        start_time = time.time()
        distances_file = distances.copy()
        n_groups = len(holes) +1
        n_nodes = len(nodes)
        filename = 'data/' + str(n_groups) + 'test' +str(n_nodes)
        with open(filename + '.gtsp', 'w') as f:
                f.write('NAME: ' + filename + '\n')
                f.write('TYPE: GTSP' + '\n')
                f.write('COMMENT: ' + str(n_nodes) + ' nodes' + '\n')
                f.write('DIMENSION: ' + str(n_nodes) + '\n')
                f.write('GTSP_SETS: ' + str(n_groups) + '\n')
                f.write('EDGE_WEIGHT_TYPE: EXPLICIT' + '\n')
                f.write('EDGE_WEIGHT_FORMAT: FULL_MATRIX ' + '\n')
                f.write('EDGE_WEIGHT_SECTION' + '\n')
                for i in range(n_nodes):
                        for j in range(n_nodes):
                                if i==j :
                                        distances_file[i][j] = 10**(precision+5) -1
                                else :
                                        if distances_file[i][j]>10**7:
                                                distances_file[i][j] = distances_file[0][0]
                                        else:
                                                distances_file[i][j] = round(distances_file[i][j], precision)*10**precision
                np.savetxt(f, distances_file,fmt='%8u')
                #np.savetxt(f, distances,fmt='%.8f')
                f.write('GTSP_SET_SECTION:' + '\n')
                f.write('1 1 -1\n')
                for i,hi in enumerate(holes):
                        f.write(str(i+2) + ' ')
                        for x in indexes[hi]:
                                f.write(str(x+1) + ' ')
                        f.write(str(-1) + '\n')
                f.write('EOF')
                print("--- %s seconds to generate GLNS file ---" % (time.time() - start_time))
        return filename +'.gtsp', distances


def get_chosen_nodes_from_GLNS(fichier):
        #get list from file
        with open(fichier, 'r') as f:
                for last_line in f:
                        pass
        res = last_line.split('Int64')[1][1:-1]
        res = [int(x.strip()) for x in res.split(',')]
        #shift elements to have the list start with depot
        n = res.index(1)
        for _ in range(n):
                res.append(res.pop(0))
        #go back to python indexing
        for i in range(len(res)):
                res[i]-=1
        #Add depot node at the end of the tour
        res.append(0)
        return res


def show_tour(pathsGTSP, distancesGTSP, nodes, path_nodes):
        start_time = time.time()
        paths = []
        d=0
        for i,j in zip(path_nodes,path_nodes[1:]):
                if j<i:
                        paths.append(pathsGTSP[i][j].reverse())
                else:
                        paths.append(pathsGTSP[i][j])
                d+=distancesGTSP[i][j]
        path = concatenate_paths(paths)
        ps.client.basic.problem.addPath(path)
        d2=path.length()
        print("--- %s seconds to show tour ---" % (time.time() - start_time))
        for x in path_nodes:
                print(str(nodes[x][1]) + '->')
        return d, d2


def show_approximate_tour(qhomes, base_paths, nodes, path_nodes, handles_to_configs, distancesGTSP,  ponderator = 20):
        start_time = time.time()
        armPlanner=InStatePlanner()
        armPlanner.setEdge(loop_free)
        armPlanner.plannerType="kPRM*"
        armPlanner.cproblem.setParameter('kPRM*/numberOfNodes', Any(TC_long,200))
        armPlanner.createEmptyRoadmap()
        r=robot.rankInConfiguration["tiago/root_joint"]
        paths = []
        d=0
        d_approx = 0
        if path_nodes[0]!=0:
                raise ValueError("first node should be 0")
        if path_nodes[-1]!=0:
                raise ValueError("last node should be 0")
        for i,j in progressbar_iterable(zip(path_nodes,path_nodes[1:]), "compute chosen paths"):
                d_approx+=distancesGTSP[i][j]
                if i==0:
                        q_home,h = nodes[j]
                        q_home=list(q_home)
                        index = qhomes.index(q_home)
                        path_q0_to_q = base_paths[i][index]

                        configs=handles_to_configs[h]
                        for config in configs:
                                if config[2]==q_home:
                                        q_pre_grasp=config[0]
                                        q_grasp = config[1]
                        try:
                                path_q_to_h = armPlanner.computePath(q_home,q_pre_grasp, True)
                                hand_path = armPlanner.computePath(q_pre_grasp, q_grasp, True)
                                paths.extend([path_q0_to_q,path_q_to_h,hand_path])
                        except Exception as e:
                                print(e)
                        d+= path_q0_to_q.length() + path_q_to_h.length()
                elif j==0:
                        q_home,h = nodes[i]
                        q_home=list(q_home)
                        index = qhomes.index(q_home)
                        path_q_to_q0 = base_paths[index][j]

                        try:
                                hand_path = armPlanner.computePath(q_grasp, q_pre_grasp, True)
                                path_h_to_q = armPlanner.computePath(q_pre_grasp,q_home,True)
                                paths.extend([hand_path, path_h_to_q, path_q_to_q0])
                        except Exception as e:
                                print(e)
                        d+= path_q_to_q0.length() + path_h_to_q.length()
                else:
                        q1,h1=nodes[i]
                        q1 = list(q1)
                        q2,h2=nodes[j]
                        q2 = list(q2)
                        if q1[r:r+4]==q2[r:r+4]:
                                q_pre_1 = q_pre_grasp
                                q_grasp_1 = q_grasp
                                configs=handles_to_configs[h2]
                                for config in configs:
                                        if config[2]==q1:
                                                q_pre_grasp=config[0]
                                                q_grasp = config[1]
                                try:
                                        hand_path_1 = armPlanner.computePath(q_grasp_1,q_pre_1,True)
                                        path_h1_to_h2 = armPlanner.computePath(q_pre_1,q_pre_grasp,True)
                                        hand_path_2 = armPlanner.computePath(q_pre_grasp,q_grasp,True)
                                        paths.extend([hand_path_1, path_h1_to_h2, hand_path_2])
                                except Exception as e:
                                        print(e)
                                d+= path_h1_to_h2.length()
                        else:
                                index_q1 = qhomes.index(q1)
                                index_q2 = qhomes.index(q2)
                                path_q1_to_q2 = base_paths[index_q1][index_q2]

                                q_pre_1 = q_pre_grasp
                                q_grasp_1 = q_grasp

                                configs=handles_to_configs[h2]
                                for config in configs:
                                        if config[2]==q2:
                                                q_pre_grasp=config[0]
                                                q_grasp = config[1]
                                try:
                                        hand_path_1 = armPlanner.computePath(q_grasp_1,q_pre_1,True)
                                        path_h1_to_q1 = armPlanner.computePath(q_pre_1,q1,True)
                                        path_q2_to_h2 = armPlanner.computePath(q2,q_pre_grasp,True)
                                        hand_path_2 = armPlanner.computePath(q_pre_grasp,q_grasp,True)
                                        paths.extend([hand_path_1, path_h1_to_q1, path_q1_to_q2, path_q2_to_h2, hand_path_2])
                                except Exception as e:
                                        print(e)
                                d+= path_q1_to_q2.length()*ponderator + path_h1_to_q1.length() + path_q2_to_h2.length()
        path = concatenate_paths(paths)
        d2=path.length()
        ps.client.basic.problem.addPath(path)
        print("--- %s seconds to show approx tour ---" % (time.time() - start_time))
        for x in path_nodes:
                print(str(nodes[x][1]) + '->')
        return d_approx, d, d2


def show_approximate_tour_and_iterate(qhomes, base_paths, nodes, path_nodes, handles_to_configs, distancesGTSP,
                          old_distancesGTSP, pathsGTSP,  ponderator = 20):
        armPlanner=InStatePlanner()
        armPlanner.setEdge(loop_free)
        armPlanner.plannerType="kPRM*"
        armPlanner.cproblem.setParameter('kPRM*/numberOfNodes', Any(TC_long,300))
        armPlanner.createEmptyRoadmap()
        r=robot.rankInConfiguration["tiago/root_joint"]
        paths = []
        d=0
        cpt = 0
        d_approx = 0
        old_dist = 0
        if path_nodes[0]!=0:
                raise ValueError("first node should be 0")
        if path_nodes[-1]!=0:
                raise ValueError("last node should be 0")
        for i,j in progressbar_iterable(zip(path_nodes,path_nodes[1:]), "compute chosen paths"):
                d_approx+=distancesGTSP[i][j]
                old_dist += old_distancesGTSP[i][j]
                path_to_check = pathsGTSP[i][j]
                if path_to_check:
                        paths.append(path_to_check)
                        d+= distancesGTSP[i][j]
                else:
                        if i==0:
                                q_home,h = nodes[j]
                                q_home=list(q_home)
                                index = qhomes.index(q_home)
                                path_q0_to_q = base_paths[i][index]

                                configs=handles_to_configs[h]
                                for config in configs:
                                        if config[2]==q_home:
                                                q_pre_grasp=config[0]
                                                q_grasp = config[1]
                                try:
                                        path_q_to_h = armPlanner.computePath(q_home,q_pre_grasp, True)
                                        hand_path = armPlanner.computePath(q_pre_grasp, q_grasp, True)
                                        current_path = concatenate_paths([path_q0_to_q,path_q_to_h,hand_path])
                                        paths.append(current_path)
                                except Exception as e:
                                        print(e)
                                new_d = path_q0_to_q.length() + path_q_to_h.length()
                                d+=new_d
                        elif j==0:
                                q_home,h = nodes[i]
                                q_home=list(q_home)
                                index = qhomes.index(q_home)
                                path_q_to_q0 = base_paths[index][j]

                                configs=handles_to_configs[h]
                                for config in configs:
                                        if config[2]==q_home:
                                                q_pre_grasp=config[0]
                                                q_grasp = config[1]

                                try:
                                        hand_path = armPlanner.computePath(q_grasp, q_pre_grasp, True)
                                        path_h_to_q = armPlanner.computePath(q_pre_grasp,q_home,True)
                                        current_path = concatenate_paths([hand_path, path_h_to_q, path_q_to_q0])
                                        paths.append(current_path)
                                except Exception as e:
                                        print(e)
                                new_d = path_q_to_q0.length() + path_h_to_q.length()
                                d+=new_d
                        else:
                                q1,h1=nodes[i]
                                q1 = list(q1)
                                q2,h2=nodes[j]
                                q2 = list(q2)
                                if q1[r:r+4]==q2[r:r+4]:
                                        configs=handles_to_configs[h1]
                                        for config in configs:
                                                if config[2]==q1:
                                                        q_pre_1=config[0]
                                                        q_grasp_1 = config[1]
                                        configs=handles_to_configs[h2]
                                        for config in configs:
                                                if config[2]==q1:
                                                        q_pre_grasp=config[0]
                                                        q_grasp = config[1]
                                        try:
                                                hand_path_1 = armPlanner.computePath(q_grasp_1,q_pre_1,True)
                                                path_h1_to_h2 = armPlanner.computePath(q_pre_1,q_pre_grasp,True)
                                                hand_path_2 = armPlanner.computePath(q_pre_grasp,q_grasp,True)
                                                current_path = concatenate_paths([hand_path_1, path_h1_to_h2, hand_path_2])
                                                paths.append(current_path)
                                        except Exception as e:
                                                print(e)
                                        new_d = path_h1_to_h2.length()
                                        d+=new_d
                                else:
                                        index_q1 = qhomes.index(q1)
                                        index_q2 = qhomes.index(q2)
                                        path_q1_to_q2 = base_paths[index_q1][index_q2]

                                        configs=handles_to_configs[h1]
                                        for config in configs:
                                                if config[2]==q1:
                                                        q_pre_1=config[0]
                                                        q_grasp_1 = config[1]

                                        configs=handles_to_configs[h2]
                                        for config in configs:
                                                if config[2]==q2:
                                                        q_pre_grasp=config[0]
                                                        q_grasp = config[1]
                                        try:
                                                hand_path_1 = armPlanner.computePath(q_grasp_1,q_pre_1,True)
                                                path_h1_to_q1 = armPlanner.computePath(q_pre_1,q1,True)
                                                path_q2_to_h2 = armPlanner.computePath(q2,q_pre_grasp,True)
                                                hand_path_2 = armPlanner.computePath(q_pre_grasp,q_grasp,True)
                                                current_path = concatenate_paths([hand_path_1, path_h1_to_q1, path_q1_to_q2, path_q2_to_h2, hand_path_2])
                                                paths.append(current_path)
                                        except Exception as e:
                                                print(e)
                                        new_d = path_q1_to_q2.length()*ponderator + path_h1_to_q1.length() + path_q2_to_h2.length()
                                        d+=new_d
                        distancesGTSP[i][j] = distancesGTSP[j][i] = new_d
                        pathsGTSP[i][j] = current_path
                        pathsGTSP[j][i] = current_path.reverse()
                        cpt += 1
        path = concatenate_paths(paths)
        d2=path.length()
        ps.client.basic.problem.addPath(path)
        print("updates : " + str(cpt))
        return old_dist, d_approx, d, d2

# def computeBetterPath(planner, q1, q2, resetRoadmap = False ):
#         candidate1 = planner.computePath(q1,q2,resetRoadmap)
#         candidate2 = planner.computePath(q1,q2,resetRoadmap)
#         return candidate1 if candidate1.length()<candidate2.length() else candidate2

def compute_GLNS(handles,homes_to_holes, handles_to_configs, homes_list, ponderator=20, precision = 3):
        base_paths, base_distances = compute_base_matrix(homes_list)
        homes_to_holes_distances,nodes,indexes = compute_arm_matrixes(homes_to_holes, handles_to_configs)
        write_data((homes_to_holes_distances,nodes,indexes),'data/arms.txt')
        pathsGTSP, distancesGTSP = compute_cost_matrix(nodes, homes_list, base_paths, base_distances,
                                               homes_to_holes, homes_to_holes_distances, ponderator)
        f,d = generate_GLNS_file(distancesGTSP, nodes, indexes, handles, precision)
        return f,d, nodes, pathsGTSP, distancesGTSP


def compute_additional_GLNS(handles,new_homes_to_holes, homes_to_holes, handles_to_configs, homes_list, ponderator=20, precision = 3):
        base_paths, base_distances = compute_base_matrix(homes_list)
        old_homes_to_holes_distances,old_nodes,old_indexes = read_data('data/arms.txt')
        homes_to_holes_distances,nodes,indexes = compute_additional_arm_matrixes(new_homes_to_holes, handles_to_configs,
                                                                                 old_homes_to_holes_distances,old_nodes,old_indexes)
        write_data((homes_to_holes_distances,nodes,indexes),'data/arms.txt')
        pathsGTSP, distancesGTSP = compute_cost_matrix(nodes, homes_list, base_paths, base_distances,
                                                       homes_to_holes, homes_to_holes_distances, ponderator)
        f,d = generate_GLNS_file(distancesGTSP, nodes, indexes, handles, precision)
        return f,d, nodes, pathsGTSP, distancesGTSP

def compute_approximative_GLNS(handles, homes_to_holes, handles_to_configs, homes_list, ponderator=20, precision = 3):
        base_paths, base_distances = compute_base_matrix(homes_list)
        homes_to_holes_distances,nodes,indexes = compute_approximate_arm_matrixes(homes_to_holes, handles_to_configs)
        distancesGTSP = compute_approximate_cost_matrix(nodes, homes_list, base_distances,
                                                        homes_to_holes, homes_to_holes_distances, ponderator)
        f,d = generate_GLNS_file(distancesGTSP, nodes, indexes, handles, precision)
        return f,d, homes_list, base_paths, nodes, indexes, distancesGTSP

def iterate(handles, homes_to_holes, handles_to_configs, homes_list, ponderator=20, precision = 3, max_iter = 250):
        base_paths, base_distances = compute_base_matrix(homes_list)
        homes_to_holes_distances,nodes,indexes = compute_approximate_arm_matrixes(homes_to_holes, handles_to_configs)
        distancesGTSP = compute_approximate_cost_matrix(nodes, homes_list, base_distances,
                                                        homes_to_holes, homes_to_holes_distances, ponderator)
        filename,d = generate_GLNS_file(distancesGTSP, nodes, indexes, handles, precision)
        with open("script.jl", 'w') as f:
                f.write("using GLNS\n")
                f.write("GLNS.solver(\""+filename+"\", output=\"data/tour.txt\")")
        os.system("julia script.jl")
        path_nodes = get_chosen_nodes_from_GLNS('data/tour.txt')
        n = len(nodes)
        pathsGTSP = np.full((n,n), None)
        old_dists = distancesGTSP.copy()
        with open('data/results.csv', mode='w') as results:
                res_writer = csv.writer(results, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

                res_writer.writerow(['Initial approx weighted length','iterated length', 'weighted length', 'gepetto length', 'time'])
                d_old, d_approx, d_pond, d_gep = show_approximate_tour_and_iterate(homes_list, base_paths, nodes,
                                                                                       path_nodes,
                                                                                       handles_to_configs, distancesGTSP,
                                                                                       old_dists,
                                                                                       pathsGTSP, ponderator=20)
                res_writer.writerow([d_old, d_approx, d_pond, d_gep])
                best = d_pond
                best_path = path_nodes
                print( "BEST : " + str(best))
                current = d_approx
                cpt = 0
                tries = 0
                while (current < (best - 1e-5) or tries <2) and cpt<max_iter:
                        print(current < best or tries <2, cpt<max_iter, tries)
                        start_time = time.time()
                        if current >= best :
                                tries += 1
                        else:
                                tries = 0
                        f,d = generate_GLNS_file(distancesGTSP, nodes, indexes, handles, precision)
                        os.system("julia script.jl")
                        path_nodes = get_chosen_nodes_from_GLNS('data/tour.txt')
                        d_old, d_approxiter, d_pond, d_gep = show_approximate_tour_and_iterate(homes_list, base_paths, nodes,
                                                                                        path_nodes,
                                                                                        handles_to_configs, distancesGTSP,
                                                                                        old_dists,
                                                                                        pathsGTSP, ponderator=20)
                        #print(d_old, d_approxiter, d_pond, d_gep)
                        t_iter = time.time() - start_time
                        res_writer.writerow([d_old, d_approxiter, d_pond, d_gep, t_iter])
                        current = d_approxiter
                        if d_pond < best:
                                print("UPDATE iter " + str(cpt) + " " + str(best) + "-->" + str(d_pond))
                                best = d_pond
                                best_path = path_nodes
                        cpt += 1
                        print(current, best)
        print(str(cpt) + " iterations")
        return(best_path, distancesGTSP, nodes, indexes, homes_list, base_paths, path_nodes, handles_to_configs, old_dists)






"""
holes=part_handles
homes_to_holes, handles_to_configs, homes_list = generateConfigs(holes, 1,100)   #2 configs par trou
homes_to_holes, handles_to_configs, homes_list, newhomes_to_holes, \
newhandles_to_configs = generateAdditionalConfigs(holes,homes_to_holes, handles_to_configs, homes_list,1,100)


f,d, homes_list, base_paths, nodes, indexes, distancesGTSP = compute_approximative_GLNS(holes, homes_to_holes,
                                                                               handles_to_configs, homes_list, ponderator=20)
path_nodes = get_chosen_nodes_from_GLNS('data/tour.txt')
show_approximate_tour(homes_list, base_paths, nodes, path_nodes, handles_to_configs, distancesGTSP,ponderator = 20)

n = len(nodes)
pathsGTSP = pathsGTSP = np.full((n,n), None)
old_dists = distancesGTSP.copy()
path_nodes = get_chosen_nodes_from_GLNS('data/tour.txt')
show_approximate_tour_and_iterate(homes_list, base_paths, nodes, path_nodes, handles_to_configs, distancesGTSP, old_dists, pathsGTSP, \
ponderator = 20)
f,d = generate_GLNS_file(distancesGTSP, nodes, indexes, holes, precision=3)


best_path, distancesGTSP, nodes, indexes, homes_list, base_paths, path_nodes, handles_to_configs, old_dists = iterate(holes, \
homes_to_holes, handles_to_configs, homes_list, ponderator=20)


f,d, nodes, pathsGTSP, distancesGTSP= compute_GLNS(holes,homes_to_holes, handles_to_configs, homes_list, ponderator=20)
path_nodes = get_chosen_nodes_from_GLNS('data/tour.txt')
show_tour(pathsGTSP, distancesGTSP, nodes,path_nodes)

f,d, nodes, pathsGTSP, distancesGTSP= compute_additional_GLNS(holes,newhomes_to_holes, homes_to_holes,
                                                              newhandles_to_configs, homes_list, ponderator=20)
path_nodes = get_chosen_nodes_from_GLNS('data/tour.txt')
show_tour(pathsGTSP, distancesGTSP, nodes,path_nodes)



write_data((homes_to_holes, handles_to_configs, homes_list),'data/configs.txt')
homes_to_holes, handles_to_configs, homes_list = read_data('data/configs.txt') 



write_data((homes_to_holes, handles_to_configs, homes_list, newhomes_to_holes, newhandles_to_configs),'data/configs2.txt')
homes_to_holes, handles_to_configs, homes_list, newhomes_to_holes, \
newhandles_to_configs = read_data('data/configs2.txt')



base_paths, base_distances = compute_base_matrix(homes_list)
homes_to_holes_distances,nodes,indexes = compute_arm_matrixes(homes_to_holes, handles_to_configs)
pathsGTSP, distancesGTSP = compute_cost_matrix(nodes, homes_list, base_paths, base_distances,
                                               homes_to_holes, homes_to_holes_distances, 10)
f,d = generate_GLNS_file(distancesGTSP, nodes, indexes, holes, 2)




path_nodes = get_chosen_nodes_from_GLNS('data/tour.txt')
show_tour(pathsGTSP, d, nodes,path_nodes)
"""
#show a path
#ps.client.basic.problem.addPath(pathsGTSP[1][2])

#to show home configs in gepetto-gui: (assuming v=vf.createViewer())
# for hi in part_handles:
#       for configs in handles_to_configs[hi]:
#               v(configs[1])
#               time.sleep(0.3)
