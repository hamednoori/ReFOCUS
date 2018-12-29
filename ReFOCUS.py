#!/usr/bin/env python
##### Author: Mahboobe Rezaee #####
from __future__ import division

import os
import ast
import sys
import subprocess
import signal
import socket
import logging
import thread
import time
import tempfile
import math
import random
import networkx as nx
from collections import defaultdict, deque
from math import log
import sumolib
from k_shortest_paths import k_shortest_paths
from optparse import OptionParser
from bs4 import BeautifulSoup
dict_edgeRSUs={}
dict_lane={}
laneallgraph=[]
edgeallgraph=[]
edgeallsgraph=[]
visit_bfs=[]
dict_fc={}
list_source={}
list_present_network=[]
list_vehicle_set_route=[]

TMax=0

# We need to import Python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Environment variable SUMO_HOME not defined")
    
import traci

class UnusedPortLock:
    lock = thread.allocate_lock()

    def __init__(self):
        self.acquired = False

    def __enter__(self):
        self.acquire()

    def __exit__(self):
        self.release()

    def acquire(self):
        if not self.acquired:
            UnusedPortLock.lock.acquire()
            self.acquired = True

    def release(self):
        if self.acquired:
            UnusedPortLock.lock.release()
            self.acquired = False

def find_unused_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
    sock.bind(('127.0.0.1', 0))
    sock.listen(socket.SOMAXCONN)
    ipaddr, port = sock.getsockname()
    sock.close()
    
    return port

def terminate_sumo(sumo):
    if sumo.returncode == None:
        os.kill(sumo.pid, signal.SIGTERM)
        time.sleep(0.5)
        if sumo.returncode == None:
            print os.__file__
            #os.kill(sumo.pid, signal.SIGKILL)
            time.sleep(1)
            if sumo.returncode == None:
                time.sleep(10)
    
	
	
	
	
	
def build_road_graph(network):                
    # Input   
    f = open(network, 'r')
    data = f.read()
    soup = BeautifulSoup(data)
    f.close()
    sys.stdout = open('RSU.txt','wt')

	
    edges_length={}
    for edge_tag in soup.findAll("edge"):
	  edge_id = edge_tag["id"]
	  lane_tag = edge_tag.find("lane")
	  edge_length = int(float(lane_tag["length"]))
	  edges_length[edge_id] = edge_length
	  edgeallsgraph.append(edge_id)
	  lane_tagall= edge_tag.findAll("lane")
	  laneid=[]
	  for lane in lane_tagall:
	   laneid.append(lane["id"])
	   laneallgraph.append(lane["id"])
	   dict_lane[edge_id]= laneid
        
	    
        # lane_tag = edge_tag.find("lane")
		
	
	# print(lane_tagall)
	
	
	# print("edge_id:{0}".format(edge_id)) 
	# laneid=[]
	# for lane in lane_tagall:
	 # laneid.append(lane["id"]) 
	 # laneallgraph.append(lane["id"])
	 # dict_lane[edge_id]= laneid
	# # print("dict_lane[{0}] = {1}".format(edge_id,dict_lane[edge_id])) 
        
    graph = nx.DiGraph() 
    for edg_tag in soup.findAll("edge"):
	 edg_id = edg_tag["id"]
	 if edg_id.startswith(":") : continue
	 edgeallgraph.append(edg_id)	
    
    for connection_tag in soup.findAll("connection"):
        source_edge = connection_tag["from"]        
        dest_edge = connection_tag["to"]
        #list_source.append(source_edge)       
        graph.add_edge(source_edge.encode("ascii"), dest_edge.encode("ascii"), length=edges_length[source_edge], weight=0 , TT=0, Counter=0)
        print("source= {0}    dest= {1}    length= {2}     weight= 0  TT=0 Counter=0" .format(source_edge.encode("ascii"),dest_edge.encode("ascii"),edges_length[source_edge]))
        print("\t\t\t")
    
    return graph
def bfs_edges(G, source, reverse=False):
    if reverse and isinstance(G, nx.DiGraph):
        neighbors = G.predecessors_iter
    else:
        neighbors = G.neighbors_iter
    visited = set([source])
    visit_bfs.append(source)
    queue=deque()
    # front = d.popleft()
    # u = front[0]
    # level = front[1]
    queue.append((source, 0))
    
    while queue:
        
        parent, children = queue[0]
        front= queue.popleft()
        u= front[0]
        print(u)
        level = front[1]
        print(level)
        if level >= 3:
		   break
	level+=1
        for v in G.neighbors(u):
            #child = next(children)
            if v not in visited:
                yield parent, v
                visited.add(v)
                visit_bfs.append(v) 
                logging.debug("visited %s::::" % visited)
                queue.append((v, level))
                
        #except StopIteration:
            #queue.popleft()
        print(visited)

    
	
def Road_RSU_congestion_index(edgelist,rsuid):####################################################3
          #for rsuid in RSU_ids:
			 list_of_vehicle=[]
			 congestedRoads=[]
			 vehiclecongested=[]
			 Sum_Zone_RSUs={}
			 Sum_Zone=0
			 Sum_Lane_L=0
			 Sum_Lane_WedgeZ=0
			 for edge in edgelist[rsuid]:
				  logging.debug("edgelist (%s, %s)" % (rsuid, edge))
				  lanelen = traci.lane.getLength(edge + '_' + '0')
				  Max_speed_LaneZ = traci.lane.getMaxSpeed(edge + '_' + '0')
				  MeanSpeedZ= traci.edge.getLastStepMeanSpeed(edge)
				  Kjam = lanelen / (5+2.5)
				  number_vehicles = traci.edge.getLastStepVehicleNumber(edge)
				  Wedgev = 1 - (MeanSpeedZ / Max_speed_LaneZ)
				  Wedgek = (number_vehicles / Kjam)
				  WedgeZ = (Wedgev + Wedgek) / 2
					#logging.debug("wedgewedgewedge : %s" % wlane)
				   #w += wlane # for each lane
				  Sum_Lane_L += lanelen
				  Sum_Lane_WedgeZ  += lanelen * WedgeZ
				  logging.debug("Sum_Lane_WedgeZ : %s" % Sum_Lane_WedgeZ )
				  logging.debug("sum_lane_len : %s" % Sum_Lane_L )
				  #logging.debug("j: %s" % f)
				  #s=traci.edge.getLastStepVehicleNumber(edge)
				  #logging.debug("getLastStepVehicleNumber: %s" % s)
				 
			 Sum_Zone = float(Sum_Lane_WedgeZ  / Sum_Lane_L )
			 logging.debug("Sum_ZoneSum_Zone: %s" % Sum_Zone)
			 return float(Sum_Zone)
					
			 
			 			 
def update_travel_time_on_roads(graph,time, begin_of_cycle):
    # sys.stdout = open('RSUu.txt','wt')
    sys.stdout = open("v.txt","wt")
    # for edge in list_source:
	 # dict_fc[edge]= traci.edge.getLastStepVehicleNumber(edge)
    for road in graph.nodes_iter():
        travel_time = traci.edge.getAdaptedTraveltime(road.encode("ascii"), time)
        
        dict_fc[road]= traci.edge.getLastStepVehicleNumber(road)
       # print("Road:{0}" .format(road))
       # print("getAdaptedTraveltime:{0}" .format(travel_time))
        if travel_time <= 0:
            travel_time = traci.edge.getTraveltime(road.encode("ascii"))
          #  print("travel_time:{0}" .format(travel_time))            

        for successor_road in graph.successors_iter(road):
            # print("successor_road:{0}" .format(successor_road))           
            if begin_of_cycle:
	        # print("begin_of_cycle:{0}" .format(begin_of_cycle))
                graph.edge[road][successor_road]["weight"] = travel_time
		#print("travel_time:{0}" .format(travel_time))
		#print("graph.edge[road][successor_road][weigh]:{0}" .format(graph.edge[road][successor_road]["weight"]))

            else:
		#print("begin_of_cycle:{0}" .format(begin_of_cycle))

                t = (graph.edge[road][successor_road]["weight"] + travel_time) / 2
                t = t if t > 0 else travel_time
                graph.edge[road][successor_road]["weight"] = t
                print("tttttttttttttttttt:{0}" .format(t))
	
def get_paths_and_weights(graph, simple_paths):
    paths_and_weights = {}
    # sys.stdout = open('uv.txt','wt')

    for path in simple_paths:
        weight = 0
        for i in range(len(path) - 1):
            u = path[i]
            v = path[i + 1]
            # print("u:{0}" .format(u))
            # print("v:{0}" .format(v))

            weight += graph.edge[u][v]["weight"]
            # print("weight:{0}" .format(weight))
            # print("tuple(path):{0}" .format(tuple(path)))
            # print("path:{0}" .format(path))

        paths_and_weights[tuple(path)] = weight
    return paths_and_weights 


	
                                                
def calculate_boltzmann_k_value(weights, max_weight, temperature=1):
    k = 0
    for weight in weights:
        normalized_weight = weight / max_weight
        k += math.exp(-(normalized_weight / temperature)) 
        
    return k
            
def choose_route(paths_and_weights, possible_paths, k, max_weight,vehicle, list_present_network,graph, temperature=1):  
    route = None
    highest_choice_prob = 0
    # path_update= {}
    # list_path=[]
    list_choice_prob=[]
    dict_choice_prob={}
    conges_lenghts=[]
    TotalLenght_Lanes=[]
    for path in possible_paths:
       TotalLenght_Lane=0
       conges_lenghtt=0
       for edge in path:
		     
         Lenght_Lane = traci.lane.getLength(edge + '_' + '0')
         TotalLenght_Lane += traci.lane.getLength(edge + '_' + '0')

         congestion_path = Lenght_Lane / (5+2.5)
         conges_lenghtt += (dict_fc[edge] / congestion_path)#* (log((dict_fc[edge]/congestion_path) * weight)) 

       conges_lenghts.append(conges_lenghtt)
       TotalLenght_Lanes.append(TotalLenght_Lane)

       
    for path in possible_paths:
      # if path not in list_path:
        logging.debug("path::::(%s)" % (path))
        N_fc=0
        Entropy=0
        conges_lenght=0
        congestion_path=0
        Lenght_Lane=0
        boltzmann_probb=0
        number_vehicle=0
        list_max=[]
        # for vehicle in list_vehicle_set_route:
         # source = traci.vehicle.getRoadID(vehicle)
         # if source.startswith(":"): continue
         # route = traci.vehicle.getRoute(vehicle)
         # logging.debug("Route::::(%s)" % (route))
         # destination = route[-1]
         # logging.debug("sourcesourcesourcesourcesourcesourcesourcesourcesourcesourcesourcesourcesourcesource::::(%s)" % (source))
        
            # print("u:{0}" .format(u))
            # print("v:{0}" .format(v))
        # path_weight = paths_and_weights[tuple(path)]
        # logging.debug("path_weight::::(%s)" % (path_weight))
        # normalized_path_weight = path_weight / max_weight
         
        # for edge in path:
         # congestion_path = Lenght_Lane / (5+2.5)
         # Entropy = (dict_fc[edge]/congestion_path)
         # list_max.append(Entropy)
        #minx= min(list_max)
        
        for edge in path:
		     # if edge == path[i]:
			   # u = path[i]
			   # v = path[i + 1]
			   # weight = graph.edge[u][v]["weight"]
         Lenght_Lane = traci.lane.getLength(edge + '_' + '0')
         number_vehicle += traci.lane.getLength(edge + '_' + '0')
         congestion_path = Lenght_Lane / (5+2.5)
         conges_lenght += (dict_fc[edge] / congestion_path)#* (log((dict_fc[edge]/congestion_path) * weight)) 
         
		 
         
		   
		 
        
        #N_fc = conges_lenght  
        path_weight = paths_and_weights[tuple(path)]
        logging.debug("path_weight::::(%s)" % (path_weight))
        normalized_path_weight = path_weight / max_weight
        logging.debug("normalized_path_weight::::(%s)" % (normalized_path_weight))
        #boltzmann_prob = math.exp(-(normalized_path_weight / 0.1)) / k
        
        logging.debug("boltzmann_probb::::(%s)" % ((conges_lenght / max(conges_lenghts))))

        #boltzmann_prob= (math.exp(-((normalized_path_weight ** 3) / (50 ** 1.5))) + (math.exp(-((normalized_path_weight ** 3) / (50** 1.5))))) / k
        #rand= random.random()
        #logging.debug("boltzmann_prob::::(%s)" % (boltzmann_prob))		
        choice_prob = ((conges_lenght / max(conges_lenghts)) * normalized_path_weight)
        logging.debug("choice_prob::::(%s)" % (choice_prob))
        logging.debug("conges_lenght::::(%s)" % (conges_lenght))
        logging.debug("max(conges_lenghts)::::(%s)" % (max(conges_lenghts)))

        list_choice_prob.append(choice_prob)
        dict_choice_prob[choice_prob]= path
    min_choice_prob = min(list_choice_prob)
    route = dict_choice_prob[min_choice_prob]
    if vehicle in list_present_network:
        routid = traci.vehicle.getRoute(vehicle)
        destination = routeid[-1]
       # logging.debug("source::::(%s)" % (source))
        # for edge in routeid[routeid.index(source):routeid.index(destination)]:
            # #if route.index(source)==edge:continue
            # dict_fc[edge] = dict_fc[edge] - 1
            # dict_fc[destination] = dict_fc[destination] - 1
	        
        for edge in route:
		  #if routeid.index(source)==edge:continue
		  dict_fc[edge] = dict_fc[edge] + 1
	
	
	    	
        
    else:
        for edge in route:	
            dict_fc[edge] = dict_fc[edge] + 1
	
    	
        
        # if choice_prob > highest_choice_prob:
		   
		    # highest_choice_prob = choice_prob
		    # route = path
		    # for edge in route:
			 # dict_fc[edge] = dict_fc[edge] + 1
		  
            
            
            
            # path_update[tuple(path)]= paths_and_weights[tuple(path)] + 0.1
            # list_path.append(path)
      # else:
       # path_weight = path_update[tuple(path)]
       # normalized_path_weight = path_weight / max_weight
       # boltzmann_prob = math.exp(-(normalized_path_weight / temperature)) / k
        
       # choice_prob = random.random() * boltzmann_prob
        
       # if choice_prob > highest_choice_prob:
            # highest_choice_prob = choice_prob
            # route = path
            # path_update[tuple(path)]= paths_and_weights[tuple(path)] + 0.1 	  
	    
    return route
                 
 

def reroute_vehicles_RSUs(subgraph_g,list_of_vehicle,edgelist,congestedRoadsss, congestedRoads,graph,buffered_paths, list_present_network):######################
 simple_paths = []
 listvehicle=[]
 listcongesvehicle=[]
 sys.stdout = open('edgee.txt','wt')
 # for vehicle in list_vehicle_set_route:
    # if vehicle in list_of_vehicle:
     # source = traci.vehicle.getRoadID(vehicle)
     # #if source.startswith(":"): continue
     # route = traci.vehicle.getRoute(vehicle)
     # #logging.debug("Route::::(%s)" % (route))
     # destination = route[-1]
     # logging.debug("source::::(%s)" % (source))
     # for edge in route[route.index(source):route.index(destination)]:
      # if route.index(source)==edge:continue
      # dict_fc[edge] = dict_fc[edge] + 1
      # dict_fc[destination] = dict_fc[destination] + 1
	  
         
 # for vv in list_of_vehicle:
  # for v in vv:
   # listvehicle.append(v)
 # for vc in vehiclecongested:
  # for v in vc:
   # listcongesvehicle.append(v)
 
 for vehicle in list_of_vehicle:
  aux=[] 
  route=[]
  del aux[:]
  del route[:]
  # if vehicle in listcongesvehicle: continue
  source = traci.vehicle.getRoadID(vehicle)
  if source.startswith(":"): continue
  route = traci.vehicle.getRoute(vehicle)
  logging.debug("Route::::(%s)" % (route))
  destination = route[-1]
  #logging.debug("Route(%s)" % (route))
  logging.debug("edgelist::::(%s)" % (edgelist))
  L=[]
  # for ege in edgelist:
   # for rou in route:
    # if ege==rou:
	  # L.append(route.index(rou))
  # LastEdgemax=max(L)
  # LastEdge = route[LastEdgemax]
  Route=route[route.index(source):route.index(destination)]
  logging.debug("RRRoute::::(%s)" % (Route))
  if destination in edgelist:
    for road in Route: #congestedRoads:##########################
     if len(aux) > 0:continue
     if road in congestedRoads:#route:
       
       if source != destination:
                logging.debug("Calculating shortest paths for pair (%s, %s)" % (source, destination))
		paths=[]
		simple_paths=[]
		k_paths_lengths, k_paths = k_shortest_paths(graph, source, destination, 7, "weight")
		if k_paths_lengths==source and k_paths==destination:
		 logging.debug("no reroute for pair in RSU (%s, %s,%s)" % (source, destination, vehicle))
		 auxr= route[0:route.index(destination)]
		 traci.vehicle.setRoute(vehicle,auxr)
		 logging.debug("set path no reroute pair in RSU (%s, %s,%s ,%s)" % (source, destination, vehicle,auxr))
		else:
		  for path in k_paths:
		   paths.append(path)
		   simple_paths.append(path)
		  paths_and_weights = get_paths_and_weights(graph, simple_paths)
		  weights = paths_and_weights.values()
		  max_weight = max(weights)
		  k = calculate_boltzmann_k_value(weights, max_weight)
		  logging.debug("Calculating choose_route for pair in RSU::::: (%s, %s, %s)" % (source, destination, vehicle))
		  new_route = choose_route(paths_and_weights, simple_paths, k, max_weight,vehicle, list_present_network, graph)
		  #new_route = nx.shortest_path(graph, source, destination,"weight")

		  # if LastEdge!=destination:
		  logging.debug("set route for pair in RSU:::: (%s, %s,%s)" % (source,destination, vehicle))
		  aux = route[0:route.index(source)]
		  aux += new_route
		  logging.debug(" aux for pair in RSU ::::::%s" % aux)
		  # logging.debug(" road  for pair in RSU %s" % road)
		  traci.vehicle.setRoute(vehicle, aux)
		  list_vehicle_set_route.append(vehicle)
		   
		  
  
  
  
  else:
    cloud_server(vehicle,route, source, destination,congestedRoadsss, graph, list_present_network)
    
  
		   
def cloud_server(vehicle,route, source, destination,congestedRoadsss, graph, list_present_network):
        # logging.debug("Calculating shortest paths for cloud_server pair (%s, %s)" % (source, destination))
 aux=[]
 Route=route[route.index(source):route.index(destination)]
 logging.debug("RRRoute in cloud request rsu ::::(%s)" % (Route))
 for road in Route:#congestedRoadsss:
  if len(aux) > 0:continue
  if road in congestedRoadsss:#route:
	if source != destination:
		paths=[]
		simple_paths=[]
		k_paths_lengths, k_paths = k_shortest_paths(graph, source, destination, 7, "weight")
		
		for path in k_paths:
		   paths.append(path)
		   simple_paths.append(path)
		paths_and_weights = get_paths_and_weights(graph, simple_paths)
		weights = paths_and_weights.values()
		max_weight = max(weights)
		k = calculate_boltzmann_k_value(weights, max_weight)
		logging.debug("Calculating choose_route for pair cloud_server that request RSU(%s, %s, %s)" % (source, destination, vehicle))
		new_route = choose_route(paths_and_weights, simple_paths, k, max_weight,vehicle, list_present_network,graph)
		#new_route = nx.shortest_path(graph, source, destination,"weight")

		aux = route[0:route.index(source)]
		aux += new_route
                logging.debug("path for pair cloud_server that request RSU(%s, %s, %s, %s)" % (source, destination, vehicle, aux))

		traci.vehicle.setRoute(vehicle, aux)
		list_vehicle_set_route.append(vehicle)
		
		
		
def cloud_reroute_vehicles(graph,list_of_vehiclecloud,congestedRoadsss, buffered_paths, list_present_network):###########################
    simple_paths = []
    listvehicle=[]
    listcongesvehicle=[]
    for vv in list_of_vehiclecloud:
     for v in vv:
      listvehicle.append(v)
    # for vehicle in list_vehicle_set_route:
     # if vehicle in listvehicle:
      # source = traci.vehicle.getRoadID(vehicle)
      # #if source.startswith(":"): continue
      # route = traci.vehicle.getRoute(vehicle)
      # #logging.debug("Route::::(%s)" % (route))
      # destination = route[-1]
      # logging.debug("source::::(%s)" % (source))
      # for edge in route[route.index(source):route.index(destination)]:
       # if route.index(source)==edge:continue
       # dict_fc[edge] = dict_fc[edge] + 1
       # dict_fc[destination] = dict_fc[destination] + 1
    # for vc in vehiclecongesteds:
     # for v in vc:
      # listcongesvehicle.append(v)
    
    for vehicle in listvehicle:
        aux=[]
	# if vehicle in listcongesvehicle: continue
        source = traci.vehicle.getRoadID(vehicle)
        if source.startswith(":"): continue
        route = traci.vehicle.getRoute(vehicle)
        destination = route[-1]
        Route=route[route.index(source):route.index(destination)]
        logging.debug("RRRoute in cloud ::::(%s)" % (Route))
        for road in route:#congestedRoadsss:
         if len(aux) > 0:continue
         if road in congestedRoadsss:#route:
          if source != destination:
		    # We have not calculated the paths for this given (source, destination) pair
            logging.debug("Calculating shortest paths for pair in cloud server (%s, %s ,%s)" % (source, destination, vehicle))
            paths = []
            simple_paths = []
            k_paths_lengths, k_paths = k_shortest_paths(graph, source, destination, 7, "weight")
            for path in k_paths:
			 paths.append(path)
			 simple_paths.append(path)
            paths_and_weights = get_paths_and_weights(graph, simple_paths)
            weights = paths_and_weights.values()
            max_weight = max(weights)
            k = calculate_boltzmann_k_value(weights, max_weight)
            new_route = choose_route(paths_and_weights, simple_paths, k, max_weight, vehicle, list_present_network, graph)
            #new_route = nx.dijkstra_path(graph, source, destination,"weight")

            aux = route[0:route.index(source)]
            aux += new_route
            traci.vehicle.setRoute(vehicle, aux)
            list_vehicle_set_route.append(vehicle)
			
             
		
def run(network, begin, end, interval):###############################
    logging.debug("Building road  complete graph")         
    road_graph_travel_time = build_road_graph(network)
    logging.debug("Building road subgraph")  
    # subg=subgraph_for_EachOfRsu(network,road_graph_travel_time)
    logging.debug("Distribution of RSUs in the map") 
    buffered_paths = {}	
    list_of_vehicle=[]
    subgraph_g={}
    dicttt_edgeRSUs={}
    f = open('RSUsLocation.xml', 'r')
    data = f.read()
    soup = BeautifulSoup(data)
    f.close()
    RSU_ids=[]
    RSU_x={}
    RSU_y={}
    edgelist={}
    closestEdge=[]
    edg_ids=[]
    listalledge=[]

    
    for RSU_tag in soup.findAll("poly"):
     
      RSU_id = RSU_tag["id"]
      RSU_ids.append(RSU_id)
      RSU_center = RSU_tag["center"]
      RSU_x[RSU_id], RSU_y[RSU_id] = RSU_center.split(",")
	
      x =float(RSU_x[RSU_id])
      y =float(RSU_y[RSU_id])
      logging.debug("Set of edges within the reach of each RSU")
      list_EdgeId=[]
      list_NormalEdges=[]
      list_junctions=[]
      net = sumolib.net.readNet(network)
      edges = net.getNeighboringEdges(x, y, 1000)
      for ege in edges:
	    closestEdge , dist = ege
	    #print ("{0} closestEdge= {1}" .format(RSU_id ,closestEdge ))
	    list_NormalEdges.append(str(closestEdge))
	    #print ("{0} list_NormalEdges= {1}" .format(RSU_id ,list_NormalEdges ))
	    
	    data = list_NormalEdges[0].split('id=')[1]
	    EdgeId = data.split(' ')[0]
	    
	    
		
           
	    listalledge.append(ast.literal_eval(EdgeId))
	    list_EdgeId.append(ast.literal_eval(EdgeId))
	    #########################################
	  # print("{0} list_EdgeId= {1}" .format(RSU_id ,list_EdgeId ))##########################
	  
          # JunctionFrom = data.split('from=')[1]
	  # JunctionFrom = JunctionFrom.split(' ')[0]
       	  # list_junctions.append(ast.literal_eval(JunctionFrom))
          # JunctionTo = data.split('to=')[-1]
          # JunctionTo = JunctionTo.split('/>')[0]
          # list_junctions.append(ast.literal_eval(JunctionTo))
	    del list_NormalEdges[:]# : means all memebers
      # print ("{0} list_junctions:::: {1}" .format(RSU_id ,list_junctions ))
	  # ############################################################################################
      # subgraph_g[RSU_tag]=road_graph_travel_time.subgraph(list_junctions)
	  # ############################################################################################
      # print ("{0} graph_list_junctions:::: {1}" .format(RSU_id ,subgraph_g[RSU_tag].nodes() ))
	  # ############################################################################################
      subgraph_g[RSU_id]=road_graph_travel_time.subgraph(list_EdgeId)## SUBGRAPH  SUBGRAPH   SUBGRAPH   SUBGRAPH  ##
	  ##############################################################################################
      # print ("{0} graph_list_EdgeIdedge:::: {1}" .format(RSU_id ,   subgraph_g[RSU_id].edges() ))
	  
      dicttt_edgeRSUs[RSU_id]=list_EdgeId
      
      # print ("{0} dict_edgeRSUs:::: {1}" .format(RSU_id ,dicttt_edgeRSUs[RSU_id] ))
      edgelist[RSU_id]= list_EdgeId
      logging.debug("edgelist[RSU_id]edgelist[%s]:%s " % (RSU_id,edgelist[RSU_id]))
      # print ("{0} edgelist:::: {1}" .format(RSU_id ,edgelist[RSU_id] ))

      # del list_EdgeId[:]
      # del list_junctions[:]
      del list_NormalEdges[:]
    logging.debug("Finding all simple paths")
    
    # Used to enhance performance only
    buffered_paths = {}
    
    logging.debug("Running simulation now")    
    step = 1
    # The time at which the first re-routing will happen
    rerouting_step = begin #800
    # The time at which a cycle for collecting travel time measurements begins
    travel_time_cycle_begin = interval  #600
    duration = 600
    periodic  = 1
    logging.debug("stepstepstepstepstepstep: %d" % step)
    #dict_edge_ave_speed
    zone_conges={}
    while step == 1 or traci.simulation.getMinExpectedNumber() > 0:
        logging.debug("Minimum expected number of vehicles: %d" % traci.simulation.getMinExpectedNumber())
        logging.debug("stepstepstepstepstepstep: %d" % step)
        traci.simulationStep()
        detect_vehicle_in_RSU_other=[]
        logging.debug("Simulation time %d" % step)  
        sys.stdout = open("edgee.txt","wt")
        
        if periodic >=600 and periodic <= end and periodic%duration==0:
		   for rsuid in RSU_ids:
		    if len(edgelist[rsuid]) > 0:
			 logging.debug("rsuid for zone_conges start %s" % rsuid)
			 logging.debug("rsuid edgelist[%s]  %s" % (rsuid,edgelist[rsuid]))
			 zone_conges[rsuid] = float(Road_RSU_congestion_index(edgelist, rsuid))
			 logging.debug("rsuid for zone_conges end %s" % zone_conges[rsuid])
			 
        if step >= travel_time_cycle_begin and travel_time_cycle_begin <= end and step%interval == 0:
            periodic  = 1
            logging.debug("Updating travel time on roads at simulation time %d" % step)
            list_present_network=[]
			################ Update Travel Time ##############################
            update_travel_time_on_roads(road_graph_travel_time, step, travel_time_cycle_begin == step)
            
            i=0
        #if rerouting_step == step and begin <= rerouting_step <= end:
            logging.debug("Re-routing step at simulation time %d" % step)
			########################################################################
            for edge in listalledge:
			 list_present_network.append(traci.edge.getLastStepVehicleIDs(edge))
			
            for vehicle in list_present_network:#list_vehicle_set_route:
               for v in vehicle :# if vehicle in list_present_network:
                    source = traci.vehicle.getRoadID(v)
                    dict_fc[source] = dict_fc[source] + 1
                    #if source.startswith(":"): continue
                    # route = traci.vehicle.getRoute(v)
                   # #logging.debug("Route::::(%s)" % (route))
                    # destination = route[-1]
                    # logging.debug("source::::(%s)" % (source))
                    # for edge in route[route.index(source):route.index(destination)]:
                        # if route.index(source)==edge:continue
                        # dict_fc[edge] = dict_fc[edge] + 1
                        # dict_fc[destination] = dict_fc[destination] + 1
			
			
			
			
			
			
			
	#########################################################################################################################################		
	   ########################################## select congested Roads for rerouting by cloud server####################################	   

	    congestedRoadsss=[]
	    for edge in edgeallgraph:
		 #for lanelist in dict_lane[edge]:
		    #for lane in lanelist:
			   listspeedss=[]
			   listvss=[]
			   Max_speed_LaneC = traci.lane.getMaxSpeed(edge + '_' + '0')# Get Maximum Allowed speed
			   listvss.append(traci.edge.getLastStepVehicleIDs(edge))
			   MeanSpeeds= traci.edge.getLastStepMeanSpeed(edge)
			   logging.debug("getLastStepMeanSpeed: %s" % MeanSpeeds)
			   sumspeedss=0
			   averagespeedss=0
			   jjj=0
			   wwwedge=0
			   listvehicless=[]
			   vehiclecongestedss=[]
			   # for vv in listvss:
				# for v in vv:
					# listvehicless.append(v)
					# #logging.debug("listvehicle: %s" % listvehicless)
					
			   # if len(listvehicless)!= 0: 
				# for vid in listvehicless: 
					 # logging.debug("vid in vehicle: %s" % vid)
					 # listspeedss.append(traci.vehicle.getSpeed(vid))
					 # sumspeedss += traci.vehicle.getSpeed(vid)
					 # logging.debug("traci.vehicle.getSpeed(vid): %s" % traci.vehicle.getSpeed(vid))
					 # jjj+=1
			   # if jjj>0 and listspeedss is not None :
					   # logging.debug("listspeedmax : %s" % listspeedss)
					   # maxspeedss = max(listspeedss) # V_max
					   # logging.debug("maxspeed: %s" % max_speed_Lane)
					   # logging.debug("j: %s" % jjj)
					   
					   # averagespeedss = sumspeedss / jjj# V_ave
					   # logging.debug("averagespeed: %s" % averagespeedss)
			   # if averagespeedss!=0:
			   Wedge_cloud = 1-(MeanSpeeds / Max_speed_LaneC)
			   logging.debug("Wedge_cloud: %s" % Wedge_cloud)
			   if Wedge_cloud >=0.5:
				 congestedRoadsss.append(edge)
		#############################################  select vehicle for rerouting by cloud server  #######################################	   
	    list_of_vehiclecloud=[]
	    for edge in edgeallgraph:
			 if edge in listalledge:continue
			 list_of_vehiclecloud.append(traci.edge.getLastStepVehicleIDs(edge))
			 
 	    if len(congestedRoadsss) != 0 :
			logging.debug("cloud_reroute_vehicles is staaaaaaaaaaaaaaaaaaaaaaaart: " )  
			cloud_reroute_vehicles(road_graph_travel_time,list_of_vehiclecloud,congestedRoadsss,buffered_paths, list_present_network)
			logging.debug("cloud_reroute_vehicles is eeeeeeeeeeeeeeeeeeeeeeeeeend: " )
	#########################################################################################################################################		
		#######################################################  Re-Routing for each RSU ################################################
            for rsuid in RSU_ids:
			 list_of_vehicle=[]
			 congestedRoads=[]
			 vehiclecongested=[]
			 list_present=[]
			 if len(edgelist[rsuid])== 0:continue
			 logging.debug("len(edgelist[rsuid]) is not none %s" % rsuid)
			 for edge in edgelist[rsuid]:
				   del visit_bfs[:]
				   logging.debug("edgelist (%s, %s)" % (rsuid, edge))
				  #print ("lane in edge{0} = {1}" .format(edgee ,dict_lane[edge] ))
				  #for lane in dict_lane[edge]:
                                  # logging.debug("Simulation for RSU%d -> dict_lane[%s] = lane %s" % (i , edge, lane))
                    
				   #listspeed=[]
				   MeanSpeed=0
				   number_vehicle=0
				   
				   
				   max_speed_Lane = traci.lane.getMaxSpeed(edge + '_' + '0')# Get Maximum Allowed speed
				   Lenght_Lane = traci.lane.getLength(edge + '_' + '0')
				   logging.debug("Lenght_Lane: %s" % Lenght_Lane)
				   LastStepLength = traci.edge.getLastStepLength(edge)
				   logging.debug("LastStepLength: %s" % LastStepLength)
				   listv=traci.edge.getLastStepVehicleIDs(edge) # Get the all vehicle in lane
				   number_vehicle = traci.edge.getLastStepVehicleNumber(edge)
				   logging.debug("number_vehicle: %s" % number_vehicle)
				   logging.debug("listvvvvvv: %s" % listv)
				   MeanSpeed= traci.edge.getLastStepMeanSpeed(edge)
				   logging.debug("getLastStepMeanSpeed: %s" % MeanSpeed)
				   # sumspeed=0
				   # averagespeed=0
				   # j=0
				   # wedge=0
				   # listvehicle=[]
				   # vehiclecongested=[]
				   
					 #logging.debug("listvehicle: %s" % listvehicle)
					
				   # if len(detect_vehicle_in_RSU_other)!= 0:
				      # for vv in listv:
				        # #for v in vv:
						  # if vv not in detect_vehicle_in_RSU_other:
						   # list_of_vehicle.append(vv)
						   # logging.debug("listvehicle: %s" % list_of_vehicle)
						   # detect_vehicle_in_RSU_other.append(vv)
						   # logging.debug("detect_vehicle_in_RSU_other: %s" % detect_vehicle_in_RSU_other)
					        
							
				   # else:
				      # for vv in listv:
				        # #for v in vv:
						  # list_of_vehicle.append(vv)
						  # logging.debug("listvehicle: %s" % list_of_vehicle)
						  # detect_vehicle_in_RSU_other.append(vv)
						  # logging.debug("detect_vehicle_in_RSU_other: %s" % detect_vehicle_in_RSU_other)
                     				  
				  
				    # for vid in listvehicle: 
					 # logging.debug("vid in vehicle: %s" % vid)
					 # listspeed.append(traci.vehicle.getSpeed(vid))
					 # sumspeed += traci.vehicle.getSpeed(vid)
					 # logging.debug("traci.vehicle.getSpeed(vid): %s" % traci.vehicle.getSpeed(vid))
					 # j+=1
				   # if j>0 and listspeed is not None :
				    # #logging.debug("listspeedmax : %s" % listspeed)
					# maxspeed = max(listspeed) # V_max
					# logging.debug("maxspeed: %s" % maxspeed)
					# logging.debug("j: %s" % j)
					   
					# averagespeed = sumspeed / j# V_ave
					# logging.debug("averagespeed: %s" % averagespeed)
				   
					# if averagespeed!=0:
				   Wedge = 1 - (MeanSpeed / max_speed_Lane)
				   logging.debug("Wedge: %s" % Wedge)
				   Denc=0
				   # sys.stdout = open('e.xml','wt')
				   for lane in dict_lane[edge]:
				    
				    number_vehicles = traci.lane.getLastStepVehicleNumber(lane)
				    if number_vehicles !=0 and Lenght_Lane >= 10:
				      Kjam = Lenght_Lane / (5+2.5) ####is obtained from    length of road / (avg vehicle length + min gap).
				      Density = number_vehicles / Kjam
				      if number_vehicles > Kjam:  
				       logging.debug("number_vehicles > Kjam: (%s , %s)" % (number_vehicles , Kjam) )
					 
					 

				    else:
				     Density=0
				    
				    
				    Denc += Density
				    logging.debug("DencDencDenc:  %s" % Denc )
				   ZoneC = zone_conges[rsuid]
				   logging.debug("zone_conges zone_conges zone_conges zone_conges zone_conges %s" % ZoneC)
                     
				    #wedge = 100 - ((averagespeed / max_speed_Lane)* 100
                     
                     
                     
				    #congested= wedge + dict_zone[rsuid]
				    #s= dict_zone[rsuid]
				    #logging.debug("dict_zone[rsuid] %d" % s)
					# if wedge >= 0.5 and  j > 3:
				   sum_conges= (Wedge+ZoneC+Denc)/3
				   logging.debug("sum_conges %s" % sum_conges)
				   if sum_conges >= 0.3:
				        print(list(bfs_edges(subgraph_g[rsuid], edge, reverse=False)))
				        congestedRoads.append(edge)
				        logging.debug("visit_bfs: %s" % visit_bfs)
				        for v in visit_bfs:
				            if v in list_present: continue
				            #if v in list_present: continue
				            list_present.append(v)
				            if v.startswith(":"): continue
				            vehiclecongested.append(traci.edge.getLastStepVehicleIDs(v))
				        
				   if len(detect_vehicle_in_RSU_other)!= 0:
				      for vv in vehiclecongested:
				        for v in vv:
						  if v not in detect_vehicle_in_RSU_other:
						   list_of_vehicle.append(v)
						   logging.debug("listvehicle: %s" % list_of_vehicle)
						   detect_vehicle_in_RSU_other.append(v)
						   logging.debug("detect_vehicle_in_RSU_other: %s" % detect_vehicle_in_RSU_other)
					        
							
				   else:
				      for vv in vehiclecongested:
				        for v in vv:
						  list_of_vehicle.append(v)
						  logging.debug("listvehicle: %s" % list_of_vehicle)
						  detect_vehicle_in_RSU_other.append(v)
						  logging.debug("detect_vehicle_in_RSU_other: %s" % detect_vehicle_in_RSU_other)
                     				  
				   
				         
			 if len(congestedRoads) > 0 :
			   logging.debug("Simulation for  %s" % rsuid)
			   logging.debug("Simulation time %d" % step)
			   logging.debug("congestedRoads %s" % congestedRoads)
			   logging.debug("list_of_vehicle %s" % list_of_vehicle)
			   logging.debug("detect_vehicle_in_RSU_other: %s" % detect_vehicle_in_RSU_other)
			   reroute_vehicles_RSUs(subgraph_g[rsuid],list_of_vehicle,edgelist[rsuid],congestedRoadsss,congestedRoads,road_graph_travel_time,buffered_paths, list_present_network)
			   #del list_present_network[:]
			  
		     
            rerouting_step += interval
            travel_time_cycle_begin = step + 1
        step += 1 
        periodic +=1
   
    time.sleep(10)
    logging.debug("Simulation finished")
    traci.close()
    sys.stdout.flush()
    time.sleep(10)
 
 
                
def start_simulation(sumo, scenario, network, begin, end, interval, output):
    logging.debug("Finding unused port")
    
    unused_port_lock = UnusedPortLock()
    unused_port_lock.__enter__()
    remote_port = find_unused_port()
    
    logging.debug("Port %d was found" % remote_port)
    
    logging.debug("Starting SUMO as a server")
    
    sumo = subprocess.Popen(["D:\\E\\d\\sumo-0.25.0\\bin\\sumo-gui.exe", "-c", "C:\\Users\\mahboobe\\Downloads\\london.sumo.cfg", "--tripinfo-output", output,"--device.emissions.probability", "1.0"  , "--remote-port", str(remote_port)], stdout=sys.stdout, stderr=sys.stderr)    
    unused_port_lock.release()
            
    try:     
        traci.init(remote_port)    
        run(network, begin, end, interval)
    except Exception:
        logging.exception("Something bad happened")
    finally:
        logging.exception("Terminating SUMO")  
        terminate_sumo(sumo)
        unused_port_lock.__exit__()
        
def main():
    # Option handling
    parser = OptionParser()
    parser.add_option("-c", "--command", dest="command", default="sumo", help="The command used to run SUMO [default: %default]", metavar="COMMAND")
    parser.add_option("-s", "--scenario", dest="scenario", default="london.sumo.cfg", help="A SUMO configuration file [default: %default]", metavar="FILE")
    parser.add_option("-n", "--network", dest="network", default="london.net.xml", help="A SUMO network definition file [default: %default]", metavar="FILE")    
    parser.add_option("-b", "--begin", dest="begin", type="int", default=800, action="store", help="The simulation time (s) at which the re-routing begins [default: %default]", metavar="BEGIN")
    parser.add_option("-e", "--end", dest="end", type="int", default=7200, action="store", help="The simulation time (s) at which the re-routing ends [default: %default]", metavar="END")
    parser.add_option("-i", "--interval", dest="interval", type="int", default=600, action="store", help="The interval (s) at which vehicles are re-routed [default: %default]", metavar="INTERVAL")
    parser.add_option("-o", "--output", dest="output", default="ReFOCUS.xml", help="The XML file at which the output must be written [default: %default]", metavar="FILE")

    parser.add_option("-l", "--logfile", dest="logfile", default=os.path.join(tempfile.gettempdir(), "sumo-ReFOCUS.log"), help="log messages to logfile [default: %default]", metavar="FILE")
    (options, args) = parser.parse_args()
    
    logging.basicConfig(filename=options.logfile, level=logging.DEBUG)
    logging.debug("Logging to %s" % options.logfile)
    
    if args:
        logging.warning("Superfluous command line arguments: \"%s\"" % " ".join(args))
        
    start_simulation(options.command, options.scenario, options.network, options.begin, options.end, options.interval, options.output)
    
if __name__ == "__main__":
    main()    
    
#"--device.hbefa.probability", "1.0",