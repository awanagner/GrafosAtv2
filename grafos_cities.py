import math
import json
from math import radians, cos, sin, asin, sqrt


class Grafo:
    def Limits(self, src, dest, custo):
        if src != dest:
            self.adjMat[src][dest] = custo
            self.adjMat[dest][src] = custo
        else:
            a=0

    def __init__(self, vertice):
        self.v = vertice
        self.adjMat = [[0 for i in range(self.v)] for j in range(self.v)]

    def FindNeigh(self, src):
        lst = []
        for i in range(len(self.adjMat[src])):
            if self.adjMat[src][i] > 0:
                lst.append(i)
        return lst
    
    def Shortest_Dijkstra(self,src,dest):
        dct = {}
        for i in range(len(self.adjMat)):
            temp = {}
            x = self.FindNeigh(i)
            for j in x:
                temp[j] = self.adjMat[i][j]
            dct[i] = temp
        
        StartPoint = src
        FinalPoint = dest
        notvisitednode = dct
        inf = 9999999
        shortest_dist = {}
        pred = {}
        path = []


        for node in notvisitednode:
            shortest_dist[node] = inf
        shortest_dist[StartPoint] = 0

        while notvisitednode:
            min_Node = None
            for node in notvisitednode:
                if min_Node is None:
                    min_Node = node
                elif shortest_dist[node] < shortest_dist[min_Node]:
                    min_Node = node

            for filho, peso in dct[min_Node].items():
                if peso + shortest_dist[min_Node] < shortest_dist[filho]:
                    shortest_dist[filho] = peso + shortest_dist[min_Node]
                    pred[filho] = min_Node
            notvisitednode.pop(min_Node)
        no_atual = FinalPoint


        while no_atual != StartPoint:
            try:
                path.insert(0,no_atual)
                no_atual = pred[no_atual]
            except KeyError:
                print('NOT POSSIBLE')
                break
    
    
        path.insert(0,StartPoint)
      
      
        if shortest_dist[FinalPoint] != inf:
            print('SHORTEST DISTANCE:', str(shortest_dist[FinalPoint]))
            return path

#######
with open('cities.json') as json_cities:
    cidades = json.load(json_cities)




list_latitude = []
list_longgitude = []
list_names = []

for i in cidades: 
    list_latitude.append(i['latitude'])
    list_longgitude.append(i['longgitude'])
    list_names.append(i['city'])



def haversine(lat1, lat2, long1, long2):
    
    long1, lat1, long2, lat2 = map(radians, [long1, lat1, long2, lat2])

    dist_long = long2 - long1 
    dist_lat = lat2 - lat1 
    r = 6371 
    c = 2 * asin(sqrt(a))  
    a = (sin(dist_lat/2)**2 + cos(lat1)) * (cos(lat2) * sin(dist_long/2)**2)
    return c * r

g = Grafo(1000)

print('WHATS THE MAXIMUM DISTANCE BETWEEN THOSE 2 CITIES?')
in_dist = int(input())

for i in range(1000):
    for j in range(1000):
        dist = haversine(list_longgitude[i], list_latitude[i], list_longgitude[j], list_latitude[j])
        if(dist<in_dist):
            g.Limits(i, j, dist)

##########################

print('ORIGIN CITY')
origin = input()
while True:
    if origin in list_names:
        index1 = list_names.index(origin)
        break
    else:
        print('NOT FOUND, TRY AGAIN!')
        origin = input()

print('DESTINY CITY')
destiny = input()
while True:
    if destiny in list_names:
        index2 = list_names.index(destiny)
        break
    else:
        print('NOT FOUND, TRY AGAIN!')
        destiny = input()


path = g.Shortest_Dijkstra(index1, index2)


list_try = []
for i in range(len(path)):
    aux = path[i]
    name = list_names[aux]
    list_try.append(name)


print('SHORTEST PATH:', list_try)