import numpy as np
import Global_Par as Gp
import time as t
import jhmmtg as jh
import junction_init as ji
import tgeaa as tg
import networkx as nx
import dij_test1 as dij
import math
import random
import re


def get_position(mobile_file_path):
    x_max = 0
    y_max = 0
    z_max = 0
    with open(mobile_file_path, 'r') as f:
        movement_list = []
        init_position_list = []
        item_list = []
        key = 0
        for line in f:
            line_list = re.split('[\s]', line)
            if line_list[0] != '':
                item_list.append(float(line_list[2]))
                item_list.append(float(line_list[3][8:-1]))
                if float(line_list[5]) > x_max:
                    x_max = float(line_list[5])
                if float(line_list[6]) > y_max:
                    y_max = float(line_list[6])
                if float(line_list[7][0:-1]) > z_max:
                    z_max = float(line_list[7][0:-1])
                item_list.append(float(line_list[5]))
                item_list.append(float(line_list[6]))
                item_list.append(float(line_list[7][0:-1]))
                movement_list.append(item_list)
                item_list = []
            else:
                key = key + 1
                # 将节点编号写入列表
                if key % 3 == 1:
                    item_list.append(int(line_list[1][7:-1]))
                # 将节点的位置(x,y)写入列表
                if key % 3 != 0:
                    item_list.append(float(line_list[4]))
                if key % 3 == 0:
                    item_list.append(float(line_list[4]))
                    init_position_list.append(item_list)
                    item_list = []
        print(x_max)
        print(y_max)
        print(z_max)
        movement_matrix = np.mat(movement_list)
        init_position_matrix = np.mat(init_position_list)
        return movement_matrix, init_position_matrix

movement_matrix, init_position_matrix = get_position('tiexi1.tcl')
node_num = init_position_matrix.shape[0]
veh_mm = [[[0 for i in range(80)] for i in range(80)]for i in range(node_num)]
# 控制器初始化

node_taj_x = [[] for i in range(node_num)]
node_taj_y = [[] for i in range(node_num)]
node_inter = [[] for i in range(node_num)]
node_seg = [[] for i in range(node_num)]

G = nx.DiGraph()
node_delay = [0.03 for i in range(292)]


for taj in movement_matrix:
    node_taj_x[int(taj[0, 1])].append(taj[0, 2])
    node_taj_y[int(taj[0, 1])].append(taj[0, 3])
    inter, seg = jh.junction_judge(taj[0, 2], taj[0, 3], taj[0, 1])
    node_inter[int(taj[0, 1])].append(inter)
    node_seg[int(taj[0, 1])].append(seg)

ji.veh_segement_martix = [[[] for i in range(4)] for i in range(80)]
# 位置数据处理
ji.inti()

for key, veh in enumerate(node_inter):
    for i in range(len(veh)-1):
        if node_inter[key][i] != node_inter[key][i+1]:
            veh_mm[key][node_inter[key][i]][node_inter[key][i+1]] += 1

for i in range(80):
    for j in range(80):
        if ji.adj_martix[i][j] != 0:
            G.add_edge(i, j, weight = ji.adj_martix[i][j])

path = []


def inti():
    node_delay = [0.03 for i in range(node_num)]
    path.clear()


def cal_delay(s,d,t,s_inter,d_inter):
    sum = 0
    a = 0
    dis = dij.Dijkstra(G, s_inter, d_inter)

    for i in range(len(dis)-1):
        a += veh_mm[s][dis[i]][dis[i+1]]
        sum += ji.adj_martix[dis[i+1]][dis[i]]
    v = math.sqrt(pow((node_taj_y[s][t] - node_taj_y[s][t-1]),2) + pow((node_taj_x[s][t] - node_taj_x[s][t-1]),2))
    a *= 0.9
    a -= 0.1*sum/v
    return a


def cal_dis(i, j, node_list):
    return pow(node_list[i].position[0] - node_list[j].position[0], 2) + pow(node_list[i].position[1] - node_list[j].position[1], 2)


def routing(s,d,node_list,t):
    path.append(s)
    if (len(path)>=100):
        path.clear()
        path.append(s)
        path.append(d)
        return
    if cal_dis(s, d, node_list) < pow(Gp.com_dis, 2):
        path.append(d)
        return
    max = -9999
    maxi = 0
    for node in node_list:
        if node.node_id != s and path.count(node.node_id)==0:
            if cal_dis(s, node.node_id,node_list) < pow(Gp.com_dis, 2):
                a = cal_delay(node.node_id, d, t, node.junction[0],node_list[d].junction[0])
                if a > max:
                    max = a
                    maxi = node.node_id
    routing(maxi, d, node_list, t)
    return