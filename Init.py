# 初始化，创建节点 控制器 通信节点类


import Node as Nd
import SDVN_Controller as Sc
import random
import Global_Par as Gp
import networkx as nx
import math

# 初始化,创建节点对象,并将各个节点的初始位置和节点编号赋值
def init_node(node_id_position, controller):
    node_list = []
    for i in range(node_id_position.shape[0]):
        node_list.append(Nd.Node(int(node_id_position[i][0, 0]), controller))
    return node_list

# 初始化控制器对象
def init_controller(node_num):
    G = nx.DiGraph()
    return Sc.SDVNController(G, node_num)

def init_controller_list(node_num, controller_num, position_list):
    G = nx.Digraph()
    controller_list = []
    for i in range(controller_num):
        a = Sc.SDVNController(G, node_num)
        a.position = position_list[i]
        controller_list.append(a)
    return controller_list

# 获取通信的节点,node_num为节点数量,com_node_rate为通信节点的比例
def get_communication_node(node_num, node_list):
    n = 40

    com_nodelist = []
    com_nodelist_pair = []
    for i in range(node_num):
        if len(com_nodelist) < n:
            node_id = round(random.random() * node_num)-1
            # if node_id not in com_nodelist:
            if node_id not in com_nodelist:
                node_id_1 = round(random.random() * node_num)-1
                if node_id_1 not in com_nodelist:
                    dis = math.sqrt(pow(node_list[node_id].position[0] - node_list[node_id_1].position[0], 2) + pow(
                        node_list[node_id].position[1] - node_list[node_id_1].position[1], 2))
                    if 3500 < dis < 4000:
                        com_nodelist.append(node_id)
                        com_nodelist.append(node_id_1)
                        com_nodelist_pair.append([node_id,node_id_1])
        else:
            break
    return com_nodelist_pair


# 获取通信的节点,node_num为节点数量,com_node_rate为通信节点的比例
def geo_get_communication_node(node_num, num):
    n = round((node_num * Gp.com_node_rate)/2) * 2
    com_nodelist = []
    for i in range(node_num):
        if len(com_nodelist) < n:
            node_id = round(random.random() * node_num)
            # if node_id not in com_nodelist:
            if node_id not in com_nodelist and (node_id < 1 or node_id > 3):
                com_nodelist.append(node_id)
        else:
            break
    des = [[] for i in range(node_num)]
    for i in range(node_num):
        while True:
            if len(des[i]) != num:
                node_id = round(random.random() * node_num)
                if node_id == i:
                    continue
                if node_id not in des[i] and (node_id < 1 or node_id > 3):
                    des[i].append(node_id)
                    continue
            else:
                break
    return com_nodelist, des


