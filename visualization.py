import numpy as np
import re
import matplotlib.pyplot as plt


# 获取车辆节点的数量和仿真时间
def get_simparamter(configfile_path):
    with open(configfile_path, 'r') as f:
        for line in f:
            if line.find('set opt(nn)') >= 0:
                linelist = re.split('[\s]', line)
                nnode = int(float(linelist[2]))
            if line.find('set opt(stop)') >= 0:
                linelist = re.split('[\s]', line)
                simtime = int(float(linelist[2]))
    return nnode, simtime


# 获取车量节点的初始位置以及运动轨迹的变化情况
def get_position(mobilefile_path):
    with open(mobilefile_path, 'r') as f:
        movement_list = []
        initposition_list = []
        itemlist = []
        key = 0
        for line in f:
            key = key + 1
            linelist = re.split('[\s]', line)
            if linelist[0] != '':
                itemlist.append(float(linelist[2]))
                itemlist.append(float(linelist[3][8:-1]))
                itemlist.append(float(linelist[5]))
                itemlist.append(float(linelist[6]))
                itemlist.append(float(linelist[7][0:-1]))
                movement_list.append(itemlist)
                itemlist = []
            else:
                if key % 3 == 0:
                    itemlist.append(int(linelist[1][7:-1]))
                    itemlist.append(float(linelist[4]))
                if key % 3 == 1:
                    itemlist.append(float(linelist[4]))
                if key % 3 == 2:
                    initposition_list.append(itemlist)
                    itemlist = []
        movement_matrix = np.mat(movement_list)
        initposition_matrix = np.mat(initposition_list)
        return movement_matrix, initposition_matrix


def update_node_position(movement_matrix, nodeposition, start_t, update_period, animation, nodelist, com_nodelist):
    # nodeposition节点位置的变化情况，共有6个元素，包括时间，节点编号，当前x坐标，当前y坐标，目的位置x坐标，目的位置y坐标,节点移动速度
    print('开始时间:', start_t)
    activeroute = []
    currentmove = movement_matrix[np.nonzero(movement_matrix[:, 0].A == start_t)[0], :]
    for value in currentmove:
        for i in range(2, 4):
            nodeposition[int(value[0, 1]), i + 2] = value[0, i]
    speed_x = nodeposition[:, 4] - nodeposition[:, 2]
    speed_y = nodeposition[:, 5] - nodeposition[:, 3]
    for i in range(0, int(1.0 / update_period)):
        nodeposition[:, 2] = nodeposition[:, 2] + speed_x * update_period
        nodeposition[:, 3] = nodeposition[:, 3] + speed_y * update_period
        Nodeid_position = nodeposition[:, [1, 2, 3]]
        print(Nodeid_position)
#        if nodelist == [] or com_nodelist == []:
#            nodelist.extend(aodv.init_node(Nodeid_position))
#            com_nodelist.extend(aodv.get_communication_node(Nodeid_position.shape[0], aodv.comnode_rate))
 #           print('所有通信节点:', com_nodelist)
  #      activeroute = aodv.simulation(Nodeid_position, nodelist, com_nodelist, i, activeroute)
        print(activeroute)
        if animation == True:
            plt.clf()
            plt.plot(nodeposition[:, 2], nodeposition[:, 3], '.m')
            plt.pause(0.01)


def controll_movement(animation):
    np.set_printoptions(suppress=True)
    configfile_path = r'grid.config.tcl'
    mobilefile_path = r'tiexi1.tcl'
    nnode, simtime = get_simparamter(configfile_path)
    movement_matrix, initposition_matrix = get_position(mobilefile_path)
    initposition_arranged = initposition_matrix[np.lexsort(initposition_matrix[:, ::-1].T)]
    global nodeposition
    nodeposition = initposition_arranged[0]
    nodeposition = np.insert(nodeposition, 0, values=np.zeros(nnode), axis=1)
    nodeposition = np.column_stack((nodeposition, nodeposition[:, 2:4]))
    nodeposition = np.insert(nodeposition, 6, values=np.zeros(nnode), axis=1)
    plt.ion()
    nodelist = []
    com_nodelist = []
    for i in range(0, simtime + 1):
        update_node_position(movement_matrix, nodeposition, i, 1, animation, nodelist, com_nodelist)


controll_movement(1)





















