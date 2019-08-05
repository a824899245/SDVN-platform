import Get_Move as Gm
import Init
import numpy as np
import Global_Par as Gp
import time as t
import jhmmtg as jh
import junction_init as ji
# import big_junction_init as bji
import big_jhmmtg as bjh
# import big_HRLB as bhr
import HRLB as hr
import re
import tibar_prediction as tp
import tgeaa as tg
node_list = []
com_node_list = []

sim_time = 309  # int(input("sim_time:"))
# 位置文件读取
movement_matrix, init_position_matrix = Gm.get_position('tiexi1.tcl')
node_num = init_position_matrix.shape[0]
# 控制器初始化
controller = Init.init_controller(node_num)

# 位置数据处理
init_position_arranged = init_position_matrix[np.lexsort(init_position_matrix[:, ::-1].T)]

node_position = init_position_arranged[0]
# node_position = np.insert(node_position, 0, values=np.zeros(node_num), axis=1)
# node_position = np.column_stack((node_position, node_position[:, 2:4]))
# node_position = np.insert(node_position, 6, values=np.zeros(node_num), axis=1)

# ji.inti()

ji.inti()
hr.grid_intiall()
jh.delete()
# 节点初始化
node_list = (Init.init_node(node_position, controller))
effi = 0
delay = 0
std2 = 0
time = 0
sum = 0
correct = 0
time1 = 0
edge_num = 0

routing_record = [[] for i in range(309)]
point_record = [[] for i in range(309)]
with open('comnode.txt', 'r') as f:
    for line in f:
        line_list = re.split(' ', line)
        item_list = []
        item_list.append(int(line_list[0]))
        item_list.append(int(line_list[1]))
        com_node_list.append(item_list)
with open('dijkstra.txt', 'r') as f:
    for line in f:
        line_list = re.split(' ', line)
        if len(line_list) == 1:
            i = int(line_list[0])
        else:
            line_p = []
            line_l = []
            for k in line_list:
                if k != '\n':
                    line_p.append(int(k))
            point_record[i].append(line_p)
            for j in range(len(line_list)-1):
                if line_list[j+1] != '\n':
                    item_list = []
                    item_list.append(int(line_list[j]))
                    item_list.append(int(line_list[j+1]))
                    line_l.append(item_list)
            routing_record[i].append(line_l)

with open('HRLB.txt', 'r') as f:
    for line in f:
        line_list = re.split(' ', line)
        if len(line_list) == 1:
            i = int(line_list[0])
        else:
            line_p = []
            line_l = []
            for k in line_list:
                if k != '\n':
                    line_p.append(int(k))
            point_record[i].append(line_p)
            for j in range(len(line_list)-1):
                if line_list[j+1] != '\n':
                    item_list = []
                    item_list.append(int(line_list[j]))
                    item_list.append(int(line_list[j+1]))
                    line_l.append(item_list)
            routing_record[i].append(line_l)

with open('PRHMM.txt', 'r') as f:
    for line in f:
        line_list = re.split(' ', line)
        if len(line_list) == 1:
            i = int(line_list[0])
        else:
            line_p = []
            line_l = []
            for k in line_list:
                if k != '\n':
                    line_p.append(int(k))
            point_record[i].append(line_p)
            for j in range(len(line_list)-1):
                if line_list[j+1] != '\n':
                    item_list = []
                    item_list.append(int(line_list[j]))
                    item_list.append(int(line_list[j+1]))
                    line_l.append(item_list)
            routing_record[i].append(line_l)
# 生成通信节点
for i in range(1):

    start_time = t.time()

    # 以秒为间隔进行
    for time in range(200, sim_time):
        # with open('Dijkstra.txt', 'a') as f:
        #     a = ""
        #     a += str(time)
        #     a += '\n'
        #     f.write(a)
        print('\nTime: %d' % time)

        # 处理位置矩阵
        current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]
        for value in current_move:
            for i in range(1, 4):
                node_position[int(value[0, 1]), i] = value[0, i+1]
        node_id_position = node_position[:, [1, 2, 3]]
        # print(node_id_position[44])

        # 所有节点更新位置，并发送hello至控制器
        for node in node_list:
            node.update_node_position(node_id_position)
            node.generate_hello(controller)

        tp.adjmartix_generate(node_list)
        tp.history_information()
        tp.reward_generate(time, node_num)
        jh.num_count()
        # 控制器更新网络全局情况
        controller.predict_position()
        controller.junction_matrix_construction(node_num)
        #
        # for i in routing_record[time]:
        #     s = i[0][0]
        #     d = i[len(i)-1][1]
        #     reward = [[0 for i in range(80)] for i in range(80)]
        #     jh.junction_reward(reward, node_list[d].junction[0])
        #     h_s1, h_s2 = jh.hidden_seq_generate(reward, node_list[s].junction[0], node_list[d].junction[0])
        #     ji.e_arrival_time[s] = 0
        #     jh.hidden_to_obverse(s, d, node_list, h_s1)
        #     jh.hidden_to_obverse(s, d, node_list, h_s2)
        #     if len(ji.edge_list) != 0:
        #         for edge in i:
        #             sum += 1
        #             for edge1 in ji.edge_list:
        #                 if edge[0] == edge1.u and edge[1] == edge1.v:
        #                     correct += 1
        #
        # jh.delete()

        for i in point_record[time]:
            s = i[0]
            d = i[len(i)-1]
            tp.forward_prediction(s, d, 1, node_num, 0, 1000000000)
            tp.backward_prediction(s, d, 1, node_num, 0, 1000000000)
            point_list = []
            if len(tp.edge_list) != 0:
                edge_num += len(tp.edge_list)
                for edge in tp.edge_list:
                    if point_list.count(edge.u) == 0:
                        point_list.append(edge.u)
                    if point_list.count(edge.v) == 0:
                        point_list.append(edge.v)
            for a in i:
                for b in point_list:
                    if a == b:
                        correct += 1
            print(correct/len(i))
            sum += (correct/len(i))
            print(sum)
            time1 += 1.0
            print(time1)
            correct = 0
        tp.delete()
    print('\n')
    print(sum/time1)
    print(edge_num/time1)
