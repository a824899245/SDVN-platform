#routing test
#before this run file_io

import Get_Move as Gm
import Init
import numpy as np
import Global_Par as Gp
import time as t
import jhmmtg as jh
import junction_init as ji
import big_junction_init as bji
import big_jhmmtg as bjh
import big_HRLB as bhr
import HRLB as hr
import tgeaa as tg
node_list = []
com_node_list = []

head = []
tail = []
with open('big_400.tcl', 'r') as f:
    for line in f:
        print(line)
        if line[2] == 'o':
            tail.append(line)
        else:
            head.append(line)

print('1')

with open('tiexi1.tcl', 'w') as f:
    for line in head:
        f.write(line)
    for line in tail:
        f.write(' ')
        f.write(line)
sim_time = 500  # int(input("sim_time:"))
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

bji.inti()
hr.grid_intiall()
# 节点初始化
node_list = (Init.init_node(node_position, controller))
effi = 0
delay = 0
std2 = 0
# 生成通信节点
# 输入循环次数
round = 10
velo = [[0 for i in range(2)] for i in range(len(node_list))]
acc = [[0 for i in range(2)] for i in range(len(node_list))]
last_node_id_position = [[0 for i in range(2)] for i in range(len(node_list))]
last_velo = [[0 for i in range(2)] for i in range(len(node_list))]
for i in range(round):

    start_time = t.time()

    # 以秒为间隔进行
    for time in range(0, sim_time):
        print('\nTime: %d' % time)
        # with open('history.txt', 'a') as f:
        #     a = ""
        #     a += str(time)
        #     a += '\n'
        #     f.write(a)
        # 处理位置矩阵
        current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]
        for value in current_move:
            for i in range(1, 4):
                node_position[int(value[0, 1]), i] = value[0, i+1]
        node_id_position = node_position[:, [1, 2,3]]

        # if time%50 == 0:
        #     np.savetxt('position600_' + str(time), node_id_position)
        #  np.savetxt('position1000_'+str(500),node_id_position)
        #  np.savetxt('position1000_'+str(750),node_id_position)
        #  np.savetxt('position1000_'+str(999),node_id_position)

        # if time != 0:
        #     for i in range(len(node_id_position)):
        #         velo[i][0] = node_id_position[i][0,0] - last_node_id_position[i][0,0]
        #         velo[i][1] = node_id_position[i][0,1] - last_node_id_position[i][0,1]
        #
        # print(1)
        # if time >= 1:
        #     for i in range(len(node_id_position)):
        #         acc[i][0] = velo[i][0] - last_velo[i][0]
        #         acc[i][1] = velo[i][1] - last_velo[i][1]
        #
        # for i in range(len(node_id_position)):
        #     last_velo[i][0] = velo[i][0]
        #     last_velo[i][1] = velo[i][1]
        #     last_node_id_position[i] = node_id_position[i]
        # print(node_id_position[44])
        # 所有节点更新位置，并发送hello至控制器
        for node in node_list:
            node.update_node_position(node_id_position)
            node.generate_hello(controller)
        jh.num_count()
        # 控制器更新网络全局情况
        controller.predict_position()
        controller.junction_matrix_construction(node_num)

        com_node_list.clear
        com_node_list.extend(Init.get_communication_node(node_num,node_list))
        cn = np.array(com_node_list)
        aaaaa = str(i)+'-'+str(time)
        np.savetxt(aaaaa, cn)
        for num in range(20):
            controller.calculate_path(com_node_list[num][0], com_node_list[num][1], node_list, node_num)

        # 所有通信节点生成数据包并发送请求至控制器
            node_list[com_node_list[num][0]].generate_request(com_node_list[num][1], controller, 1024, t.time())

        # 控制器处理路由请求
        print('\nrouting request')
        controller.resolve_request(node_list)

        # 所有节点处理错误路由修复请求
        print('\nerror request')
        controller.resolve_error(node_list)
        print('\nforward')

        # 所有节点开始转发分组
        for node in node_list:
            node.forward_pkt_to_nbr(node_list, controller)

        bjh.delete()
        hr.grid_delete(node_list)

    end_time = t.time()
    effi += end_time-start_time
    delay += Gp.sum
    Gp.sum = 0

    std2 += np.std(Gp.record, ddof=1)
    Gp.record.clear()
print('\ncalculation time:\n')
print(effi/round/sim_time)
print('\ndelay:\n')
print(delay/round)
print('\njitter:\n')
print(std2/round)
print('\ndelivery ratio:\n')
print((sim_time*round-Gp.fail_time)/sim_time/round)
