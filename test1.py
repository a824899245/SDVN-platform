import Get_Move as Gm
import Init
import numpy as np
import Global_Par as Gp
node_list = []
com_node_list = []

node_num, sim_time = Gm.get_sim_parameter('grid.config.tcl')
movement_matrix, init_position_matrix = Gm.get_position('grid.mobility.tcl')
controller = Init.init_controller(0, node_num)
init_position_arranged = init_position_matrix[np.lexsort(init_position_matrix[:, ::-1].T)]

node_position = init_position_arranged[0]
node_position = np.insert(node_position, 0, values=np.zeros(node_num), axis=1)
node_position = np.column_stack((node_position, node_position[:, 2:4]))
node_position = np.insert(node_position, 6, values=np.zeros(node_num), axis=1)
node_list = (Init.init_node(node_position, controller))
com_node_list.extend(Init.get_communication_node(node_num))

# for time in range(sim_time):
time = 0
print('Time: %d' % time)
current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]
for value in current_move:
    for i in range(2, 4):
        node_position[int(value[0, 1]), i] = value[0, i]
        node_id_position = node_position[:, [1, 2, 3]]
    # print(node_id_position[44])
for node in node_list:
    node.update_node_position(node_id_position)
    node.generate_hello(controller)
controller.predict_position()
for com_node in com_node_list:
    node_list[com_node[0]].generate_request(com_node[1], controller)
controller.resolve_request(node_list)
for node in com_node_list:
    node_list[node[0]].generate_pkt(node[1], 1024)
for node in com_node_list:
    node_list[node[0]].forward_pkt_to_nbr(node_list)
print("hello")
time = 1
print('Time: %d' % time)
current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]
for value in current_move:
    for i in range(2, 4):
        node_position[int(value[0, 1]), i] = value[0, i]
        node_id_position = node_position[:, [1, 2, 3]]
    # print(node_id_position[44])
for node in node_list:
    node.update_node_position(node_id_position)
    node.generate_hello(controller)
controller.predict_position()
for com_node in com_node_list:
    node_list[com_node[0]].generate_request(com_node[1], controller)
controller.resolve_request(node_list)
for node in com_node_list:
    node_list[node[0]].generate_pkt(node[1], 1024)
for node in com_node_list:
    node_list[node[0]].forward_pkt_to_nbr(node_list)

time = 2
print('Time: %d' % time)
current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]
for value in current_move:
    for i in range(2, 4):
        node_position[int(value[0, 1]), i] = value[0, i]
        node_id_position = node_position[:, [1, 2, 3]]
    # print(node_id_position[44])
for node in node_list:
    node.update_node_position(node_id_position)
    node.generate_hello(controller)
controller.predict_position()
for com_node in com_node_list:
    node_list[com_node[0]].generate_request(com_node[1], controller)
controller.resolve_request(node_list)
for node in com_node_list:
    node_list[node[0]].generate_pkt(node[1], 1024)
for node in com_node_list:
    node_list[node[0]].forward_pkt_to_nbr(node_list)

time = 3
print('Time: %d' % time)
current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]
for value in current_move:
    for i in range(2, 4):
        node_position[int(value[0, 1]), i] = value[0, i]
        node_id_position = node_position[:, [1, 2, 3]]
    # print(node_id_position[44])
for node in node_list:
    node.update_node_position(node_id_position)
    node.generate_hello(controller)
controller.predict_position()
for com_node in com_node_list:
    node_list[com_node[0]].generate_request(com_node[1], controller)
controller.resolve_request(node_list)
for node in com_node_list:
    node_list[node[0]].generate_pkt(node[1], 1024)
for node in com_node_list:
    node_list[node[0]].forward_pkt_to_nbr(node_list)

time = 4
print('Time: %d' % time)
current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]
for value in current_move:
    for i in range(2, 4):
        node_position[int(value[0, 1]), i] = value[0, i]
        node_id_position = node_position[:, [1, 2, 3]]
    # print(node_id_position[44])
for node in node_list:
    node.update_node_position(node_id_position)
    node.generate_hello(controller)
controller.predict_position()
for com_node in com_node_list:
    node_list[com_node[0]].generate_request(com_node[1], controller)
controller.resolve_request(node_list)
for node in com_node_list:
    node_list[node[0]].generate_pkt(node[1], 1024)
for node in com_node_list:
    node_list[node[0]].forward_pkt_to_nbr(node_list)

time = 5
print('Time: %d' % time)
current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]
for value in current_move:
    for i in range(2, 4):
        node_position[int(value[0, 1]), i] = value[0, i]
        node_id_position = node_position[:, [1, 2, 3]]
    # print(node_id_position[44])
for node in node_list:
    node.update_node_position(node_id_position)
    node.generate_hello(controller)
controller.predict_position()
for com_node in com_node_list:
    node_list[com_node[0]].generate_request(com_node[1], controller)
controller.resolve_request(node_list)
for node in com_node_list:
    node_list[node[0]].generate_pkt(node[1], 1024)
for node in com_node_list:
    node_list[node[0]].forward_pkt_to_nbr(node_list)



