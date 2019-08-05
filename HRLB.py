import junction_init as ji
import Global_Par as Gp
import math
x = []
y = []
junction_grid = []
grid_sub = [[] for i in range(48)]
grid_num = [[] for i in range(48)]
grid_trans = [[0 for i in range(48)] for i in range(48)]
grid_adj = [[0 for i in range(48)] for i in range(48)]
max_n = 1
max_t = 1
g_s = []
ss = []
dd = []
seg_seq = []
vehicle_seq = []
vehicle_s = []


def grid_intiall():
    for i in range(48):
        if i-1 >= 0:
            grid_adj[i][i-1] = 1
        if i+1 < 48:
            grid_adj[i][i+1] = 1
        if i - 6 >= 0:
            grid_adj[i][i - 6] = 1
        if i + 6 < 48:
            grid_adj[i][i + 6] = 1
        if i % 6 == 0 and i != 0:
            grid_adj[i][i-1] = 0
        if i % 6 == 5 and i != 47:
            grid_adj[i][i+1] = 0

    for key, i in enumerate(ji.junction_position):
        if i[0] != -1:
            x.append(i[0])
            y.append(i[1])
            junction_grid.append(int(i[0]/500)+int(i[1]/200)*6)
            grid_sub[int(i[0]/500)+int(i[1]/200)*6].append(key)

        else:
            junction_grid.append(-1)


def grid_delete(node_list):
    max_n = 1
    max_t = 1
    g_s.clear()
    ss.clear()
    dd.clear()
    seg_seq.clear()
    vehicle_seq.clear()
    vehicle_s.clear()
    # for node in node_list:
    #     if node.grid != -1 and node.grid != int(node.position[0] / 500) + int(node.position[1] / 500) * 13:
    #         grid_trans[node.grid][int(node.position[0] / 500) + int(node.position[1] / 500) * 13] += 1
    #     node.grid = int(node.position[0] / 500) + int(node.position[1] / 500) * 13
    #     grid_num[int(node.position[0] / 500) + int(node.position[1] / 500) * 13].append(node.node_id)
    for i in grid_num:
        if len(i) > max_n:
            max_n = len(i)

    for i in grid_trans:
        sum = 0
        for j in i:
            sum += j
        if sum > max_t:
            max_t = sum


def cal_os(grid_id):
    result = len(grid_num[grid_id])/max_n
    sum = 0
    for i in grid_trans[grid_id]:
        sum += i
    result += sum/max_t
    return result


def cal_seg(s_inter, d_inter, node_list):
    veh_list = []
    result = ji.junction_distance[s_inter][d_inter]
    if d_inter-s_inter == 1:
        result += (ji.num_segement_martix[s_inter][1]+ji.num_segement_martix[d_inter][3])
        for i in ji.veh_segement_martix[s_inter][1]:
            veh_list.append(i)
        for i in ji.veh_segement_martix[d_inter][3]:
            veh_list.append(i)
    else:
        if d_inter-s_inter == -1:
            result += (ji.num_segement_martix[s_inter][3] + ji.num_segement_martix[d_inter][1])
            for i in ji.veh_segement_martix[s_inter][3]:
                veh_list.append(i)
            for i in ji.veh_segement_martix[d_inter][1]:
                veh_list.append(i)
        else:
            if d_inter-s_inter == 6:
                result += (ji.num_segement_martix[s_inter][2] + ji.num_segement_martix[d_inter][0])
                for i in ji.veh_segement_martix[s_inter][2]:
                    veh_list.append(i)
                for i in ji.veh_segement_martix[d_inter][0]:
                    veh_list.append(i)
            else:
                if d_inter-s_inter == -6:
                    result += (ji.num_segement_martix[s_inter][0] + ji.num_segement_martix[d_inter][2])
                    for i in ji.veh_segement_martix[s_inter][0]:
                        veh_list.append(i)
                    for i in ji.veh_segement_martix[d_inter][2]:
                        veh_list.append(i)
    dis = 0
    for i in veh_list:
        for j in veh_list:
            if i != j:
                dis += math.sqrt(pow(node_list[i].position[0] - node_list[j].position[0], 2) + pow(node_list[i].position[1] - node_list[j].position[1], 2))
    dis = dis / 2
    if len(veh_list)*(len(veh_list)-1)/2!=0:
        result += dis/(len(veh_list)*(len(veh_list)-1)/2)
    return result


def grid_seq(s,x, d, g_s):
    if len(g_s)>100:
        g_s.clear()
        g_s.append(s)
        g_s.append(d)
        return
    g_s.append(x)
    if grid_adj[x][d] == 1:
        g_s.append(d)
        return
    maxn = 0
    maxi = 0
    for i in range(48):
        if grid_adj[x][i] == 1 and g_s.count(i) == 0:
            a = cal_os(i)
            if a > maxn:
                maxn = a
                maxi = i
    grid_seq(s, maxi, d, g_s)
    return


def seg_gen(g_s, node_list):
    for i in range(len(g_s)-1):
        maxx = 99999
        s = 0
        d = 0
        for s_inter in grid_sub[g_s[i]]:
            for d_inter in grid_sub[g_s[i+1]]:
                if ji.adj_martix[s_inter][d_inter] != 0:
                    a = cal_seg(s_inter, d_inter, node_list)
                    if a < maxx:
                        maxx = a
                        s = s_inter
                        d = d_inter
        ss.append(s)
        dd.append(d)
    for i in range(len(ss)):
        if i == 0:
            seg_seq.append(ss[i])
            seg_seq.append(dd[i])
        else:
            if ss[i] != seg_seq[len(seg_seq)-1]:
                seg_seq.append(ss[i])
            seg_seq.append(dd[i])


def poss_veh_gen():
    for i in range(len(seg_seq)-1):
        if seg_seq[i+1] - seg_seq[i] == 1:
            for j in ji.veh_segement_martix[seg_seq[i]][1]:
                vehicle_seq.append(j)
            for k in ji.veh_segement_martix[seg_seq[i+1]][3]:
                vehicle_seq.append(k)
        else:
            if seg_seq[i + 1] - seg_seq[i] == -1:
                for j in ji.veh_segement_martix[seg_seq[i]][3]:
                    vehicle_seq.append(j)
                for k in ji.veh_segement_martix[seg_seq[i + 1]][1]:
                    vehicle_seq.append(k)
            else:
                if seg_seq[i + 1] - seg_seq[i] == 10:
                    for j in ji.veh_segement_martix[seg_seq[i]][2]:
                        vehicle_seq.append(j)
                    for k in ji.veh_segement_martix[seg_seq[i + 1]][0]:
                        vehicle_seq.append(k)
                else:
                    if seg_seq[i + 1] - seg_seq[i] == -10:
                        for j in ji.veh_segement_martix[seg_seq[i]][0]:
                            vehicle_seq.append(j)
                        for k in ji.veh_segement_martix[seg_seq[i + 1]][2]:
                            vehicle_seq.append(k)


def cal_dis(i, j, node_list):
    return pow(node_list[i].position[0] - node_list[j].position[0], 2) + pow(node_list[i].position[1] - node_list[j].position[1], 2)


def vehicle_search(s, x, d, node_list, vehicle_s):
    if len(vehicle_s) > 100:
        vehicle_s.clear()
        vehicle_s.append(s)
        vehicle_s.append(d)
        return
    vehicle_s.append(x)
    if cal_dis(x, d, node_list) < pow(Gp.com_dis, 2):
        vehicle_s.append(d)
        return
    minx = 9999
    mini = 0
    for i in vehicle_seq:
        if cal_dis(x, i, node_list) < pow(Gp.com_dis, 2) and cal_dis(x, d, node_list) - cal_dis(i, d, node_list) > 0 and vehicle_s.count(i) == 0:
            if 1/cal_dis(i, d, node_list) < minx:
                minx = 1/cal_dis(i, d, node_list)
                mini = i
    vehicle_search(s, mini, d, node_list, vehicle_s)
    return


def routing(s, d, node_list):
    grid_seq(node_list[s].grid, node_list[s].grid, node_list[d].grid, g_s)
    seg_gen(g_s, node_list)
    poss_veh_gen()
    vehicle_search(s, s, d, node_list, vehicle_s)
    grid_delete(node_list)
    return (vehicle_s)
