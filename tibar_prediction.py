import math
import re
import Global_Par as gp
adj = [[0 for i in range(500)] for i in range(500)]
history = [[[0 for i in range(500)] for i in range(500)] for i in range(210)]
reward = [[0 for i in range(500)] for i in range(500)]
history_part = [[0 for i in range(500)] for i in range(500)]
ave = [0 for i in range(500)]
edge_list = []
visited = [0 for i in range(500)]
maxtime = 6

class edge:
    def __init__(self, u, v, t, d):
        self.u = u  # hello请求列表
        self.v = v  # 路由请求列表
        self.t = t  # 错误请求列表
        self.d = d  # 邻接矩阵


def insort_right(a, x, lo=0, hi=None):
    """Insert item x in list a, and keep it sorted assuming a is sorted.

    If x is already in a, insert it to the right of the rightmost x.

    Optional args lo (default 0) and hi (default len(a)) bound the
    slice of a to be searched.
    """

    if lo < 0:
        raise ValueError('lo must be non-negative')
    if hi is None:
        hi = len(a)
    while lo < hi:
        mid = (lo+hi)//2
        if x.t < a[mid].t:
            hi = mid
        else:
            lo = mid+1
    a.insert(lo, x)


q = 0.5
w = 0.1
def cal_dis(i, j, node_list):
    return math.sqrt(pow(node_list[i].position[0] - node_list[j].position[0], 2) + pow(node_list[i].position[1] - node_list[j].position[1], 2))

def adjmartix_generate(node_list):
    for nodei in node_list:
        for nodej in node_list:
            d = cal_dis(nodei.node_id, nodej.node_id, node_list)
            if d < gp.com_dis:
                adj[nodei.node_id][nodej.node_id] = d


def history_information():
    with open('300history.txt', 'r') as f:
        for line in f:
            line_list = re.split(' ', line)
            if len(line_list) == 1:
                a = int(line_list[0])
            else:
                for i in range(len(line_list)-1):
                    if line_list[i+1] != '\n':
                        history[a][int(line_list[i])][int(line_list[i+1])] += 1


def reward_generate(time, node_num):
    for i in range(node_num):
        for j in range(node_num):
            for k in range(200):
                if history[k][i][j] == 1:
                    history_part[i][j] += (time-k)
    for i in range(node_num):
        for j in range(node_num):
            if adj[i][j] != 0:
                if history_part[i][j] == 0:
                    reward[i][j] = q / adj[i][j]
                else:
                    if history_part[i][j] != 0:
                        reward[i][j] = q / adj[i][j] + w / history_part[i][j]
            else:
                reward[i][j] = 0
    num = 0
    for i in range(node_num):
        for j in range(node_num):
            if reward[i][j] != 0:
                num += 1
                ave[i] += reward[i][j]
        ave[i] = ave[i]/num
        num = 0


def forward_prediction(s, d, t, node_num, s_t, e_t):
    if visited[s] == 1 and visited[d] == 1:
        return
    if s == d or t > maxtime:
        return
    for i in range(node_num):
        if reward[s][i] >= ave[s]:
            visited[i] = 1
            visited[d] = 1
            insort_right(edge_list, edge(s, i, s_t, adj[s][i]))
            forward_prediction(i, d, t+1, node_num, s_t+adj[s][i], e_t)


def backward_prediction(s, d, t, node_num, s_t, e_t):
    if visited[s] == 1 and visited[d] == 1:
        return
    if s == d or t > maxtime:
        return
    for i in range(node_num):
        if reward[i][d] >= ave[i]:
            for edgei in edge_list:
                if edgei.v == i:
                    e_t = edgei.t+edgei.d + adj[i][d]
            visited[s] = 1
            visited[i] = 1
            backward_prediction(s, i, t + 1, node_num, s_t, e_t - adj[i][d])
            insort_right(edge_list, edge(i, d, e_t-adj[i][d], adj[i][d]))


def delete():
    adj = [[0 for i in range(500)] for i in range(500)]
    reward = [[0 for i in range(500)] for i in range(500)]
    history_part = [[0 for i in range(500)] for i in range(500)]
    ave = [0 for i in range(500)]
    edge_list = []
    visited = [0 for i in range(500)]
