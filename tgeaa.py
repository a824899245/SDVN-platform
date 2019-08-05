import Global_Par as Gp


def earliest_arrival(edge_list, source, des, num):
    e_arrival_time = [Gp.MAX for i in range(num)]
    prev = [[0 for i in range(0)] for i in range(num)]
    e_arrival_time[source] = 0
    for edge in edge_list:
        if edge.t >= e_arrival_time[edge.u]:
            if edge.t + edge.d < e_arrival_time[edge.v]:
                prev[edge.v].clear
                prev[edge.v].append(edge.u)
                e_arrival_time[edge.v] = edge.t + edge.d
            else:
                if edge.t + edge.d == e_arrival_time[edge.v]:
                    prev[edge.v].append(edge.u)
    return e_arrival_time, prev


def s_routing(prev, source, des, routing):
    if des == source:
        routing.append(source)
        return 1
    if prev[des]:
        s_routing(prev, source, prev[des][0], routing)
        routing.append(des)
    else:
        return 0


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


class edge:
    def __init__(self, u, v, t, d):
        self.u = u  # hello请求列表
        self.v = v  # 路由请求列表
        self.t = t  # 错误请求列表
        self.d = d  # 邻接矩阵


# edge_list = []
# insort_right(edge_list, edge(0, 1, 0, 1))
# insort_right(edge_list, edge(0, 1, 2, 1))
# insort_right(edge_list, edge(0, 2, 4, 1))
# insort_right(edge_list, edge(0, 3, 9, 1))
# insort_right(edge_list, edge(0, 6, 10, 1))
# insort_right(edge_list, edge(1, 4, 3, 1))
# insort_right(edge_list, edge(1, 5, 3, 1))
# insort_right(edge_list, edge(2, 5, 7, 1))
# insort_right(edge_list, edge(3, 6, 10, 1))
# insort_right(edge_list, edge(4, 7, 2, 1))
# insort_right(edge_list, edge(4, 8, 6, 1))
# insort_right(edge_list, edge(5, 8, 8, 1))
# insort_right(edge_list, edge(5, 6, 9, 1))
# insort_right(edge_list, edge(6, 9, 9, 1))
# insort_right(edge_list, edge(6, 9, 8, 1))
#
# a, b = earliest_arrival(edge_list, 0, 9, 10)
# print(a)
# print(b)
#
# for i in range(1, 10):
#     routing = []
#     s_routing(b, 0, i, routing)
#     print(routing)
