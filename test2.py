import bisect

data = [1, 2, 3, 4, 5, 6, 7]

for d in data[::-1]:
    print(d)
    a = d
    data.remove(a)


q = 1
print(q)
q += 1
print(q)


a = [[0 for i in range(0)] for i in range(80)]
a[0].append(1)
a[1].append(2)
print(a)

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

qq = []
qq.append(edge(0, 1, 0, 1))
qq.append(edge(0, 2, 0, 1))
qq.append(edge(1, 2, 1, 1))
qq.append(edge(1, 2, 2, 1))
qq.append(edge(1, 2, 4, 1))
insort_right(qq, edge(0, 1, 5, 1))