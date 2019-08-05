import random

r = [[-1 for i in range(9)] for i in range(9)]

de = 8
for i in range(9):
    if i + 3 >= 0 and i + 3 <= 8:
        r[i][i+3] = 0
    if i - 3 >= 0 and i - 3 <= 8:
        r[i][i-3] = 0
for i in range(3):
    r[i*3+0][i*3+1] = 0
    r[i*3+1][i*3+0] = 0

r[5][8] = 100
r[7][8] = 100
r[8][8] = 100
# r[4][1] = 80
# r[4][5] = 80
# r[8][5] = 80
# r[3][4] = 60
# r[7][8] = 60
# r[7][4] = 60
# r[6][3] = 40
# r[6][7] = 40

for i in range(9):
    print(r[i])

q = [[0 for i in range(9)] for i in range(9)]


def ql(re, qe, node, d, num):
    if node == d:
        return
    a = random.randint(0, num-1)
    if re[node][a] >= 0:
        max = 0
        for i in range(num):
            if qe[a][i] > max:
                max = qe[a][i]
        qe[node][a] = qe[node][a] + re[node][a] + 0.8 * max
        ql(re, qe, a, d, num)


# for i in range(4000):
#     ql(r, q, 6, de, 9)
#     for j in range(9):
#         print(q[j])
#     print('\n')
# print('\n')



