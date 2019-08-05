import networkx as nx
import Global_Par as Gp


def bellman_ford(g, start, end, node_num):
    distance = []
    for i in range(node_num):
        distance.append(Gp.MAX)
    distance[start] = 0
    pre = []
    for i in range(node_num):
        pre.append(0)
    for i in range(node_num-1):
        for edge in g.edges:
            if distance[edge[0]] != Gp.MAX:
                if distance[edge[0]] + (g[edge[0]][edge[1]]['weight']) < distance[edge[1]]:
                    distance[edge[1]] = distance[edge[0]]+g[edge[0]][edge[1]]['weight']
                    pre[edge[1]] = edge[0]
    if distance[end] == Gp.MAX:
        return None
    path = [end]
    node = end
    while node != start:
        node = pre[node]
        path.append(node)
    path.reverse()
    return path

