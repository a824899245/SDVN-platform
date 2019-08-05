import Init

com_node_list = []
com_node_list.extend(Init.get_communication_node(496-1))

with open('comnode.txt', 'w') as f:
    for i in com_node_list:
        a = ""
        a += str(i[0])
        a += ' '
        a += str(i[1])
        a += '\n'
        f.write(a)