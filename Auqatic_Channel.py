import math

import numpy as np
import Global_Par as Gp
f = 4

s = 0.5
n0 = -50
nlos = 0.1
nnlos = 21
fc = 2400000000

ps = 1
a = 5.0188
b = 0.3511
u = 2
h = 100
power = 100
band = 10*1000*1000
snr_threshold = -20
def A2Gchannel (re,d,model):
    disdict = dict.fromkeys(d, 0)
    ratedict = dict.fromkeys(d, 0)
    for i in d:
        disdict[i] += 1
    dd = np.array(list(disdict.keys()))
    dd2 = np.array(list(disdict.keys()))
    af = 0.11*(f*f)/(1+f*f) + 44*(f*f)/(4100+(f*f))+0.000275*(f*f)+0.003
    for i in range(len(dd2)):
        dd2[i] = 2* 10*math.log10(dd2[i]/1000)
    af = af* dd/1000 +dd2
    af = pow(10,af/10)

    n = 50-18*math.log2(f)
    n = pow(10,n/10)
    # pllos = np.zeros(len(tt))
    # plnlos = np.zeros(len(tt))
    # plos = np.zeros(len(tt))
    # theta = np.zeros(len(tt))
    # ex = np.zeros(len(tt))
    # power = np.zeros(len(tt))
    snr = np.zeros(len(af))
    rate = np.zeros(len(af))

    # for i in range(len(tt)):
    #     pllos[i] = 20 * math.log10(tt[i]) + nlos
    #     plnlos[i] = 20 * math.log10(tt[i]) + nnlos
    #     if model == 1:
    #         if h >= dd[i]:
    #             plos[i] = 0
    #         else:
    #             theta[i] = math.asin(h / dd[i])
    #             plos[i] = 1 / (1 + a * math.exp(1 - b * (180 / math.pi * theta[i] - a)))
    #     else:
    #         plos[i] = 0
    #     ex[i] = plos[i] * pllos[i] + (1 - plos[i]) * plnlos[i]
    #     power[i] = 300 * pow(10, (-ex[i] / 10))
    # sex = power.sum() + pow(10, (-170) / 10)
    for i in range(len(af)):
        snr[i] = 300/(af[i]*n)
        snr_db = 10*math.log10(snr[i])
        if (snr_db) < snr_threshold:
            rate[i] = 0
        else:
            ratedict[dd[i]] = math.log2(1 + snr[i]) * band
            rate[i] = math.log2(1 + snr[i]) * band

    return rate

    # power = 300
    # np1 = pow(10,(-170)/10) + power * pow(10,(-95/10)) + power * pow(10,(-105/10)) + + power * pow(10,(-111/10))
    # print(np1)
    #
    # power = 300
    # power = power * pow(10,(-ex/10))
    # snr = power / np1
    # print(snr)
    # r = math.log2(1+snr)*67000
    # print(r)
    # print(1024*8/(r)*1000)
def A2Gchannel_1 (d,model,po):
    # disdict = dict.fromkeys(d, 0)
    # ratedict = dict.fromkeys(d, 0)
    # for i in d:
    #     disdict[i] += 1
    # dd = np.array(list(disdict.keys()))
    tt = 4 * math.pi * fc * d / 299792458
    pllos = np.zeros(len(tt))
    plnlos = np.zeros(len(tt))
    plos = np.zeros(len(tt))
    theta = np.zeros(len(tt))
    ex = np.zeros(len(tt))
    power = np.zeros(len(tt))
    snr = np.zeros(len(tt))
    sinr = np.zeros(len(tt))

    for i in range(len(tt)):
        pllos[i] = 20 * math.log10(tt[i]) + nlos
        plnlos[i] = 20 * math.log10(tt[i]) + nnlos
        if model == 1:
            if h >= dd[i]:
                plos[i] = 0
            else:
                theta[i] = math.asin(h / dd[i])
                plos[i] = 1 / (1 + a * math.exp(1 - b * (180 / math.pi * theta[i] - a)))
        else:
            plos[i] = 0
        ex[i] = plos[i] * pllos[i] + (1 - plos[i]) * plnlos[i]
        power[i] = po[i] * pow(10, (-ex[i] / 10))
    sex = power.sum() + pow(10, (-170) / 10)
    for i in range(len(tt)):
        snr[i] = power[i] / (sex - power[i])
        sinr[i] = 10*math.log10(snr[i])
        

    return sinr


def delay(d,model):
    d = np.sort(d)
    disdict = dict.fromkeys(d, 0)
    disdict1 = dict.fromkeys(d, 0)
    ddict = dict.fromkeys(d, 0)
    for i in d:
        disdict[i] += 1
        disdict1[i] += 1
    for i in disdict:
        disdict[i] = disdict[i]*1024*8
    dis = list(disdict.keys())
    re_size = list(disdict.values())
    sum_d = 0
    for i in range(len(re_size)):
        rate = (A2Gchannel(re_size, np.array(dis), model))
        if rate[0] == 0:
            Gp.gar_veh += 1
            return ddict
        else:
            part_d = re_size[i]/rate[0]
            sum_d += part_d
            for j in range(len(re_size)-i):
                re_size[j+i] = re_size[j+i] - rate[j] * part_d
            mindis = dis.pop(0)
            ddict[mindis] = sum_d*1000/disdict1[mindis]
    return ddict


def A2G_channel_1(c_v_co_association,c_v_ch_association,t_v_co_association,t_v_ch_association,c_v_power,t_v_power,node_position, control_position):
    vehicle_num = np.size(c_v_co_association,0)
    control_num = np.size(c_v_co_association,1)
    channel_num = np.size(c_v_ch_association,1)

    # node_position = []
    # control_position = np.array(controller.controller_position) 
    # for i in node_list:
    #     node_position.appedn(i.position)
    
    # node_position = np.array(node_position)


    dist = np.zeros((vehicle_num, control_num), dtype=np.float)
    for i in range(vehicle_num):
        for j in range(control_num):
            dist[i, j] = np.sqrt(np.sum((node_position[i, :] - control_position[j, :])**2, axis=0))
    
    association = [[] for i in range(channel_num)]
 
    for i in range(vehicle_num):
        for j in range(channel_num):
            if t_v_ch_association[i,j] == 1:
                for k in range(control_num):
                    if t_v_co_association[i,k] == 1:
                        kk = k
                        break
                association[j].append([i,kk,1])
                break

    for i in range(vehicle_num):
        for j in range(channel_num):
            if c_v_ch_association[i,j] == 1:
                for k in range(control_num):
                    if c_v_co_association[i,k] == 1:
                        kk = k
                        break
                association[j].append([i,kk,0])
                break

    c_sinr = [0 for i in range(vehicle_num)]
    t_sinr = [0 for i in range(vehicle_num)]

    for i in association:
        dis = []
        for j in i:
            dis.append(dist[j[0],j[1]])

        power = []
        for k in i:
            if k[2] == 0:
                power.append(c_v_power[k[0]])
            elif k[2] == 1:
                power.append(t_v_power[k[0]])

        sinr = A2Gchannel_1(np.array(dis),3,power)

        for k in range(len(sinr)):
            if i[k][2] == 0:
                c_sinr[i[k][0]] = sinr[k]
            elif i[k][2] == 1:
                t_sinr[i[k][0]] = sinr[k]

    return c_sinr,t_sinr






if __name__ == "__main__":
    c_a = np.array([[1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0],
                    [0,0,0,1],
                    [0,0,0,1]])

    t_a = np.array([[1,0,0,0],
                    [1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0],
                    [0,0,0,1]])    

    c_c = np.array([[1,0,0],
                    [0,1,0],
                    [0,0,1],
                    [1,0,0],
                    [0,1,0]])

    t_c = np.array([[1,0,0],
                    [1,0,0],
                    [0,1,0],
                    [0,0,1],
                    [0,0,1]])        

    c_po = np.array([100,200,200,300,300])

    t_po = np.array([200,100,300,300,300])

    n_p = np.array([[0,0,0],
                    [20,0,0],
                    [45,0,0],
                    [69,0,0],
                    [99,0,0]])


    c_p = np.array([[12,0,0],
                    [36,0,0],
                    [57,0,0],
                    [79,0,0]])

    c_sinr,t_sinr = A2G_channel_1(c_a,c_c,t_a,t_c,c_po,t_po,n_p,c_p)
    # delaydic = delay(np.array([100, 200, 300, 400, 500, 300, 300, 200]), 1)
    # print(delaydic)
    # aaa = np.array(list(delaydic.values()))
    # print(aaa.mean())

    # delaydic = delay(np.array([100, 200, 300, 400, 500, 300, 300, 200]), 1)
    # print(delaydic)
    # aaa = np.array(list(delaydic.values()))
    # print(aaa.mean())

# ddd = 0
# re = [8192,8192,8192,8192]
# rate,delay = (A2Gchannel(re,np.array([100,200,300,400]),1))
# print(rate)
# ddd += re[0]/rate[0]
# for i in range(4):
#     re[i] = re[i] - rate[i]*ddd
#
# rate,delay = (A2Gchannel(re,np.array([200,300,400]),1))
# print(rate)
#
# dddd = re[1]/rate[0]
# ddd += dddd
# for i in range(3):
#     re[i+1] = re[i+1] - rate[i]*dddd
# rate,delay = (A2Gchannel(re,np.array([300,400]),1))
# print(rate)
# dddd = re[2]/rate[0]
# ddd += dddd
# for i in range(2):
#     re[i+2] = re[i+2] - rate[i]*dddd
# rate,delay = (A2Gchannel(re,np.array([400]),1))
# print(rate)
# dddd = re[3]/rate[0]
# ddd += dddd
# for i in range(1):
#     re[i+3] = re[i+3] - rate[i]*dddd
# print("A2G delay are ")
# print(delay)
# print("Average delay is ")
# print(np.mean(delay))
#
# ddd = 0
# re = [8192,8192,8192,8192]
# rate,delay = (A2Gchannel(re,np.array([100,200,300,400]),1))
# print(rate)
# ddd += re[0]/rate[0]
# for i in range(4):
#     re[i] = re[i] - rate[i]*ddd
#
# rate,delay = (A2Gchannel(re,np.array([200,300,400]),1))
# print(rate)
#
# dddd = re[1]/rate[0]
# ddd += dddd
# for i in range(3):
#     re[i+1] = re[i+1] - rate[i]*dddd
# rate,delay = (A2Gchannel(re,np.array([300,400]),1))
# print(rate)
# dddd = re[2]/rate[0]
# ddd += dddd
# for i in range(2):
#     re[i+2] = re[i+2] - rate[i]*dddd
# rate,delay = (A2Gchannel(re,np.array([400]),1))
# print(rate)
# dddd = re[3]/rate[0]
# ddd += dddd
# for i in range(1):
#     re[i+3] = re[i+3] - rate[i]*dddd
#
# print("a2g delay")
# print(ddd*1000)
#
# ddd = 0
# re = [8192,8192,8192,8192]
# rate,delay = (A2Gchannel(re,np.array([100,200,300,400]),0))
# print(rate)
# ddd += re[0]/rate[0]
# for i in range(4):
#     re[i] = re[i] - rate[i]*ddd
#
# rate,delay = (A2Gchannel(re,np.array([200,300,400]),0))
# print(rate)
#
# dddd = re[1]/rate[0]
# ddd += dddd
# for i in range(3):
#     re[i+1] = re[i+1] - rate[i]*dddd
# rate,delay = (A2Gchannel(re,np.array([300,400]),0))
# print(rate)
# dddd = re[2]/rate[0]
# ddd += dddd
# for i in range(2):
#     re[i+2] = re[i+2] - rate[i]*dddd
# rate,delay = (A2Gchannel(re,np.array([400]),0))
# print(rate)
# dddd = re[3]/rate[0]
# ddd += dddd
# for i in range(1):
#     re[i+3] = re[i+3] - rate[i]*dddd
#
# print("Ground network")
# print(ddd*1000)