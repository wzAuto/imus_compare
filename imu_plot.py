import sys
import matplotlib.pyplot as plt
import numpy as np
import struct

def read(path):
    file = open(path,"rb")
    data = file.read(8+4*6)
    result = []
    while data:
        t_xyz = struct.unpack('dffffff', data)
        result.append(t_xyz)
        data = file.read(8+4*6)
    return np.array(result)

def unbios_accum(data, index):
    mean = np.mean(data[:,index])
    data[:,index] = data[:,index] - mean
    diff_sec = data[1:, 0] - data[:-1, 0]
    result = [0]
    for sec, measurement in zip(diff_sec, data[1:,index]):
        result.append(result[-1] + measurement * sec)
    return np.array(result)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("you may input lsm6dsm bin and mpu6050 bin")
        exit(0)
    lsm_data = read(sys.argv[1])
    plt.plot(lsm_data[1:,0] - lsm_data[:-1, 0], '.')
    plt.show()

    # mpu_data = read(sys.argv[2])
    # print("diff_tick: ", lsm_data[1,0] - lsm_data[0,0])
    # print("diff_tick: ", mpu_data[1,0] - mpu_data[0,0])
    # lsm_accum = unbios_accum(lsm_data, 6)
    # mpu_accum = unbios_accum(mpu_data, 6)
    # plt.plot(lsm_accum, '.', label = "lsm6dsm")
    # plt.plot(mpu_accum, '.', label = "mpu6050")
    # plt.legend()
    # plt.show()