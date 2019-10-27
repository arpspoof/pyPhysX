import matplotlib.pyplot as plt

target = []
with open('/home/zhiqiy/target.txt') as f:
    line = f.readline()
    while line:
        s = line.strip().split(' ')
        nums = map(lambda x: float(x), s)
        target.append(list(nums))
        line = f.readline()

sparse = []
with open('/home/zhiqiy/sparse.txt') as f:
    line = f.readline()
    while line:
        s = line.strip().split(' ')
        nums = map(lambda x: float(x), s)
        sparse.append(list(nums))
        line = f.readline()

aba = []
with open('/home/zhiqiy/aba.txt') as f:
    line = f.readline()
    while line:
        s = line.strip().split(' ')
        nums = map(lambda x: float(x), s)
        aba.append(list(nums))
        line = f.readline()

for i in range(3, 43):
    print("dimension ", i)
    tcol = list(map(lambda r: r[i], target))
    acols = list(map(lambda r: r[i], sparse))
    acola = list(map(lambda r: r[i], aba))
    plt.plot(tcol, 'g')
    plt.plot(acols, 'b')
    plt.plot(acola, 'r')
    plt.show()
    