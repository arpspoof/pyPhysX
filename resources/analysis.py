import sys
import math
import matplotlib.pyplot as plt

f = int(sys.argv[1])
m = sys.argv[2]
j = sys.argv[3]

pos = []
for c in range(4):
    pos.append([])
    file = '/home/zhiqiy/Documents/SCA2020/accuracy/{3}/{0}/{1}/{2}.txt'.format(m, c, j, f)
    with open(file) as fp:
        for line in fp:
            spline = line.strip()
            if spline != '':
                splt = spline.split(',')
                pos[c].append([ float(splt[0]), float(splt[1]), float(splt[2]) ])

err = [[], [], []]
for c in range(3):
    for t in range(len(pos[0])):
        x0 = pos[3][t][0]
        y0 = pos[3][t][1]
        z0 = pos[3][t][2]
        x = pos[c][t][0]
        y = pos[c][t][1]
        z = pos[c][t][2]
        xe = x - x0
        ye = y - y0
        ze = z - z0
        err[c].append(math.sqrt(xe**2+ye**2+ze**2))

print(err)

plt.plot(err[0], 'r')
plt.plot(err[1], 'g')
plt.plot(err[2], 'b')
plt.show()
