import sys
import math
import matplotlib.pyplot as plt
from pylab import *
from matplotlib.font_manager import FontProperties

simfreqs = [0, 30, 60, 120, 300, 600]

model = sys.argv[1]
c = int(sys.argv[2])
motion = sys.argv[3]
link = sys.argv[4]

pos = []
for freq in simfreqs:
    ar = []
    ctrl = c if freq > 0 else 3
    freqused = freq if ctrl != 3 else 30
    file = '/home/zhiqiy/Documents/SCA2020/accuracy-{4}/{3}/{0}/{1}/{2}.txt'.format(motion, ctrl, link, freqused, model)
    with open(file) as fp:
        for line in fp:
            spline = line.strip()
            if spline != '':
                splt = spline.split(',')
                ar.append([ float(splt[0]), float(splt[1]), float(splt[2]) ])
    pos.append(ar)

# relative to kinematic
err = [[], [], [], [], []]
for i in range(5):
    for t in range(len(pos[0])):
        x0 = pos[0][t][0]
        y0 = pos[0][t][1]
        z0 = pos[0][t][2]
        x = pos[i+1][t][0]
        y = pos[i+1][t][1]
        z = pos[i+1][t][2]
        xe = x - x0
        ye = y - y0
        ze = z - z0
        err[i].append(math.sqrt(xe**2+ye**2+ze**2))

time = []
for t in range(len(pos[0])):
    time.append(t / 30)


params = {
   'pgf.texsystem': 'pdflatex',
   'font.family': 'serif',
   'text.usetex': True,
   'pgf.rcfonts': False,
   'axes.labelsize': 12,
   'font.size': 8,
   'xtick.labelsize': 10,
   'ytick.labelsize': 10,
   'figure.figsize': [6, 4]
}
rcParams.update(params)

from matplotlib import rc
rc('font',**{'family':'serif','serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)

method = ''
if c == 0:
    method = 'MABA'
elif c == 1:
    method = 'sparse factorization'
elif c == 2:
    method = 'dense factorization'

axes(frameon=0)
grid(linestyle=':')
plot(time, err[0], color='#b33b3b', linewidth=1)
plot(time, err[1], color='#b3803b', linewidth=1)
plot(time, err[2], color='#7da537', linewidth=1)
plot(time, err[3], color='#2f4877', linewidth=1)
plot(time, err[4], color='#852c6d', linewidth=1)

title('Tracking error with respect to time step $t$', fontsize=12)
xlabel('Time (s)')
ylabel('Error distance')

if model == 'dog':
    ylim(0, 3.0)
    legendloc = 1
else:
    ylim(0, 3.5)
    legendloc = 4

legend = legend(["$t = 1/30$", "$t = 1/60$", "$t = 1/120$", "$t = 1/300$", "$t = 1/600$"], 
    loc=legendloc, fancybox=True, shadow=False);
frame = legend.get_frame()
frame.set_facecolor('0.9')
frame.set_edgecolor('0.9')

fig1 = plt.gcf()
plt.show()
plt.draw()
fig1.savefig('/home/zhiqiy/Documents/SCA2020/{0}-{1}-{2}.pgf'.format(model, link, method), dpi=100)
