import subprocess
import sys

for ctrl in range(3):
    print('running test for control', ctrl)

    repeat = 20

    total = 0
    total_aba = 0
    total_spd = 0

    for i in range(repeat):
        result = subprocess.run(sys.argv[1:] + ['-c', str(ctrl)], stdout=subprocess.PIPE).stdout.decode('utf-8')
        time = int(result.strip().split()[-3])
        aba = int(result.strip().split()[-2])
        spd = int(result.strip().split()[-1])
        total += time
        total_aba += aba
        total_spd += spd

    avgFPS = repeat * 10000000000 / total
    spdFPS = repeat * 10000000000 / total_spd
    abaPlusSpdFPS = repeat * 10000000000 / (total_aba + total_spd)
    totalAbaPlusSpdFPS = repeat * 10000000000 / (total_aba + total)
    totalMinusAbaFPS = repeat * 10000000000 / (-total_aba + total)

    if ctrl == 0:
        componentFPS = abaPlusSpdFPS
        aloneFPS = totalAbaPlusSpdFPS
        embedFPS = avgFPS

        embedpercent = total_spd / total
        alonepercent = (total_spd + total_aba) / (total + total_aba)
    else:
        componentFPS = spdFPS
        aloneFPS = avgFPS
        embedFPS = totalMinusAbaFPS

        embedpercent = total_spd / (total - total_aba)
        alonepercent = total_spd / total

    print('method:', ctrl, 'component:', componentFPS, 'alone:', aloneFPS, alonepercent, 'embed:', embedFPS, embedpercent)
