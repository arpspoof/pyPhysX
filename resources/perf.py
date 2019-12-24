import subprocess
import sys

repeat = 20

total = 0
total_aba = 0
total_spd = 0

for i in range(repeat):
    result = subprocess.run(sys.argv[1:], stdout=subprocess.PIPE).stdout.decode('utf-8')
    time = int(result.strip().split()[-3])
    aba = int(result.strip().split()[-2])
    spd = int(result.strip().split()[-1])
    total += time
    total_aba += aba
    total_spd += spd
    print("run", i, "time is", time, "; aba is", aba, "; spd is", spd)

print("average FPS =", repeat * 10000000000 / total)
print("spd FPS =", repeat * 10000000000 / total_spd)
print("aba + spd FPS =", repeat * 10000000000 / (total_aba + total_spd))
print("total aba + spd FPS =", repeat * 10000000000 / (total_aba + total))
