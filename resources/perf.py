import subprocess
import sys

repeat = 20

total = 0
for i in range(repeat):
    result = subprocess.run(sys.argv[1:], stdout=subprocess.PIPE).stdout.decode('utf-8')
    time = int(result.strip().split()[-1])
    total += time
    print("run", i, "time is", time)

print("average FPS =", repeat * 10000000000 / total)
