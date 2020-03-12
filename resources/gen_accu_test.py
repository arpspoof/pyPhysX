import subprocess
import sys
import os

simfreqs = [30, 60, 120, 300, 600]

model = sys.argv[1]
motion = sys.argv[2]
ctrlFreq = int(sys.argv[3])
repeat = int(sys.argv[4])

for ctrl in range(4):
    for freq in simfreqs:
        print('running test for control', ctrl, 'freq', freq)

        dmpFolder = '/home/zhiqiy/Documents/SCA2020/accuracy-{0}/{1}/{2}/{3}/'.format(model, freq, motion, ctrl)
        os.system('mkdir -p {0}'.format(dmpFolder))
        
        cmd = '/home/zhiqiy/pyPhysX/build/poseTracking -m 3 -h 1 --dmpRepeat={6} -f {0} -t {1} -m {2}.txt -c {3} --dmpFolder={4} {5}'.format(
            1.0 / ctrlFreq, 1.0 / freq, motion, ctrl, dmpFolder, ' '.join(sys.argv[5:]), repeat
        )
        os.system(cmd)
