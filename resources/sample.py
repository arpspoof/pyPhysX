from pyPhysX import *

foundation = Foundation()
scene = foundation.CreateScene(SceneDescription(), 0.016)

material = scene.CreateMaterial(1.0, 1.0, 0.0)

plane = scene.CreatePlane(material, vec3(0, 1, 0), 0)
plane.SetupCollisionFiltering(1, 2 | 4)

articulation = scene.CreateArticulation("../resources/humanoid.urdf", material, vec3(0, 3.55 + 0.2, 1.5))

p = 500
d = 100

articulation.SetKPs([p]*28)
articulation.SetKDs([d]*28)

articulation.SetFixBaseFlag(True)

articulation2 = scene.CreateArticulation("../resources/humanoid.urdf", material, vec3(0, 3.55 + 0.2, -1.5))

articulation2.SetKPs([p]*28)
articulation2.SetKDs([d]*28)

articulation2.SetFixBaseFlag(True)

targetPositions = [ 0.998819, 0.010960, -0.047140, -0.004159, 0.999996, 0.002537, 0.000256, 0.001070, 0.949948, 0.020403, -0.059407, 0.306028, -0.195258, 0.999520, 0.016056, 0.020116, 0.017256, 0.985617, -0.063945, 0.093094, -0.125710, 0.171284, 0.986347, -0.017107, 0.091650, -0.135749, -0.453371, 0.975329, 0.126891, -0.033021, 0.177601, 0.965989, 0.188903, -0.141940, 0.105041, 0.579958 ]

'''
q = [1, 0, 0, 0]
targetPositions = q*6 + [0]*4 + q*2
'''


import time

pos1 = []
pos2 = []
v1 = []
v2 = []

start_t = time.time()
for _ in range(100):
    articulation.AddSPDForces(targetPositions, scene.timeStep)
    articulation2.AddSPDForcesABA(targetPositions, scene.timeStep)
    pos1.append(articulation.GetJointPositionsQuaternion())
    v1.append(articulation.GetJointVelocitiesPack4())
    pos2.append(articulation2.GetJointPositionsQuaternion())
    v2.append(articulation2.GetJointVelocitiesPack4())
    scene.Step()
end_t = time.time()

print("total time is ", int((end_t - start_t) * 1000000.0))


import matplotlib.pyplot as plt

print("positions")
for j in range(7, len(pos1[0])):
    print("dimension ", j)
    l1 = []
    l2 = []
    for i in range(len(pos1)):
        l1.append(pos1[i][j])
        l2.append(pos2[i][j])
    plt.plot(l1, 'b')
    plt.plot(l2, 'r')
    if j >= 7:
        l3 = [targetPositions[j - 7]] * len(pos1)
        plt.plot(l3, 'g')
    plt.show()


'''
class GlutHandler(GlutRendererCallback):
    def __init__(self):
        GlutRendererCallback.__init__(self)
    def keyboardHandler(self, key):
        pass
    def beforeSimulationHandler(self):
        articulation.AddSPDForces(targetPositions, scene.timeStep)
        articulation2.AddSPDForcesABA(targetPositions, scene.timeStep)

handler = GlutHandler()

renderer = GlutRenderer.GetInstance()
renderer.AttachScene(scene, handler)
renderer.StartRenderLoop()
'''

foundation.Dispose()
