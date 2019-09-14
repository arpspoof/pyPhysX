from pyPhysX import *

foundation = Foundation()
scene = foundation.CreateScene(SceneDescription(), 0.001)

material = scene.CreateMaterial(1.0, 1.0, 0.0)

plane = scene.CreatePlane(material, vec3(0, 1, 0), 0)
plane.SetupCollisionFiltering(1, 2 | 4)

articulation = scene.CreateArticulation("../resources/humanoid.urdf", material, vec3(0, 3.75, 0))

articulation.GetLinkByName("right_ankle").SetupCollisionFiltering(2, 1 | 4)
articulation.GetLinkByName("left_ankle").SetupCollisionFiltering(4, 1 | 2)

nDof = articulation.GetNDof()

articulation.SetKPs([10000.0] * nDof)
articulation.SetKDs([400.0] * nDof)

targetPositions = [
    0.9988130000, 0.0094850000, -0.0475600000, -0.0044750000, # 0-3
    0.9649400000, 0.0243690000, -0.0575550000, 0.2549220000, #8-11
    0.9927550000, -0.0209010000, 0.0888240000, -0.0781780000, #22-25
    0.9659420000, 0.1884590000, -0.1422460000, 0.1058540000, #31-34
    1.0000000000,0.0000000000, 0.0000000000, 0.0000000000, #4-7
    0.9854980000, -0.0644070000, 0.0932430000, -0.1262970000, #17-20
    -0.2491160000, #12
    -0.3915320000, #26
    0.5813480000, #35
    0.1705710000, #21
    0.9993660000, 0.0099520000, 0.0326540000, 0.0100980000, #13-16
    0.9828790000, 0.1013910000, -0.0551600000, 0.1436190000, #27-30
]

'''
import time

start_t = time.time()
for _ in range(10000):
    articulation.AddSPDForces(targetPositions, scene.timeStep)
    scene.Step()
end_t = time.time()

print("total time is ", int((end_t - start_t) * 1000000.0))
'''

class GlutHandler(GlutRendererCallback):
    def __init__(self):
        GlutRendererCallback.__init__(self)
    def keyboardHandler(self, key):
        pass
    def beforeSimulationHandler(self):
        articulation.AddSPDForces(targetPositions, scene.timeStep)

handler = GlutHandler()

renderer = GlutRenderer.GetInstance()
renderer.AttachScene(scene, handler)
renderer.StartRenderLoop()


foundation.Dispose()
