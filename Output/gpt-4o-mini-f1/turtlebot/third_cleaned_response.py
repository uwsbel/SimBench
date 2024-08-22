import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens
import random


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
robot.Initialize()  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)


manager = sens.ChSensorManager(system)

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.5, 0, 0.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)


update_rate = 5.0

horizontal_samples = 800
vertical_samples = 300

horizontal_fov = 2 * chrono.CH_PI  
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6

lag = 0

collection_time = 1. / update_rate  

valid_radius = 60.0

sample_radius = 0.003

vertical_sample_radius = 0.0005

horizontal_sample_radius = 0.0005

divergence_angle = 0.0002

divergence_angle_vert = 0.0002

divergence_angle_vert_large = 0.0005

divergence_angle_vert_large_large = 0.001

divergence_angle_horiz_large = 0.001

divergence_angle_horiz_large_large = 0.002

divergence_angle_horiz_large_large_large = 0.003

divergence_angle_horiz_large_large_large_large = 0.005

divergence_angle_horiz_large_large_large_large_large = 0.01

divergence_angle_horiz_large_large_large_large_large_large = 0.02

divergence_angle_horiz_large_large_large_large_large_large_large = 0.03

divergence_angle_horiz_large_large_large_large_large_large_large_large = 0.06

divergence_angle_horiz_large_large_large_large_large_large_large_large_large = 0.12

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large = 0.25

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large = 0.5

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large = 1.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large = 2.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 4.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 8.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 16.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 32.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 64.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 128.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 256.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 512.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 1024.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 2048.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 4096.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 8192.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 16384.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 32768.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 65536.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 131072.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 262144.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 524288.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 1048576.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 2097152.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 4194304.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 8388608.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 16777216.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 33554432.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 67108864.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 134217728.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 268435456.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 536870912.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 1073741824.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 2147483648.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 4294967296.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 8589934592.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 17179869184.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 34359738368.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 68719476736.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 137438953472.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 274877906944.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 549755813888.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 1099511627776.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 2199023255552.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 4398046511104.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 8796093022208.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 17592186044416.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 35184372088832.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 70368744177664.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 140737488355328.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 281474976710656.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 562949953421312.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 1125899906842624.0

divergence_angle_horiz_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large_large = 2251799813685248.0

divergence_angle_horiz_large_large_large_large_large_large_large_large
print("error happened with only start ```python")