import pychrono as pc
import pychrono.visualization.irrlicht as irr


pc.Initialize()


scene = pc.Scene()


floor = pc.ChBodyEasyBox(10, 0.1, 10, 1000, False)
crankshaft = pc.ChBodyEasyBox(0.5, 1, 0.5, 1, False)
connecting_rod = pc.ChBodyEasyBox(0.5, 1, 0.5, 1, False)
piston = pc.ChBodyEasyBox(0.5, 1, 0.5, 1, False)


scene.Add(floor)
scene.Add(crankshaft)
scene.Add(connecting_rod)
scene.Add(piston)


floor.SetPos(pc.ChVectorD(0, -5, 0))
crankshaft.SetPos(pc.ChVectorD(0, 0, 0))
connecting_rod.SetPos(pc.ChVectorD(1, 0, 0))
piston.SetPos(pc.ChVectorD(3, 0, 0))


crankshaft_joint = pc.ChLinkRevolute()
crankshaft_joint.Initialize(crankshaft, floor, pc.ChVectorD(0, 0, 0), pc.ChVectorD(0, 0, 1))
scene.Add(crankshaft_joint)

connecting_rod_joint = pc.ChLinkRevolute()
connecting_rod_joint.Initialize(crankshaft, connecting_rod, pc.ChVectorD(1, 0, 0), pc.ChVectorD(0, 1, 0))
scene.Add(connecting_rod_joint)

piston_joint = pc.ChLinkRevolute()
piston_joint.Initialize(connecting_rod, piston, pc.ChVectorD(3, 0, 0), pc.ChVectorD(0, 1, 0))
scene.Add(piston_joint)


motor = pc.ChLinkMotor()
motor.Initialize(crankshaft_joint, True, True)
motor.SetTargetSpeed(1)
scene.Add(motor)


vis = irr.ChIrrApp(scene, 'Crank-Slider Mechanism')
vis.SetCamera(pc.ChVectorD(1, 1, -5), pc.ChVectorD(0, 0, 0))
vis.SetLightType(irr.ChIrrApp.LIGHT_DIRECTIONAL)
vis.AddLogo(irr.ChIrrApp.LOGO_CHRONO)
vis.AddSkyBox()
vis.AssetAddTexture(irr.ChIrrApp.TEXTURE_FLOOR, 'textures/grass.png')


vis.Run()


pc.Terminate()