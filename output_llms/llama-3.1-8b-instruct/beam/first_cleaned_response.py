import chrono as chrono
import chrono.irrlicht as irrlicht


chrono.Initialize()


sys = chrono.ChSystem()


ground = chrono.ChBody()
ground.SetBodyType(chrono.ChBodyType.STATIC)
ground.SetCollisionShape(chrono.ChCollisionModel.CreateBox(10, 1, 1))
sys.Add(ground)


beam = chrono.ChBody()
beam.SetBodyType(chrono.ChBodyType.BEAM)
beam.SetCollisionShape(chrono.ChCollisionModel.CreateBox(1, 1, 1))
beam.SetMass(1)
beam.SetInertia(1)
sys.Add(beam)


node = chrono.ChBody()
node.SetBodyType(chrono.ChBodyType.STATIC)
node.SetCollisionShape(chrono.ChCollisionModel.CreateSphere(0.1))
sys.Add(node)


beam_fe = chrono.ChBeamFiniteElement()
beam_fe.SetBeam(beam)
beam_fe.SetNode1(node)
beam_fe.SetNode2(chrono.ChNodeBody.GetNode(beam))
beam_fe.SetLength(1)
beam_fe.SetCrossSection(chrono.ChBeamCrossSection.CreateBox(0.1, 0.1))
beam_fe.SetMaterial(chrono.ChMaterialSurface.CreateDefault())
sys.Add(beam_fe)


ground.SetPosition(chrono.ChVector(0, 0, 0))
beam.SetPosition(chrono.ChVector(0, 1, 0))
node.SetPosition(chrono.ChVector(0, 2, 0))
beam_fe.SetPosition(chrono.ChVector(0, 1.5, 0))


engine = irrlicht.CreateIrrlichtDevice(irrlicht.VideoMode(800, 600, irrlicht.EVFT_32))
view = engine.getSceneManager().addCameraSceneNode()
view.setTarget(chrono.ChVector(0, 1, 0))


while True:
    sys.DoStep()
    engine.beginScene()
    view.beginScene()
    sys.DrawAll()
    view.endScene()
    engine.endScene()
    engine.nextFrame()