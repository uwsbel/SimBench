import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the Chrono system
chrono.SetChronoDataPath("../../../data/")

# Create the physical system
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a mesh, that is a container for groups of elements and their referenced nodes.
mesh = chrono.ChMesh()

# Create some nodes. These are the points used to define the beam elements.
nodeA = chrono.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
nodeB = chrono.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(2, 0, 0)))
nodeC = chrono.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(4, 0, 0)))

# Add nodes to the mesh
mesh.AddNode(nodeA)
mesh.AddNode(nodeB)
mesh.AddNode(nodeC)

# Create beam section & material
msection = chrono.ChBeamSectionEulerAdvanced()
beam_wy = 0.1
beam_wz = 0.2
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetGshearModulus(0.01e9 * 0.3)
msection.SetBeamRaleyghDamping(0.000)

# Create beam elements
beamAB = chrono.ChElementBeamEuler()
beamAB.SetNodes(nodeA, nodeB)
beamAB.SetSection(msection)

beamBC = chrono.ChElementBeamEuler()
beamBC.SetNodes(nodeB, nodeC)
beamBC.SetSection(msection)

# Add beam elements to the mesh
mesh.AddElement(beamAB)
mesh.AddElement(beamBC)

# Add the mesh to the system
system.Add(mesh)

# Create the Irrlicht visualization
application = chronoirr.ChIrrApp(system, "Beam FEA Demo", chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, 3, -3))

# Add the mesh visualization
mvisualizebeamA = chrono.ChVisualShapeBeam(beamAB, 0.01, 20, 0.01)
mvisualizebeamA.SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.5, 0.8, 0.8)))
beamAB.AddVisualShape(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeBeam(beamBC, 0.01, 20, 0.01)
mvisualizebeamC.SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.5, 0.8, 0.8)))
beamBC.AddVisualShape(mvisualizebeamC)

# Initialize the Irrlicht application
application.AssetBindAll()
application.AssetUpdateAll()

# Set the time step and simulation end time
time_step = 0.01
end_time = 10.0

# Run the simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    # Apply a force to the end node
    nodeC.SetForce(chrono.ChVectorD(0, -2, 0))

    # Advance the simulation
    application.DoStep()

    # Break the loop if the end time is reached
    if system.GetChTime() > end_time:
        break

    application.EndScene()

# Clean up
application.GetDevice().drop()