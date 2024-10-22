import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Initialize the PyChrono environment
chrono.ChSystemSMC()

# Initialize the Irrlicht visualization engine
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('ANCF Cable Beam Simulation')
vis.Initialize()

# Create a material for the cable elements
material = chrono.ChMaterialSurfaceSMC()
material.SetFriction(0.3)

# Create a mesh for the cable elements
mesh = chrono.ChMesh()

# Define the cable element properties
num_elements = 10  # Number of cable elements
element_length = 0.5  # Length of each cable element
cable_radius = 0.01  # Radius of the cable

# Create the cable elements and add them to the mesh
for i in range(num_elements):
    nodeA = chrono.ChNodeFEAxyz(chrono.ChFrameD(chrono.ChVectorD(i * element_length, 0, 0)))
    nodeB = chrono.ChNodeFEAxyz(chrono.ChFrameD(chrono.ChVectorD((i + 1) * element_length, 0, 0)))

    element = chrono.ChElementCableANCF(nodeA, nodeB, cable_radius, material)
    mesh.AddElement(element)

# Create a body for the ground and add it to the system
ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True, chrono.ChFrameD(chrono.ChVectorD(0, 0, -0.05)))
chrono.GetSystem().Add(ground)

# Create a body for the hinged end of the beam and add it to the system
hinge_body = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hinge_body.SetBodyFixed(True)
chrono.GetSystem().Add(hinge_body)

# Create a revolute joint between the hinged end and the ground
hinge_joint = chrono.ChLinkLockRevolute()
hinge_joint.Initialize(hinge_body, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
chrono.GetSystem().Add(hinge_joint)

# Add the mesh to the system
chrono.GetSystem().Add(mesh)

# Set gravity
chrono.GetSystem().Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Set the initial position of the cable elements
for node in mesh.GetNodes():
    node.SetPos(chrono.ChVectorD(node.GetPos().x(), 0, 0))

# Set the visualization properties for the cable elements
for element in mesh.GetElements():
    element.SetVisualizationType(chrono.ChVisualizationFEA.E_BEAM_ELEM)
    element.SetVisualizationColor(chrono.ChColor(0.8, 0.8, 0.8))

# Simulation loop
while vis.Run():
    chrono.GetSystem().DoStepDynamics(0.01)

    # Update the visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Print the nodal positions
    for node in mesh.GetNodes():
        print(f'Node position: {node.GetPos()}')