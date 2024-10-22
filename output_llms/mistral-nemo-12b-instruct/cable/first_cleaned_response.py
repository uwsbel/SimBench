import pychrono as pc
import numpy as np


pc.set_log_level(pc.LogLevel.INFO)
sim = pc.Simulation()
renderer = pc.Renderer()
sim.set_renderer(renderer)


ground = pc.ChBodyEasyBox(10, 10, 0.1, density=1000, material=pc.ChMaterialSurfaceNSC())
ground.SetPos(np.array([0, 0, -0.05]))
sim.Add(ground)


hinge = pc.ChLinkLock()
hinge.Initialize(ground, None, pc.VecD3(0, 0, 0), pc.ChFrameD())
sim.Add(hinge)


num_elements = 10
beam = pc.ChCable()
beam.SetNumElements(num_elements)
beam.SetSection(0.01, 0.01)  
beam.SetMaterial(pc.ChMaterialSurfaceNSC())
beam.SetYieldStrength(1e6)
beam.SetYoungModulus(2e7)
beam.SetDensity(8000)
beam.SetInitialLength(1)
beam.SetFixedEnd(True, False)
beam.SetPos(np.array([0, 0, 0.5]))
sim.Add(beam)


sim.SetGravity(pc.VecD3(0, 0, -9.81))


beam_vis = pc.ChVisualShapeCable()
beam_vis.SetCable(beam)
beam_vis.SetFilled(True)
beam_vis.SetColor(pc.ChColor(0.8, 0.6, 0.4))
renderer.Add(beam_vis)


while sim.GetTime() < 5:
    sim.DoStepDynamics(0.01)

    
    for i in range(num_elements + 1):
        node_pos = beam.GetNodeAbsPos(i)
        renderer.AddPoint(node_pos, pc.ChColor(1, 0, 0))

    renderer.Render()