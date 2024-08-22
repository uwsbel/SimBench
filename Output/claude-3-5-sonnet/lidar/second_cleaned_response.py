import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    side = 2.0  
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVectorD(0, 0, 0))
    box.SetBodyFixed(True)
    
    
    texture = chrono.ChVisualMaterial()
    texture.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    box.GetVisualShape(0).SetMaterial(0, texture)
    
    mphysicalSystem.Add(box)

    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box,                    
        update_rate,            
        offset_pose,            
        horizontal_samples,     
        vertical_samples,       
        horizontal_fov,         
        max_vert_angle,         
        min_vert_angle,         
        100.0,                  
        sens.LidarBeamShape_RECTANGULAR,  
        sample_radius,          
        divergence_angle,       
        divergence_angle,       
        return_mode             
    )
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    
    manager.AddSensor(lidar)

    
    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor(
        box,                    
        update_rate,            
        offset_pose_2d,         
        horizontal_samples,     
        1,                      
        horizontal_fov,         
        0,                      
        0,                      
        100.0,                  
        sens.LidarBeamShape_RECTANGULAR,  
        sample_radius,          
        divergence_angle,       
        divergence_angle,       
        return_mode             
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    
    manager.AddSensor(lidar_2d)

    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    t1 = time.time()

    while ch_time < end_time:
        
        orbit_pose = chrono.ChFrameD(
            chrono.ChVectorD(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1
            ),
            chrono.Q_from_AngAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))
        )
        lidar.SetOffsetPose(orbit_pose)
        lidar_2d.SetOffsetPose(orbit_pose)

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('3D XYZI buffer received. Resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('3D Max Value: {0}'.format(np.max(xyzi_data)))

        
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print('2D XYZI buffer received. Resolution: {0}x{1}'.format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height))
            print('2D Max Value: {0}'.format(np.max(xyzi_data_2d)))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_C_PI
max_vert_angle = chrono.CH_C_PI / 12
min_vert_angle = -chrono.CH_C_PI / 6
lag = 0
collection_time = 1. / update_rate
sample_radius = 2
divergence_angle = 0.003


step_size = 1e-3
end_time = 40.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"

main()