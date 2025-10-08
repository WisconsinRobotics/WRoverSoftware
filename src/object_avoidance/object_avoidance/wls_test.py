import depthai as dai
import cv2
import numpy as np
import time


class SectorDepthClassifier():

    X_PIXEL_OFFSET = np.float32(648.040894)
    Y_PIXEL_OFFSET = np.float32(360)
    FOCAL_LENGTH = np.float32(563.33333)
    GAP_THRESHOLD = 2 # The minimum distance between two obstacles such that the rover can fit.

   
    def cb(self, depth_full):
        start_time = time.time()
        # Decode and crop depth image
      
        mask = (depth_full == 0) | (depth_full == np.nan)
        depth_full[mask] = np.float32(10)
        depth_threshold = 2
        H,W = depth_full.shape        

        
        rows = (np.arange(depth_full.shape[0], dtype=np.float32) - self.Y_PIXEL_OFFSET) / self.FOCAL_LENGTH
        mask = -1*depth_full * rows[:, None] < -0.5
        depth_full[mask] = np.float32(10)

        
        # list of all min values of each vertical sector. values are in m
        min_list = np.min(depth_full, axis = 0)
        # list of where objects are
        gap_list = (min_list <= depth_threshold).astype(int)

        
        d = np.diff(gap_list)

        starts = np.nonzero(d == -1)[0]
        ends = np.nonzero(d == 1)[0] + 1

        if not gap_list[-1]:
            ends = np.concatenate((ends, [gap_list.size - 1]))
        if not gap_list[0]:
            starts = np.concatenate(([0], starts))

        gaps = list(zip(starts, ends))


        # code is optimized till here

        thetas = []
        distance_monitor_list = []
        for gap in gaps:
            ux1 = gap[0]
            ux2 = gap[1]
            
            theta1 = np.arctan((ux1 - self.X_PIXEL_OFFSET)/self.FOCAL_LENGTH) 
            theta2 = np.arctan((ux2 - self.X_PIXEL_OFFSET)/self.FOCAL_LENGTH)

            d1 = min_list[ux1]/np.cos(theta1)
            d2 = min_list[ux2]/np.cos(theta2)
            
            # Calculating the theta for each gap
            
            theta = theta2 - theta1
            thetas.append(theta)
            gap_distance = np.sqrt(d1**2 + d2**2 - (2*d1*d2*np.cos(theta)))
            distance_monitor_list.append(gap_distance)

        print("theta: ", (np.array(thetas)*180)/3.14)
        print("list of gaps :",gaps)
        print("list of distance between gaps :", distance_monitor_list, "\n\n")
        
        depth_full = cv2.normalize(depth_full, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_full = cv2.cvtColor(depth_full, cv2.COLOR_GRAY2BGR)

                
        for gap in gaps:
            start_point, end_point = (gap[0], 0), (gap[1], 719)
            color = (0, 255, 0)
            depth_full = cv2.rectangle(depth_full, start_point, end_point, color, -1)

            # Publish overlay

        cv2.imshow("obstacle avoidance", depth_full)
        cv2.waitKey(1)
        
        end_time = time.time() - start_time
        print(end_time)



with dai.Pipeline() as pipeline:
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(1280, 720)
    stereo.setLeftRightCheck(True)


    monoLeftOut = monoLeft.requestOutput((1280, 720))
    monoRightOut = monoRight.requestOutput((1280, 720))

    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    rightOut = monoRightOut.createOutputQueue()
    stereoOut = stereo.depth.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        stereoFrame = stereoOut.get()

        assert stereoFrame.validateTransformations()
        # depth = processDepthFrame(stereoFrame.getCvFrame())
        depth = stereoFrame.getCvFrame().astype(np.float32) / 1000.0
        obj = SectorDepthClassifier()
        obj.cb(depth)
        
    pipeline.stop()