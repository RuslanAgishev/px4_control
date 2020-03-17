import sys
import time
import openvr

openvr.init(openvr.VRApplication_Scene)
poses = []  # will be populated with proper type after first call
for i in range(100):
    poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
    hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]
    print(hmd_pose.mDeviceToAbsoluteTracking)
    sys.stdout.flush()
    time.sleep(0.2)
openvr.shutdown()
