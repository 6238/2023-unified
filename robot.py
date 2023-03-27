import depthai as dai
import numpy as np
import cv2
from networktables import NetworkTables as nt
# Create pipeline
pipeline = dai.Pipeline()
# This might improve reducing the latency on some systems
pipeline.setXLinkChunkSize(0)

# Define source and output
rgb = pipeline.create(dai.node.ColorCamera)
rgb.setFps(30)
rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
rgb.setIspScale(numerator=10, denominator=27)
rgb.setPreviewSize(640,400)
rgb.setVideoSize(640,400)
rgb.setInterleaved(False)

monoLeft = pipeline.create(dai.node.MonoCamera)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight = pipeline.create(dai.node.MonoCamera)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

depth = pipeline.create(dai.node.StereoDepth)
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY) # Focuses on getting a lot of usable data points rather than one good one
depth.setDepthAlign(dai.CameraBoardSocket.RGB)
depth.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutRgb.setStreamName("rgb")

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("depth")

rgb.video.link(xoutRgb.input)
depth.depth.link(xoutDepth.input)

nt.initialize("roborio-6238-frc.local")
sd = nt.getTable('SmartDashboard')

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    print(device.getUsbSpeed())
    rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    diffs = np.array([])
    while True:
        img = rgbQueue.get()
        depth = depthQueue.get()

        colorFrame = img.getCvFrame()
        depthFrame = depth.getFrame()

        green = (colorFrame[..., 0] <= 200) & (colorFrame[..., 1] >= 200) & (colorFrame[..., 2] >= 200)
        maskedDepth = depthFrame * green

        depthFrameColor = cv2.normalize(maskedDepth, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depthFrameColor = cv2.equalizeHist(depthFrameColor)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_PINK)

        avgDepth = np.median(maskedDepth[np.nonzero(maskedDepth)])

        sd.putNumber("Distance", avgDepth)

        cv2.imshow("Color", colorFrame)
        cv2.imshow("Depth", depthFrameColor)
        print(f"Depth: {avgDepth}")

        # Latency in miliseconds
        latencyMs = (dai.Clock.now() - img.getTimestamp()).total_seconds() * 1000
        diffs = np.append(diffs, latencyMs)
        print('Latency: {:.2f} ms, Average latency: {:.2f} ms, Std: {:.2f}'.format(latencyMs, np.average(diffs), np.std(diffs)))

        cv2.waitKey(1)
