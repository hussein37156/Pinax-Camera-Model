#!/usr/bin/env python3
import rospy
import numpy as np
import cv2 as cv
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

mapx_left = None
mapy_left = None
mapx_right = None
mapy_right = None

# ====== Camera Parameters ======
baseline = 0.12  # meters
MIN_DEPTH = 0.3
MAX_DEPTH = 10.0
h, w = 360, 640
fx = 350
baseline=0.12
# ====== ROS Globals ======
bridge = CvBridge()
left_image = None
right_image = None
lock = threading.Lock()

# ====== Stereo Matchers ======
window_size = 5
min_disp = 0
num_disp = 16 * 6

left_matcher = cv.StereoSGBM_create(
    minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=window_size,
    P1=8 * 3 * window_size**2,
    P2=32 * 3 * window_size**2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=2,
    preFilterCap=63,
    mode=cv.STEREO_SGBM_MODE_SGBM_3WAY
)

right_matcher = cv.ximgproc.createRightMatcher(left_matcher)
wls_filter = cv.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls_filter.setLambda(8000)
wls_filter.setSigmaColor(1.5)
clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
def init():
    mapPath = "/home/hussein/pinax_ws/src/Pinax-camera-model/depth_estimation/scripts/maps"
    global mapx_left, mapy_left , mapx_right, mapy_right
    mapx_left = np.loadtxt(f"{mapPath}/MapX_left.txt", dtype=np.float32, delimiter=",")
    mapy_left = np.loadtxt(f"{mapPath}/MapY_left.txt", dtype=np.float32, delimiter=",")
    mapx_right = np.loadtxt(f"{mapPath}/MapX_right.txt", dtype=np.float32, delimiter=",")
    mapy_right = np.loadtxt(f"{mapPath}/MapY_right.txt", dtype=np.float32, delimiter=",")
    
    
    rospy.init_node('depth_map_generation', anonymous=False)
    rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, left_image_call_back)
    rospy.Subscriber("/zed/zed_node/right/image_rect_color", Image, right_image_call_back)
    
def enhance_contrast(image_bgr):
    # Convert to LAB color space
    lab = cv.cvtColor(image_bgr, cv.COLOR_BGR2LAB)
    l, a, b = cv.split(lab)

    # Apply CLAHE to the L-channel
    cl = clahe.apply(l)

    # Merge and convert back to BGR
    limg = cv.merge((cl, a, b))
    result = cv.cvtColor(limg, cv.COLOR_LAB2BGR)
    return result

# ====== Callbacks ======
def left_image_call_back(data):
    global left_image ,mapx_left, mapy_left ,bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        img_bgr = cv.cvtColor(cv_image, cv.COLOR_RGBA2BGR)
        correctedImg = cv.remap(img_bgr, mapx_left, mapy_left, cv.INTER_LINEAR)
        enhancedImg = enhance_contrast(correctedImg)
        with lock:
            left_image = enhancedImg
    except Exception as e:
        rospy.logerr(f"Left image callback error: {e}")

def right_image_call_back(data):
    global right_image ,mapx_right, mapy_right ,bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        img_bgr = cv.cvtColor(cv_image, cv.COLOR_RGBA2BGR)
        correctedImg = cv.remap(img_bgr, mapx_right, mapy_right, cv.INTER_LINEAR)
        enhancedImg = enhance_contrast(correctedImg)
        with lock:
            right_image = enhancedImg
    except Exception as e:
        rospy.logerr(f"Right image callback error: {e}")

# ====== Utility Functions ======
def preprocess_image(img):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    gray = cv.equalizeHist(gray)
    return gray

def postprocess_disparity(disp):
    disp = disp.astype(np.float32) / 16.0
    disp = cv.medianBlur(disp, 5)
    kernel = np.ones((5, 5), np.uint8)
    disp = cv.morphologyEx(disp, cv.MORPH_CLOSE, kernel)
    disp[disp <= 0] = 0
    return disp

# ====== Main Processing Thread ======
def processing_loop():
    global left_image, right_image

    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        with lock:
            l_img = left_image.copy() if left_image is not None else None
            r_img = right_image.copy() if right_image is not None else None

        if l_img is None or r_img is None:
            rate.sleep()
            continue

        try:
            left_gray = preprocess_image(l_img)
            right_gray = preprocess_image(r_img)

            disp_left = left_matcher.compute(left_gray, right_gray)
            disp_right = right_matcher.compute(right_gray, left_gray)

            filtered_disp = wls_filter.filter(disp_left, l_img, disparity_map_right=disp_right)
            disparity = postprocess_disparity(filtered_disp)

            with np.errstate(divide='ignore'):
                depth_image = (fx * baseline) / (disparity + 1e-6)
            depth_image[disparity <= 0] = 0
            depth_image[depth_image > MAX_DEPTH] = 0

            depth_msg = bridge.cv2_to_imgmsg(depth_image.astype(np.float32), encoding="32FC1")
            depth_msg.header.stamp = rospy.Time.now()
            depth_msg.header.frame_id = "depth_frame"
            depth_map_pub.publish(depth_msg)

        except Exception as e:
            rospy.logerr(f"Processing error: {e}")
        
        rate.sleep()

# ====== Main Entry ======
if __name__ == "__main__":
    init()
    depth_map_pub = rospy.Publisher("/depth_image", Image, queue_size=1)
    processing_thread = threading.Thread(target=processing_loop)
    processing_thread.start()

    rospy.spin()
