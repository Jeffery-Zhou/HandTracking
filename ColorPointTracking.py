## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import math

def getDist(depth_f, color_f, (x, y)):
    #Use pixel value of  depth-aligned color image to get 3D axess
    aligned_depth_frame = depth_f
    depth = aligned_depth_frame.get_distance(x, y)
    color_frame = color_f
    color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    dx, dy, dz = rs.rs2_deproject_pixel_to_point(
        depth_intrin, [x, y], depth)

    distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
    return [dx, dy, dz]


def getAvgDist(depth_f, color_f, lst):
    dist_lst = []
    for i in lst:
        dist_lst.append(getDist(depth_f, color_f, (i[0], i[1]) ))
    return np.average(np.asarray(dist_lst), axis=0)

def click_event(event, x, y, flags, params):
    img = images
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:

        # displaying the coordinates
        # on the Shell
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        print img[y][x]


        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, str(x) + ',' +
                    str(y), (x, y), font,
                    1, (255, 0, 0), 2)
        cv2.imshow('image', img)

    # checking for right mouse clicks
    if event == cv2.EVENT_RBUTTONDOWN:

        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)

        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        cv2.putText(img, str(b) + ',' +
                    str(g) + ',' + str(r),
                    (x, y), font, 1,
                    (255, 255, 0), 2)
        cv2.imshow('image', img)

def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)


    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)
    clipping_distance_in_meters = 1  # 1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale


    try:
        while True:
            frames = pipeline.wait_for_frames()

            colorizer = rs.colorizer()
            align = rs.align(rs.stream.color)
            aligned_frames = align.process(frames)

            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame         = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                continue

            # Image
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())


            colorized_depth = np.asanyarray(
                colorizer.colorize(aligned_depth_frame).get_data())

            grey_color = 153
            # clipping_distance =1.0
            depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
            bg_removed = np.where((depth_image_3d > clipping_distance) | (
                depth_image_3d <= 0), grey_color, color_image)

            global images
            # images = np.hstack((bg_removed, colorized_depth))
            images = np.hstack((bg_removed, colorized_depth))
            images = bg_removed
            color = (223, 180, 88)

# ----------------------------- add new color points -----------------------------------
            red_pos = []
            green_pos = []
            blue_pos = []
            white_pos = []

            for x in range(280, 480):
                for y in range(160, 375):
                    r, g, b = images[y][x]
                    if r >= 86 and r <= 130 and g >= 105 and g <= 140 and b >= 248:
                        # print("Found color at %s,%s!" % (x, y))
                        red_pos.append((x, y))
                    if r >= 75 and r <= 140 and g >= 150 and g <= 198 and b >= 25 and b <= 98:
                        green_pos.append((x, y))
                    if r >= 170 and r <= 230 and g >= 165 and g <= 245 and b >= 40 and b <= 120:
                        blue_pos.append((x, y))
                    if r >= 250 and r <= 256 and g >= 250 and g <= 256 and b >= 40 and b <= 256:
                        white_pos.append((x, y))



            avg_pos_lst = []
            avg_red_pos = getAvgDist(aligned_depth_frame, color_frame, red_pos)
            avg_green_pos = getAvgDist(aligned_depth_frame, color_frame, red_pos)
            avg_blue_pos = getAvgDist(aligned_depth_frame, color_frame, red_pos)
            avg_white_pos = getAvgDist(aligned_depth_frame, color_frame, red_pos)

            avg_pos_lst.append(avg_red_pos)
            avg_pos_lst.append(avg_green_pos)
            avg_pos_lst.append(avg_blue_pos)
            avg_pos_lst.append(avg_white_pos)

            print(avg_red_pos)
# ----------------------------- add new color points -----------------------------------

            # print avg_pos_lst
            with open('marker_pose.txt', 'a') as f:
                if avg_red_pos.any() and avg_green_pos.any() and avg_blue_pos.any() and avg_white_pos.any():
                    for i in avg_pos_lst:
                        # print np.asarray(i)
                        f.write('%s,' % i)
                    f.write('\n')



            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            red = (255, 0, 0)
            # cv2.circle(images, (320, 240), 5, red, -1)
            cv2.rectangle(images, (280, 160), (480, 375), (0, 255, 0), 2)

# ----------------------------------------------------------------
            # show the detected pixel
            # for i in red_pos:
            #     cv2.circle(images, i, 2, red, -1)

            # for i in green_pos:
            #     cv2.circle(images, i, 2, red, -1)

            # for i in blue_pos:
            #     cv2.circle(images, i, 2, red, -1)

            # for i in white_pos:
            #     cv2.circle(images, i, 2, red, -1)
# ----------------------------------------------------------------

            cv2.imshow('RealSense', images)
            cv2.setMouseCallback('RealSense', click_event)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print "I'm done"
                break
    finally:
        pipeline.stop()


if __name__ == '__main__':
    main()
