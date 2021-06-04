import pyrealsense2 as rs
import numpy as np
import cv2
# from open3d import *
import open3d as o3d
import time
import logging

# -------------------------------
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
# -------------------------------

def cropPCD(pcd):
    # max_dis = 0.20/1000
    max_x = 0.1/1000
    max_y = 0.1/1000

    pcd_points = np.asarray(pcd.points)
    color_lst = np.asarray(pcd.colors)
    pos_col_lst = np.concatenate((pcd_points, color_lst), axis=1)

    pos_col_lst = pos_col_lst[(pos_col_lst[:, 2] < -0.00074)]
    pos_col_lst = pos_col_lst[(pos_col_lst[:, 2] > -0.00094)]
    pos_col_lst = pos_col_lst[(pos_col_lst[:, 0] < max_x)]
    pos_col_lst = pos_col_lst[(pos_col_lst[:, 0] > -max_x)]
    pos_col_lst = pos_col_lst[(pos_col_lst[:, 1] < max_y)] 
    pos_col_lst = pos_col_lst[(pos_col_lst[:, 1] > -max_y)] 
    pcd.points = o3d.utility.Vector3dVector(pos_col_lst[:,0:3])
    pcd.colors = o3d.utility.Vector3dVector(pos_col_lst[:, 3:])

    # o3d.visualization.draw_geometries([pcd])
    timestr = time.strftime("%Y%m%d-%H%M%S")
    o3d.io.write_point_cloud("%s.pcd" % (timestr), pcd)
    logging.info("%s.pcd output successfully!" % (timestr))
    saveRes(pos_col_lst, timestr)



def saveRes(pos_col_lst, timestr):
    # put the RGB value list here
    color_maker_lst = [
        [0.42352941176470588, 0.47058823529411764, 0.94901960784313721], [0.396078431372549, 0.51764705882352946, 0.96078431372549022], [0.45490196078431372, 0.93333333333333335, 0.89411764705882357], [0.47058823529411764, 0.59215686274509804, 0.21176470588235294], [0.6470588235294118, 0.52941176470588236, 0.17647058823529413], [0.40392156862745099, 0.24705882352941178, 0.32941176470588235], [0.75686274509803919, 0.792156862745098, 0.80784313725490198], [0.90196078431372551, 0.93725490196078431, 0.83921568627450982]
        ]
    color_range = 0.02
    target_pos_lst = []
    for i in color_maker_lst:
        avg_pos = getAvgPostion(pos_col_lst, i, color_range)
        avg_pos = avg_pos*1000
        print(avg_pos)
        target_pos_lst.append(avg_pos)
    # print(target_pos_lst)
    logging.info("%s.pcd Start to extract the average points." % (timestr))
    target_pos_lst.insert(0, timestr)

    with open('marker_pose.txt', 'a') as f:
        for i in target_pos_lst:
            f.write('%s,' % i)
        f.write('\n')
    logging.info("%s.pcd Extracted average points into the file." % (timestr))



def getAvgPostion(all_ary, col_lst, rg_v):
    # @all_ary: corlor point lst
    # @col_lst: col_lst
    # range value
    col_lst = np.array(col_lst)
    max_rg = col_lst + rg_v
    min_rg = col_lst - rg_v
    all_ary = all_ary[(all_ary[:, 3] >=min_rg[0])]
    all_ary = all_ary[(all_ary[:, 4] >=min_rg[1])]
    all_ary = all_ary[(all_ary[:, 5] >=min_rg[2])]
    all_ary = all_ary[(all_ary[:, 3] <=max_rg[0])]
    all_ary = all_ary[(all_ary[:, 4] <=max_rg[1])]
    all_ary = all_ary[(all_ary[:, 5] <=max_rg[2])]  
    # print(all_ary[:, :3])
    return np.average(all_ary[:, :3], axis=0)  


if __name__ == '__main__':
    pipeline = rs.pipeline()
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # if device_product_line == 'L500':
    #     config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    # else:
    #     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


    pipeline.start(config)
    align = rs.align(rs.stream.color)
    pointcloud = o3d.geometry.PointCloud()
        
    try:
        n = 100 # collection total frames
        while n > 0:
            # dt0=datetime.now()
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            profile = frames.get_profile()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            # ====================== show depth and color img ===============================
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape
            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((color_image, depth_colormap))
            
            # Store images here!
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', images)
            # key = cv2.waitKey(1)

            # if key & 0xFF == ord('q') or key == 27:
            #     cv2.destroyAllWindows()
            #     break
            # ====================== ===============================
            img_depth = o3d.geometry.Image(depth_colormap)
            img_color = o3d.geometry.Image(color_image)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth, convert_rgb_to_intensity=False)
            intrinsics = profile.as_video_stream_profile().get_intrinsics()
            pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            pointcloud.points = pcd.points
            pointcloud.colors = pcd.colors
            print()
            print("================================= Handling ================================")
            cropPCD(pointcloud)
            time.sleep(1)

            n -= 1
            # break
    finally:
        pipeline.stop()
