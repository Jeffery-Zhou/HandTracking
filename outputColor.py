import open3d as o3d
import numpy as np

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) Afther picking points, press q for close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run() 
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud("20210604-171520.pcd")
    color_lst = np.asarray(pcd.colors)
    select_lst = pick_points(pcd)
    # print(select_lst)
    # select_lst = [7422, 7593, 7289, 7460, 6996, 7326, 7022, 7352]
    color_maker_lst = [list(color_lst[i]) for i in select_lst]
    print("The color intenstiy list is as below: ")
    print(color_maker_lst)
