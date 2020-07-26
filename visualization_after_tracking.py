import os
import numpy as np
import struct
import open3d as o3d
from alfred.fusion.common import draw_3d_box, compute_3d_box_lidar_coords
import argparse
import cv2


def multiple_replace(file):
  f = open(file, "r+")
  text = f.readlines()
  f.seek(0)
  f.truncate()
  for line in text:
      line = line.replace('Pedestrian','1')
      line = line.replace('Car', '2')
      line = line.replace('Cyclist', '3')
      line = line.replace('Truck', '4')
      line = line.replace('Tricar', '5')
      f.write(line)
  f.close()

def color_set(label):
    # https: // www.sojson.com / rgb.html
    if label == 1:
        color = [x/255. for x in [255,0,255]] # Pedestrian - Magenta
    if label == 2:
        color = [x/255. for x in [255, 0, 0]] # Car - Red1
    if label == 3:
        color = [x/255. for x in [255,255, 1]] # Cyclist - Yellow
    if label == 4:
        color = [x/255. for x in [0,255,255]] # Truck - Cyan
    if label == 5:
        color = [x/255. for x in [160,32,240]] # Tricar - Purple
    return color


def read_bin_velodyne(path):
    pc_list=[]
    with open(path,'rb') as f:
        content=f.read()
        pc_iter=struct.iter_unpack('ffff', content)
        for idx,point in enumerate(pc_iter):
            pc_list.append([point[0], point[1], point[2]])
    return np.asarray(pc_list, dtype=np.float32)


def render_option(opt, background_color = np.asarray([0, 0, 0]), point_size=1 ,line_width=5):
        opt.background_color = background_color
        opt.point_size = point_size
        opt.line_width = line_width
        # opt.point_color_option = o3d.visualization.PointColorOption.Color

def view_control(ctr, field_of_view_step=0.1, zoom=0.1, look_at=None, front=None):
    if front is None:
        front = [0, -np.pi / 4, np.pi / 4]
    if look_at is None:
        look_at = [0, 0, 0]
    ctr.change_field_of_view(step=field_of_view_step)
    ctr.set_zoom(zoom)
    ctr.set_lookat(look_at)
    ctr.set_front(front)
    # ctr.set_up([i, 0, 0])


def visualize(lidar_dir, dets_dir):

    # lidar_dir ='data/test_lidar'
    # dets_dir = 'results/video1_backup'

    filename = os.listdir(lidar_dir)
    file_number = len(filename)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Open3D-Trk')

    opt = vis.get_render_option()
    render_option(opt, point_size=1, line_width=5)

    vis2 = o3d.visualization.Visualizer()
    vis2.create_window(window_name='Open3D-Focus-Trk')
    opt2 = vis2.get_render_option()
    render_option(opt2, point_size=2, line_width=10)

    pcd = o3d.geometry.PointCloud()

    for i in range(file_number):
        vis.clear_geometries()
        vis2.clear_geometries()
        path = os.path.join(lidar_dir, filename[i])
        print(path)
        example = read_bin_velodyne(path)
        # From numpy to Open3D
        pcd.points = o3d.utility.Vector3dVector(example)
        # pcd.colors = o3d.utility.Vector3dVector( [0,0,1] )

        dets_path = os.path.join(dets_dir, filename[i][:-4] + '.txt')
        multiple_replace(dets_path)
        dets = np.loadtxt(dets_path, delimiter = ' ')  # load detections, N x 16 : label 1,2,3,4,5,6,7,w,l,h,x,y,z,theta,score

        for d in dets:
            xyz = np.array([d[[13, 14, 15]]])
            hwl = np.array([d[[10, 11, 12]]])
            r_y = [d[16]]
            pts3d = compute_3d_box_lidar_coords(xyz, hwl, angles=r_y, origin=(0.5, 0.5, 0.5), axis=2)
            lines = [[0, 1], [1, 2], [2, 3], [3, 0],
                     [4, 5], [5, 6], [6, 7], [7, 4],
                     [0, 4], [1, 5], [2, 6], [3, 7]]

            color = color_set(d[2])
            colors = [color for i in range(len(lines))]
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(pts3d[0])
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.colors = o3d.utility.Vector3dVector(colors)
            vis.add_geometry(line_set)
            vis2.add_geometry(line_set)
        vis.add_geometry(pcd)
        vis2.add_geometry(pcd)

        vis.add_geometry(pcd)
        vis2.add_geometry(pcd)
        ctr = vis.get_view_control()
        view_control(ctr, zoom=0.1, look_at=[0,0,0], front=[0, -np.pi/4, np.pi/4])
        ctr2 = vis2.get_view_control()
        view_control(ctr2, zoom=0.02, look_at=[0, 10, 0], front=[0, -np.pi/3, np.pi/4])

        # ctr.set_up([i, 0, 0])
        vis.poll_events()
        vis2.poll_events()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--dets_dir', default='data/test_lidar', help='point cloud detection result .txt file')
    parser.add_argument('-l', '--lidar_dir', default='results/video1_backup', help='lidar .bin file')
    args = parser.parse_args()
    visualize(args.dets_dir, args.lidar_dir)