from __future__ import print_function
import matplotlib

matplotlib.use('Agg')
import os, numpy as np, time
from AB3DMOT_libs.model import AB3DMOT
from Xinshuo_PyToolbox.xinshuo_io import load_list_from_folder, fileparts
import struct
import open3d as o3d
from alfred.fusion.common import compute_3d_box_lidar_coords
import argparse


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


def render_option(opt, background_color=np.asarray([0, 0, 0]), point_size=1, line_width=5):
        opt.background_color = background_color
        opt.point_size = point_size
        opt.line_width = line_width


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


def set_line(dets):
    xyz = np.array([dets[[3,4,5]]])
    hwl = np.array([dets[[0,1,2]]])
    r_y  = [dets[6]]
    pts3d = compute_3d_box_lidar_coords(xyz, hwl, angles=r_y, origin=(0.5, 0.5, 0.5), axis=2)
    lines = [[0, 1], [1, 2], [2, 3], [3, 0],
             [4, 5], [5, 6], [6, 7], [7, 4],
             [0, 4], [1, 5], [2, 6], [3, 7]]

    color = color_set(dets[9])
    colors = [color for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(pts3d[0])
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def visualize(dets_dir, lidar_dir, eval_dir):
    # dets_dir = 'data/test_dets'
    # lidar_dir = 'data/test_lidar'
    # eval_dir = 'results/video1'

    det_id2str = {1: 'Pedestrian', 2: 'Car', 3: 'Cyclist', 4: 'Truck', 5: 'Tricar'} # deecamp
    seq_file_list, num_seq = load_list_from_folder(dets_dir)
    total_time = 0
    total_frames = len(seq_file_list)
    if not os.path.exists(eval_dir):
        os.makedirs(eval_dir)

    # =============================visual window==================================
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Open3D-Trk')
    opt = vis.get_render_option()
    render_option(opt, background_color=np.asarray([0.15, 0, 0]), point_size=1, line_width=5)

    vis2 = o3d.visualization.Visualizer()
    vis2.create_window(window_name='Open3D-Focus-Trk')
    opt2 = vis2.get_render_option()
    render_option(opt2, background_color=np.asarray([0, 0, 0]), point_size=2)
    pcd = o3d.geometry.PointCloud()

    mot_tracker = AB3DMOT()

    seq_count = 0
    for frame_num, seq_file in enumerate(seq_file_list):
        _, seq_name, _ = fileparts(seq_file)
        eval_file = os.path.join(eval_dir, seq_name + '.txt')
        eval_file = open(eval_file, 'w')
        multiple_replace(seq_file)
        seq_dets = np.loadtxt(seq_file, delimiter=' ')  # load detections, N x 16 : label 1,2,3,4,5,6,7,w,l,h,x,y,z,theta,score

        print_str = '[#] processing: Name: %s, Current frame: %d / %d   \r' % (seq_name, seq_count, num_seq)
        print(print_str)

        # get irrelevant information associated with an object, not used for associationg
        dets = seq_dets[:, [10, 8, 9, 11, 12, 13, 14]] # reorder to h, w, l, x, y, z, theta
        additional_info = seq_dets[:, [14, 0, 4,5,6,7, 15]] # r_y, label, 2d bbox *4, score
        dets_all = {'dets': dets, 'info': additional_info}

        # =============================visualization==================================
        vis.clear_geometries()
        vis2.clear_geometries()

        bin_velodyne = read_bin_velodyne(os.path.join(lidar_dir,seq_name + '.bin'))
        # From numpy to Open3D
        pcd.points = o3d.utility.Vector3dVector(bin_velodyne)

        # important
        start_time = time.time()
        trackers = mot_tracker.update(dets_all)
        cycle_time = time.time() - start_time
        total_time += cycle_time

        # saving results, loop over each tracklet
        for d in trackers:
            bbox3d_tmp = d[0:7]  # h, w, l, x, y, z, theta in camera coordinate
            id_tmp = d[7]
            ori_tmp = d[8]
            type_tmp = det_id2str[d[9]]
            bbox2d_tmp_trk = d[10:14]
            conf_tmp = d[14]

            # =============================visualization==================================
            if conf_tmp > 0:
                line_set = set_line(d)
                vis.add_geometry(line_set)
                vis2.add_geometry(line_set)

            # save in tracking format, for 3D MOT evaluation
            str_to_srite = '%d %d %s 0 0 %f %f %f %f %f %f %f %f %f %f %f %f %f\n' % (frame_num, id_tmp,
                                                                                      type_tmp, ori_tmp,
                                                                                      bbox2d_tmp_trk[0],
                                                                                      bbox2d_tmp_trk[1],
                                                                                      bbox2d_tmp_trk[2],
                                                                                      bbox2d_tmp_trk[3],
                                                                                      bbox3d_tmp[0], bbox3d_tmp[1],
                                                                                      bbox3d_tmp[2], bbox3d_tmp[3],
                                                                                      bbox3d_tmp[4], bbox3d_tmp[5],
                                                                                      bbox3d_tmp[6],
                                                                                      conf_tmp)
            eval_file.write(str_to_srite)
        # =============================visualization==================================
        vis.add_geometry(pcd)
        vis2.add_geometry(pcd)
        ctr = vis.get_view_control()
        view_control(ctr, zoom=0.1, look_at=[0,0,0], front=[0, -np.pi/4, np.pi/4])
        ctr2 = vis2.get_view_control()
        view_control(ctr2, zoom=0.02, look_at=[0, 10, 0], front=[0, -np.pi/3, np.pi/4])
        vis.poll_events()
        vis2.poll_events()

        seq_count += 1
        eval_file.close()
    print('Total Tracking took: %.3f for %d frames or %.1f FPS' % (total_time, total_frames, total_frames / total_time))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--dets_dir', default='data/test_dets', help='point cloud detection result .txt file')
    parser.add_argument('-l', '--lidar_dir', default='data/test_lidar', help='lidar .bin file')
    parser.add_argument('-e', '--eval_dir', default='results/video1', help='where tracking result saved')
    args = parser.parse_args()
    visualize(args.dets_dir, args.lidar_dir, args.eval_dir)
