# Tracking and Visualization for Point Cloud 3D Object Detection

>  **>>> By Deecamp Focus-Det Group <<<**

### 1 Dependency

```pyth
pip install -r requirements
```

### 2 Visualization with tracking

- **usage**

```python
python visualization_with_tracking.py -dets_dir your_dets_dir -lidar_dir your_lidar_dir -eval_dir your_eval_dir
```

or simple

```py
python visualization_with_tracking.py -d your_dets_dir -l your_lidar_dir -e your_eval_dir
```

- **description**

After obtaining 3-d object detection result based on point cloud, we track the objects by AB3DMOT model frame by frame. To visualize the result, you need to feed the detections directory (dets_dir) which includes some text files (.txt), LiDAR  directory (lidar_dir) which includes some point cloud data files (.bin), and tracking result directory  (eval_dir) .

### 3 Visualization after tracking

- **usage**

```python
python visualization_with_tracking.py -dets_dir your_dets_dir -lidar_dir your_lidar_dir
```

or simple

```py
python visualization_with_tracking.py -d your_dets_dir -l your_lidar_dir
```

- **description**

We track the objects by AB3DMOT model frame by frame after obtaining 3-d object detection result. Then tracking result will be obtained. To visualize the result, you need to feed the detections directory (dets_dir) which includes some text files (.txt), and LiDAR directory (lidar_dir) which includes some point cloud data files (.bin).

### 4 Format of 3-d object detection result format

| 0    | 1    | 2    | 3    | 4    | 5    | 6    | 7    | 8    | 9    | 10   | 11   | 12   | 13   | 14   | 15   |
| ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| type | -    | -    | -    | -    | -    | -    | -    | w    | l    | h    | x    | y    | z    | r_y  | conf |

### 5 Format of tracking result

| 0     | 1    | 2    | 3         | 4        | 5     | 6    | 7    | 8    | 9    | 10   | 11   | 12   | 13   | 14   | 15   | 16   | 17   |
| ----- | ---- | ---- | --------- | -------- | ----- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| frame | id   | type | truncated | occluded | alpha | 2d   | 2d   | 2d   | 2d   | h    | w    | l    | x    | y    | z    | r_y  | conf |



## Reference

- https://github.com/xinshuoweng/AB3DMOT
- http://www.open3d.org/docs/release/



