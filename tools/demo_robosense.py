import argparse
import glob
from pathlib import Path
import copy
import pandas
from pcdet.datasets.kitti.robosense_dataset import RobosenseDataset
import pcl
import os

import open3d as o3d

try:
    import open3d
    from visual_utils import open3d_vis_utils as V
    OPEN3D_FLAG = True
except:
    import mayavi.mlab as mlab
    from visual_utils import visualize_utils as V
    OPEN3D_FLAG = False

import numpy as np
import torch

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils


class DemoDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None, ext='.bin'):
        """
        Args:
            root_path:
            dataset_cfg:
            class_names:
            training:
            logger:
        """
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )
        self.root_path = root_path
        self.ext = ext
        data_file_list = glob.glob(str(root_path / f'*{self.ext}')) if self.root_path.is_dir() else [self.root_path]

        data_file_list.sort()
        self.sample_file_list = data_file_list

    def remove_nan_data(self, data_numpy):
        data_numpy = data_numpy
        data_pandas = pandas.DataFrame(data_numpy)
        # 删除任何包含nan的所在行 (实际有三分之一的数据无效，是[nan, nan, nan, 0.0])
        data_pandas = data_pandas.dropna(axis=0, how='any')
        data_numpy = np.array(data_pandas)

        return data_numpy

    # 根据每一帧的pcd文件名和路径single_pcd_path，
    # 得到这一帧中的点云数据，返回点云的numpy格式（M,4）
    def get_single_pcd_info(self, single_pcd_path):
        single_pcd_path = single_pcd_path
        single_pcd_points = pcl.load_XYZI(single_pcd_path)
        # 将点云数据转化为numpy格式
        single_pcd_points_np = single_pcd_points.to_array()
        # 去掉一帧点云数据中无效的点
        single_pcd_points_np = self.remove_nan_data(single_pcd_points_np)
        # print(single_pcd_points_np)
        # 将点云数据转化为list格式
        # single_pcd_points_list =single_pcd_points.to_list()

        return single_pcd_points_np

    def __len__(self):
        return len(self.sample_file_list)

    def __getitem__(self, index):
        print(self.sample_file_list[index])
        # 得到点云数据，且是有效的点云数据，返回点云的numpy格式（M,4）
        # pcd = o3d.io.read_point_cloud("/Project/pycharm_project/OpenPCDet-master/data/robosense/testing/pcd/0310001352_2.pcd")
        # print(pcd)
        # points = np.asarray(pcd.points)
        # o3d.visualization.draw_geometries([pcd])
        # points = np.delete(points,[1,2,3,4],axis=0).reshape(-1, 4)
        #
        # print(len(points))
        # print()
        single_pcd_path="/Project/pycharm_project/OpenPCDet-master/data/robosense/testing/pcd/0310000157_1.pcd"
        points = self.get_single_pcd_info(single_pcd_path)
        points = np.asarray(points).reshape(-1, 4)



        input_dict = {
            'points': points,
            'frame_id': index,
        }

        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict


def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--cfg_file', type=str, default='cfgs/kitti_models/second.yaml',
                        help='specify the config for demo')
    parser.add_argument('--data_path', type=str, default='demo_data',
                        help='specify the point cloud data file or directory')
    parser.add_argument('--ckpt', type=str, default=None, help='specify the pretrained model')
    parser.add_argument('--ext', type=str, default='.bin', help='specify the extension of your point cloud data file')

    args = parser.parse_args()

    cfg_from_yaml_file(args.cfg_file, cfg)

    return args, cfg


def main():
    args, cfg = parse_config()
    logger = common_utils.create_logger()
    logger.info('-----------------Quick Demo of OpenPCDet-------------------------')
    demo_dataset = DemoDataset(
        dataset_cfg=cfg.DATA_CONFIG, class_names=cfg.CLASS_NAMES, training=False,
        root_path=Path(args.data_path), ext=args.ext, logger=logger
    )
    logger.info(f'Total number of samples: \t{len(demo_dataset)}')

    model = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=demo_dataset)
    model.load_params_from_file(filename=args.ckpt, logger=logger, to_cpu=True)
    model.cuda()
    model.eval()
    with torch.no_grad():
        for idx, data_dict in enumerate(demo_dataset):
            logger.info(f'Visualized sample index: \t{idx + 1}')
            data_dict = demo_dataset.collate_batch([data_dict])
            load_data_to_gpu(data_dict)
            pred_dicts, _ = model.forward(data_dict)

            V.draw_scenes(
                points=data_dict['points'][:, 1:], ref_boxes=pred_dicts[0]['pred_boxes'],
                ref_scores=pred_dicts[0]['pred_scores'], ref_labels=pred_dicts[0]['pred_labels']
            )

            if not OPEN3D_FLAG:
                mlab.show(stop=True)

    logger.info('Demo done.')


if __name__ == '__main__':
    main()
