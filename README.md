# My Openpcdet 

多卡训练

```
CUDA_VISIBLE_DEVICES=3,6,7 python -m torch.distributed.launch --nproc_per_node=3 train.py --cfg_file cfgs/kitti_models/pv_rcnn.yaml --launcher pytorch
```

评测模型

```
python test.py --cfg_file cfgs/kitti_models/pv_rcnn.yaml  --ckpt ../output/kitti_models/pv_rcnn/default/ckpt/checkpoint_epoch_80.pth
```

