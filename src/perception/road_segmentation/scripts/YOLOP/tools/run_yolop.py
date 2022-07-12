import os, sys
import shutil
import time
from pathlib import Path

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

# print(sys.path)
import cv2
import torch
# import torch.backends.cudnn as cudnn
from numpy import random
# import scipy.special
# import numpy as np
import torchvision.transforms as transforms
# import PIL.Image as image

from lib.config import cfg
# from lib.config import update_config
from lib.utils.utils import select_device, time_synchronized #create_logger
from lib.models import get_net
from lib.dataset.DemoDataset1 import LoadImages
from lib.core.general import non_max_suppression, scale_coords
from lib.utils import plot_one_box,show_seg_result
from lib.core.function import AverageMeter
# from lib.core.postprocess import morphological_process, connect_lane
# from tqdm import tqdm
normalize = transforms.Normalize(
        mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
    )

transform=transforms.Compose([
            transforms.ToTensor(), #resize,
            normalize
        ])

opt_device = 'gpu'
opt_save_dir = 'src/perception/road_segmentation/scripts/YOLOP/inference/output'
opt_weights = 'src/perception/road_segmentation/scripts/YOLOP/weights/End-to-end.pth'
# opt_source = '/home/reuben/Projects/YOLOP/inference/images/0ace96c3-48481887.jpg'
opt_img_size = 320
opt_conf_thres = 0.4
opt_iou_thres = 0.3

device = select_device(logger=None, device=opt_device)
print('Using:', device)
if os.path.exists(opt_save_dir):  # output dir
    shutil.rmtree(opt_save_dir)  # delete dir
os.makedirs(opt_save_dir)  # make new dir
half = device.type != 'cpu'  # half precision only supported on CUDA

# Load model
model = get_net(cfg)
checkpoint = torch.load(opt_weights, map_location= device)
model.load_state_dict(checkpoint['state_dict'])
model = model.to(device)
if half:
    model.half()  # to FP16

# dataset = LoadImages(image, img_size=opt_img_size)
# bs = 1  # batch_size

# Get names and colors
names = model.module.names if hasattr(model, 'module') else model.names
colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]

# vid_path, vid_writer = None, None
img = torch.zeros((1, 3, opt_img_size, opt_img_size), device=device)  # init img
_ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
model.eval()

def detect(cfg, image): #check opt

    # Run inference
    # t0 = time.time()

    # inf_time = AverageMeter()
    # nms_time = AverageMeter()

    img, img_det, shapes = LoadImages(image)
    
    # for i, (img, img_det, shapes) in tqdm(enumerate(dataset),total = len(dataset)):
        # print("Shapes:\n", shapes)
    img = transform(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    # Inference
    t1 = time_synchronized()
    det_out, da_seg_out,ll_seg_out= model(img)
    t2 = time_synchronized()
        # if i == 0:
        #     print(det_out)
    inf_out, _ = det_out
    # inf_time.update(t2-t1,img.size(0))

    # Apply NMS
    t3 = time_synchronized()
    det_pred = non_max_suppression(inf_out, conf_thres=opt_conf_thres, iou_thres=opt_iou_thres, classes=None, agnostic=False)
    t4 = time_synchronized()

    # nms_time.update(t4-t3,img.size(0))
    det=det_pred[0]

    _, _, height, width = img.shape
    h,w,_=img_det.shape
    pad_w, pad_h = shapes[1][1]
    pad_w = int(pad_w)
    pad_h = int(pad_h)
    ratio = min(1, shapes[1][0][1])

    da_predict = da_seg_out[:, :, pad_h:(height-pad_h),pad_w:(width-pad_w)]
    da_seg_mask = torch.nn.functional.interpolate(da_predict, scale_factor=int(1/ratio), mode='bilinear')
        # print("Ratio:\n", ratio)
    _, da_seg_mask = torch.max(da_seg_mask, 1)
    da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()
        # da_seg_mask = morphological_process(da_seg_mask, kernel_size=7)

        
    ll_predict = ll_seg_out[:, :,pad_h:(height-pad_h),pad_w:(width-pad_w)]
    ll_seg_mask = torch.nn.functional.interpolate(ll_predict, scale_factor=int(1/ratio), mode='bilinear')
    _, ll_seg_mask = torch.max(ll_seg_mask, 1)
    ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()
    # Lane line post-processing
    # ll_seg_mask = morphological_process(ll_seg_mask, kernel_size=7, func_type=cv2.MORPH_OPEN)
    # ll_seg_mask = connect_lane(ll_seg_mask)

    img_det = show_seg_result(img_det, (da_seg_mask, ll_seg_mask), _, _, is_demo=True)

    if len(det):
        det[:,:4] = scale_coords(img.shape[2:],det[:,:4],img_det.shape).round()
        for *xyxy,conf,cls in reversed(det):
            label_det_pred = f'{names[int(cls)]} {conf:.2f}'
            plot_one_box(xyxy, img_det , label=label_det_pred, color=colors[int(cls)], line_thickness=2)

    # print('Results saved to %s' % Path(opt_save_dir))
    # print('Done. (%.3fs)' % (time.time() - t0))
    # print('inf : (%.4fs/frame)   nms : (%.4fs/frame)' % (inf_time.avg,nms_time.avg))
    return img_det           

if __name__ == '__main__':
    # vid_path = 'inference/rgb_streams/shortened_file-60fps.avi'
    # vid_path = 'inference/videos/1.mp4'
    # vid_path = 'src/perception/road_segmentation/scripts/inference/rgb_streams/video_output.mp4'
    vid_path = 'src/perception/road_segmentation/scripts/YOLOP/inference/rgb_streams/video_output.mp4'
    cap = cv2.VideoCapture(vid_path)
    with torch.no_grad():
        while cap.isOpened():
            ret, frame = cap.read()
            # frame = cv2.resize(frame, (320,540))
            # print(frame.shape)

            det_frame = detect(cfg, frame)

            cv2.imshow("Output", det_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break


cv2.destroyAllWindows()
cap.release()

# img_det = detect(cfg, cv2.imread('/home/reuben/Projects/YOLOP/inference/images/0ace96c3-48481887.jpg'))
# cv2.imshow('det', img_det)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

