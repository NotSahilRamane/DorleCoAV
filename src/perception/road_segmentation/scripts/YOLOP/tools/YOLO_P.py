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

normalize = transforms.Normalize(
        mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
    )

transform=transforms.Compose([
            transforms.ToTensor(), #resize,
            normalize
        ])

class YOLOP:
    def __init__(self, img_size=640, opt_conf_thres = 0.4, opt_iou_thres = 0.3):
        
        self.conf_thresh=opt_conf_thres
        self.iou_thresh=opt_iou_thres
        
        self.device = select_device(logger=None, device='gpu')
        weights = 'src/perception/road_segmentation/scripts/YOLOP/weights/End-to-end.pth'
        self.half = self.device.type != 'cpu'

        self.model = get_net(cfg)
        checkpoint = torch.load(weights, map_location=self.device)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model = self.model.to(self.device)
        if self.half:
            self.model.half()

        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.names))]
        
        # vid_path, vid_writer = None, None
        img = torch.zeros((1, 3, img_size, img_size), device=self.device)  # init img
        _ = self.model(img.half() if self.half else img) if self.device.type != 'cpu' else None  # run once
        self.model.eval()

    def detect(self, image):
        img, img_det, shapes = LoadImages(image)
        img = transform(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        # Inference
        det_out, da_seg_out,ll_seg_out= self.model(img)
        inf_out, _ = det_out

        det_pred = non_max_suppression(inf_out, conf_thres=self.conf_thresh, iou_thres=self.iou_thresh, classes=None, agnostic=False)
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
        # print(da_seg_mask.shape)
        # Lane line post-processing
        # ll_seg_mask = morphological_process(ll_seg_mask, kernel_size=7, func_type=cv2.MORPH_OPEN)
        # ll_seg_mask = connect_lane(ll_seg_mask)

        img_det = show_seg_result(img_det, (da_seg_mask, ll_seg_mask), _, _, is_demo=True)

        # if len(det):
        #     det[:,:4] = scale_coords(img.shape[2:],det[:,:4],img_det.shape).round()
        #     for *xyxy,conf,cls in reversed(det):
        #         label_det_pred = f'{self.names[int(cls)]} {conf:.2f}'
        #         plot_one_box(xyxy, img_det , label=label_det_pred, color=self.colors[int(cls)], line_thickness=2)

        return img_det, da_seg_mask, ll_seg_mask

# if __name__ == '__main__':

#     vid_path = 'src/perception/road_segmentation/scripts/YOLOP/inference/rgb_streams/video_output.mp4'
#     cap = cv2.VideoCapture(vid_path)

#     # fps = cap.get(cv2.CAP_PROP_FPS)
#     # size = (int(cap.get(4)), int(cap.get(3)))
#     # result = cv2.VideoWriter('out.avi', 
#     #                      cv2.VideoWriter_fourcc(*'MJPG'),
#     #                      fps, size)

#     yolo_p = YOLOP()
#     with torch.no_grad():
#         while cap.isOpened():
#             ret, frame = cap.read()
#             if ret:
#                 det_frame, _, _ = yolo_p.detect(frame)
#                 cv2.imshow("Output", det_frame)
#                 # result.write(det_frame)
#                 key = cv2.waitKey(1) & 0xFF
#                 if key == ord("q"):
#                     break
#             else:
#                 break

# cv2.destroyAllWindows()
# # result.release()
# cap.release()



# if __name__ == '__main__':
#     # vid_path = 'inference/rgb_streams/shortened_file-60fps.avi'
#     # vid_path = 'inference/videos/1.mp4'
#     vid_path = 'inference/rgb_streams/video_output.mp4'
#     cap = cv2.VideoCapture(vid_path)
#     fps = cap.get(cv2.CAP_PROP_FPS)
#     size = (int(cap.get(3)), int(cap.get(4)))
#     result = cv2.VideoWriter('out.mp4', 
#                          cv2.VideoWriter_fourcc(*'MJPG'),
#                          fps, size)
#     with torch.no_grad():
#         while cap.isOpened():
#             ret, frame = cap.read()
#             if ret:
#             # frame = cv2.resize(frame, (640,540))
#                 print(frame.shape)

#                 det_frame = detect(frame)
#                 result.write(det_frame)
#                 cv2.imshow("Output", det_frame)
#                 key = cv2.waitKey(1) & 0xFF
#                 if key == ord("q"):
#                     break
#             else:
#                 break


# cv2.destroyAllWindows()
# cap.release()
# result.release()
