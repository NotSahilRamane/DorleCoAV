from yolov5 import YOLO_Fast
import cv2

img = cv2.imread('scripts/image.jpg')
# print(img)


yolo = YOLO_Fast(nms_thresh=0.9, model = '/home/reuben/perception_modules/src/perception/camera_object_detector/scripts/req_files/onnx_models/yolov6s.onnx')

_, _, _, nums, detections = yolo.object_detection(img, visualise=True)
print('Nums:', nums)

cv2.imshow("image", detections)
 
cv2.waitKey(0)

cv2.destroyAllWindows()