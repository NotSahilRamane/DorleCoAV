

classesFile = "./src/perception/camera_object_detector/req_files/coco_classes.txt"
with open(classesFile, 'rt') as f:
            classes = f.read().rstrip('\n').split('\n')

print(classes)