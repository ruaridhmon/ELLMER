#!/home/kinova/dev/kinova/env1/bin/python3
import open3d as o3d

import cv2
import numpy as np
import supervision as sv
import argparse
import torch
import torchvision
from std_msgs.msg import String
from kortex_driver.msg import TargetCoordinates, TargetCoordinatesArray
import os
import sys
import rospy
import json

current_directory = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current_directory)
sys.path.append(parent_directory)

from groundingdino.util.inference import Model
from segment_anything import SamPredictor
from MobileSAM.setup_mobile_sam import setup_model

class AzureKinect:
    def __init__(self, device):
        self.config = o3d.io.AzureKinectSensorConfig()
        self.device = device
        self.align_depth_to_color = 1

    def start(self):
        self.sensor = o3d.io.AzureKinectSensor(self.config)
        if not self.sensor.connect(self.device):
                raise RuntimeError('Failed to connect to sensor')

    def frames(self):
        while 1:

            rgbd = self.sensor.capture_frame(self.align_depth_to_color)
            if rgbd is None:
                continue
            color, depth = np.asarray(rgbd.color).astype(np.uint8), np.asarray(rgbd.depth).astype(np.float32) / 1000.0
            return color, depth


def parse_args(arg_list=None):
    if arg_list is None:
        argv = rospy.myargv()[1:]
    else:
        argv = arg_list
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--MOBILE_SAM_CHECKPOINT_PATH", type=str, default="./EfficientSAM/mobile_sam.pt", help="model"
    )
    parser.add_argument(
        "--SOURCE_IMAGE_PATH", type=str, default="./assets/demo2.jpg", help="path to image file"
    )
    parser.add_argument(
        "--DEPTH_PATH", type=str, default="./depth.npy", help="path to image file"
    )
    parser.add_argument(
        "--CAPTION", type=str, default="The running dog", help="text prompt for GroundingDINO"
    )
    parser.add_argument(
        "--OUT_FILE_BOX", type=str, default="groundingdino_annotated_image.jpg", help="the output filename"
    )
    parser.add_argument(
        "--OUT_FILE_SEG", type=str, default="grounded_mobile_sam_annotated_image.jpg", help="the output filename"
    )
    parser.add_argument(
        "--OUT_FILE_BIN_MASK", type=str, default="grounded_mobile_sam_bin_mask.jpg", help="the output filename"
    )
    parser.add_argument("--BOX_THRESHOLD", type=float, default=0.4, help="")
    parser.add_argument("--TEXT_THRESHOLD", type=float, default=0.4, help="")
    parser.add_argument("--NMS_THRESHOLD", type=float, default=0.8, help="")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument(
        "--DEVICE", type=str, default=device, help="cuda:[0,1,2,3,4] or cpu"
    )
    return parser.parse_args(argv)

def update_classes_callback(msg):
    global CLASSES
    new_classes = msg.data.split(", ")
    CLASSES = new_classes
    rospy.loginfo(f"Updated classes to {CLASSES}")

def segment(sam_predictor: SamPredictor, image: np.ndarray, xyxy: np.ndarray) -> np.ndarray:
    sam_predictor.set_image(image)
    result_masks = []
    for box in xyxy:
        masks, scores, logits = sam_predictor.predict(
            box=box,
            multimask_output=True
        )
        index = np.argmax(scores)
        result_masks.append(masks[index])
    return np.array(result_masks)

def rgbd_pc(image, depth, binary_mask_all, intrinsic, transformation_matrix):
    color_trans = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    

    all_target = []
    all_pcds = []

    for object in range(len(binary_mask_all)):
        binary_mask = binary_mask_all[object]

        color_trans[binary_mask == 0] = np.array([128, 128, 128])
        color_trans[binary_mask == 255] = np.array([0, 0, 0])

        depth_o3d = o3d.geometry.Image(depth)
        color_o3d = o3d.geometry.Image(color_trans)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d, depth_scale=1.0,
                                                                    convert_rgb_to_intensity=False)

        
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

        pcd.transform(transformation_matrix)

        index = np.where((pcd.colors == np.array([0, 0, 0])).all(axis=1))

        target = np.array(pcd.points)[index].mean(axis=0)

        all_target.append(target)
        all_pcds.append(pcd)

    # o3d.visualization.draw_geometries([pcd])
    # print(f"Target coordinates: {target}")

    return all_target



global CLASSES
CLASSES = ["coffee", "black kettle", "spoon handle"]


def vision_module(args):
  import warnings
  import logging
  warnings.filterwarnings("ignore", category=UserWarning, module="transformers")
  logging.getLogger("transformers").setLevel(logging.ERROR)
  
  rospy.init_node('coordinate_publisher')
  rospy.Subscriber("/update_classes", String, update_classes_callback)
  pub = rospy.Publisher('target_coordinates', TargetCoordinatesArray, queue_size=10)

  camera_number = 0

  cam = AzureKinect(camera_number)
  cam.start()

  # camera 0 is close from robot. camera 1 is away from robot.
  intrinsic0 = o3d.camera.PinholeCameraIntrinsic(1280, 720, 607.6044921875,607.41522216796875,640.840576171875,364.01950073242188)
  intrinsic1 = o3d.camera.PinholeCameraIntrinsic(1280, 720, 612.41802978515625,612.31890869140625,640.6007080078125,364.61968994140625)
  intrinsic_array = [intrinsic0, intrinsic1]

  intrinsic = intrinsic_array[camera_number]
    

  DEVICE = args.DEVICE
  script_dir = os.path.dirname(os.path.abspath(__file__))
  parent_dir = os.path.dirname(script_dir)
  grandparent_dir = os.path.dirname(parent_dir)
  GROUNDING_DINO_CONFIG_PATH = os.path.join(grandparent_dir, 'GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py')
  GROUNDING_DINO_CHECKPOINT_PATH = os.path.join(grandparent_dir, 'groundingdino_swint_ogc.pth')
  grounding_dino_model = Model(model_config_path=GROUNDING_DINO_CONFIG_PATH, model_checkpoint_path=GROUNDING_DINO_CHECKPOINT_PATH) # out--------
  MOBILE_SAM_CHECKPOINT_PATH = os.path.join(grandparent_dir, 'EfficientSAM/mobile_sam.pt')
  checkpoint = torch.load(MOBILE_SAM_CHECKPOINT_PATH)
  mobile_sam = setup_model()
  mobile_sam.load_state_dict(checkpoint, strict=True)
  mobile_sam.to(device=DEVICE)
  sam_predictor = SamPredictor(mobile_sam)
  SOURCE_IMAGE_PATH = args.SOURCE_IMAGE_PATH
  BOX_THRESHOLD = args.BOX_THRESHOLD
  TEXT_THRESHOLD = args.TEXT_THRESHOLD
  NMS_THRESHOLD = args.NMS_THRESHOLD
  box_annotator = sv.BoxAnnotator()
  mask_annotator = sv.MaskAnnotator()
  msg = TargetCoordinates()

  current_script_directory = os.path.dirname(__file__)
  json_file_path = os.path.join(current_script_directory, 'camera_to_robot_transformation/camera0_transformation_matrix.json')  

  with open(json_file_path, 'r') as f:
      transformation_matrix = json.load(f)

  transformation_matrix = np.array(transformation_matrix)      


  try:
        while not rospy.is_shutdown(): 

            target_all = return_target_coordinates(cam, grounding_dino_model,BOX_THRESHOLD,TEXT_THRESHOLD,
                                      NMS_THRESHOLD, sam_predictor, mask_annotator, box_annotator,
                                      intrinsic, CLASSES, transformation_matrix)            
        
            coordinate_array_msg = TargetCoordinatesArray()
            coordinate_array_msg.targetcoordinates = []

            if target_all is None: continue

            for target in target_all:
                if target is None: continue
                coordinate = TargetCoordinates(x=target[0], y=target[1], z=target[2])
                coordinate_array_msg.targetcoordinates.append(coordinate)

            pub.publish(coordinate_array_msg)

            if cv2.waitKey(1) == ord('q'):
                break

  except rospy.ROSInterruptException:
    print("ROS shutdown, stopping...")



def return_target_coordinates(cam, grounding_dino_model,BOX_THRESHOLD,TEXT_THRESHOLD,
                              NMS_THRESHOLD, sam_predictor, mask_annotator, box_annotator,
                              intrinsic, CLASSES, transformation_matrix):
    
    color, depth = cam.frames()
    image = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

    detections = grounding_dino_model.predict_with_classes(
        image=image,
        classes=CLASSES,
        box_threshold=BOX_THRESHOLD,
        text_threshold=TEXT_THRESHOLD
    )

    labels = [f'{CLASSES[detection[3]]} {detection[2]:.2f}' for detection in detections]


    nms_idx = torchvision.ops.nms(
        torch.from_numpy(detections.xyxy),
        torch.from_numpy(detections.confidence),
        NMS_THRESHOLD
    ).numpy().tolist()

    
    detections.xyxy = detections.xyxy[nms_idx]
    detections.confidence = detections.confidence[nms_idx]
    detections.class_id = detections.class_id[nms_idx]


    detections.mask = segment(
        sam_predictor=sam_predictor,
        image=cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
        xyxy=detections.xyxy
    )

    annotated_image = mask_annotator.annotate(scene=image.copy(), detections=detections)
    annotated_image = box_annotator.annotate(scene=annotated_image, detections=detections, labels=labels)

    cv2.imshow("segmentation mask", annotated_image)

    binary_mask_all = detections.mask.astype(np.uint8)*255
    target_all = rgbd_pc(image, depth, binary_mask_all, intrinsic, transformation_matrix)
    target_classes = get_highest_confidence_targets(target_all, labels, CLASSES)

    return target_classes


def get_highest_confidence_targets(target_all, labels, CLASSES):
    highest_confidence_targets = {cls: None for cls in CLASSES}

    for target, label in zip(target_all, labels):
        label_parts = label.split()
        class_name = ' '.join(label_parts[0:-1])
        confidence = float(label_parts[-1])

        if highest_confidence_targets[class_name] is None or \
           highest_confidence_targets[class_name][1] < confidence:
            highest_confidence_targets[class_name] = (target, confidence)

    output_targets = []
    for cls in CLASSES:
        output_targets.append(highest_confidence_targets[cls][0] if highest_confidence_targets[cls] else None)

    return output_targets

if __name__ == "__main__":
  
    vision_args = parse_args()
    vision_module(vision_args)

