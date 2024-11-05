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
import time
import json

current_directory = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current_directory)
sys.path.append(parent_directory)

from groundingdino.util.inference import Model
from segment_anything import SamPredictor
from MobileSAM.setup_mobile_sam import setup_model

class AzureKinect:
    def __init__(self):
        self.config = o3d.io.AzureKinectSensorConfig()
        self.align_depth_to_color = 1

    def start(self, device):
        self.sensor = o3d.io.AzureKinectSensor(self.config)
        if not self.sensor.connect(device):
                raise RuntimeError('Failed to connect to sensor')

    def frames(self):
        while 1:
            rgbd = self.sensor.capture_frame(self.align_depth_to_color)
            if rgbd is None:
                continue
            color, depth = np.asarray(rgbd.color).astype(np.uint8), np.asarray(rgbd.depth).astype(np.float32) / 1000.0
            return color, depth
    
    def disconnect(self):
        self.sensor.disconnect()


def parse_args(arg_list=None):
    if arg_list is None:
        argv = rospy.myargv()[1:]  # Extract only user-supplied arguments, ignoring those added by ROS
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

# Prompting SAM with detected boxes
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


def get_detections(image, depth, binary_mask_all, intrinsic):
    color_trans = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    from pupil_apriltags import Detector

    at_detector = Detector(
    families="tagStandard41h12",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
    )

    color_trans = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    at_detector.detect(color_trans)

    detections = at_detector.detect(color_trans, estimate_tag_pose=True, camera_params=[613.4286499023438,613.3960571289062,639.1371459960938,365.3634948730469],tag_size=0.14)

    return detections


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

  # 0 denotes the camera close to robot, 1 denotes the camera away from robot
  intrinsic0 = o3d.camera.PinholeCameraIntrinsic(1280, 720, 607.6044921875,607.41522216796875,640.840576171875,364.01950073242188) 
  intrinsic1 = o3d.camera.PinholeCameraIntrinsic(1280, 720, 612.41802978515625,612.31890869140625,640.6007080078125,364.61968994140625)
  intrinsic_array = [intrinsic0, intrinsic1]
  camera_array = ['camera0_transformation_matrix.json','camera1_transformation_matrix.json']

#   device = [0, 1]
  device = [0]
  cam = AzureKinect()     
  imgs, deps = [], []
  for d in device:
      cam.start(d)
      intrinsic = intrinsic_array[d]

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
      camera_save_dir = camera_array[d]
      
      try:
              target_all = return_target_coordinates(cam, grounding_dino_model,BOX_THRESHOLD,TEXT_THRESHOLD,
                                          NMS_THRESHOLD, sam_predictor, mask_annotator, box_annotator,
                                          intrinsic, CLASSES, camera_save_dir)            
              
      except rospy.ROSInterruptException:
          print("ROS shutdown, stopping...")



def return_target_coordinates(cam, grounding_dino_model,BOX_THRESHOLD,TEXT_THRESHOLD,
                              NMS_THRESHOLD, sam_predictor, mask_annotator, box_annotator,
                              intrinsic, CLASSES, camera_save_dir):
    
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

    print(f'Width: {intrinsic.width}, Height: {intrinsic.height}')
    print(f'fx: {intrinsic.intrinsic_matrix[0, 0]}, fy: {intrinsic.intrinsic_matrix[1, 1]}')
    print(f'cx: {intrinsic.intrinsic_matrix[0, 2]}, cy: {intrinsic.intrinsic_matrix[1, 2]}\n')

    detections = get_detections(image, depth, binary_mask_all, intrinsic)

    calibrate_camera(detections, camera_save_dir)

    return

def calibrate_camera(detections, camera_save_dir):
    M_CA = np.eye(4)
    M_CA[:3, :3] = detections[0].pose_R
    M_CA[:3, 3] = detections[0].pose_t.squeeze()   

    # transformation from the april tap to the robot base
    T_RA = [0.04,0.46, 0]
    R_RA = [[0,1,0],
            [1,0,0],
            [0,0,-1]]

    M_RA = np.zeros((4, 4))
    M_RA[:3, :3] = R_RA    
    M_RA[:3, 3] = T_RA     
    M_RA[3, 3] = 1         

    M_RC = M_RA @ np.linalg.inv(M_CA)

    # check M_RC and check the .json file is updated
    import pdb; pdb.set_trace()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    target_dir = os.path.join(script_dir, 'camera_to_robot_transformation')
    os.makedirs(target_dir, exist_ok=True)
    file_path = os.path.join(target_dir, camera_save_dir)
    print(f'File path: {file_path}')

    with open(file_path, 'w') as f:
        json.dump(M_RC.tolist(), f)

    print("Transformation matrix saved successfully.")      
    print("Current Working Directory:", os.getcwd())
    return


if __name__ == "__main__":
    vision_args = parse_args()
    vision_module(vision_args)

