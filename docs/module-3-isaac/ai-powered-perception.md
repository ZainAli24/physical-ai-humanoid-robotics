---
sidebar_position: 5
title: AI-Powered Perception
description: Implement object detection, segmentation, pose estimation, and depth perception for robotic manipulation using Isaac Sim
---

# AI-Powered Perception

## Prerequisites

Before starting this chapter, you should have:

- ✅ Completed Isaac Platform Architecture chapter
- ✅ Understanding of computer vision fundamentals (CNNs, image classification)
- ✅ Isaac Sim 2023.1.0 or newer installed
- ✅ Python deep learning frameworks (PyTorch or TensorFlow)
- ✅ Familiarity with ROS 2 sensor messages
- ✅ NVIDIA RTX GPU (8GB+ VRAM for inference)

**Estimated Reading Time**: 30-35 minutes

---

## Introduction

Robots must **perceive** their environment to act intelligently. Unlike traditional rule-based vision (template matching, color thresholding), **AI-powered perception** uses deep learning models to understand complex, unstructured scenes. Isaac Sim provides:

- **Synthetic Data Generation**: Photorealistic images with automatic ground-truth labeling
- **Pre-trained Models**: NVIDIA-optimized models for object detection, segmentation, pose estimation
- **Domain Randomization**: Sim-to-real transfer techniques to reduce the reality gap
- **Sensor Simulation**: RGB, depth, segmentation, lidar for comprehensive perception

This chapter demonstrates how to:
1. Generate synthetic training data with Isaac Sim
2. Deploy pre-trained vision models (DOPE, FoundationPose, DetectNet)
3. Implement 6D pose estimation for manipulation
4. Process depth data for obstacle avoidance
5. Create perception pipelines for pick-and-place tasks

**Why AI-Powered Perception?**
- **Robustness**: Handles occlusions, lighting variations, object diversity
- **Generalization**: Trained models work on unseen object instances
- **Speed**: GPU-accelerated inference (30-240 FPS)
- **Integration**: Easy connection to manipulation planners (MoveIt, Isaac manipulators)

**Learning Objectives**:
1. Generate synthetic datasets with semantic labels in Isaac Sim
2. Deploy object detection models (YOLO, DetectNet, GroundingDINO)
3. Estimate 6D object poses using DOPE and FoundationPose
4. Process depth images for 3D scene reconstruction
5. Build complete perception-action pipelines for manipulation

---

## Synthetic Data Generation

### Why Synthetic Data?

Real-world data collection is:
- **Expensive**: Manual labeling costs $0.10-$1.00 per bounding box
- **Slow**: 1000-image dataset = weeks of collection + annotation
- **Limited**: Hard to cover edge cases (unusual poses, lighting, occlusions)

Isaac Sim generates **millions of labeled images** automatically:
- Automatic ground-truth (bounding boxes, segmentation masks, 3D poses)
- Infinite scene variations (lighting, camera angles, object arrangements)
- Rare event simulation (objects falling, extreme occlusions)

### Setting Up a Data Generation Scene

```python
# synthetic_data_gen.py
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.replicator.core import Writer, AnnotatorRegistry
import omni.replicator.core as rep
import random

# Initialize world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add camera
camera = rep.create.camera(position=(1.5, 1.5, 1.0), look_at=(0, 0, 0.5))

# Load object assets
assets_root = get_assets_root_path()
object_paths = [
    f"{assets_root}/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd",
    f"{assets_root}/Isaac/Props/YCB/Axis_Aligned/004_sugar_box.usd",
    f"{assets_root}/Isaac/Props/YCB/Axis_Aligned/005_tomato_soup_can.usd",
    f"{assets_root}/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd"
]

# Randomizer function
def randomize_scene():
    """Place objects in random positions with random orientations"""
    for i, obj_path in enumerate(object_paths):
        x = random.uniform(-0.3, 0.3)
        y = random.uniform(-0.3, 0.3)
        z = random.uniform(0.2, 0.8)

        rotation = (random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360))

        obj_prim = add_reference_to_stage(
            obj_path,
            f"/World/Object_{i}"
        )

        # Set transform
        rep.modify.pose(
            obj_prim,
            position=(x, y, z),
            rotation=rotation
        )

    # Randomize lighting
    light_intensity = random.uniform(500, 2000)
    rep.modify.attribute(
        "/World/defaultLight",
        "intensity",
        light_intensity
    )

# Register randomizer
rep.randomizer.register(randomize_scene)

# Define output writers
output_dir = "./synthetic_data"

# RGB writer
rgb_writer = rep.WriterRegistry.get("BasicWriter")
rgb_writer.initialize(
    output_dir=f"{output_dir}/rgb",
    rgb=True,
    bounding_box_2d_tight=True,  # Object detection labels
    semantic_segmentation=True,   # Segmentation masks
    distance_to_camera=True       # Depth images
)

# Generate 1000 frames
with rep.trigger.on_frame(num_frames=1000):
    rep.randomizer.randomize_scene()

rep.orchestrator.run()
print(f"Generated 1000 synthetic images in {output_dir}")
```

### Data Output Format

Isaac Replicator generates:

```
synthetic_data/
├── rgb/
│   ├── rgb_0000.png
│   ├── rgb_0001.png
│   └── ...
├── bounding_box_2d_tight/
│   ├── bbox_0000.json
│   └── ...
├── semantic_segmentation/
│   ├── seg_0000.png
│   └── ...
└── distance_to_camera/
    ├── depth_0000.npy
    └── ...
```

**Bounding Box JSON Format**:
```json
{
  "objects": [
    {
      "class": "cracker_box",
      "bbox": [120, 85, 245, 310],
      "confidence": 1.0,
      "occlusion": 0.15
    }
  ]
}
```

---

## Object Detection with Isaac Sim

### Using Pre-Trained DetectNet

**DetectNet** is NVIDIA's object detection model optimized for robotics.

```python
# detectnet_inference.py
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
import torch
import torchvision
import numpy as np

# Enable Isaac sensor extension
enable_extension("omni.isaac.sensor")

from omni.isaac.sensor import Camera

class ObjectDetector:
    def __init__(self, model_path="detectnet_v2.pth", confidence_threshold=0.5):
        """Initialize object detector"""
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.load(model_path).to(self.device)
        self.model.eval()
        self.confidence_threshold = confidence_threshold

    def detect(self, image):
        """
        Detect objects in image.

        Args:
            image: NumPy array (H, W, 3) in RGB format

        Returns:
            List of detections: [{"class": str, "bbox": [x1,y1,x2,y2], "confidence": float}]
        """
        # Preprocess
        image_tensor = torch.from_numpy(image).permute(2, 0, 1).float() / 255.0
        image_tensor = image_tensor.unsqueeze(0).to(self.device)

        # Inference
        with torch.no_grad():
            predictions = self.model(image_tensor)

        # Post-process
        detections = []
        boxes = predictions[0]["boxes"].cpu().numpy()
        scores = predictions[0]["scores"].cpu().numpy()
        labels = predictions[0]["labels"].cpu().numpy()

        for box, score, label in zip(boxes, scores, labels):
            if score > self.confidence_threshold:
                detections.append({
                    "class": self.idx_to_class(label),
                    "bbox": box.tolist(),
                    "confidence": float(score)
                })

        return detections

    def idx_to_class(self, idx):
        """Map class index to name"""
        class_names = ["background", "cracker_box", "sugar_box", "soup_can", "mustard_bottle"]
        return class_names[idx]

# Usage in Isaac Sim
world = World()
camera = Camera(
    prim_path="/World/Camera",
    position=[1.5, 0, 1.0],
    frequency=30  # 30 Hz
)

detector = ObjectDetector("detectnet_v2.pth")

for _ in range(100):
    world.step()

    # Get camera image
    rgb_image = camera.get_rgba()[:, :, :3]  # Drop alpha channel

    # Detect objects
    detections = detector.detect(rgb_image)

    for det in detections:
        print(f"Detected {det['class']} at {det['bbox']} (confidence: {det['confidence']:.2f})")
```

---

## 6D Pose Estimation

### What is 6D Pose?

**6D Pose** = 3D position (x, y, z) + 3D orientation (roll, pitch, yaw)

Critical for:
- **Grasping**: Align gripper with object orientation
- **Placement**: Put objects in specific poses
- **Manipulation**: Track object movement during contact

### DOPE (Deep Object Pose Estimation)

**DOPE** estimates 6D poses from RGB images using CNN keypoints.

```python
# dope_pose_estimation.py
from omni.isaac.dope import DOPE
from omni.isaac.core import World
import numpy as np

class PoseEstimator:
    def __init__(self):
        """Initialize DOPE model"""
        self.dope = DOPE()
        self.dope.load_model("dope_cracker_box.pth")

    def estimate_pose(self, rgb_image, camera_intrinsics):
        """
        Estimate 6D pose from RGB image.

        Args:
            rgb_image: (H, W, 3) RGB image
            camera_intrinsics: 3x3 camera matrix

        Returns:
            {
                "position": [x, y, z],
                "rotation": [qw, qx, qy, qz],  # Quaternion
                "confidence": float
            }
        """
        # Run DOPE inference
        detections = self.dope.infer(rgb_image)

        if len(detections) == 0:
            return None

        # Take highest-confidence detection
        best_det = max(detections, key=lambda d: d["score"])

        # Extract 2D keypoints
        keypoints_2d = best_det["projected_points"]

        # PnP (Perspective-n-Point) to recover 3D pose
        pose_3d = self.solve_pnp(keypoints_2d, camera_intrinsics)

        return {
            "position": pose_3d["translation"],
            "rotation": pose_3d["quaternion"],
            "confidence": best_det["score"]
        }

    def solve_pnp(self, keypoints_2d, camera_matrix):
        """Solve PnP problem to get 6D pose"""
        import cv2

        # 3D model keypoints (object-centric coordinates)
        object_points = np.array([
            [0.0, 0.0, 0.0],      # Center
            [0.1, 0.0, 0.0],      # +X
            [0.0, 0.1, 0.0],      # +Y
            [0.0, 0.0, 0.1],      # +Z
            # ... (more keypoints)
        ], dtype=np.float32)

        image_points = np.array(keypoints_2d, dtype=np.float32)

        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            distCoeffs=None
        )

        if not success:
            return None

        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

        return {
            "translation": tvec.flatten().tolist(),
            "quaternion": quaternion
        }

    def rotation_matrix_to_quaternion(self, R):
        """Convert 3x3 rotation matrix to quaternion"""
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            # Handle degenerate cases
            # (simplified for brevity)
            w, x, y, z = 1, 0, 0, 0

        return [w, x, y, z]

# Usage
world = World()
camera = world.scene.add_camera("/World/Camera")
pose_estimator = PoseEstimator()

for _ in range(100):
    world.step()

    rgb_image = camera.get_rgba()[:, :, :3]
    camera_matrix = camera.get_intrinsics_matrix()

    pose = pose_estimator.estimate_pose(rgb_image, camera_matrix)

    if pose:
        print(f"Object at position: {pose['position']}")
        print(f"Object orientation (quaternion): {pose['rotation']}")
```

---

## Depth Perception

### Processing Depth Images

```python
# depth_processing.py
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
import numpy as np

enable_extension("omni.isaac.sensor")
from omni.isaac.sensor import Camera

class DepthProcessor:
    def __init__(self, camera):
        self.camera = camera

    def get_depth_map(self):
        """Get depth map from camera (in meters)"""
        depth_image = self.camera.get_depth()  # Shape: (H, W)
        return depth_image

    def find_nearest_obstacle(self, depth_map):
        """Find closest obstacle in depth map"""
        min_depth = np.min(depth_map[depth_map > 0])  # Ignore zero (invalid)
        return min_depth

    def segment_foreground_background(self, depth_map, threshold=2.0):
        """Separate foreground (< threshold) from background"""
        foreground_mask = depth_map < threshold
        return foreground_mask

    def point_cloud_from_depth(self, depth_map, camera_intrinsics):
        """
        Convert depth map to 3D point cloud.

        Args:
            depth_map: (H, W) depth in meters
            camera_intrinsics: 3x3 camera matrix

        Returns:
            (N, 3) point cloud in camera frame
        """
        H, W = depth_map.shape
        fx, fy = camera_intrinsics[0, 0], camera_intrinsics[1, 1]
        cx, cy = camera_intrinsics[0, 2], camera_intrinsics[1, 2]

        # Create pixel grid
        u, v = np.meshgrid(np.arange(W), np.arange(H))

        # Back-project to 3D
        z = depth_map
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # Stack into (H*W, 3) array
        points = np.stack([x, y, z], axis=-1).reshape(-1, 3)

        # Filter invalid points
        valid_mask = points[:, 2] > 0
        points = points[valid_mask]

        return points

# Usage
world = World()
camera = Camera(
    prim_path="/World/Camera",
    position=[2.0, 0, 1.0]
)

processor = DepthProcessor(camera)

for _ in range(100):
    world.step()

    depth_map = processor.get_depth_map()

    # Find nearest obstacle
    min_dist = processor.find_nearest_obstacle(depth_map)
    print(f"Nearest obstacle: {min_dist:.2f}m")

    # Generate point cloud
    intrinsics = camera.get_intrinsics_matrix()
    point_cloud = processor.point_cloud_from_depth(depth_map, intrinsics)
    print(f"Point cloud: {point_cloud.shape[0]} points")
```

---

## Complete Perception Pipeline: Pick and Place

### Integration with Manipulation

```python
# pick_and_place_perception.py
from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

class PickAndPlaceController:
    def __init__(self):
        # Initialize world
        self.world = World()
        self.world.scene.add_default_ground_plane()

        # Add robot
        self.robot = self.world.scene.add(Franka(
            prim_path="/World/Franka",
            name="franka"
        ))

        # Add camera mounted on robot
        from omni.isaac.sensor import Camera
        self.camera = Camera(
            prim_path="/World/Franka/panda_hand/camera",
            position=[0, 0, 0.1],  # Relative to end-effector
            orientation=[0.707, 0, 0.707, 0]  # Look downward
        )

        # Initialize perception
        self.object_detector = ObjectDetector("detectnet.pth")
        self.pose_estimator = PoseEstimator()

    def detect_and_pick(self):
        """Main perception-action loop"""
        # 1. Move to observation pose
        self.robot.move_to_observation_pose()
        self.world.step()

        # 2. Capture image
        rgb_image = self.camera.get_rgba()[:, :, :3]

        # 3. Detect objects
        detections = self.object_detector.detect(rgb_image)

        if len(detections) == 0:
            print("No objects detected")
            return False

        # 4. Estimate pose of first detected object
        target_object = detections[0]
        pose_6d = self.pose_estimator.estimate_pose(
            rgb_image,
            self.camera.get_intrinsics_matrix()
        )

        if pose_6d is None:
            print("Pose estimation failed")
            return False

        # 5. Plan grasp
        grasp_pose = self.compute_grasp_pose(pose_6d)

        # 6. Execute grasp
        success = self.robot.grasp_at_pose(grasp_pose)

        if success:
            print(f"Successfully grasped {target_object['class']}")

            # 7. Place at target location
            place_pose = [0.5, 0.3, 0.2]  # Target position
            self.robot.place_at_pose(place_pose)
            print("Object placed")

        return success

    def compute_grasp_pose(self, object_pose):
        """Convert object 6D pose to grasp pose"""
        # Simple top-down grasp
        grasp_position = object_pose["position"]
        grasp_position[2] += 0.1  # Approach from above

        grasp_orientation = [1, 0, 0, 0]  # Vertical gripper

        return {
            "position": grasp_position,
            "orientation": grasp_orientation
        }

    def run(self, num_cycles=10):
        """Run pick-and-place for multiple objects"""
        for i in range(num_cycles):
            print(f"\n=== Cycle {i+1}/{num_cycles} ===")
            success = self.detect_and_pick()

            if not success:
                print("Cycle failed, retrying...")
                continue

# Run the pipeline
controller = PickAndPlaceController()
controller.run(num_cycles=5)
```

---

## Domain Randomization for Sim-to-Real Transfer

### Why Randomization?

Models trained on synthetic data often fail on real robots due to the **reality gap** (differences in textures, lighting, physics).

**Domain Randomization** bridges this gap by training on **diverse** synthetic data:
- Random textures for objects and backgrounds
- Random lighting (intensity, color, shadows)
- Random camera parameters (position, exposure, noise)
- Random object positions and orientations

### Implementing Domain Randomization

```python
# domain_randomization.py
import omni.replicator.core as rep
import random

def randomize_textures():
    """Apply random materials to objects"""
    objects = ["/World/Object_0", "/World/Object_1", "/World/Object_2"]

    for obj_path in objects:
        # Random PBR material
        rep.randomizer.materials(
            obj_path,
            metallic=random.uniform(0, 1),
            roughness=random.uniform(0.2, 1.0),
            base_color=(
                random.uniform(0, 1),
                random.uniform(0, 1),
                random.uniform(0, 1)
            )
        )

def randomize_lighting():
    """Randomize scene lighting"""
    intensity = random.uniform(500, 3000)
    color_temp = random.randint(2000, 8000)  # Kelvin

    rep.modify.attribute("/World/Light", "intensity", intensity)
    rep.modify.attribute("/World/Light", "colorTemperature", color_temp)

def randomize_camera():
    """Add realistic camera noise and distortion"""
    camera_path = "/World/Camera"

    # Motion blur
    rep.modify.attribute(camera_path, "motion:motionBlurScale", random.uniform(0, 0.5))

    # Exposure
    exposure = random.uniform(0.5, 2.0)
    rep.modify.attribute(camera_path, "exposure", exposure)

# Register randomizers
with rep.trigger.on_frame():
    randomize_textures()
    randomize_lighting()
    randomize_camera()
```

---

## Key Takeaways

- **Synthetic data generation** in Isaac Sim provides unlimited labeled training data with automatic ground-truth
- **Object detection models** (DetectNet, YOLO) enable robots to identify objects in cluttered scenes
- **6D pose estimation** (DOPE, FoundationPose) is critical for precise manipulation tasks
- **Depth perception** enables obstacle avoidance and 3D scene understanding
- **Complete perception pipelines** integrate detection, pose estimation, and manipulation for autonomous pick-and-place
- **Domain randomization** bridges the sim-to-real gap by training on diverse synthetic data

AI-powered perception transforms robots from blind actuators into intelligent agents that understand and interact with complex, unstructured environments.

---

**Previous Chapter**: [Isaac Platform Architecture](./isaac-sdk-overview.md)
**Next Chapter**: [Advanced RL and Sim-to-Real Transfer](./reinforcement-learning.md)

In the next chapter, we'll explore advanced reinforcement learning techniques and strategies for transferring policies trained in simulation to physical robots.
