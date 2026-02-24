"""
Vision Module for Object Detection and Ghost Computation

This module provides:
- Camera initialization with calibration
- Board detection (black tape border)
- Cube/object detection by color
- Ghost/mirror position computation
- Coordinate conversion (pixel -> board mm -> robot mm)

The ghost logic is preserved EXACTLY from the original identifyVideo.py:
- Mirror is computed about a vertical divider line
- Mirror happens in INNER BOARD PIXEL COORDINATES
- Then converted to world coordinates
"""

import cv2
import numpy as np
import pickle
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any

from config import (
    BOARD, WARP, COLORS, CAMERA, SMOOTHING, DEBUG,
    MIN_CUBE_AREA_PX, CUBE_ASPECT_MIN, CUBE_ASPECT_MAX,
    ColorRange
)


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class Detection:
    """A detected object"""
    color: str                      # Color name (e.g., "BLUE")
    box_xywh: Tuple[int, int, int, int]  # Bounding box in inner coords (x, y, w, h)
    center_inner_px: Tuple[float, float]  # Center in inner pixel coords
    center_board_mm: Tuple[float, float]  # Center in board mm coords
    confidence: float = 1.0
    bgr_draw: Tuple[int, int, int] = (255, 255, 255)

    @property
    def cx(self) -> float:
        return self.center_inner_px[0]

    @property
    def cy(self) -> float:
        return self.center_inner_px[1]


@dataclass
class GhostPosition:
    """A ghost/mirrored position"""
    original: Detection             # The original detection
    ghost_box_xywh: Tuple[int, int, int, int]  # Ghost box in inner coords
    ghost_center_inner_px: Tuple[float, float]  # Ghost center in inner pixels
    ghost_center_board_mm: Tuple[float, float]  # Ghost center in board mm
    divider_x_px: float            # Divider position used for mirroring


# =============================================================================
# CAMERA AND CALIBRATION
# =============================================================================

class CameraManager:
    """Manages camera capture and calibration"""

    def __init__(self, calibration_dir: Optional[Path] = None):
        self.cap: Optional[cv2.VideoCapture] = None
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.map1: Optional[np.ndarray] = None
        self.map2: Optional[np.ndarray] = None
        self.calibration_dir = calibration_dir

    def load_calibration(self, calibration_dir: Path) -> bool:
        """Load camera calibration from pickle files"""
        try:
            matrix_path = calibration_dir / CAMERA.matrix_file
            dist_path = calibration_dir / CAMERA.dist_file

            with open(matrix_path, 'rb') as f:
                self.camera_matrix = pickle.load(f)
            with open(dist_path, 'rb') as f:
                self.dist_coeffs = pickle.load(f)

            print(f"Loaded camera calibration from {calibration_dir}")
            return True
        except Exception as e:
            print(f"Warning: Could not load calibration: {e}")
            print("Proceeding without distortion correction")
            return False

    def open(self, preferred_index: int = CAMERA.index) -> bool:
        """Open camera, trying multiple indices"""
        for idx in range(preferred_index, preferred_index + 5):
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA.height)
                self.cap = cap
                print(f"Opened camera at index {idx}")

                # Initialize undistortion maps if calibration loaded
                if self.camera_matrix is not None:
                    ret, frame = cap.read()
                    if ret:
                        self._init_undistort_maps(frame.shape)
                return True
            cap.release()

        print("Could not open any camera")
        return False

    def _init_undistort_maps(self, frame_shape: Tuple[int, ...]):
        """Initialize undistortion maps for fast remapping"""
        if self.camera_matrix is None:
            return

        h, w = frame_shape[:2]
        new_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.dist_coeffs, None, new_matrix, (w, h), cv2.CV_16SC2
        )

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read and undistort a frame"""
        if self.cap is None:
            return False, None

        ret, frame = self.cap.read()
        if not ret:
            return False, None

        # Apply undistortion if maps available
        if self.map1 is not None and self.map2 is not None:
            frame = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)

        return True, frame

    def release(self):
        """Release camera"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None


# =============================================================================
# GEOMETRY HELPERS
# =============================================================================

def order_points(pts: np.ndarray) -> np.ndarray:
    """Order 4 points as: TL, TR, BR, BL"""
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    rect[0] = pts[np.argmin(s)]      # Top-left
    rect[2] = pts[np.argmax(s)]      # Bottom-right
    rect[1] = pts[np.argmin(diff)]   # Top-right
    rect[3] = pts[np.argmax(diff)]   # Bottom-left
    return rect


def apply_homography(H: np.ndarray, pts_xy: List[Tuple[float, float]]) -> np.ndarray:
    """Apply homography to a list of points"""
    pts = np.asarray(pts_xy, dtype=np.float32).reshape(-1, 1, 2)
    out = cv2.perspectiveTransform(pts, H)
    return out.reshape(-1, 2)


def clamp_roi(x: int, y: int, w: int, h: int, W: int, H: int) -> Tuple[int, int, int, int]:
    """Clamp ROI to image bounds"""
    x = max(0, min(int(x), W - 1))
    y = max(0, min(int(y), H - 1))
    w = max(1, min(int(w), W - x))
    h = max(1, min(int(h), H - y))
    return x, y, w, h


# =============================================================================
# BOARD DETECTION
# =============================================================================

class BoardDetector:
    """Detects the black tape border of the board"""

    def __init__(self):
        self.last_corners: Optional[np.ndarray] = None
        self.roi: Optional[Tuple[int, int, int, int]] = None
        self.alpha = SMOOTHING.corner_alpha

    def detect(self, frame_bgr: np.ndarray, debug: bool = False
               ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Detect black border corners.

        Returns:
            (corners, debug_image): corners as 4x2 array [TL, TR, BR, BL], or None
        """
        H_img, W_img = frame_bgr.shape[:2]

        if self.roi is None:
            x0, y0, w0, h0 = 0, 0, W_img, H_img
        else:
            x0, y0, w0, h0 = clamp_roi(*self.roi, W_img, H_img)

        crop = frame_bgr[y0:y0+h0, x0:x0+w0]
        blurred = cv2.GaussianBlur(crop, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Detect black tape
        mask = cv2.inRange(hsv, (0, 0, 0), (179, 90, 90))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)

        edges = cv2.Canny(mask, 50, 150)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, (frame_bgr.copy() if debug else None)

        expected_ar = BOARD.outer_width_mm / BOARD.outer_height_mm
        roi_area = float(w0 * h0)

        best = None
        best_score = 1e9

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 0.01 * roi_area:
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue

            quad = order_points(approx.reshape(4, 2).astype(np.float32))
            xs, ys = quad[:, 0], quad[:, 1]
            bw = float(xs.max() - xs.min())
            bh = float(ys.max() - ys.min())
            if bw < 50 or bh < 50:
                continue

            ar = bw / bh if bh > 0 else 0
            ar = ar if ar >= 1 else 1 / ar
            exp = expected_ar if expected_ar >= 1 else 1 / expected_ar

            fill = area / roi_area
            if fill > 0.85:
                continue

            score = abs(ar - exp) * 2.0 + abs(fill - 0.20) * 1.0
            if score < best_score:
                best_score = score
                best = quad

        if best is None:
            if contours:
                cnt = max(contours, key=cv2.contourArea)
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect).astype(np.float32)
                corners_local = order_points(box)
            else:
                return None, (frame_bgr.copy() if debug else None)
        else:
            corners_local = best

        # Offset to full image coordinates
        corners = corners_local.copy()
        corners[:, 0] += x0
        corners[:, 1] += y0

        # Smooth corners
        if self.last_corners is None:
            self.last_corners = corners
        else:
            self.last_corners = (1 - self.alpha) * self.last_corners + self.alpha * corners

        # Debug visualization
        dbg = None
        if debug:
            dbg = frame_bgr.copy()
            cv2.rectangle(dbg, (x0, y0), (x0 + w0, y0 + h0), (255, 255, 0), 2)
            cv2.polylines(dbg, [self.last_corners.astype(int)], True, (0, 255, 0), 3)

        return self.last_corners.copy(), dbg

    def set_roi_from_corners(self, pad: int = 80):
        """Set ROI from last detected corners"""
        if self.last_corners is not None:
            xs = self.last_corners[:, 0]
            ys = self.last_corners[:, 1]
            x_min, x_max = int(xs.min()), int(xs.max())
            y_min, y_max = int(ys.min()), int(ys.max())
            self.roi = (x_min - pad, y_min - pad,
                        (x_max - x_min) + 2 * pad, (y_max - y_min) + 2 * pad)

    def reset(self):
        """Reset tracking state"""
        self.last_corners = None
        self.roi = None


# =============================================================================
# PERSPECTIVE WARP
# =============================================================================

def warp_board(frame_bgr: np.ndarray, outer_corners: np.ndarray
               ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Warp the board to a top-down view.

    Returns:
        (warped_outer, H_img2warp, H_warp2img): Warped image and homography matrices
    """
    dst = np.array([
        [0, 0],
        [WARP.warp_width - 1, 0],
        [WARP.warp_width - 1, WARP.warp_height - 1],
        [0, WARP.warp_height - 1]
    ], dtype=np.float32)

    H_img2warp = cv2.getPerspectiveTransform(outer_corners.astype(np.float32), dst)
    H_warp2img = np.linalg.inv(H_img2warp)
    warped = cv2.warpPerspective(frame_bgr, H_img2warp, (WARP.warp_width, WARP.warp_height))

    return warped, H_img2warp, H_warp2img


def extract_inner_board(warped_outer: np.ndarray) -> np.ndarray:
    """Extract the inner playing area from warped outer board"""
    margin = WARP.margin_px
    return warped_outer[margin:WARP.warp_height - margin,
                        margin:WARP.warp_width - margin].copy()


# =============================================================================
# DIVIDER DETECTION
# =============================================================================

class DividerDetector:
    """Detects the vertical divider line in the middle of the board"""

    def __init__(self):
        self.x_smooth: Optional[float] = None
        self.beta = SMOOTHING.divider_beta

    def detect(self, inner_bgr: np.ndarray) -> Optional[float]:
        """
        Detect vertical divider line.

        Returns:
            x_px: X position of divider in inner pixel coordinates, or None
        """
        h, w = inner_bgr.shape[:2]
        gray = cv2.cvtColor(inner_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        edges = cv2.Canny(gray, 40, 120)

        # Ignore top/bottom edges
        cut = int(0.03 * h)
        edges[:cut, :] = 0
        edges[h-cut:, :] = 0

        # Column energy
        col = edges.sum(axis=0).astype(np.float32)

        # Smooth
        k = max(11, (w // 80) | 1)
        col = cv2.GaussianBlur(col.reshape(1, -1), (k, 1), 0).ravel()

        # Search near center
        cx = w // 2
        win = int(0.18 * w)
        x0 = max(0, cx - win)
        x1 = min(w - 1, cx + win)

        region = col[x0:x1+1].copy()
        if region.max() < 1e-3:
            return self.x_smooth  # Return last known if detection fails

        # Find two peaks (tape edges)
        peaks = []
        region_work = region.copy()
        suppress = max(10, w // 60)

        for _ in range(2):
            i = int(np.argmax(region_work))
            peaks.append(x0 + i)
            lo = max(0, i - suppress)
            hi = min(region_work.size, i + suppress + 1)
            region_work[lo:hi] = 0

        peaks.sort()
        x_mid = (peaks[0] + peaks[1]) / 2.0

        # Smooth
        if self.x_smooth is None:
            self.x_smooth = x_mid
        else:
            self.x_smooth = (1 - self.beta) * self.x_smooth + self.beta * x_mid

        return self.x_smooth

    def reset(self):
        """Reset smoothing state"""
        self.x_smooth = None


# =============================================================================
# CUBE/OBJECT DETECTION
# =============================================================================

def create_color_mask(hsv: np.ndarray, color: ColorRange) -> np.ndarray:
    """Create mask for a color range"""
    mask = cv2.inRange(hsv, np.array(color.lower, np.uint8), np.array(color.upper, np.uint8))

    # Handle wraparound colors (like red)
    if color.lower2 is not None and color.upper2 is not None:
        mask2 = cv2.inRange(hsv, np.array(color.lower2, np.uint8), np.array(color.upper2, np.uint8))
        mask = cv2.bitwise_or(mask, mask2)

    return mask


def find_cubes_from_mask(mask: np.ndarray) -> List[Tuple[int, int, int, int, float, float]]:
    """
    Find cube bounding boxes from a color mask.

    Returns:
        List of (x, y, w, h, cx, cy) tuples
    """
    # Morphological cleanup
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_CUBE_AREA_PX:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        aspect = w / float(h) if h > 0 else 0
        if not (CUBE_ASPECT_MIN <= aspect <= CUBE_ASPECT_MAX):
            continue

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue

        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        boxes.append((x, y, w, h, cx, cy))

    return boxes


def detect_objects(inner_bgr: np.ndarray) -> List[Detection]:
    """
    Detect all colored cubes in the inner board image.

    Args:
        inner_bgr: BGR image of inner board area

    Returns:
        List of Detection objects
    """
    hsv = cv2.cvtColor(inner_bgr, cv2.COLOR_BGR2HSV)

    detections = []

    for color in COLORS:
        mask = create_color_mask(hsv, color)
        boxes = find_cubes_from_mask(mask)

        for (x, y, w, h, cx, cy) in boxes:
            # Convert center to board mm
            center_mm = (cx / WARP.px_per_mm, cy / WARP.px_per_mm)

            det = Detection(
                color=color.name,
                box_xywh=(x, y, w, h),
                center_inner_px=(cx, cy),
                center_board_mm=center_mm,
                bgr_draw=color.bgr_draw
            )
            detections.append(det)

    return detections


# =============================================================================
# GHOST/MIRROR COMPUTATION
# =============================================================================

def mirror_box_across_vertical_line(box_xywh: Tuple[int, int, int, int],
                                     x_div_px: float
                                     ) -> Tuple[int, int, int, int]:
    """
    Mirror a bounding box across a vertical line.

    EXACTLY preserves original identifyVideo.py behavior:
    - Mirror center X across x_div_px
    - Keep Y unchanged
    - Keep width/height unchanged

    Args:
        box_xywh: (x, y, w, h) bounding box
        x_div_px: X position of vertical divider line

    Returns:
        (xg, yg, w, h) mirrored bounding box
    """
    x, y, w, h = box_xywh

    # Original center
    cx = x + w / 2.0
    cy = y + h / 2.0

    # Mirror center across divider
    cx_ghost = 2.0 * x_div_px - cx

    # Reconstruct box from mirrored center
    xg = int(round(cx_ghost - w / 2.0))
    yg = int(round(cy - h / 2.0))

    return (xg, yg, w, h)


def clip_box_to_image(box_xywh: Tuple[int, int, int, int],
                      W: int, H: int) -> Tuple[int, int, int, int]:
    """Clip bounding box to image bounds"""
    x, y, w, h = box_xywh
    x = int(x)
    y = int(y)
    w = int(w)
    h = int(h)

    x2 = x + w
    y2 = y + h

    x = max(0, min(W - 1, x))
    y = max(0, min(H - 1, y))
    x2 = max(0, min(W, x2))
    y2 = max(0, min(H, y2))

    w = max(1, x2 - x)
    h = max(1, y2 - y)

    return (x, y, w, h)


def compute_ghost_position(detection: Detection,
                           divider_x_px: float,
                           inner_size: Tuple[int, int]) -> GhostPosition:
    """
    Compute the ghost (mirrored) position for a detection.

    The ghost is the mirrored location across the vertical divider line.
    Mirror computation happens in INNER BOARD PIXEL COORDINATES,
    exactly as in the original identifyVideo.py.

    Args:
        detection: The original detection
        divider_x_px: X position of divider in inner pixel coords
        inner_size: (width, height) of inner board image

    Returns:
        GhostPosition with all computed coordinates
    """
    inner_w, inner_h = inner_size

    # Mirror the bounding box
    ghost_box = mirror_box_across_vertical_line(detection.box_xywh, divider_x_px)
    ghost_box = clip_box_to_image(ghost_box, inner_w, inner_h)

    # Compute ghost center
    xg, yg, wg, hg = ghost_box
    ghost_cx = xg + wg / 2.0
    ghost_cy = yg + hg / 2.0

    # Convert to board mm
    ghost_mm = (ghost_cx / WARP.px_per_mm, ghost_cy / WARP.px_per_mm)

    return GhostPosition(
        original=detection,
        ghost_box_xywh=ghost_box,
        ghost_center_inner_px=(ghost_cx, ghost_cy),
        ghost_center_board_mm=ghost_mm,
        divider_x_px=divider_x_px
    )


# =============================================================================
# TARGET SELECTION
# =============================================================================

def choose_target(detections: List[Detection],
                  priority_colors: Optional[List[str]] = None,
                  prefer_closest_to: Optional[Tuple[float, float]] = None
                  ) -> Optional[Detection]:
    """
    Choose which detection to pick up.

    Args:
        detections: List of all detections
        priority_colors: If provided, prefer these colors (in order)
        prefer_closest_to: If provided, prefer detection closest to this point (mm)

    Returns:
        Selected Detection, or None if no detections
    """
    if not detections:
        return None

    candidates = detections.copy()

    # Filter by priority colors if specified
    if priority_colors:
        for color in priority_colors:
            matching = [d for d in candidates if d.color == color]
            if matching:
                candidates = matching
                break

    # If prefer closest, sort by distance
    if prefer_closest_to is not None:
        px, py = prefer_closest_to
        candidates.sort(key=lambda d: (
            (d.center_board_mm[0] - px) ** 2 +
            (d.center_board_mm[1] - py) ** 2
        ))

    return candidates[0] if candidates else None


# =============================================================================
# MAIN VISION PIPELINE
# =============================================================================

class VisionPipeline:
    """
    Complete vision pipeline for object detection and ghost computation.

    This class encapsulates all vision processing:
    - Camera capture and undistortion
    - Board detection and warping
    - Divider detection
    - Object detection
    - Ghost position computation
    """

    def __init__(self, calibration_dir: Optional[Path] = None):
        self.camera = CameraManager(calibration_dir)
        self.board_detector = BoardDetector()
        self.divider_detector = DividerDetector()

        # State
        self.last_frame: Optional[np.ndarray] = None
        self.last_warped: Optional[np.ndarray] = None
        self.last_inner: Optional[np.ndarray] = None
        self.H_warp2img: Optional[np.ndarray] = None
        self.divider_x_px: Optional[float] = None

    def initialize(self, calibration_dir: Optional[Path] = None) -> bool:
        """Initialize camera and load calibration"""
        if calibration_dir:
            self.camera.load_calibration(calibration_dir)

        return self.camera.open()

    def process_frame(self) -> Tuple[List[Detection], Optional[float]]:
        """
        Process one frame from camera.

        Returns:
            (detections, divider_x_px): List of detections and divider position
        """
        ret, frame = self.camera.read()
        if not ret or frame is None:
            return [], None

        self.last_frame = frame

        # Detect board
        corners, _ = self.board_detector.detect(frame, debug=DEBUG.show_windows)
        if corners is None:
            return [], None

        # Warp to top-down view
        warped, _, self.H_warp2img = warp_board(frame, corners)
        self.last_warped = warped

        # Extract inner area
        inner = extract_inner_board(warped)
        self.last_inner = inner

        # Detect divider
        self.divider_x_px = self.divider_detector.detect(inner)

        # Detect objects
        detections = detect_objects(inner)

        return detections, self.divider_x_px

    def compute_pick_point(self, detection: Detection) -> Tuple[float, float]:
        """
        Compute pick point in board mm coordinates.

        Args:
            detection: Detection to pick

        Returns:
            (x_mm, y_mm): Pick point in board coordinates
        """
        return detection.center_board_mm

    def compute_ghost_point(self, detection: Detection) -> Optional[Tuple[float, float]]:
        """
        Compute ghost (drop) point in board mm coordinates.

        Args:
            detection: Detection to compute ghost for

        Returns:
            (x_mm, y_mm): Ghost point in board coordinates, or None if no divider
        """
        if self.divider_x_px is None or self.last_inner is None:
            return None

        inner_h, inner_w = self.last_inner.shape[:2]
        ghost = compute_ghost_position(detection, self.divider_x_px, (inner_w, inner_h))

        return ghost.ghost_center_board_mm

    def get_all_ghosts(self, detections: List[Detection]) -> List[GhostPosition]:
        """Compute ghost positions for all detections"""
        if self.divider_x_px is None or self.last_inner is None:
            return []

        inner_h, inner_w = self.last_inner.shape[:2]
        ghosts = []

        for det in detections:
            ghost = compute_ghost_position(det, self.divider_x_px, (inner_w, inner_h))
            ghosts.append(ghost)

        return ghosts

    def release(self):
        """Release resources"""
        self.camera.release()

    def reset_tracking(self):
        """Reset all tracking state"""
        self.board_detector.reset()
        self.divider_detector.reset()
