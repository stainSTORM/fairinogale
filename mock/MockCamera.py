"""A camera on the mock robot's gripper: it renders the AprilTags of a made-up scene.

With robot.mock and a camera whose source is "mock", the whole chain runs without hardware: the
mock arm drives to the look point, this camera draws the station's tags as seen from there, the
detector finds them and the points are corrected. mock_shift moves the stations after "teaching",
so that there is something to correct.
"""

import cv2
import numpy as np

from vision import DICTIONARY, Camera, T_from_pose


class MockCamera(Camera):
    def __init__(self, name: str, spec: dict, tcp_pose, tag_size: float):
        super().__init__(name, spec)
        self.tcp_pose, self.tag_size = tcp_pose, tag_size  # tcp_pose(): the arm's pose right now
        self.width, self.height = spec.get("width", 1280), spec.get("height", 720)
        self.scene: dict[int, np.ndarray] = {}  # tag id -> 4x4 pose of the tag in the user frame
        self.sizes: dict[int, float] = {}       # tags of another size than tag_size

    def place(self, ids, look_pose, distance: float = 220.0, spacing: float = 80.0, row: float = 0.0,
              size: float | None = None):
        """Puts tags side by side in front of the camera as it stands at look_pose, tilted by 20 deg."""
        tilt = T_from_pose([0, row, distance, 180 + 20, 0, 0])  # the tags face the camera
        for k, i in enumerate(ids):
            offset = np.eye(4)
            offset[0, 3] = (k - (len(ids) - 1) / 2) * spacing
            self.scene[i] = T_from_pose(look_pose) @ self.mount @ tilt @ offset
            if size:
                self.sizes[i] = size

    def move(self, ids, D: np.ndarray) -> None:
        """Moves tags by a 4x4 transform given in the user frame."""
        for i in ids:
            self.scene[i] = D @ self.scene[i]

    def grab(self) -> np.ndarray:
        K = self.matrix(self.width, self.height)
        to_camera = np.linalg.inv(T_from_pose(self.tcp_pose()) @ self.mount)
        fine = 3  # drawn three times finer and averaged down, so that edges are not jagged
        size = (self.width * fine, self.height * fine)
        image = np.full(size[::-1], 150, np.uint8)
        cells = 10  # 8 cells of tag and one cell of white margin on each side
        for i, pose in self.scene.items():
            h = self.sizes.get(i, self.tag_size) / 8 * cells / 2
            corners = (to_camera @ pose @ np.array([[-h, h, 0, 1], [h, h, 0, 1], [h, -h, 0, 1],
                                                    [-h, -h, 0, 1]], dtype=float).T).T[:, :3]
            if (corners[:, 2] < 10).any():
                continue  # behind the camera
            pixels = (corners @ K.T)[:, :2] / corners[:, 2:3] * fine + (fine - 1) / 2
            picture = cv2.copyMakeBorder(DICTIONARY.generateImageMarker(i, 8 * 40), 40, 40, 40, 40,
                                         cv2.BORDER_CONSTANT, value=255)
            a, b = -0.5, picture.shape[0] - 0.5  # the picture's outer edges (pixel centres are whole numbers)
            warp = cv2.getPerspectiveTransform(np.float32([[a, a], [b, a], [b, b], [a, b]]),
                                               np.float32(pixels))
            mask = cv2.warpPerspective(np.full_like(picture, 255), warp, size, flags=cv2.INTER_NEAREST)
            drawn = cv2.warpPerspective(picture, warp, size, flags=cv2.INTER_NEAREST)
            image = np.where(mask > 127, drawn, image)
        image = cv2.resize(image, (self.width, self.height), interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(cv2.GaussianBlur(image, (5, 5), 0.8), cv2.COLOR_GRAY2BGR)
