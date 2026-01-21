import cv2
import numpy as np
try:
    from pupil_apriltags import Detector
except ImportError as e:
    raise ImportError(
        "pupil-apriltags is not installed. "
        "Run: pip install pupil-apriltags"
    ) from e


AT = Detector(
    families="tag36h11",
    nthreads=2,
    quad_decimate=1.0,  
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0,
)


def crop_from_window_perc(image, window_perc):
    """window_perc = [x_min, y_min, x_max, y_max] in 0..100"""
    h, w = image.shape[:2]
    x0 = int(w * window_perc[0] / 100.0)
    y0 = int(h * window_perc[1] / 100.0)
    x1 = int(w * window_perc[2] / 100.0)
    y1 = int(h * window_perc[3] / 100.0)

    x0 = max(0, min(w-1, x0)); x1 = max(1, min(w, x1))
    y0 = max(0, min(h-1, y0)); y1 = max(1, min(h, y1))

    roi = image[y0:y1, x0:x1]
    return roi, (x0, y0, x1, y1)




def find_apriltags(image_bgr, tuning_params, camera_params=None, tag_size_m=None):
    """
    Returns:
      detections: list of dicts with id, corners_px, center_px, (optional) pose_R_t
      out_image: annotated full image
    """
    window = [
        tuning_params["x_min"], tuning_params["y_min"],
        tuning_params["x_max"], tuning_params["y_max"],
    ]
    roi_bgr, (x0, y0, x1, y1) = crop_from_window_perc(image_bgr, window)

    
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)


    estimate_pose = (camera_params is not None) and (tag_size_m is not None)
    dets = AT.detect(
        gray,
        estimate_tag_pose=estimate_pose,
        camera_params=camera_params,
        tag_size=tag_size_m,
    )


    out = image_bgr.copy()
    cv2.rectangle(out, (x0, y0), (x1, y1), (255, 0, 0), 2)

    results = []
    for d in dets:
        # d.corners are in ROI coordinates; shift to full-image coords
        corners = d.corners.copy()
        corners[:, 0] += x0
        corners[:, 1] += y0

        center = (float(d.center[0] + x0), float(d.center[1] + y0))

        pts = corners.astype(int).reshape(-1, 1, 2)
        cv2.polylines(out, [pts], True, (0, 255, 0), 2)
        cv2.circle(out, (int(center[0]), int(center[1])), 4, (0, 0, 255), -1)
        cv2.putText(out, f"id={d.tag_id}", (int(center[0]) + 5, int(center[1]) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        item = {
            "id": int(d.tag_id),
            "corners_px": corners.tolist(), 
            "center_px": center,
            "decision_margin": float(d.decision_margin),
        }

        if estimate_pose:
            item["pose_R"] = d.pose_R.tolist()
            item["pose_t"] = d.pose_t.tolist()  
        results.append(item)

    return results, out
