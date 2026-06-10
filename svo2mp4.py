import pyzed.sl as sl
import cv2

svo_path = "/home/volcani/workspaces/zedx_ws/zed_odom_recordings/7_6_26/c10b/HD1080_SN49953233_08-43-37.svo2"
out_path = svo_path.replace(".svo2", ".mp4")

init = sl.InitParameters()
init.set_from_svo_file(svo_path)
init.svo_real_time_mode = False

cam = sl.Camera()
status = cam.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print("Failed to open SVO:", status)
    exit(1)

info = cam.get_camera_information()
w = info.camera_configuration.resolution.width
h = info.camera_configuration.resolution.height
fps = info.camera_configuration.fps
total = cam.get_svo_number_of_frames()

print(f"Resolution: {w*2}x{h} @ {fps}fps, Total frames: {total}")

out = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w*2, h))
left = sl.Mat()
right = sl.Mat()
frame = 0

while True:
    if cam.grab() != sl.ERROR_CODE.SUCCESS:
        break
    cam.retrieve_image(left, sl.VIEW.LEFT)
    cam.retrieve_image(right, sl.VIEW.RIGHT)
    import numpy as np
    combined = np.concatenate([left.get_data()[:, :, :3], right.get_data()[:, :, :3]], axis=1)
    out.write(combined)
    frame += 1
    if frame % 30 == 0:
        print(f"Progress: {frame}/{total} frames ({100*frame//total}%)")

cam.close()
out.release()
print(f"Done! Saved to {out_path}")
