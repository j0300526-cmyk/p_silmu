#!/usr/bin/env python3
"""Probe a local V4L2 camera and MediaPipe. Does not store video or images."""
import argparse
from datetime import datetime
import glob
import json
import os
from pathlib import Path
import sys
from zoneinfo import ZoneInfo


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='auto')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    devices = ([args.device] if args.device != 'auto'
               else sorted(glob.glob('/dev/video*'), key=lambda p: int(p.removeprefix('/dev/video'))))
    report = {'checked_at': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
              'ready': False, 'devices': devices, 'attempts': []}
    if not devices:
        report['reason'] = 'No /dev/video* device. Connect a USB webcam and retry.'
        try:
            model = Path('/sys/class/dmi/id/product_name').read_text().strip()
            if model == 'Nitro ANV16S-41':
                option = 'gpiolib_acpi.ignore_interrupt=AMDI0030:00@11'
                report.update(
                    reason='Built-in camera absent. This laptop matches a reported Linux camera power issue.',
                    laptop_model=model, suggested_boot_option=option,
                    option_in_current_boot=option in Path('/proc/cmdline').read_text().split(),
                    grub_fix_staged=Path('/etc/default/grub.d/99-omx-laptop-camera.cfg').is_file(),
                    fix_script='sudo bash scripts/fix_laptop_camera.sh --apply',
                    note='The boot workaround requires a reboot after application; camera operation remains unverified.')
        except OSError:
            pass
    else:
        try:
            import cv2
            import mediapipe as mp
            report['opencv'] = cv2.__version__
            report['mediapipe'] = mp.__version__
            if not hasattr(mp, 'solutions'):
                raise RuntimeError('MediaPipe solutions API unavailable')
            for device in devices:
                attempt = {'device': device, 'readable': os.access(device, os.R_OK | os.W_OK)}
                report['attempts'].append(attempt)
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                try:
                    if not cap.isOpened():
                        attempt['reason'] = 'V4L2 open failed'
                        continue
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    frame = None
                    for _ in range(5):
                        ok, frame = cap.read()
                        if not ok or frame is None:
                            break
                    if not ok or frame is None:
                        attempt['reason'] = 'Video frames unavailable'
                        continue
                    with mp.solutions.hands.Hands(max_num_hands=2) as hands:
                        result = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                    report.update(ready=True, selected_device=device,
                                  frame_size=[int(frame.shape[1]), int(frame.shape[0])],
                                  detected_hands=len(result.multi_hand_landmarks or []))
                    break
                finally:
                    cap.release()
            if not report['ready']:
                report['reason'] = 'No readable video stream. Check permissions or another app using the camera.'
        except Exception as exc:
            report['reason'] = str(exc)
    output = json.dumps(report, ensure_ascii=False, indent=2)
    print(output)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output + '\n')
    return 0 if report['ready'] else 2


if __name__ == '__main__':
    sys.exit(main())
