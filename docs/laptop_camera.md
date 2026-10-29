# Acer 내장 웹캠 복구

검증한 개발 노트북은 Acer Nitro ANV16S-41, BIOS V1.14, Ubuntu 22.04 / kernel 6.8.0-138이다.
복구 전에는 내장 카메라가 USB 목록과 /dev/video*에 없었다. UVC 드라이버 파일은 설치돼 있었으며
차단 설정은 발견되지 않았다.

동일 모델·BIOS에서 ACPI GPIO 11 처리로 내장 USB 카메라 전원이 꺼지는 문제가 보고됐다.
이 PC에서도 옵션 적용·재부팅 뒤 카메라가 나타났고 실제 프레임을 확인했다.
커널 로그의 Ignoring interrupt on pin 11, USB 0408:4035, /dev/video0·video1을 확인했다.
640×480 영상과 MediaPipe 손 추적, 실제 손 목표에 따른 Gazebo 관절 이동까지 검증했다.

- [수정 작성자의 설명과 재현 기록](https://github.com/abduvaliy-engineer/acer-nitro-anv16s-camera-fix)
- [상위 커널 수정 6da1f3437435](https://git.kernel.org/pub/scm/linux/kernel/git/andy/linux-gpio-intel.git/commit/?h=fixes&id=6da1f3437435c82eaa15b121022474bb358ffe20)

프로젝트의 scripts/fix_laptop_camera.sh는 외부 설치 스크립트를 실행하지 않는다.
정확한 모델·ACPI 장치·커널 매개변수 지원을 확인한 후, 아래 옵션 하나를 추가하는
/etc/default/grub.d/99-omx-laptop-camera.cfg를 작성하고 update-grub을 실행한다.

~~~text
gpiolib_acpi.ignore_interrupt=AMDI0030:00@11
~~~

기존 부팅 인자는 유지한다. /var/backups/omx-camera.*에 수정 전 설정을 백업하고
GRUB 생성이 실패하면 복구한다. 이 스크립트는 재부팅하지 않는다.

~~~bash
# 변경 내용 확인
bash scripts/fix_laptop_camera.sh --dry-run

# 열린 관리자 인증 터미널에서 적용하거나 직접 실행
sudo bash scripts/fix_laptop_camera.sh --apply
~~~

적용 로그에 OMX_CAMERA_BOOT_FIX_APPLIED가 나타난 후 작업을 저장하고 재부팅한다.
재부팅 뒤에는 아래 명령이 실제 프레임과 MediaPipe를 검사하고 Gazebo·MoveIt·손 제어를 시작한다.

~~~bash
bash scripts/run_hand_control.sh
~~~

한 손을 보여주고 Space로 활성화한 뒤 잠시 고정한다. 손목 이동은 팔, 같은 손의 pinch는 그리퍼다.
진단 결과는 artifacts/install/camera_report.json, 적용 로그는 laptop_camera_fix.log에 남는다.
손 추종은 실제 카메라 프레임과 JointState 이동을 확인하기 전까지 검증 완료로 표시하지 않는다.
현재 관측 기록은 artifacts/install/live_hand_verification.json에 있다.

되돌리기:

~~~bash
sudo bash scripts/fix_laptop_camera.sh --remove
~~~

제거도 다음 재부팅부터 적용된다.
