#!/usr/bin/env bash
# Narrow GRUB workaround for the ANV16S-41 camera power issue; never reboots.
set -euo pipefail
option='gpiolib_acpi.ignore_interrupt=AMDI0030:00@11'
config=/etc/default/grub.d/99-omx-laptop-camera.cfg
action="${1:---dry-run}"
case "$action" in --dry-run|--apply|--remove) ;; *) echo 'Use --dry-run, --apply or --remove' >&2; exit 2;; esac
if [[ "$(cat /sys/class/dmi/id/sys_vendor)" != Acer ||
      "$(cat /sys/class/dmi/id/product_name)" != 'Nitro ANV16S-41' ||
      "$(cat /sys/class/dmi/id/product_family)" != 'Acer Nitro V 16S AI' ]]; then
  echo 'Stopped: this workaround targets only Acer Nitro ANV16S-41.' >&2
  exit 1
fi
[[ -f /etc/default/grub && -f /boot/grub/grub.cfg && -d /etc/default/grub.d ]]
[[ -d /sys/bus/acpi/devices/AMDI0030:00 && -r /sys/module/gpiolib_acpi/parameters/ignore_interrupt ]]
command -v update-grub >/dev/null
echo "Model: $(cat /sys/class/dmi/id/product_name), BIOS: $(cat /sys/class/dmi/id/bios_version)"
echo "Boot option: $option"
echo "Managed file: $config"
echo 'Source: https://github.com/abduvaliy-engineer/acer-nitro-anv16s-camera-fix'
content=$(cat <<'EOF'
# Managed by the OMX project: Acer ANV16S-41 internal camera power workaround.
# Remove with: sudo bash scripts/fix_laptop_camera.sh --remove
case " ${GRUB_CMDLINE_LINUX_DEFAULT:-} " in
  *" gpiolib_acpi.ignore_interrupt=AMDI0030:00@11 "*) ;;
  *) GRUB_CMDLINE_LINUX_DEFAULT="${GRUB_CMDLINE_LINUX_DEFAULT:+$GRUB_CMDLINE_LINUX_DEFAULT }gpiolib_acpi.ignore_interrupt=AMDI0030:00@11" ;;
esac
EOF
)
if [[ -L "$config" ]] || { [[ -e "$config" ]] && ! cmp -s "$config" <(printf '%s\n' "$content"); }; then
  echo 'Stopped: existing configuration is not the file managed by this script.' >&2
  exit 1
fi
if [[ "$action" == --dry-run ]]; then
  printf '\n%s\n' "$content"
  echo 'Apply: sudo bash scripts/fix_laptop_camera.sh --apply'
  echo 'A reboot is required after successful application; this script does not reboot.'
  exit 0
fi
if [[ "$EUID" != 0 ]]; then
  echo 'Administrator authentication is required. Run this script with sudo in a terminal.' >&2
  exit 1
fi
if [[ "$action" == --remove && ! -e "$config" ]]; then
  echo 'No OMX camera workaround is installed.'
  exit 0
fi
backup=$(mktemp -d /var/backups/omx-camera.XXXXXX)
cp -p /boot/grub/grub.cfg "$backup/grub.cfg"
if [[ -e "$config" ]]; then cp -p "$config" "$backup/camera.cfg"; fi
if [[ "$action" == --apply ]]; then
  temporary=$(mktemp /etc/default/grub.d/.omx-camera.XXXXXX)
  printf '%s\n' "$content" > "$temporary"
  chmod 0644 "$temporary"
  mv "$temporary" "$config"
else
  rm -- "$config"
fi
if ! update-grub; then
  if [[ -e "$backup/camera.cfg" ]]; then
    cp -p "$backup/camera.cfg" "$config"
  else
    rm -f -- "$config"
  fi
  cp -p "$backup/grub.cfg" /boot/grub/grub.cfg
  echo "GRUB generation failed; restored configuration. Backup: $backup" >&2
  exit 1
fi
if [[ "$action" == --apply ]]; then
  grep -F -- "$option" /boot/grub/grub.cfg >/dev/null
  echo "OMX_CAMERA_BOOT_FIX_APPLIED; backup: $backup"
else
  echo "OMX_CAMERA_BOOT_FIX_REMOVED; backup: $backup"
fi
echo 'Reboot when your work is saved, then run: bash scripts/run_hand_control.sh'
