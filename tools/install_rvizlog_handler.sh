#!/usr/bin/env bash
# Register only rviz_map_*.bag files for double-click opening on this desktop.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MIME_DIR="${HOME}/.local/share/mime/packages"
APP_DIR="${HOME}/.local/share/applications"
mkdir -p "${MIME_DIR}" "${APP_DIR}"

cat >"${MIME_DIR}/uav-rvizlog.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<mime-info xmlns="http://www.freedesktop.org/standards/shared-mime-info">
  <mime-type type="application/x-uav-rvizlog">
    <comment>RViz 地图快照</comment>
    <glob pattern="rviz_map_*.bag" weight="90"/>
  </mime-type>
</mime-info>
XML

cat >"${APP_DIR}/uav-rvizlog.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=RViz 地图快照
Comment=打开并旋转查看无人机地图快照
Exec=${PROJECT_DIR}/tools/open_rvizlog.sh %f
MimeType=application/x-uav-rvizlog;
NoDisplay=true
Terminal=false
Categories=Science;Graphics;
EOF

update-mime-database "${HOME}/.local/share/mime"
if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "${APP_DIR}"
fi
xdg-mime default uav-rvizlog.desktop application/x-uav-rvizlog
echo "已注册双击打开 rviz_map_*.bag：$(xdg-mime query default application/x-uav-rvizlog)"
