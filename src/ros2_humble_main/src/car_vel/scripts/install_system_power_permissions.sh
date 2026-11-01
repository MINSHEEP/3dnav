#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "请使用 sudo 运行：sudo bash $0 <登录用户名>" >&2
    exit 1
fi

TARGET_USER="${1:-${SUDO_USER:-}}"
if [[ -z "${TARGET_USER}" || "${TARGET_USER}" == "root" ]]; then
    echo "请指定运行机器人程序的普通用户，例如：sudo bash $0 a" >&2
    exit 1
fi
if [[ ! "${TARGET_USER}" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]]; then
    echo "用户名格式无效：${TARGET_USER}" >&2
    exit 1
fi
if ! /usr/bin/id "${TARGET_USER}" >/dev/null 2>&1; then
    echo "系统中不存在用户：${TARGET_USER}" >&2
    exit 1
fi

for required in /usr/bin/systemctl /usr/bin/sudo /usr/sbin/visudo; do
    if [[ ! -x "${required}" ]]; then
        echo "缺少必要程序：${required}" >&2
        exit 1
    fi
done

SUDOERS_PATH="/etc/sudoers.d/90-robot-system-power"
TEMP_FILE="$(mktemp /tmp/robot-system-power.XXXXXX)"
trap 'rm -f -- "${TEMP_FILE}"' EXIT

printf '%s ALL=(root) NOPASSWD: /usr/bin/systemctl reboot, /usr/bin/systemctl poweroff\n' \
    "${TARGET_USER}" >"${TEMP_FILE}"
chmod 0440 "${TEMP_FILE}"
/usr/sbin/visudo -cf "${TEMP_FILE}"
/usr/bin/install -o root -g root -m 0440 "${TEMP_FILE}" "${SUDOERS_PATH}"
/usr/sbin/visudo -cf "${SUDOERS_PATH}"

echo "已为用户 ${TARGET_USER} 配置两项固定权限："
echo "  /usr/bin/systemctl reboot"
echo "  /usr/bin/systemctl poweroff"
echo "没有授予 shell、通配 systemctl 或其他无密码 sudo 权限。"
