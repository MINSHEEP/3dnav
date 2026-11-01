#!/usr/bin/env bash
# Read-only host load capture. Does not call ROS services or send commands.
set -euo pipefail

if [[ ! -d /proc/self ]]; then
  echo "请在小电脑的 Linux 终端运行此采集脚本。" >&2
  exit 2
fi

sample_count="${NAV_LOAD_SAMPLES:-60}"
if [[ ! "$sample_count" =~ ^[0-9]+$ ]] ||
   (( 10#$sample_count < 2 || 10#$sample_count > 300 )); then
  echo "NAV_LOAD_SAMPLES 必须为 2 到 300。" >&2
  exit 2
fi

umask 077
load_file="$(mktemp "${HOME}/nav_runtime_load_$(date +%Y%m%d_%H%M%S)_XXXXXX.log")"
trap 'printf "\n运行负载记录保存在：%s\n" "$load_file"' EXIT
echo "只采集运行负载，不控制机器狗；约 ${sample_count} 秒，可按 Ctrl+C 提前结束。"
{
  date --iso-8601=seconds
  printf '逻辑处理器数量：'
  getconf _NPROCESSORS_ONLN
  free -h
  printf '\n进程对应关系（只含PID、父PID和名称，不含命令参数）：\n'
  ps -eo pid,ppid,comm --sort=pid
  # Keep each frame's CPU/memory header and highest-CPU processes, not its tail.
  top -b -d 1 -n "$sample_count" -w 200 |
    awk '/^top -/ {remaining=35} remaining>0 {print; remaining--; fflush()}'
} | tee "$load_file"
