#!/usr/bin/env bash
# 本地预览 Go2 教材 —— http://127.0.0.1:8000
set -euo pipefail
cd "$(dirname "$0")"

if ! command -v mkdocs >/dev/null 2>&1; then
  echo "[!] 未检测到 mkdocs,先装依赖:"
  echo "    pip install -r requirements.txt"
  exit 1
fi

mkdocs serve -a 127.0.0.1:8000
