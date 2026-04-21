#!/usr/bin/env bash
# 构建静态站点到 ./site/
set -euo pipefail
cd "$(dirname "$0")"
mkdocs build --clean
echo "[ok] 构建完成,入口:./site/index.html"
