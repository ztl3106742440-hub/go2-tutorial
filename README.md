<!-- 小co改动 2026-09-16：按用户要求整理仓库定位、功能、克隆运行说明和合作入口。 -->
# Go2 ROS2 Tutorial · 机器狗开发教程与项目导航

Unitree Go2 + ROS 2 中文实验教程，涵盖环境通信、运动接口、传感器、SLAM、导航、语音与视觉，并提供机器人项目分类导航。

[在线阅读](https://ztl3106742440-hub.github.io/go2-tutorial/) · [项目分类与合作](docs/projects/index.md)

## 从哪里开始

| 需求 | 入口 |
| --- | --- |
| 学习 Go2 / ROS 2 二次开发 | `docs/00-overview/` 与教材章节 |
| 获取二维建图导航代码 | [go2-ros2-navigation](https://github.com/ztl3106742440-hub/go2-ros2-navigation) |
| 获取实机接口与 Python SDK 示例 | [go2-ros2-sdk](https://github.com/ztl3106742440-hub/go2-ros2-sdk) |
| 了解巡检、视觉、语音和移动端 | [项目分类](docs/projects/index.md) |

## 本地预览

完成下方克隆命令后执行：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m mkdocs serve
# 构建静态站点
python -m mkdocs build
```

浏览器打开终端显示的本地地址，默认 http://127.0.0.1:8000 。

## 内容边界

教材源文件位于 `docs/`，配置为 `mkdocs.yml`。项目介绍明确区分公开源码、私有系统、历史验证和待实现能力。内部设备配置、凭据、原始采集数据与完整商业/课程交付资料不在此公开。

## 获取与更新

安装 Git 后执行：

```bash
git clone https://github.com/ztl3106742440-hub/go2-tutorial.git
cd go2-tutorial
# 在没有本地未提交改动时获取更新
git pull --ff-only
```

保留自己的修改：先 `git switch -c my-experiment`，再 `git add <修改的文件>`、`git commit -m "说明修改目的"`。没有本仓库写权限时先 Fork，再向自己的仓库推送分支。

## 交流与合作

有 **Go2 机器狗二次开发、ROS 2 集成、导航与感知实验、机器人教学或项目合作** 需求，欢迎通过 [GitHub Issues](https://github.com/ztl3106742440-hub/go2-tutorial/issues) 联系，说明需求目标、硬件、系统版本和期望交付内容。

涉及项目私有资料时，请先在公开 Issue 留下不敏感的需求概要，约定联系渠道后再交流。项目维护者：[TIlor](https://github.com/ztl3106742440-hub)。
