# Go2 机器狗实验指导书

面向本科小白的 Unitree Go2 + ROS2 二次开发实验教程。源文件为 Markdown，通过 [MkDocs-Material](https://squidfunk.github.io/mkdocs-material/) 构建为可交互的静态网站。

---

## 本地预览

```bash
# 1. 装依赖(只需一次)
pip install -r requirements.txt

# 2. 启动预览(每次)
./serve.sh
# 浏览器打开 http://127.0.0.1:8000,改动 Markdown 会自动热重载
```

构建成静态站点:

```bash
./build.sh
# 产物在 ./site/,可直接扔服务器 / 打压缩包分享
```

---

## 目录结构

```
go2实验指导书/
├── mkdocs.yml              # MkDocs 配置(主题/导航树/插件)
├── requirements.txt        # Python 依赖
├── serve.sh                # 本地预览
├── build.sh                # 构建静态站
├── README.md               # 本文件
├── docs/                   # ★ 教材源文件(Markdown)
│   ├── index.md            # 首页
│   ├── assets/             # 图片/视频/CSS
│   ├── 00-overview/        # 开篇
│   ├── 01-foundation/      # 基础篇:环境与通信
│   ├── 02-packages/        # 功能包开发
│   ├── 03-communication/   # ROS2 通信机制
│   ├── 04-perception/      # 感知与建图
│   ├── 05-interaction/     # 交互(语音/视觉)
│   └── 06-integration/     # 综合篇
└── meta/                   # ★ 编写规范(不参与构建)
    ├── 编写指南.md           # 所有章节的元规范,写/改章节前必读
    ├── 命名约定.md           # 工作空间/包/节点命名规范
    └── 章节大纲.md           # 总章节规划与进度
```

---

## 命名/路径约定(给所有协作者)

- **顶层目录**用中文:`go2实验指导书/`
- **内部目录**用英文:`docs/04-perception/` → URL 干净
- **章节文件名**用数字前缀 + 英文短名:`11-slam-2d.md`
- **侧边栏标题**在 `mkdocs.yml` 的 `nav` 里用中文指定
- **教材正文不出现任何本地路径**(代码路径只存在于 Codex 交接文件中)

---

## 给 Codex 的指引

所有填充章节的任务,**先读 `meta/编写指南.md` 和 `meta/命名约定.md`**,再对照样章 `docs/04-perception/11-slam-2d.md` 的结构复刻。
