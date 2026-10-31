# EPGM Platform & Infrastructure (`epgm-platform`)

[ZH] EPGM 生产级云原生环境（EMQX + Redis + Postgres + Celery + FastAPI）与 ESP32 嵌入式端工程。
[EN] EPGM production-grade cloud-native infrastructure and ESP32 embedded edge firmware.

## 结构说明 / Structure
- `docker-compose.yml`: 一键启动生产级分布式 IoT 消息与图处理集群
- `dashboard/`: 现代 Web Dark-Mode 实时因果拓扑图可视化控制台 (`index.html`)
- `firmware/`: ESP32-S3 原生 C/C++ 嵌入式工程
  - `src/epgm_core.c & .h`: 纯 C 语言轻量级拓扑图运行时
  - `include/protocol_schema.h`: 端云二进制与 MQTT 消息规范
- `scripts/`: 一键安装与环境启动脚本 (`setup_env.ps1`, `setup_env.sh`)

## 快速启动 / Getting Started
```powershell
# Windows
.\scripts\setup_env.ps1

# 启动 Docker 生产集群
docker-compose up -d
```
启动后访问：
- EMQX 控制台: `http://localhost:18083` (admin / epgm_secret)
- API 文档: `http://localhost:8000/docs`
- 拓扑大屏: 打开 `dashboard/index.html`
