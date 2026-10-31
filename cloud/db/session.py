"""
[ZH] EPGM 云端持久化数据库连接引擎与会话生命周期管理。
[EN] EPGM Cloud Persistence Database Connection Engine and Session Lifecycle Management.
"""

import os
import logging
from typing import Generator
from contextlib import contextmanager
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, Session
from .models import Base, Robot, TelemetryLog, GraphSnapshot

logger = logging.getLogger("DatabaseSession")

# 默认读取环境变量，支持本地开发与容器化部署
raw_db_url = os.getenv("DATABASE_URL", "")

if not raw_db_url:
    # 未指定时自适应检测可用的驱动
    try:
        import psycopg2
        raw_db_url = "postgresql://epgm_admin:epgm_db_password@localhost:5432/epgm_knowledge"
    except ImportError:
        try:
            import psycopg
            raw_db_url = "postgresql+psycopg://epgm_admin:epgm_db_password@localhost:5432/epgm_knowledge"
        except ImportError:
            raw_db_url = "sqlite:///epgm_knowledge.db"
else:
    # 环境变量指定了 postgresql:// 但未装 psycopg2 时，自适应降级 psycopg3 或 sqlite
    if raw_db_url.startswith("postgresql://") and not raw_db_url.startswith("postgresql+"):
        try:
            import psycopg2
        except ImportError:
            try:
                import psycopg
                raw_db_url = raw_db_url.replace("postgresql://", "postgresql+psycopg://", 1)
            except ImportError:
                logger.warning("No PostgreSQL driver found (neither psycopg2 nor psycopg). Falling back to SQLite.")
                raw_db_url = "sqlite:///epgm_knowledge.db"

DATABASE_URL = raw_db_url

# 针对 SQLite 内存或文件库增加特殊连接参数兼容单元测试
is_sqlite = DATABASE_URL.startswith("sqlite")

engine_kwargs = {
    "pool_pre_ping": True,  # 探针自动检测并重连失效空闲连接
}

if is_sqlite:
    # SQLite 内存库单线程安全配置
    engine_kwargs["connect_args"] = {"check_same_thread": False}
else:
    # 针对 PostgreSQL 生产级连接池调优，并限制快速连接超时防止无容器时长时间挂起
    engine_kwargs["pool_size"] = 10
    engine_kwargs["max_overflow"] = 20
    engine_kwargs["connect_args"] = {"connect_timeout": 2}

engine = create_engine(DATABASE_URL, **engine_kwargs)

SessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=engine,
    expire_on_commit=False,
)


def init_db() -> None:
    """
    [ZH] 自动初始化并创建所有未就绪的数据表。
    [EN] Initialize and create all missing database tables.
    """
    try:
        logger.info(f"Initializing database tables using engine: {engine.url}")
        Base.metadata.create_all(bind=engine)
        logger.info("Database tables initialized successfully.")
    except Exception as e:
        logger.error(f"Failed to initialize database tables: {e}", exc_info=True)
        raise


@contextmanager
def get_db_session() -> Generator[Session, None, None]:
    """
    [ZH] 上下文管理器，提供事务自动提交与异常安全回滚。
    [EN] Context manager providing transaction auto-commit and safe rollback.
    """
    session: Session = SessionLocal()
    try:
        yield session
        session.commit()
    except Exception as e:
        session.rollback()
        logger.error(f"Database session rolled back due to error: {e}")
        raise
    finally:
        session.close()


def get_db() -> Generator[Session, None, None]:
    """
    [ZH] FastAPI 依赖注入 (Depends) 会话生成器。
    [EN] FastAPI dependency generator for request-scoped database sessions.
    """
    session: Session = SessionLocal()
    try:
        yield session
    finally:
        session.close()
