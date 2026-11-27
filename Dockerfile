# Fairino arm as an Arkitekt app, for the openUC2 OS (os-rpi) and any Docker host.
#   docker build -t fairinogale .
#   docker run -p 8080:8080 -v ~/fairino:/config fairinogale   -> http://localhost:8080/
FROM python:3.12-slim
COPY --from=ghcr.io/astral-sh/uv:0.9 /uv /bin/uv
ENV UV_COMPILE_BYTECODE=1 UV_LINK_MODE=copy UV_PYTHON_DOWNLOADS=never \
    UV_PROJECT_ENVIRONMENT=/opt/venv PATH=/opt/venv/bin:$PATH PYTHONUNBUFFERED=1
WORKDIR /app
COPY pyproject.toml uv.lock .python-version ./
RUN uv sync --frozen --no-dev --no-install-project
# Only the Linux SDK module is needed; it is pure Python.
COPY fairino-python-sdk/linux/fairino/Robot.py sdk/
COPY app.py arkitekt_bind.py teach.py vision.py vision_ui.py debug.py tags.py config.example.yaml ./
COPY mock mock
COPY control-points control-points
# config.yaml, the teach points and the stored login (.state/) live in /config.
ENV FAIRINO_SDK_DIR=/app/sdk FAIRINO_CONFIG=/config/config.yaml
RUN useradd -u 1000 -m app && mkdir /config && chown app /config
USER app
# 8080: the binding page (PORT). 8093: the vision page (vision.port in config.yaml).
EXPOSE 8080 8093
CMD ["python", "app.py"]
