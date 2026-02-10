FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8

# ── Base tools ──────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    gnupg \
    lsb-release \
    python3 \
    python3-pip \
    python3-venv \
    software-properties-common \
    sudo \
    wget \
  && rm -rf /var/lib/apt/lists/*

# ── vcstool & colcon ───────────────────────────────────────────────────────
RUN pip3 install --break-system-packages \
    vcstool \
    colcon-common-extensions

# ── OSRF apt repo (for Gazebo dependencies) ────────────────────────────────
RUN curl -s https://packages.osrfoundation.org/gazebo.gpg \
      --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update

# ── Install Gazebo deps from packages.apt manifests ────────────────────────
# Clone sources temporarily just to read their packages.apt files,
# install everything, then discard the clones.
COPY gz-jetty.repos /tmp/gz-jetty.repos
RUN mkdir -p /tmp/gz_src && \
    vcs import --recursive /tmp/gz_src < /tmp/gz-jetty.repos && \
    apt-get install -y --no-install-recommends \
      $(sort -u $(find /tmp/gz_src -iname "packages-$(lsb_release -cs).apt" \
                    -o -iname 'packages.apt' | grep -v '/\.git/') \
        | sed '/gz\|sdf/d' | tr '\n' ' ') && \
    rm -rf /tmp/gz_src /tmp/gz-jetty.repos /var/lib/apt/lists/*

# ── Non-root user matching host UID (overridable at build time) ────────────
ARG USER_NAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN if getent group $USER_GID >/dev/null; then \
      existing_group=$(getent group $USER_GID | cut -d: -f1); \
      groupmod -n $USER_NAME "$existing_group"; \
    else \
      groupadd --gid $USER_GID $USER_NAME; \
    fi && \
    if getent passwd $USER_UID >/dev/null; then \
      existing_user=$(getent passwd $USER_UID | cut -d: -f1); \
      usermod -l $USER_NAME -d /home/$USER_NAME -m "$existing_user"; \
    else \
      useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USER_NAME; \
    fi && \
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USER_NAME

USER $USER_NAME
WORKDIR /home/$USER_NAME/gazebo_ws

COPY --chown=$USER_NAME:$USER_NAME docker/entrypoint.bash /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]
CMD ["bash"]
