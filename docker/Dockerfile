FROM ubuntu:22.04

ARG USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG WORKSPACE=/home/$USERNAME/workspace

# CoppeliaSim variables
# Needs to use underscore instead of dot
ARG COPPELIA_VERSION="4_10_0"
ARG COPPELIA_REVISION="rev0"
ARG COPPELIA_OS="Ubuntu22_04"

ENV USERNAME=${USERNAME}
ENV HOME=/home/$USERNAME
ENV WORKSPACE=${WORKSPACE}
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV TZ=Etc/UTC
# CoppeliaSim variables
ENV COPPELIA_DIR=/opt/CoppeliaSim
ENV COPPELIA_VERSION=${COPPELIA_VERSION}
ENV COPPELIA_REVISION=${COPPELIA_REVISION}
ENV COPPELIA_OS=${COPPELIA_OS}
ENV COPPELIA_FULL_VERSION=${COPPELIA_VERSION}_${COPPELIA_REVISION}
ENV COPPELIA_FILENAME=CoppeliaSim_Edu_V${COPPELIA_FULL_VERSION}_${COPPELIA_OS}.tar.xz
ENV COPPELIA_URL=https://downloads.coppeliarobotics.com/V${COPPELIA_FULL_VERSION}/${COPPELIA_FILENAME}

# Set shell to use pipefail option
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install sudo (not present in all minimal images)
RUN mkdir -p ${WORKSPACE} && \
    apt-get update && apt-get install -y sudo \
    && rm -rf /var/lib/apt/lists/*    # keep image slim

# Create group & user, give a home dir and bash shell
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd  --uid  $USER_UID \
            --gid  $USER_GID \
            --create-home \
            --shell /bin/bash \
            $USERNAME && \
    # Allow the new user to run any command via sudo with no password
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod -aG sudo $USERNAME

# Copy scripts
WORKDIR ${WORKSPACE}
COPY requirements.txt .
COPY src src
COPY scripts scripts

# Basic build essentials + X11 + Python
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-venv \
    python3-pip \
    python3-dev \
    sudo \
    unzip \
    build-essential \
    git \
    wget \
    curl \
    nano \
    htop \
    ca-certificates \
    libglu1-mesa \
    libxi6 \
    libxrender1 \
    libxext6 \
    libxkbcommon-x11-0 \
    libqt5widgets5 \
    libqt5gui5 \
    libqt5core5a \
    libx11-xcb1 \
    libxcb-xinerama0 \
    xvfb \
    x11-utils \
    socat \
    # Install these packages so that CoppeliaSim doesn't show errors
    libzmq5 \
    libsodium23 \
    libavcodec-dev \
    libavformat-dev \
    libgl1-mesa-dev \
    libswscale-dev \
    # Graphics libraries
    mesa-utils \
    libgl1 \
    libglx-mesa0 \
    libegl1 \
    libvulkan1 \
    vulkan-tools && \
    rm -rf /var/lib/apt/lists/* && \
    # refresh linker cache after installing libzmq5 libsodium23 libraries
    ldconfig

# Install Python packages in a virtual environment
RUN python3 -m venv $HOME/venv && \
    echo "source $HOME/venv/bin/activate" >> $HOME/.bashrc && \
    echo '#!/bin/bash' >> /usr/local/bin/python && \
    echo 'exec $HOME/venv/bin/python "$@"'  >> /usr/local/bin/python && \
    chmod +x /usr/local/bin/python && \
    ln -sf /usr/local/bin/python /usr/local/bin/python3 && \
    echo '#!/bin/bash' >> /usr/local/bin/ipython && \
    echo 'exec $HOME/venv/bin/python -m IPython "$@"' >> /usr/local/bin/ipython && \
    chmod +x /usr/local/bin/ipython && \
    ln -sf /usr/local/bin/ipython /usr/local/bin/ipython3 && \
    ln -sf $HOME/venv/bin/python /usr/local/bin/python && \
    ln -sf $HOME/venv/bin/python /usr/local/bin/python3 && \
    # Install these packages in the default python3 so Coppelia doesn't show errors
    python3 -m pip install --no-cache-dir pyzmq cbor2 && \
    # Upgrade pip and install requirements in venv
    $HOME/venv/bin/pip install --upgrade pip && \
    $HOME/venv/bin/pip install --no-cache-dir -r requirements.txt

# Extract CoppeliaSim files
RUN mkdir -p ${COPPELIA_DIR} && \
    mkdir -p $HOME/data && \
    wget ${COPPELIA_URL} -O /tmp/coppeliaSim.tar.xz && \
    tar -xJf /tmp/coppeliaSim.tar.xz -C ${COPPELIA_DIR} --strip-components=1 && \
    rm /tmp/coppeliaSim.tar.xz && \
    # Make the script executable
    chmod +x ${COPPELIA_DIR}/coppeliaSim.sh && \
    # Convenient symlink
    ln -sf ${COPPELIA_DIR}/coppeliaSim.sh /usr/local/bin/coppeliaSim

ENV PATH="/usr/local/bin:${PATH}"

# Expose remote API ports (classic 19997, ZMQ 23000-23050 for multi-sim)
EXPOSE 19997 23000-23050

# Set up colored prompt and ls alias for the user
RUN echo "force_color_prompt=yes" >> $HOME/.bashrc && \
echo "alias ls='ls --color=auto'" >> $HOME/.bashrc && \
echo "source $HOME/venv/bin/activate" >> $HOME/.bashrc
# echo "export LD_LIBRARY_PATH" >> $HOME/.bashrc && \
# echo "export PKG_CONFIG_PATH" >> $HOME/.bashrc

# Create the restart_coppelia.sh script
RUN echo '#!/bin/bash' > ${HOME}/restart_coppelia.sh && \
    echo 'pkill -f CoppeliaSim || true' >> ${HOME}/restart_coppelia.sh && \
    echo '${HOME}/command.sh &' >> ${HOME}/restart_coppelia.sh && \
    chmod +x ${HOME}/restart_coppelia.sh && \
    chown ${USERNAME}:${USERNAME} ${HOME}/restart_coppelia.sh

# Add alias to .bashrc
RUN echo "alias coppeliasim='${HOME}/start_coppeliasim.sh'" >> ${HOME}/.bashrc
