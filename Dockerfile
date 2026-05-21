FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl ca-certificates \
    python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool \
    ros-humble-rviz2 \
    ros-humble-rqt-image-view \
    ros-humble-ros-gz \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-vision-msgs ros-humble-tf2-ros ros-humble-robot-localization \
    ros-humble-teleop-twist-keyboard \
    ros-humble-xacro ros-humble-robot-state-publisher \
    libopencv-dev libeigen3-dev libglew-dev libssl-dev \
    libboost-all-dev libgl1-mesa-dev libglu1-mesa-dev \
    x11-apps mesa-utils \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --default-timeout=300 --retries 10 \
    'numpy<2' scipy transforms3d

RUN pip3 install --no-cache-dir --default-timeout=300 --retries 10 \
    opencv-python-headless

RUN pip3 install --no-cache-dir --default-timeout=300 --retries 10 \
    --index-url https://download.pytorch.org/whl/cpu \
    torch torchvision

RUN pip3 install --no-cache-dir --default-timeout=300 --retries 10 \
    ultralytics

RUN pip3 install --no-cache-dir --default-timeout=300 --retries 10 \
    langchain langchain-google-genai google-generativeai

RUN pip3 install --no-cache-dir --default-timeout=300 --retries 10 \
    djitellopy

# Voice I/O deps + dotenv for API key loading
RUN apt-get update && apt-get install -y --no-install-recommends \
    portaudio19-dev libsndfile1 \
    mpg123 ffmpeg espeak libespeak-dev \
    ros-humble-cv-bridge \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --default-timeout=300 --retries 10 \
    python-dotenv sounddevice

RUN pip3 install --no-cache-dir --default-timeout=300 --retries 10 \
    openai-whisper

RUN pip3 install --no-cache-dir --default-timeout=300 --retries 10 \
    gtts pyttsx3

RUN apt-get update && apt-get install -y --no-install-recommends \
    libepoxy-dev libwayland-dev libxkbcommon-dev \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/autobee/third_party && cd /opt/autobee/third_party \
 && git clone --recursive https://github.com/stevenlovegrove/Pangolin.git \
 && cd Pangolin && mkdir build && cd build \
 && cmake .. -DCMAKE_BUILD_TYPE=Release \
 && make -j"$(nproc)" && make install && ldconfig

RUN mkdir -p /root/autobee_ws/src
WORKDIR /root/autobee_ws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "[ -f /root/autobee_ws/install/setup.bash ] && source /root/autobee_ws/install/setup.bash" >> /root/.bashrc

ENV QT_X11_NO_MITSHM=1
ENV NO_AT_BRIDGE=1

CMD ["/bin/bash"]
