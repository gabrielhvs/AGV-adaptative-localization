FROM ubuntu:24.04
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,display
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get -y full-upgrade && apt-get install -y --no-install-recommends software-properties-common && add-apt-repository universe
RUN apt-get install -y --no-install-recommends curl gnupg lsb-release ca-certificates
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt-get install -y --no-install-recommends \
build-essential zip \
libboost-all-dev libalglib-dev \
libsqlite3-dev sqlite3 \
libkf5networkmanagerqt-dev modemmanager-qt-dev libkf5modemmanagerqt6  \
libgl1-mesa-dev \
libxcb-xinerama0 libxcb-icccm4 libxcb-image0 \
libxcb1 libxcb-keysyms1 libxcb-render-util0 \
libxrender1 libxkbcommon-x11-0 libfontconfig1 \
libpulse0 libpulse-mainloop-glib0 libasound2-dev \
libkf5solid5 libkf5solid-dev \
libsm6 libice6 libxext6 \
libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
locales \
git openssh-client \
make cmake \
sudo
RUN apt-get -y --no-install-recommends install libcurl4-openssl-dev libssl-dev uuid-dev zlib1g-dev libpulse-dev

RUN sudo apt install -y --no-install-recommends ros-jazzy-desktop \
ros-dev-tools \
ros-jazzy-ros-gz* \
ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ackermann-msgs

#Others Tools
RUN sudo apt update && apt-get install -y --no-install-recommends nano
RUN sudo apt update && apt-get install -y --no-install-recommends psmisc #KILLALL

RUN apt-get -qq clean && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 && dpkg-reconfigure locales

# Add group & user
RUN groupadd -r user && useradd --create-home --gid user user && echo 'user ALL=NOPASSWD: ALL' > /etc/sudoers.d/user

USER user
WORKDIR /home/user
ENV HOME /home/user

RUN /bin/sh -c "echo alias killg='killall gzclient && killall gzserver && killall rosmaster' >> ~/.bashrc"
RUN /bin/sh -c "echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc"
