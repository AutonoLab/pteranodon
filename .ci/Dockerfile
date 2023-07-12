# Tests Dockerfile

FROM ubuntu:20.04

ARG GIT_REF_NAME

ENV WORKSPACE_DIR /root
ENV PTERANODON_DIR ${WORKSPACE_DIR}/pteranodon

ENV TZ=Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Add docker-compose-wait tool -------------------
#ENV WAIT_VERSION 2.7.2
#ADD https://github.com/ufoscout/docker-compose-wait/releases/download/$WAIT_VERSION/wait /wait
#RUN chmod +x /wait

# Install deps
RUN apt-get update && \
    apt-get install -y curl git python3-pip netcat&& \
    apt-get -y autoremove && \
    apt-get clean autoclean && \
    rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Checkout current version of pteranodon
RUN git clone https://github.com/AutonoLab/pteranodon.git ${PTERANODON_DIR}
RUN echo "REF ${GIT_REF_NAME}"
RUN git -C ${PTERANODON_DIR} checkout "${GIT_REF_NAME}"
RUN git -C ${PTERANODON_DIR} submodule init
RUN git -C ${PTERANODON_DIR} pull --recurse-submodules
RUN git -C ${PTERANODON_DIR} submodule update --init --recursive

RUN apt install -y git-all
RUN apt install -y libgirepository1.0-dev gcc libcairo2-dev pkg-config python3-dev gir1.2-gtk-3.0
RUN apt install -y python3-gi python3-gi-cairo gir1.2-gtk-3.0
RUN apt install -y libopencv-dev

RUN pip3 install ${PTERANODON_DIR}/.
RUN pip3 install -r ${PTERANODON_DIR}/requirements.txt
RUN pip3 install -r ${PTERANODON_DIR}/requirements-test.txt

WORKDIR "${PTERANODON_DIR}"

RUN sudo apt install software-properties-common
RUN sudo add-apt-repository universe
RUN sudo apt update && sudo apt install curl -y
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sudo apt update
RUN sudo apt upgrade
RUN sudo apt install ros-foxy-ros-base python3-argcomplete
RUN source /opt/ros/foxy/setup.bash
