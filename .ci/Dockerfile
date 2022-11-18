# Tests Dockerfile

FROM ubuntu:18.04

ARG GIT_REF_NAME

ENV WORKSPACE_DIR /root
ENV PTERANODON_DIR ${WORKSPACE_DIR}/pteranodon

# Add docker-compose-wait tool -------------------
ENV WAIT_VERSION 2.7.2
ADD https://github.com/ufoscout/docker-compose-wait/releases/download/$WAIT_VERSION/wait /wait
RUN chmod +x /wait

# Install deps
RUN apt-get install python3.7 | echo y && \ 
    apt-get install -y curl git python3.7-pip && \
    #apt-get update && \
    apt-get -y autoremove && \
    apt-get clean autoclean && \
    rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Checkout current version of pteranodon
RUN git clone https://github.com/AutonoLab/pteranodon.git ${PTERANODON_DIR}
RUN echo "REF ${GIT_REF_NAME}"
RUN git -C ${PTERANODON_DIR} checkout "${GIT_REF_NAME}"
RUN git -C ${PTERANODON_DIR} submodule update --init --recursive # Future proofing

RUN pip3 install -r ${PTERANODON_DIR}/requirements.txt
RUN pip3 install -r ${PTERANODON_DIR}/requirements-test.txt

WORKDIR "${PTERANODON_DIR}"