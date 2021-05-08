FROM ryuichiueda/emcl-test-env:latest
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN cd /catkin_ws/src && \
    git clone https://github.com/ryuichiueda/emcl.git

RUN source ~/.bashrc && \
    cd /catkin_ws && \
    catkin_make
