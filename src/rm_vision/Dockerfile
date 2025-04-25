FROM fateryu/ros2_humble_openvino

USER root

ARG SSH_PRIVATE_KEY

RUN apt-get update && apt-get install openssh-client -y && \
    rm -rf /var/lib/apt/lists/*

# write ssh private key
RUN mkdir -p /root/.ssh && \
    echo -e "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/id_rsa && \
    ssh-keyscan e.coding.net >> /root/.ssh/known_hosts


# create workspace
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws/

# clone projects
RUN cd src && git clone git@e.coding.net:scnu-pioneer/vision/rm_auto_aim.git --depth=1 && \
    git clone git@e.coding.net:scnu-pioneer/vision/rm_buff.git --depth=1 && \
    git clone git@e.coding.net:scnu-pioneer/vision/ros2_hik_camera.git --depth=1 && \
    git clone git@e.coding.net:scnu-pioneer/vision/rm_gimbal_description.git --depth=1 && \
    git clone git@e.coding.net:scnu-pioneer/vision/rm_serial_driver.git --depth=1 && \
    git clone git@e.coding.net:scnu-pioneer/vision/rm_vision.git --depth=1 && \
    git clone git@e.coding.net:scnu-pioneer/vision/rm_auto_record.git --depth=1

# install dependencies and some tools
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y && \
    apt-get install ros-humble-foxglove-bridge wget htop vim nano -y && \
    rm -rf /var/lib/apt/lists/*

# build
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# setup .zshrc
RUN echo -e 'export TERM=xterm-256color\n\
source /ros_ws/install/setup.zsh\n\
eval "$(register-python-argcomplete3 ros2)"\n\
eval "$(register-python-argcomplete3 colcon)"\n\
bindkey -s "\e\e" "\C-asudo \C-e"\n'\
>> /root/.zshrc

# source entrypoint setup
RUN sed --in-place --expression \
    '$isource "/ros_ws/install/setup.zsh"' \
    /ros_entrypoint.sh
