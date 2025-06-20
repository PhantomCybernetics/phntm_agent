ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

RUN echo "Building docker image with ROS_DISTRO=$ROS_DISTRO"

RUN apt-get update -y --fix-missing
RUN apt-get install -y pkg-config

RUN apt-get install -y python3-setuptools
RUN apt-get install -y python3-termcolor
RUN apt-get install -y python3-pyee

# init workspace
ENV ROS_WS=/ros2_ws
RUN mkdir -p $ROS_WS/src

# docker ctrl
RUN apt-get install -y python3-docker

# wifi scanning
RUN apt-get install -y iw wireless-tools libiw-dev

# create a venv, install pip-only deps
RUN apt-get install -y python3-venv
RUN mkdir -p $ROS_WS/ros2_py_venv
RUN python3 -m venv $ROS_WS/ros2_py_venv
RUN . $ROS_WS/ros2_py_venv/bin/activate && \
    pip install iwlib && \
    deactivate

# wifi ctrl via shared /var/run/wpa_supplicant/ (also needs shared /tmp)
RUN apt-get install -y wpasupplicant

# generate entrypoint script
RUN echo '#!/bin/bash \n \
set -e \n \
\n \
# setup ros environment \n \
source "/opt/ros/'$ROS_DISTRO'/setup.bash" \n \
test -f "/ros2_ws/install/setup.bash" && source "/ros2_ws/install/setup.bash" \n \
\n \
exec "$@" ' > /ros_entrypoint.sh
RUN chmod a+x /ros_entrypoint.sh

# source underlay on every login
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /root/.bashrc
RUN echo 'test -f "/ros2_ws/install/setup.bash" && source "/ros2_ws/install/setup.bash"' >> /root/.bashrc
# activate python venv on ~/.bashrc source
RUN echo 'export PATH="/ros2_ws/ros2_py_venv/bin:$PATH"' >> /root/.bashrc
RUN echo 'export PYTHONPATH="/ros2_ws/ros2_py_venv/lib/python3.12/site-packages:${PYTHONPATH:-}"' >> /root/.bashrc

WORKDIR $ROS_WS

# clone and install phntm interfaces
RUN git clone https://github.com/PhantomCybernetics/phntm_interfaces.git /ros2_ws/src/phntm_interfaces
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src/phntm_interfaces --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --packages-select phntm_interfaces

# install phntm agent
COPY ./ $ROS_WS/src/phntm_agent
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    rosdep install -i --from-path src/phntm_agent --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --packages-select phntm_agent

# pimp up prompt with hostame and color
RUN echo "PS1='\${debian_chroot:+(\$debian_chroot)}\\[\\033[01;35m\\]\\u@\\h\\[\\033[00m\\] \\[\\033[01;34m\\]\\w\\[\\033[00m\\] '"  >> /root/.bashrc

WORKDIR $ROS_WS

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]