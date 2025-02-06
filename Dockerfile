FROM ghcr.io/ca-giken/rosnoetic-base:main

WORKDIR /root

RUN cd catkin_ws/src/eel-ros1
RUN npm install

ENTRYPOINT ["/bin/bash", "-c", "entrypoint.sh"]