FROM ghcr.io/ca-giken/rosnoetic-base:main

WORKDIR /root

# Install Eel and Chromium
RUN pip install eel
RUN apt-get update \
    && apt-get install -y libgtk2.0-0 libgtk-3-0 libnotify-dev libgconf-2-4 libnss3 libxss1 libasound2 libxtst6 xauth xvfb libgbm-dev fonts-ipafont \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN wget 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x869689FE09306074' \
    -O '/etc/apt/trusted.gpg.d/phd-chromium.asc'
RUN echo "deb https://freeshell.de/phd/chromium/focal/" \
    | tee /etc/apt/sources.list.d/phd-chromium.list
RUN apt-get update \
    && apt-get install -y chromium \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

COPY ./entrypoint.sh /root/entrypoint.sh

# Temporarily install rovi for catkin build
# Rovi setup(Aravis)
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV ORGE_RTT_MODE=Copy
RUN apt-get update -q && apt-get install -y \
    g++ automake intltool libgstreamer*-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.0.tar.xz
RUN tar -xvf aravis-0.6.0.tar.xz
RUN cd aravis-0.6.0 && ./configure && make && make install

# Rovi_industrial setup
RUN pip install git+https://github.com/UniversalRobots/RTDE_Python_Client_Library.git

# ENTRYPOINT ["/bin/bash", "-c", "/root/entrypoint.sh"]