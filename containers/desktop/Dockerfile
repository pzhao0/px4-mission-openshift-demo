ARG BASE_IMAGE
FROM ${BASE_IMAGE}
USER root

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends libqt5svg5 xvfb x11vnc x11-utils novnc websockify \
    && rm -rf /var/lib/apt/lists/*

COPY start-vnc.sh novnc_server.py /etc/mission-sim/
COPY gui.config /usr/local/share/gazebo-vnc/gui.config
WORKDIR /tmp
USER 1001

EXPOSE 6080/tcp
ENTRYPOINT ["/bin/bash", "/etc/mission-sim/start-vnc.sh"]
