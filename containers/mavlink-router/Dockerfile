ARG BASE_IMAGE
FROM ${BASE_IMAGE} AS build

RUN apk add --no-cache \
    gcc \
    g++ \
    git \
    pkgconf \
    meson \
    ninja \
    linux-headers

WORKDIR /src/mavlink-router
ARG ROUTER_REV
RUN git clone --no-checkout \
    https://github.com/mavlink-router/mavlink-router.git . \
    && git checkout --detach "${ROUTER_REV}" \
    && git submodule update --init --recursive \
    && meson setup -Dsystemdsystemunitdir=/usr/lib/systemd/system --buildtype=release build . \
    && ninja -C build

FROM ${BASE_IMAGE}

RUN apk add --no-cache libstdc++
COPY --from=build /src/mavlink-router/build/src/mavlink-routerd /usr/local/bin/mavlink-routerd

COPY --from=build /src/mavlink-router/LICENSE /usr/share/doc/mavlink-router/COPYING
USER 1001
EXPOSE 5760/tcp
ENTRYPOINT ["mavlink-routerd"]
