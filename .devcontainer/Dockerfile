FROM mcr.microsoft.com/devcontainers/cpp:1-debian-11

ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="none"

# Optionally install the cmake for vcpkg
COPY ./reinstall-cmake.sh /tmp/

RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
        chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi \
    && rm -f /tmp/reinstall-cmake.sh

# [Optional] Uncomment this section to install additional vcpkg ports.
# RUN su vscode -c "${VCPKG_ROOT}/vcpkg install <your-port-name-here>"

# [Optional] Uncomment this section to install additional packages.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends usbutils gcc-arm-none-eabi gdb-arm-none-eabi \
    && apt-get clean

# Install the JLink package from host machine
# Copy the JLink .deb file into the container
COPY ./JLink_Linux_V824_x86_64.deb /tmp/
RUN dpkg --unpack /tmp/JLink_Linux_V824_x86_64.deb
RUN rm /var/lib/dpkg/info/jlink.postinst -f
# dpkg configure, if it fails, ignore the error
RUN dpkg --configure jlink || true
RUN apt install -yf 