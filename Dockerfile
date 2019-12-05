ARG FROM_IMAGE=ruffsl/librealsense:development

# multi-stage for caching
FROM $FROM_IMAGE AS cache

# copy overlay source
ENV REALSENSE_WS /opt/realsense_ws
RUN mkdir -p $REALSENSE_WS/src
WORKDIR $REALSENSE_WS
COPY ./ src/ros2_intel_realsense

# copy manifests for caching
WORKDIR /opt
RUN find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp
    # && find ./ -name "COLCON_IGNORE" | \
    #   xargs cp --parents -t /tmp

# multi-stage for building
FROM $FROM_IMAGE AS build

# install CI dependencies
RUN apt-get update && apt-get install -q -y \
      ccache \
      lcov \
    && rm -rf /var/lib/apt/lists/*

# copy overlay manifests
ENV REALSENSE_WS /opt/realsense_ws
COPY --from=cache /tmp/realsense_ws $REALSENSE_WS
WORKDIR $REALSENSE_WS

# install overlay dependencies
RUN . $OVERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths \
        src \
        $OVERLAY_WS/src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy overlay source
COPY --from=cache $REALSENSE_WS ./

# build overlay source
ARG REALSENSE_MIXINS="release ccache"
RUN . $OVERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin \
        $REALSENSE_MIXINS \
      --event-handlers console_direct+

# source overlay from entrypoint
RUN sed --in-place \
      's|^source .*|source "$REALSENSE_WS/install/setup.bash"|' \
      /ros_entrypoint.sh
