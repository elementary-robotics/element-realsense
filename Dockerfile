FROM elementaryrobotics/atom

# Want to copy over the contents of this repo to the code
#	section so that we have the source
ADD . /code

#
# Build and install librealsense
#
ARG RS_VERSION=v2.16.3
RUN apt update
RUN apt install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
WORKDIR /tmp
RUN git clone https://github.com/IntelRealSense/librealsense.git
WORKDIR /tmp/librealsense
RUN git checkout $RS_VERSION
RUN mkdir build
WORKDIR /tmp/librealsense/build
RUN cmake ../ -DCMAKE_BUILD_TYPE=Release
RUN make -j16 && make install
RUN pip3 install pyrealsense2

#
# Install other dependencies
#
WORKDIR /code
RUN DEBIAN_FRONTEND=noninteractive apt install -y tzdata
RUN apt install -y libopencv-dev libpcl-dev
# RUN pip3 install -r requirements.txt

# Finally, specify the command we should run when the app is launched
WORKDIR /code
RUN chmod +x launch.sh
CMD ["/bin/bash", launch.sh"]
