FROM elementaryrobotics/element-realsense-base:7969c396b780b4feb43d6845dd346aa81d783e36

# Want to copy over the contents of this repo to the code
#	section so that we have the source
ADD . /code

# Build transformation_estimation
WORKDIR /code
RUN make

#
# Install python dependencies
#
RUN pip3 install -r requirements.txt

# Finally, specify the command we should run when the app is launched
WORKDIR /code
RUN chmod +x launch.sh
CMD ["/bin/bash", "launch.sh"]
