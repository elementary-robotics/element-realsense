FROM elementaryrobotics/element-realsense-base:1d92b548818d3b01a8b4baae3ed355da9183239c

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
