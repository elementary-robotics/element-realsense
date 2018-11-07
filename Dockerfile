FROM elementaryrobotics/element-realsense-base:d570c21bfb1142631a11bfcc697757c15fc02643

# Want to copy over the contents of this repo to the code
#	section so that we have the source
ADD . /code

#
# Install python dependencies
#
WORKDIR /code
RUN pip3 install -r requirements.txt

# Finally, specify the command we should run when the app is launched
WORKDIR /code
RUN chmod +x launch.sh
CMD ["/bin/bash", "launch.sh"]
