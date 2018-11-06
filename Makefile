################################################################################
#
# Makefile for realsense calibration element
#
################################################################################

# Which compiler to use
CXX:=g++

# Set the default target for just 'make' to all
.DEFAULT_GOAL := all

# Build directory
BUILD_DIR:=build

# Location of the sources and the sources themselves
SRC_DIR := src
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS:.cpp=.o)))
vpath %.cpp $(sort $(dir $(SRCS)))

# Name of the output binary
OUTPUT_NAME := transform_estimation

# PCL version. Defaults to 1.8
ifeq ($(PCL_VERSION),)
	PCL_VERSION:=1.8
endif

# C flags for the source we're building
CFLAGS := \
	-O3 \
	-Wall \
	-Werror \
	-Wsign-compare \
	-Wmaybe-uninitialized \
	-I/usr/include/pcl-$(PCL_VERSION) \
	-I/usr/include/eigen3 \
	-I/usr/local/include/atom

# Linker flags
LDFLAGS := \
	-lopencv_core \
	-lopencv_imgcodecs \
	-lopencv_calib3d \
	-lpcl_common \
	-latom

# Build the source files
$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	@ echo "Compiling $<"
	@ $(CXX) -c $(CFLAGS) -o $@ $<

$(BUILD_DIR)/$(OUTPUT_NAME): $(OBJS) | $(BUILD_DIR)
	@ echo "Linking $@"
	@ $(CXX) $(OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR):
	@ echo "Creating $@"
	@ mkdir $@

.PHONY: clean
clean:
	# Get rid of the build
	@ rm -rf $(BUILD_DIR)

.PHONY: all
all: $(BUILD_DIR)/$(OUTPUT_NAME)

.PHONY: run
run: $(BUILD_DIR)/$(OUTPUT_NAME)
	$(BUILD_DIR)/$(OUTPUT_NAME)
