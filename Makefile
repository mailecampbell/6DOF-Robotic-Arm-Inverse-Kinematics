# Makefile for Robot Arm Kinematics Library
#
# Targets:
#   make test       - Build and run test harness
#   make lib        - Build static library
#   make clean      - Remove build artifacts

CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -I./include

SRC_DIR = src
INC_DIR = include
TEST_DIR = test
BUILD_DIR = build

# Source files
LIB_SOURCES = $(SRC_DIR)/arm_kinematics.cpp $(SRC_DIR)/servo_controller.cpp
LIB_OBJECTS = $(BUILD_DIR)/arm_kinematics.o $(BUILD_DIR)/servo_controller.o

# Test files
TEST_SOURCES = $(TEST_DIR)/test_kinematics.cpp
TEST_EXECUTABLE = $(BUILD_DIR)/test_kinematics

# Library
LIBRARY = $(BUILD_DIR)/libarmkinematics.a

.PHONY: all test lib clean dirs

all: test

dirs:
	@mkdir -p $(BUILD_DIR)

# Build test executable
test: dirs $(TEST_EXECUTABLE)
	@echo "Running tests..."
	@./$(TEST_EXECUTABLE) 500

$(TEST_EXECUTABLE): $(LIB_OBJECTS) $(TEST_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $(TEST_SOURCES) $(LIB_OBJECTS)

# Build static library
lib: dirs $(LIBRARY)

$(LIBRARY): $(LIB_OBJECTS)
	ar rcs $@ $^

# Object files
$(BUILD_DIR)/arm_kinematics.o: $(SRC_DIR)/arm_kinematics.cpp $(INC_DIR)/arm_kinematics.h $(INC_DIR)/arm_config.h $(INC_DIR)/arm_math.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/servo_controller.o: $(SRC_DIR)/servo_controller.cpp $(INC_DIR)/servo_controller.h $(INC_DIR)/arm_config.h $(INC_DIR)/arm_math.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -rf $(BUILD_DIR)

# Debug build
debug: CXXFLAGS += -g -DDEBUG
debug: test
