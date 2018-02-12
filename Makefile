.PHONY: all init clean
CXX = g++
CXXFLAGS = -std=c++0x -Wall
LDLIBS = `pkg-config --libs --cflags opencv` -lGLEW -lm -lGL -lGLU -lglfw -lglut

SRC_DIR = src
OBJ_DIR = bin
LIB_DIR = -L/usr/lib
INC_DIR = -I/usr/include

SOURCE = $(wildcard $(SRC_DIR)/*.cpp main.cpp)
OBJECTS = $(addprefix $(OBJ_DIR)/,$(notdir $(SOURCE:.cpp=.o)))
EXECUTABLE = main

all: rmbin init $(OBJECTS) $(EXECUTABLE) clean

$(EXECUTABLE):
	$(CXX) $(CXXFLAGS) $(LIB_DIR) -o $@ $(OBJECTS) $(LDLIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INC_DIR) -c $< -o $@

init:
	@mkdir -p "$(OBJ_DIR)"

clean:
	@rm -rf $(OBJ_DIR)

rmbin:
	@rm -f ./main