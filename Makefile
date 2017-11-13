CXX = g++
CXXFLAGS = -Wall -std=c++0x
LDLIBS = `pkg-config --libs --cflags opencv` -lm -lGL -lGLU -lglut

OBJ_DIR = bin
LIB_DIR = -L/usr/lib
INC_DIR = -I/usr/include

SOURCE = main.cpp
OBJECTS = ${SOURCE:%.cpp=$(OBJ_DIR)/%.o}
EXECUTABLE = ${SOURCE:%.cpp=%}

all: init $(OBJECTS) $(EXECUTABLE)

$(EXECUTABLE):
	$(CXX) $(CXXFLAGS) $(LIB_DIR) -o $@ $(OBJECTS) $(LDLIBS)

$(OBJ_DIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INC_DIR) -c $< -o $@

init:
	@mkdir -p "$(OBJ_DIR)"

clean:
	@rm -rf $(OBJ_DIR) $(EXECUTABLE)