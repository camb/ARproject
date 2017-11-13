.PHONY: clean rebuild compressed doc
VPATH = src src/include
 
GPP    = g++
GCC    = gcc
LINK   = g++
DOCGEN = doxygen
 
INCDIRS = -I./src/include
LIBDIRS = 
LIBS    = `pkg-config --libs --cflags opencv` -lm -lGL -lGLU -lglut
# compiled with `pkg-config --libs --cflags opencv`
 
RELEASE_FLAGS = -Wall -O2 -flto
DEBUG_FLAGS   = -Wall -O0 -g
 
RELEASE_LINK_FLAGS = -flto
DEBUG_LINK_FLAGS   = -g
 
IFLAGS = $(INCDIRS)
CFLAGS = -std=c++0x `pkg-config --libs --cflags opencv` $(IFLAGS) $(DEBUG_FLAGS) 
LFLAGS = $(LIBS)   $(DEBUG_LINK_FLAGS)
UPXFLAGS = --best --ultra-brute
 
#----------------file utils---------------
RM    = rm -rf
CAT   = cat
ECHO  = echo
TAR   = tar
GZIP  = gzip
BZIP  = bzip2
XZ    = xz
STRIP = strip
UPX   = upx
#-----------------------------------------
 
 
EXECUTABLE = main
OBJS       = main.o
HEADERS    = 
 
all: $(EXECUTABLE)
 
$(EXECUTABLE): $(OBJS)
	$(LINK) -o $(EXECUTABLE) $(OBJS) $(LFLAGS)
 
%.o : %.c $(HEADERS)
	$(GCC) $(CFLAGS) -c -o $@ $<
 
%.o : %.cpp $(HEADERS)
	$(GPP) $(CFLAGS) -c -o $@ $<
 
clean:
	$(RM) $(EXECUTABLE) $(OBJS) *~ src/*~ src/include/*~
 
rebuild: clean all
 
compressed: all
	$(UPX) $(UPXFLAGS) $(EXECUTABLE)
 
doc:
	$(DOCGEN)