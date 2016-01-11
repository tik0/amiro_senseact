# STEMMER IMAGING
# 
# makefile for GrabConsoleExample
# usage:
# make all
# or 
# make debug 

SOURCES = Grabber.cpp

ifeq "$(MAKECMDGOALS)" "debug"
 CPPFLAGS = -D_DEBUG -ggdb3 -O0
 TGT = Grabber_d 
 CONFIGURATION = Debug
else 
 CPPFLAGS = -O3 
 TGT = Grabber 
 CONFIGURATION = Release
endif

CPPFLAGS += -I$(CVB)/include
LDFLAGS = -L$(CVB)/lib
LDLIBS= -lCVCImg -lCVCUtilities -lCVCDriver -lCVGenApi -lopencv_core -lopencv_highgui -lopencv_imgproc
CONFIGURATIONPREFIX= ./$(CONFIGURATION)/
DEPENDENCIES= $(addprefix $(CONFIGURATIONPREFIX),$(SOURCES:.cpp=.d) )  

%.o: %.cpp
	@mkdir -p $(CONFIGURATION)
	@echo Building CXX object $(CONFIGURATION)/$@
	@$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -o $(CONFIGURATION)/$@ $<

OBJECTS = $(addprefix $(CONFIGURATIONPREFIX),$(SOURCES:.cpp=.o) )

$(TGT): $(SOURCES:.cpp=.o) 
	@echo Linking CXX executable $(CONFIGURATION)/$(TGT)
	@$(CXX) $(LDFLAGS) $(OBJECTS) -o $(CONFIGURATION)/$(TGT) $(LDLIBS)

all: $(TGT)

.PHONY: clean
clean:
	@rm -rf Debug
	@rm -rf Release

debug: $(TGT)


