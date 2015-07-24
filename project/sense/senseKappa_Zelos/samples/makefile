CC=g++
LOG4CXX=true

ifeq ($(LOG4CXX), true)
	CFLAGS=-o3 -g -c  -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES -I ../../include -D_LOG4CXX_ 
	LDFLAGS= -lsdk4acquire -lsdk4control -lgevtl -lpthread -llog4cxx
else
	CFLAGS=-o3 -g -c  -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES -I ../../include 
	LDFLAGS= -lsdk4acquire -lsdk4control -lgevtl -lpthread -ldl
endif

SOURCES=main.cpp SDK4SaveBuffer.cpp

SOURCES_C=

OBJECTS=$(SOURCES:.cpp=.o) $(SOURCES_C:.c=.o)
EXECUTABLE=acquire2disk

all: $(SOURCES) $(EXECUTABLE)

clean:

	rm -f acquire2disk
	rm -f main.o
	rm -f SDK4SaveBuffer.o
		
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS) 

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
	
.c.o:
	$(CC) $(CFLAGS) $< -o $@
	
