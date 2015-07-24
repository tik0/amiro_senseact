CC=g++

LOG4CXX=false

ifeq ($(LOG4CXX), true)
	CFLAGS=-o3 -g -c  -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES -I ../../include -D_LOG4CXX_
	LDFLAGS= -lsdk4acquire -lsdk4control -lgevtl -lpthread -llog4cxx
else
	CFLAGS=-o3 -g -c  -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES -I ../../include 
	LDFLAGS= -lsdk4acquire -lsdk4control -lgevtl -lpthread -ldl
endif
SOURCES=main.cpp 

SOURCES_C=

OBJECTS=$(SOURCES:.cpp=.o) $(SOURCES_C:.c=.o)
EXECUTABLE=store

all: $(SOURCES) $(EXECUTABLE)

clean:

	rm -f store
	rm -f main.o
		
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS) 

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
	
.c.o:
	$(CC) $(CFLAGS) $< -o $@
	
