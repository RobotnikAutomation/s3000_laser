############################################################################
#			Copyright (c) 2010 Robotnik Automation, SLL 				   #
#			Makefile for standard Robotnik component    				   #
############################################################################

BUILD		=	./build/
BINDIR		=	./bin/
SRC		=	./src/
LIB 		= 	./lib/
#LIBS		=	 -L$(LIB) -lpthread -lremotelog -lcurses -lrt
INC		= 	./include/SerialDevice

CPP		    =	g++
CCFLAGS	    =	-Wall -c -g3 -I$(INC)
# -g3 -Wno-deprecated -Wall
OBJECTS =	\
			$(BUILD)SerialDevice.o \

default: $(BINDIR)$(EXE)

all: $(BINDIR)$(EXE)

$(BUILD)SerialDevice.o : $(SRC)SerialDevice.cc
			 $(CPP) $(CCFLAGS) -o $(BUILD)SerialDevice.o $(SRC)SerialDevice.cc

#$(BINDIR)$(EXE) : $(OBJECTS)
#	$(CPP) -I$(INC) -o $(BINDIR)$(EXE) $(OBJECTS) $(LIBS)

clean:
	rm -fv $(BUILD)*.o
	rm -fv $(BINDIR)$(EXE)
