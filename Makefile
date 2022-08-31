TARGET      = darwind

# compiler options
CC          = gcc
CCFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
LNKCC       = $(CX)
LNKFLAGS    = $(CXFLAGS) #-Wl,-rpath,$(DIR_THOR)/lib
FORMAT      = -m32

#---------------------------------------------------------------------
# Core components (all of these are likely going to be needed)
#---------------------------------------------------------------------
#INCLUDES   += -I$(DIR_DXL)/include/dynamixel_sdk
INCLUDES   += -I/usr/local/include/dynamixel_sdk
INCLUDES   += -I./include/
LIBRARIES  += -ldxl_x86_c
LIBRARIES  += -lrt


#---------------------------------------------------------------------
# Files
#---------------------------------------------------------------------
SOURCES = src/darwind.c 

#---------------------------------------------------------------------
# Compiling Rules
#---------------------------------------------------------------------
$(TARGET): $(SOURCES)
	$(CC) -c $(CCFLAGS) -g $(INCLUDES) $(SOURCES) -o $(TARGET) $(LIBRARIES)

all: $(TARGET)

clean:
	rm -rf $(TARGET) *~ *.a *.so *.lo

#---------------------------------------------------------------------
# End of Makefile
#---------------------------------------------------------------------
