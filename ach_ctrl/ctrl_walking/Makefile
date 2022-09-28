default: all

CFLAGS := -I./../../include  -Wall -I./include -I./src/minIni 
LFLAGS += -lpthread -ldl

CC := g++
AR = ar
ARFLAGS = cr
TARGET = darwin.a

BINARIES := test
all : $(BINARIES)

obj : $(TARGET)

LIBS := -lrt -lstdc++ -lach 

OBJS =  src/MX28.o             \
        src/math/Matrix.o      \
        src/math/Plane.o       \
        src/math/Point.o       \
        src/math/Vector.o      \
        src/JointData.o        \
        src/Kinematics.o       \
        src/MotionStatus.o     \
        src/Walking.o          \
        src/minIni/minIni.o    
#	src/LinuxMotionTimer.o

$(TARGET): $(OBJS)
	$(AR) $(ARFLAGS) $(TARGET) $(OBJS)
#	$(AR) $(ARGLAGS) $(TARGET) $(LFLAGS)
#	$(CC) -o $(TARGET) $(OBJS) $(LFLAGS)


$(BINARIES): src/test.o
	$(CC) -o $@ $< $(LIBS) $(OBJS)

%.o: %.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o

