CC = gcc

CFLAGS =

TARGET = sim.exe

SRCS = main.c Queue.c dijkstra.c
DEPS = Parameters.h

$(TARGET): $(SRCS) $(DEPS)
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET)

ifeq ($(OS),Windows_NT)
    RM = del /Q
else
    RM = rm -f
endif

clean:
	$(RM) $(TARGET)