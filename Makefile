# Makefile for disk scheduling simulator

CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2
TARGET = disk
SOURCE = disk.cpp

all: $(TARGET)

$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE)

clean:
	rm -f $(TARGET)

test: $(TARGET)
	./$(TARGET) -c

.PHONY: all clean test
