SRC=$(shell find *.cpp)
INC=$(shell find *.hh)
TARGET=build/test

$(TARGET):$(SRC) $(INC)
	mkdir -p build
	g++ -std=c++20 -g $(SRC) -o $@ 

run: $(TARGET)
	./$<

.phony:run