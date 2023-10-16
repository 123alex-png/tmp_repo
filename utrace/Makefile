SRC=$(shell find *.cpp)
INC=$(shell find *.hh)
TARGET=test

$(TARGET):$(SRC) $(INC)
	g++ -std=c++20 -g $(SRC) -o $@ 

run: $(TARGET)
	./$<

.phony:run