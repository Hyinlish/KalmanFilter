INC = kalman.o

app : $(INC) test.o
	g++ -c test.cpp
	g++ test.o $(INC) -o app

$(INC) : kalman.cpp
	g++ -c kalman.cpp

# 清理
clean:
	rm ./*.o
# 运行文件
run:
	./app