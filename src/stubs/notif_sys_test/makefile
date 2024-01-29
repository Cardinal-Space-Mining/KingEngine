build:
	clear
	g++ ./notifier.cpp ./waiter.cpp -c -I../
	g++ ../*.cpp notifier.o -o ./notifier.exe
	g++ ../*.cpp waiter.o -o ./waiter.exe
 clean:
	rm ./*.o
	rm ./*.exe
