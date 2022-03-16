setup_ubuntu:
	gcc -I"../install/include/" -L"../install/lib/" EthercatCommunication.c -lsoem -lpthread -lgsl -ldl -lgslcblas -lm -o setup_output

setup_mac:
	gcc -I"../install/include/" -L"../install/lib/" EthercatCommunication.c -lsoem -lpthread -lgsl -lcap -ldl -lgslcblas -lm -o setup_output

run_setup:
	sudo ./setup_output no_debug

run_debug_setup:
	sudo ./setup_output debug
	
clean:
	rm -f setup_output
