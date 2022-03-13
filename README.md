# ws21-kelo-500-motion-control

In *EthercatCommunication.c* change the port name to the active port on your PC in the line 
        
        if (!ecx_init(&ecx_context, "enp2s0"))
        {
        printf("Failed to initialize EtherCAT\n");
        return 0; 
        }
        
## Compilation 

To compile the project

For ubuntu users: Use **setup_ubuntu** from MakeFile. 

        setup_ubuntu:
                gcc -I"../install/include/" -L"../install/lib/" setup.c -lsoem -lpthread -lgsl -ldl -lgslcblas -lm -o setup_output

For macOS: Use **setup_mac** from MakeFile.

        setup_mac:
                gcc -I"../install/include/" -L"../install/lib/" setup.c -lsoem -lpthread -lgsl -lcap -ldl -lgslcblas -lm -o setup_output
