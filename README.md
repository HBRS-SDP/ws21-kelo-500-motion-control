# ws21-kelo-500-motion-control

In *EthercatCommunication.c* change the port name to the active port on your PC in the line 
        
        if (!ecx_init(&ecx_context, "enp2s0"))
        {
        printf("Failed to initialize EtherCAT\n");
        return 0; 
        }
