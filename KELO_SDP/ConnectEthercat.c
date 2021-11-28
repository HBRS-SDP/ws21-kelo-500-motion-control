#include "soem/ethercat.h"
#include "soem/ethercattype.h"
#include "soem/nicdrv.h"
#include "soem/ethercatbase.h"
#include "soem/ethercatmain.h"
#include "soem/ethercatconfig.h"
#include "soem/ethercatcoe.h"
#include "soem/ethercatdc.h"
#include "soem/ethercatprint.h"
#include "kelo_tulip/include/kelo_tulip/KeloDriveAPI.h"
#include <stdio.h>

int main(int argc, char *argv[])
{
  ec_slavet ecx_slave[EC_MAXSLAVE];
  int ecx_slavecount;
  ec_groupt ec_group[EC_MAXGROUP];
  uint8 esibuf[EC_MAXEEPBUF];
  uint32 esimap[EC_MAXEEPBITMAP];
  ec_eringt ec_elist;
  ec_idxstackT ec_idxstack;

  ec_SMcommtypet ec_SMcommtype;
  ec_PDOassignt ec_PDOassign;
  ec_PDOdesct ec_PDOdesc;
  ec_eepromSMt ec_SM;
  ec_eepromFMMUt ec_FMMU;
  boolean EcatError;
  int64 ec_DCtime;
  ecx_portt ecx_port;
  ecx_redportt ecx_redport;
  ecx_contextt ecx_context;
  char IOmap[4096];

  ecx_context.port = &ecx_port;
  ecx_context.slavelist = &ecx_slave[0];
  ecx_context.slavecount = &ecx_slavecount;
  ecx_context.maxslave = EC_MAXSLAVE;
  ecx_context.grouplist = &ec_group[0];
  ecx_context.maxgroup = EC_MAXGROUP;
  ecx_context.esibuf = &esibuf[0];
  ecx_context.esimap = &esimap[0];
  ecx_context.esislave = 0;
  ecx_context.elist = &ec_elist;
  ecx_context.idxstack = &ec_idxstack;

  ecx_context.ecaterror = &EcatError;
  ecx_context.DCtime = &ec_DCtime;
  ecx_context.SMcommtype = &ec_SMcommtype;
  ecx_context.PDOassign = &ec_PDOassign;
  ecx_context.PDOdesc = &ec_PDOdesc;
  ecx_context.eepSM = &ec_SM;
  ecx_context.eepFMMU = &ec_FMMU;
  ecx_context.manualstatechange = 0;

  int nWheels = 4;
  int index_to_EtherCAT[4] = {2, 3, 5, 6};

  if (!ecx_init(&ecx_context, "enp2s0")) //enp2s0: port name on our PC to initiate connection
  {
    printf("Failed to initialize EtherCAT\n");
    return 0;
  }
  else
  {
    printf("Successfully established EtherCAT connection.")
  }

  if (!ecx_config_init(&ecx_context, TRUE)) //establishing connection with slave or autoconfig slaves
  {
    printf("NO SLAVES!\n");
    return 0;
  }
  else
  {
    printf("Found %i number of slaves. \n", ecx_slavecount);
  }
  ecx_config_map_group(&ecx_context, IOmap, 0);

  // reading all slave names w.r.t their id's
  for (int i = 1; i <= ecx_slavecount; i++)
  {
    printf("slave(index) \t%i has name \t%s\n", i, ecx_slave[i].name);
  }

  // waiting for all slaves to reach SAFE_OP state
  ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

  // checking if all slaves reached SAFE_OP state
  if (ecx_slave[0].state != EC_STATE_SAFE_OP)
  {
    printf("EtherCAT slaves have not reached safe operational state\n");
    ecx_readstate(&ecx_context);

    // If not all slaves operational find out the one which has not reached safe state
    for (int i = 1; i <= ecx_slavecount; i++)
    {
      if (ecx_slave[i].state != EC_STATE_SAFE_OP)
      {
        printf("Slave %i State= %i\n", i, ecx_slave[i].state);
      }
    }
    return 0; // if any slave has not reached the safe operational state, terminate the program
  }

  //  initialising message struct
  rxpdo1_t msg;
  msg.timestamp = 1;
  msg.command1 = 0;
  msg.limit1_p = 0;
  msg.limit1_n = 0;
  msg.limit2_p = 0;
  msg.limit2_n = 0;
  msg.setpoint1 = 0;
  msg.setpoint2 = 0;

  //  bulding initial message
  for (unsigned int i = 0; i < nWheels; i++)
  {
    rxpdo1_t *ecData = (rxpdo1_t *)ecx_slave[index_to_EtherCAT[i]].outputs;
    *ecData = msg;
  }

  //  sending built message
  ecx_send_processdata(&ecx_context);

  //  setting state to operational
  ecx_slave[0].state = EC_STATE_OPERATIONAL;

  //  sending built message after setting state as operational
  ecx_send_processdata(&ecx_context);

  //  receiving the responses from slaves
  ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

  //  write the response to struct
  ecx_writestate(&ecx_context, 0);

  //  checking if state is operational (by waiting for fixed time duration)
  ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); 
  if (ecx_slave[0].state != EC_STATE_OPERATIONAL)
  {
    printf("EtherCAT slaves have not reached operational state\n");
    return 0;
  }
  else
  {
    printf("Operational state reached for all EtherCAT slaves.\n");
  }

  //  sending control commands
  int counter = 0;
  
  while (counter < 15000)
  {
    counter += 1;
    rxpdo1_t msg;
    msg.timestamp = time(NULL);
    msg.command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_VELOCITY;
    msg.limit1_p = 20;
    msg.limit1_n = -20;
    msg.limit2_p = 20; 
    msg.limit2_n = -20;
    msg.setpoint1 = 2;  
    msg.setpoint2 = -2;

    //  bulding message with uniform setpoint values
    for (unsigned int i = 0; i < nWheels; i++) 
    {
      rxpdo1_t *ecData = (rxpdo1_t *)ecx_slave[index_to_EtherCAT[i]].outputs;
      *ecData = msg;
    }

    //  sending message with control commands
    ecx_send_processdata(&ecx_context);

    //  receiving response message
    ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

    //  reading encoder pivot angle of all wheel units (radians)
    for (unsigned int i = 0; i < nWheels; i++)
    {
      txpdo1_t *ecData = (txpdo1_t *)ecx_slave[index_to_EtherCAT[i]].inputs;
      printf("Pivot angle of wheel(id) %d: %f \n", nWheels[i], ecData->encoder_pivot);
    }
  }

  return 0;
}