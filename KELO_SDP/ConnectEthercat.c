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
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_sf_trig.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit.h>
// #include <time.h>

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
    ecx_context.manualstatechange = 0; // 0

    int nWheels = 4; //just for testing purpose
    int index_to_EtherCAT[4] = {3, 5};

    if (!ecx_init(&ecx_context, "enp2s0"))
    { //port name on our PC to initiate connection
        printf("Failed to initialize EtherCAT\n");
        return 0;
    }
    if (!ecx_config_init(&ecx_context, TRUE))
    { //establish first connection with slave or autoconfig slaves
        printf("NO SLAVES!\n");
        return 0;
    }
    ecx_config_map_group(&ecx_context, IOmap, 0); //PDO - process data object

    printf("%i\n", ecx_slavecount);
    printf("%s\n", ecx_slave[1].name);

    // Reading all slave names w.r.t their no.
    for (int i = 1; i <= ecx_slavecount; i++)
    {
        printf("slave \t%i has name \t%s\n", i, ecx_slave[i].name);
    }

    // wait for all slaves to reach SAFE_OP state
    ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    if (ecx_slave[0].state != EC_STATE_SAFE_OP)
    {
        printf("EtherCAT slaves have not reached safe operational state\n");
        ecx_readstate(&ecx_context);

        // If not all slaves operational find out which one
        for (int i = 1; i <= ecx_slavecount; i++)
        {
            if (ecx_slave[i].state != EC_STATE_SAFE_OP)
            {
                printf("Slave %i State= %i\n", i, ecx_slave[i].state); //<< ecx_slave[i].state << " StatusCode=" << ecx_slave[i].ALstatuscode << " : " << ec_ALstatuscode2string(ecx_slave[i].ALstatuscode) << "\n";
            }
        }
        return 0;
    }

    rxpdo1_t msg;
    msg.timestamp = 1;
    msg.command1 = 0;
    msg.limit1_p = 0;
    msg.limit1_n = 0;
    msg.limit2_p = 0;
    msg.limit2_n = 0;
    msg.setpoint1 = 0;
    msg.setpoint2 = 0;

    for (unsigned int i = 0; i < nWheels; i++)
    {
        rxpdo1_t *ecData = (rxpdo1_t *)ecx_slave[index_to_EtherCAT[i]].outputs;
        *ecData = msg;
    }

    ecx_send_processdata(&ecx_context); //Sending process data

    ecx_slave[0].state = EC_STATE_OPERATIONAL; //Setting state to operational

    ecx_send_processdata(&ecx_context);
    ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET); //receiving response from slaves

    ecx_writestate(&ecx_context, 0);

    ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); //Checking if operational

    if (ecx_slave[0].state != EC_STATE_OPERATIONAL)
    {
        printf("EtherCAT slaves have not reached operational state\n");
        return 0;
    }
    else
    {
        printf("Operational state reached for all EtherCAT slaves.\n");
    }

    time_t endwait;
    int seconds = 5;

    // endwait = time (NULL) + seconds ;
    int cnt = 0;
    double pivots[4];
    while (cnt < 30000)
    {
        cnt += 1;
        rxpdo1_t msg;
        msg.timestamp = time(NULL); // REASON?
        msg.command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_VELOCITY;
        msg.limit1_p = 20;
        msg.limit1_n = -20;
        msg.limit2_p = 20;  // pos limit
        msg.limit2_n = -20; // neg limit
        msg.setpoint1 = 0;  // rad/sec
        msg.setpoint2 = 0;

        // printf(time (NULL));

        for (unsigned int i = 0; i < nWheels; i++) // runs all wheels
        {
            rxpdo1_t *ecData = (rxpdo1_t *)ecx_slave[index_to_EtherCAT[i]].outputs;
            *ecData = msg;
        }

        ecx_send_processdata(&ecx_context); //Sending process data

        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

        for (unsigned int i = 0; i < nWheels; i++)
        {
            txpdo1_t *ecData = (txpdo1_t *)ecx_slave[index_to_EtherCAT[i]].inputs;
            pivots[i] = ecData->encoder_pivot;
            // printf("EC_DATA: %f \n", ecData->encoder_pivot);
        }
    }
 
    return 0;
}