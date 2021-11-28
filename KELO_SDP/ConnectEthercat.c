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
    ecx_context.manualstatechange = 0; // 0 
  
  return 0;
}
