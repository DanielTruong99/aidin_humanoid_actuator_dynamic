/* It is revised by HJ at 24.01.16 */


#pragma once

/// EtherCAT Mater ///
#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>
#include <ethercatdc.h>
#include <ethercatcoe.h>
#include <ethercatfoe.h>
#include <ethercatconfig.h>
#include <ethercatprint.h>
#include <ethercat.h>
/// EtherCAT Mater ///

/// SYSTEM ///
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <iostream>
/// SYSTEM ///

////// XENOMAI3 //////
#include <alchemy/task.h>
#include <alchemy/sem.h>
#include <alchemy/mutex.h>
#include <alchemy/timer.h>
#include <trank/rtdk.h>
////// XENOMAI3 //////

#ifndef HAVE_RTNET
    /// Rename the standard functions
    /// And use theses ones to be RTnet-compatible when available

    #define rt_dev_socket     socket
    #define rt_dev_setsockopt setsockopt
    #define rt_dev_bind       bind
    #define rt_dev_recvfrom   recvfrom
    #define rt_dev_sendto     sendto
    #define rt_dev_close      close
    #define rt_dev_connect    connect
#else
    /// Use RTnet in Xenomai
    #include <rtdm/rtdm.h>
#endif

#define EC_TIMEOUTMON 500
#define INITIAL_POS 0

#define TORQUE_MODE 10

#define ELMO_VENDOR_ID 0x0000009a
#define ELMO_GOLD_PRODUCT_CODE 0x00030924

#define EVO_VENDOR_ID 0x0
#define EVO_PRODUCT_CODE 0xab123

// Number of X
#define NUM_ELMO 4
#define NUM_EVO 1 //나중에 연결하면 보는 걸로 하자
#define NUM_ACT (NUM_ELMO + NUM_EVO)

#define STATUSWORD_READY_TO_SWITCH_ON_BIT 0
#define STATUSWORD_SWITCHED_ON_BIT 1
#define STATUSWORD_OPERATION_ENABLE_BIT 2
#define STATUSWORD_FAULT_BIT 3

// ELMO WRITE PDO
#define TARGET_POSITION 0x607A0
#define TARGET_VELOCITY 0x60FF0
#define TARGET_TORQUE 0x60710
#define MAX_TORQUE 0x60720
#define CONTROL_WORD 0x60400
#define MODE_OF_OPERATION 0x60600
#define DIGITAL_OUTPUTS 0x60FE0

// ELMO READ PDO
#define POSITION_ACTUAL_VALUE 0x60640
#define TORQUE_ACTUAL_VALUE 0x60770
#define STATUS_WORD 0x60410
#define MODE_OF_OPERATION_DISPLAY 0x60610
#define VELOCITY_ACTUAL_VALUE 0x606C0
#define DC_LINK_CIRCUIT_VOLTAGE 0x60790
#define DIGITAL_INPUTS 0x60FD0
#define ANALOG_INPUT_1 0x22051
#define AUXILIARY_POSITION_ACTUAL_VALUE 0x20A00
#define CURRENT_ACTUAL_VALUE 0x60780

// EVO WRITE PDO
#define CONTROLWORD_evo 0x60401
#define TARGETPOSITION_evo 0x607A1
#define TARGETVELOCITY_evo 0x60FF1
#define TARGETTORQUE_evo 0x60B21
#define MODEofOPERATION_evo 0x60601

// EVO READ PDO
#define STATUSWORD_evo 0x60411
#define POSITION_ACTUALVALUE_evo 0x60641
#define VELOCITY_ACTUALVALUE_evo 0x606C1
#define TORQUE_ACTUALVALUE_evo 0x60771
#define MODEofOPERATION_DISPLAY_evo 0x60611

//////// Time functions ////////
int ms(double _time);
//////// Time functions ////////

///////// Namespace /////////
using namespace std;
///////// Namespace /////////
#define DCMODE 1

#if DCMODE == 1
    #ifndef SEC_IN_NSEC 
        #define SEC_IN_NSEC 1e9
    #endif
#endif
/**
 * helper macros
 */

 #define READ(slaveId, idx, sub, buf, comment)    \
 {   \
     buf=0;  \
     int __s = sizeof(buf);    \
     int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);rt_task_sleep(1e6);   \
     rt_printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment);    \
  }

#define CHECKERROR(slaveId)   \
{   \
 ec_readstate();\
 rt_printf("EC> \"%s\" %x - %x [%s] \n", (char*)ec_elist2string(), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char*)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode));    \
}

////// COMMUNICATION STRUCT //////
/// If PACK function isn't used, whole datasize should be 32 bits-multiple. So dummy data should be added to match 32 bits-multiple.
/// And data should be defined by size order from biggest to smallest.
/// If PACK function is used, it doesn't need to consider data size and order.
#pragma pack(push, 1)
struct ELMO_GOLD_IN
{
    int32_t		TargetPosition; 	// 0x607A
    int32_t		TargetVelocity; 	// 0x60FF
    int16_t		TargetTorque; 	// 0x6071
    uint16_t	MaxTorque; 	// 0x6072
    uint16_t	ControlWord; 	// 0x6040
    int8_t		ModeOfOperation; 	// 0x6060	
	int32_t		DigitalOutputs; 	// 0x60FE			
};
struct ELMO_GOLD_OUT
{
    int32_t		PositionActualValue; 	// 0x6064
    int32_t		VelocityActualValue; 	// 0x606C
    uint32_t	DCLinkCircuitVoltage; 	// 0x6079
    uint32_t	DigitalInputs; 	// 0x60FD
    int32_t		AuxiliaryPositionActualValue; 	// 0x20A0
    int16_t     AnalogInput1;   //0x2205
	int16_t		CurrentActualValue; 	// 0x6078
    int16_t		TorqueActualValue; 	// 0x6077
    uint16_t	StatusWord; 	// 0x6041
    int8_t		ModeOfOperationDisplay; 	// 0x6061
};

struct ELMO_GOLD
{
    struct ELMO_GOLD_IN 	*InParam;
    struct ELMO_GOLD_OUT 	*OutParam;
};

// #pragma pack(push, 1)
struct EVO_IN
{
    uint16_t	controlword;   // 0x6040
    int32_t		targetposition; 	// 0x607A
    int32_t		targetvelocity; 	// 0x60FF
    int32_t		targettorque; 	// 0x60B2
    uint16_t	modeofoperation; 	// 0x6060	
};
struct EVO_OUT
{
    uint16_t	statusword; 	// 0x6041
    int32_t		positionactualvalue; 	// 0x6064
    int32_t		velocityactualvalue; 	// 0x606C
    int32_t		torqueactualvalue; 	// 0x6077
    uint16_t	modeofoperationdisplay; 	// 0x6061
};

struct EVO
{
   struct EVO_IN 	*InParam;
   struct EVO_OUT 	*OutParam;
};
#pragma pack(pop)
////// COMMUNICATION STRUCT //////

/****************************************************************************/
// Class

class EthercatMaster
{
    private:
        static char IOmap[4096];

    public:
        #if NUM_ELMO != 1
            static ELMO_GOLD _elmo_gold[NUM_ELMO];
        #else
            static ELMO_GOLD _elmo_gold;
        #endif
        #if NUM_EVO != 1
            static EVO _evo[NUM_EVO];
        #else
            static EVO _evo;
        #endif
        
        EthercatMaster();
        ~EthercatMaster();

        static int init(const int8_t mode_op, const char *ifname);
        
        //int Elmosetup(uint16_t slave);
        static int initSlave();
        static int showSlaveInfo();
        static int registerParam();
        static int testDriveState(const int8_t mode_op);
        static int testDriveStateEVO(const uint16_t op_mode);
 
        static void processRxDomain();
        static int processTxDomain();
       

        static void readBuffer(const int EntryID, void* data);
        static void writeBuffer(const int EntryID, void* data);
        
        // void elmoservoOn(const int position);
        // void evoservoOn(const int position);
        // servo 통합
        static void servoOn(const int position, const std::string& type);

        void CheckEtherCATState();

        static int expectedWKC;
        static int ControlWord;
        static int controlword;


        /// DC Functions & Variables ///
        RTIME nowSub, previousSub;
        RTIME nowCheck, previousCheck;
        RTIME nowSave, previousSave;

        static int dorun;
        static int okop;
        static int64_t toff;
        static uint64_t overruns;
        static RTIME cycle_ns_ethercat; /* 1 ms */

        double timet_sub = 0;
        double timet_ecat = 0;
        double timet_save_sub = 0;
        double dtCtrl = 0;

        RTIME cycletime; // cycletime in ns 

        // void CheckEtherCATState();
        void InitDCMODE(double _ms);
        void SetLoopTime(double _freq);
        void SetLoopTimeCheck(double _freq);
        void RunEtherCATCheck();
        void WaitTime();
        static void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime);
        static void wait_period(RTIME wakeup_count);
        static void add_timespec(RTIME *ts, RTIME addtime);

        #if DCMODE == 1
            RTIME ts, ht;
            RTIME timeOffset;
            RTIME msConvert = 1000000;      // ms to sec
            RTIME tsbefore;               
            int ecatInitialcount = 0;
            int waitperiod_flag = 1;
            struct timespec waitperiod_check_time;

            struct timespec ecatcheckTime;
            unsigned int cycletimeCheck;
        #endif

        /// Sto ErrorReset variables ///
        static int sto_on[NUM_ELMO];
        static int error_flag[NUM_ELMO];
};
