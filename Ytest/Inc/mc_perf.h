/* mc_perf.h */

typedef enum {
  MEASURE_TSK_HighFrequencyTask,
  MEASURE_TSK_MediumFrequencyTaskM1
//  Others functions to measure to be added here.
}MC_PERF_FUNCTIONS_LIST_t;

/* Define max number of traces according to the list defined in MC_PERF_FUNCTIONS_LIST_t */
#define  MC_PERF_NB_TRACES  2

/* DWT (Data Watchpoint and Trace) registers, only exists on ARM Cortex with a DWT unit */
/* The DWT is usually implemented in Cortex-M3 or higher, but not on Cortex-M0(+) (ie not present on G0) */

typedef struct {
    uint32_t  StartMeasure;
    uint32_t  DeltaTimeInCycle;
    uint32_t  min;
    uint32_t  max;
} Perf_Handle_t;

typedef struct {
    bool   BG_Task_OnGoing;
    uint32_t  AccHighFreqTasksCnt;
    Perf_Handle_t MC_Perf_TraceLog[MC_PERF_NB_TRACES];
} MC_Perf_Handle_t;

void MC_Perf_Measure_Init  (MC_Perf_Handle_t * pHandle);
void MC_Perf_Clear(MC_Perf_Handle_t *pHandle);
void MC_Perf_Measure_Start (MC_Perf_Handle_t * pHandle, uint8_t i);
void MC_BG_Perf_Measure_Start (MC_Perf_Handle_t * pHandle, uint8_t i);
void MC_Perf_Measure_Stop  (MC_Perf_Handle_t * pHandle, uint8_t i);
void MC_BG_Perf_Measure_Stop  (MC_Perf_Handle_t * pHandle, uint8_t i);

float MC_Perf_GetCPU_Load( MC_Perf_Handle_t * pHandle );
float MC_Perf_GetMaxCPU_Load( MC_Perf_Handle_t * pHandle );
float MC_Perf_GetMinCPU_Load( MC_Perf_Handle_t * pHandle );
