#ifndef BQ_DATA_H
#define BQ_DATA_H

#include "bms_data_types.h"
#include "bq_comm.h"

// --- Function Prototypes ---

// Data Acquisition
BMSErrorCode_t bqGetAllCellVoltages(BMSOverallData_t *bmsData);
BMSErrorCode_t bqGetAllTemperatures(BMSOverallData_t *bmsData); 
BMSErrorCode_t bqGetStackFaultStatus(BMSOverallData_t *bmsData); 
BMSErrorCode_t bqGetBridgeFaultStatus(BMSOverallData_t *bmsData); 

// Data processing
void processRawVoltages(BMSOverallData_t *bmsData);
void processRawTemperatures(BMSOverallData_t *bmsData);
void updatePackStatistics(BMSOverallData_t *bmsData); 
void printBQDump();


// Application Logic
uint8_t estimateSOC(const BMSOverallData_t *bmsData);
void readSystemInputs(BMSOverallData_t *bmsData);
float resistanceToTemperature(float resistance); // <-- ADDED DECLARATION

// Balancing related
BMSErrorCode_t bqStartCellBalancing(BMSOverallData_t *bmsData, uint8_t moduleIndex, uint8_t cellToBalanceIndex, uint8_t duration_code);
BMSErrorCode_t bqStopCellBalancing(uint8_t moduleIndex, uint8_t cellToBalanceIndex);
BMSErrorCode_t bqCheckBalancingStatus(uint8_t moduleIndex, BMSOverallData_t *bmsData);

#endif // BQ_DATA_H