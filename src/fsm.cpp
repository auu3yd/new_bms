#include "fsm.h"
#include "config.h"
#include "bq_comm.h"
#include "bq_data.h"     
#include "fault_handler.h" 
#include "can_bus.h"
#include "B0_reg.h"      
#include <math.h>        
#include <string.h> 


FSM_State_t g_currentState;
// BMSOverallData_t g_bmsData; // Definition moved to bq_data.cpp

unsigned long g_lastFsmRunTime = 0;
unsigned long g_lastCanTransmitTime = 0;
unsigned long fault_startup_error_entry_ts = 0;
static unsigned long startup_attempt_timestamp = 0;

void read_and_print_bq_control_register(const char* stage_msg, uint8_t dev_id, uint16_t reg_addr, const char* reg_name) {
    uint8_t current_val;
    BMSErrorCode_t status = bqReadReg(dev_id, reg_addr, &current_val, 1, FRMWRT_SGL_R);
    if (status == BMS_OK) {
        BMS_DEBUG_PRINTF("%s: Dev 0x%02X Reg %s (0x%04X) = 0x%02X\n", stage_msg, dev_id, reg_name, reg_addr, current_val);
    } else {
        BMS_DEBUG_PRINTF("%s: Dev 0x%02X Reg %s (0x%04X) = READ FAILED (Error %d)\n", stage_msg, dev_id, reg_name, reg_addr, status);
    }
}

const char* fsm_state_to_string(FSM_State_t state) {
    switch (state) {
        case FSM_STATE_INITIAL: return "INITIAL";
        case FSM_STATE_STARTUP: return "STARTUP";
        case FSM_STATE_NORMAL_OPERATION: return "NORMAL_OPERATION";
        case FSM_STATE_CELL_BALANCING: return "CELL_BALANCING";
        case FSM_STATE_FAULT_COMMUNICATION: return "FAULT_COMMUNICATION";
        case FSM_STATE_FAULT_MEASUREMENT: return "FAULT_MEASUREMENT";
        case FSM_STATE_FAULT_STARTUP_ERROR: return "FAULT_STARTUP_ERROR";
        case FSM_STATE_FAULT_CRITICAL: return "FAULT_CRITICAL";
        case FSM_STATE_SHUTDOWN_PROCEDURE: return "SHUTDOWN_PROCEDURE";
        case FSM_STATE_SYSTEM_OFF: return "SYSTEM_OFF";
        default: return "UNKNOWN_STATE";
    }
}

void fsm_change_state(FSM_State_t newState) {
    if (g_currentState != newState) {
        BMS_DEBUG_PRINTF("FSM: %s -> %s\n", fsm_state_to_string(g_currentState), fsm_state_to_string(newState));
        g_currentState = newState;
    }
}

void fsm_init() {
    Serial.println("FSM Init");
    memset(&g_bmsData, 0, sizeof(BMSOverallData_t)); // Initialize global BMS data
    initFaultHandler(); 
    bqInitCommunication(); 
    can_init();

    pinMode(FAN_PIN, OUTPUT); digitalWrite(FAN_PIN, LOW);
    pinMode(RESET_PIN, INPUT_PULLDOWN); 
    pinMode(IMD_STATUS_PIN, INPUT_PULLDOWN); // Assuming pull-down if signal is active HIGH for OK
    pinMode(POS_AIR_STATUS_PIN, INPUT_PULLDOWN);
    pinMode(NEG_AIR_STATUS_PIN, INPUT_PULLDOWN);
    pinMode(IMD_ERROR_PIN, OUTPUT); digitalWrite(IMD_ERROR_PIN, LOW);
    pinMode(AMS_ERROR_PIN, OUTPUT); digitalWrite(AMS_ERROR_PIN, LOW);

    fsm_change_state(FSM_STATE_INITIAL);
}

void fsm_run() {
    if (millis() - g_lastFsmRunTime < STATE_MACHINE_INTERVAL_MS) {
        return; 
    }
    g_lastFsmRunTime = millis();

    FSM_State_t nextState = g_currentState; 

    readNFaultPin(); 
    readSystemInputs(&g_bmsData); 

    switch (g_currentState) {
        case FSM_STATE_INITIAL: action_initial(); break;
        case FSM_STATE_STARTUP: action_startup(); break;
        case FSM_STATE_NORMAL_OPERATION: action_normal_operation(); break;
        case FSM_STATE_CELL_BALANCING: action_cell_balancing(); break;
        case FSM_STATE_FAULT_COMMUNICATION: action_fault_communication(); break;
        case FSM_STATE_FAULT_MEASUREMENT: action_fault_measurement(); break;
        case FSM_STATE_FAULT_STARTUP_ERROR: action_fault_startup_error(); break;
        case FSM_STATE_FAULT_CRITICAL: action_fault_critical(); break;
        case FSM_STATE_SHUTDOWN_PROCEDURE: action_shutdown_procedure(); break;
        case FSM_STATE_SYSTEM_OFF: action_system_off(); break;
        default: Serial.println("FSM: Unknown state in action dispatch!"); break;
    }

    processBMSFaults(&g_bmsData); 

    switch (g_currentState) {
        case FSM_STATE_INITIAL: nextState = transition_initial(); break;
        case FSM_STATE_STARTUP: nextState = transition_startup(); break;
        case FSM_STATE_NORMAL_OPERATION: nextState = transition_normal_operation(); break;
        case FSM_STATE_CELL_BALANCING: nextState = transition_cell_balancing(); break;
        case FSM_STATE_FAULT_COMMUNICATION: nextState = transition_fault_communication(); break;
        case FSM_STATE_FAULT_MEASUREMENT: nextState = transition_fault_measurement(); break;
        case FSM_STATE_FAULT_STARTUP_ERROR: nextState = transition_fault_startup_error(); break;
        case FSM_STATE_FAULT_CRITICAL: nextState = transition_fault_critical(); break;
        case FSM_STATE_SHUTDOWN_PROCEDURE: nextState = transition_shutdown_procedure(); break;
        case FSM_STATE_SYSTEM_OFF: nextState = transition_system_off(); break; 
        default:
            Serial.println("FSM: Unknown state in transition dispatch! Resetting to INITIAL.");
            nextState = FSM_STATE_INITIAL;
            break;
    }

    fsm_change_state(nextState); // Use fsm_change_state to handle print only on change

    if (millis() - g_lastCanTransmitTime >= CAN_TRANSMIT_INTERVAL_MS) {
        if (g_currentState == FSM_STATE_NORMAL_OPERATION || g_currentState == FSM_STATE_CELL_BALANCING || 
            g_currentState == FSM_STATE_FAULT_MEASUREMENT || g_currentState == FSM_STATE_FAULT_COMMUNICATION ||
            g_currentState == FSM_STATE_FAULT_STARTUP_ERROR || g_currentState == FSM_STATE_FAULT_CRITICAL) { 
            can_send_bms_data(&g_bmsData); 
            g_lastCanTransmitTime = millis();
        }
    }
}

float convertTemperatureToNtcVoltage(float temperatureC) {
    float min_R = 10.0f;      
    float max_R = 500000.0f;  
    float best_R = NTC_R_REF;  
    float min_temp_diff_at_best_R = 1000.0f; // Store the diff for best_R

    int iterations = 100; 
    for (int i = 0; i < iterations; ++i) {
        float mid_R = (min_R + max_R) / 2.0f;
        if (mid_R <= 0.1f) mid_R = 0.1f; 
        
        float calc_temp = resistanceToTemperature(mid_R); // Ensure resistanceToTemperature is declared in bq_data.h
        float temp_diff = calc_temp - temperatureC;

        if (fabsf(temp_diff) < min_temp_diff_at_best_R) { // If this R is better
             min_temp_diff_at_best_R = fabsf(temp_diff);
             best_R = mid_R;
        }

        if (fabsf(temp_diff) < 0.01f) { 
            break; 
        }

        if (temp_diff < 0) { 
            max_R = mid_R;
        } else { 
            min_R = mid_R;
        }
        // best_R already updated if mid_R was better.
    }
    // BMS_DEBUG_PRINTF("Temp %.1fC -> Best R %.0f Ohm (final diff %.2fC)\n", temperatureC, best_R, min_temp_diff_at_best_R);
    
    float v_ntc_volts = (NTC_V_TSREF * best_R) / (NTC_R_DIVIDER + best_R);
    return v_ntc_volts * 1000.0f; 
}

BMSErrorCode_t applyEssentialStackConfigsForRecovery(){
    Serial.println("Applying essential stack configurations for recovery...");
    BMSErrorCode_t status = BMS_OK;
    BMSErrorCode_t current_op_status;

    uint8_t reg_val_8bit = (uint8_t)(CELLS_PER_SLAVE - 1);
    current_op_status = bqStackWrite(ACTIVE_CELL, reg_val_8bit, 1);
    if(current_op_status != BMS_OK) { Serial.println("Recovery: Failed ACTIVE_CELL"); status = current_op_status;}

    // Read-modify-write for CONTROL2 is tricky for stack. Sample writes 0x01.
    current_op_status = bqStackWrite(CONTROL2, 0x01, 1); // TSREF_EN
    if(current_op_status != BMS_OK) { Serial.println("Recovery: Failed CONTROL2"); if(status == BMS_OK) status = current_op_status;}
    
    delayMicroseconds(1); // For TSREF

    current_op_status = bqStackWrite(ADC_CTRL1, 0x0E, 1); // MAIN_GO=1, Continuous
    if(current_op_status != BMS_OK) { Serial.println("Recovery: Failed ADC_CTRL1"); if(status == BMS_OK) status = current_op_status;}
    
    // Re-enable protectors
    current_op_status = bqStackWrite(OVUV_CTRL, 0x07, 1); 
    if(current_op_status != BMS_OK) { Serial.println("Recovery: Failed OVUV_CTRL"); if(status == BMS_OK) status = current_op_status;}
    current_op_status = bqStackWrite(OTUT_CTRL, 0x05, 1); 
    if(current_op_status != BMS_OK) { Serial.println("Recovery: Failed OTUT_CTRL"); if(status == BMS_OK) status = current_op_status;}

    return status;
}

void action_initial() { }
FSM_State_t transition_initial() { return FSM_STATE_STARTUP; }

void action_startup() {
    Serial.println("Action: STARTUP");
    BMSErrorCode_t status;
    uint8_t reg_val_8bit;
    uint8_t dummy_read_buffer[MAX_READ_DATA_BYTES]; 
    
    g_bmsData.communicationFault = false; 
    // fault_startup_error_entry_ts is managed by fsm_change_state

    bqShutdownDevices(); // Ensure all devices are in shutdown state before startup
    delay(100); // Allow devices to settle after shutdown

    // 1. MCU sends WAKE ping to BQ79600-Q1.
    Serial.println("[1] Waking up BQ79600 bridge...");
    status = bqWakeUpBridge();
    if (status != BMS_OK) { g_bmsData.communicationFault = true; Serial.println("Startup Error: Bridge wakeup failed."); return; }
    delayMicroseconds(10); 

    // 2. Single device write to BQ79600-Q1 CONTROL1 [SEND_WAKE] = 1 (wake up stack devices)
    //    (BQ79600 CONTROL1 is 0x0309, SEND_WAKE is bit 5)

    // TODO: Does BQWakeUpStack() not already do this? -- in which case, why isn't it being used?

    // Serial.println("[2] Commanding bridge to wake up BQ79616 stack...");
    // status = bqWakeUpStack();
    // if (status != BMS_OK) { g_bmsData.communicationFault = true; Serial.println("Startup Error: Stack wakeup failed."); return; }
    // delayMicroseconds(10); 
    // Serial.println("[3] Auto Addressing...");
    // status = bqAutoAddressStack();
    // if (status != BMS_OK) { g_bmsData.communicationFault = true; Serial.println("Startup Error: Stack autoaddress failed."); return; }
    // delayMicroseconds(10); 
    // We need to read BQ79600.CONTROL1, set SEND_WAKE, and write it back.
    // ADDR_WR on BQ79600 should also be set here for it to take address 0.
    uint8_t bq79600_ctrl1_val;
    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_CONTROL1, &bq79600_ctrl1_val, 1, FRMWRT_SGL_R);
    if (status != BMS_OK) { g_bmsData.communicationFault = true; Serial.println("Startup Error (Daisy S2): Failed to read BQ79600.CONTROL1 for SEND_WAKE."); return; }

    bq79600_ctrl1_val |= BQ79600_CTRL1_SEND_WAKE_BIT; // Set SEND_WAKE
    bq79600_ctrl1_val |= BQ79600_CTRL1_ADDR_WR_BIT;  // Enable ADDR_WR for BQ79600

    status = bqWriteReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_CONTROL1, bq79600_ctrl1_val, 1, FRMWRT_SGL_W);
    if (status != BMS_OK) { g_bmsData.communicationFault = true; Serial.println("Startup Error (Daisy S2): Stack wakeup command (SEND_WAKE/ADDR_WR) failed."); return; }
    read_and_print_bq_control_register("After Daisy S2", BQ79600_BRIDGE_DEVICE_ID, BQ79600_CONTROL1, "BQ79600.CONTROL1");
    delayMicroseconds(10 + STACK_WAKE_PROPAGATION_DELAY_US / 1000); 

    // 3. Auto-Address (Daisy Chain / Northbound using DIR0_ADDR):
    Serial.println("Daisy Step 3: Auto-Addressing Stack (Northbound)...");
    
    // 3a. Dummy broadcast writes to OTP_ECC_DATAIN1-8 (0x036B to 0x0372) for DLL sync.
    //     Using the corrected defines from user: OTP_ECC_DATAIN1 (0x343) to OTP_ECC_DATAIN8 (0x34A)
    Serial.println("  DLL Sync (dummy broadcast writes to OTP_ECC_DATAIN1-8)...");
    const uint16_t dummy_dll_regs[] = {
        OTP_ECC_DATAIN1, OTP_ECC_DATAIN2, OTP_ECC_DATAIN3, OTP_ECC_DATAIN4,
        OTP_ECC_DATAIN5, OTP_ECC_DATAIN6, OTP_ECC_DATAIN7, OTP_ECC_DATAIN8
    };
    for (int k_reg = 0; k_reg < 8; ++k_reg) {
        status = bqBroadcastWrite(dummy_dll_regs[k_reg], 0x00, 1);
        if (status != BMS_OK) { BMS_DEBUG_PRINTF("Startup Warning (Daisy S3a): Dummy broadcast write to 0x%04X failed.\n", dummy_dll_regs[k_reg]); }
    }

    // 3b. Broadcast write to BQ79616.CONTROL1 (0x0309) to set ADDR_WR=1.
    Serial.println("  Enable ADDR_WR on stack devices (CONTROL1=0x01)...");
    // Data 0x01 sets Bit 0 (ADDR_WR)=1 in BQ79616.CONTROL1. DIR_SEL (Bit 7) should be 0 for North.
    status = bqBroadcastWrite(CONTROL1, 0x01, 1); 
    if (status != BMS_OK) { g_bmsData.communicationFault = true; Serial.println("Startup Error (Daisy S3b): Broadcast Write for stack ADDR_WR failed."); return; }
    delayMicroseconds(5);

    // 3c. Broadcast write sequential addresses to DIR0_ADDR (0x0306).
    //     (Address 0 for bridge, 1 for first BQ79616, etc.)
    //     BQ79600's CONTROL1[ADDR_WR] was set in Step 2.
    /*  ----- 3c : write sequential addresses --------------------- */
for (uint8_t i = 0; i < NUM_BQ79616_DEVICES; ++i) {
    bqBroadcastWrite(DIR0_ADDR, i + 1, 1);   // addr 1, 2, …
}

/*  NEW — Step 4: exit address-write mode  */
bqBroadcastWrite(CONTROL1, 0x00, 1);         // ADDR_WR = 0
delayMicroseconds(150);                      // ≥100 µs guard time
/*  ----------------------------------------------------------- */

/*  ----- 3d : put devices into stack mode -------------------- */
bqBroadcastWrite(COMM_CTRL, 0x02, 1);         // STACK_DEV = 1

    // 3d. Set COMM_CTRL (0x0308) for bridge, stack devices, and ToS.
    Serial.println("  Configure BQ79616s as stack devices (COMM_CTRL=0x02)...");
    status = bqBroadcastWrite(COMM_CTRL, 0x02, 1); // STACK_DEV=1, TOP_STACK=0 for BQ79616s
    delayMicroseconds(100);
    if (status != BMS_OK) { g_bmsData.communicationFault = true; Serial.println("Startup Error (Daisy S3d): Broadcast write for BQ79616 COMM_CTRL failed."); return; }

    if (NUM_BQ79616_DEVICES > 0) {
        uint8_t tos_address = NUM_BQ79616_DEVICES; 
        BMS_DEBUG_PRINTF("  Configure ToS device (Addr %d) COMM_CTRL=0x03...\n", tos_address);
        status = bqWriteReg(tos_address, COMM_CTRL, 0x03, 1, FRMWRT_SGL_W); 
        delayMicroseconds(100);
        if (status != BMS_OK) { BMS_DEBUG_PRINTF("Startup Error (Daisy S3d): Configure ToS device (Addr %d) failed.\n", tos_address); g_bmsData.communicationFault = true; return; }
    }
    Serial.println("  Configure BQ79600 (Addr 0) as base (COMM_CTRL=0x00)...");
    status = bqWriteReg(BQ79600_BRIDGE_DEVICE_ID, COMM_CTRL, 0x00, 1, FRMWRT_SGL_W); 
    delayMicroseconds(100);
    if (status != BMS_OK) { Serial.println("Startup Error (Daisy S3d): Configure bridge as base device failed."); g_bmsData.communicationFault = true; return; }

    // 3e. Dummy stack reads from OTP_ECC_DATAIN1-8.
    Serial.println("  DLL Sync (dummy stack reads OTP_ECC_DATAIN1-8)...");
    for (int k_reg = 0; k_reg < 8; ++k_reg) {
        status = bqStackRead(dummy_dll_regs[k_reg], dummy_read_buffer, 1); 
        if (status != BMS_OK && status != BMS_ERROR_CRC && status != BMS_ERROR_COMM_TIMEOUT) { 
             BMS_DEBUG_PRINTF("Startup Warning (Daisy S3e): Dummy stack read from 0x%04X failed.\n", dummy_dll_regs[k_reg]);
        }
    }

    // 3f. Verify Auto-Addressing by reading DIR0_ADDR from stack devices
    Serial.println("  Verifying stack device addresses (DIR0_ADDR)...");
    bool addressing_ok = true;
    /* buffer order after broadcast read is ToS → ... → bridge */
    uint8_t addr_read_buffer[NUM_BQ79616_DEVICES + 1];
    if (NUM_BQ79616_DEVICES > 0) {
        status = bqBroadcastRead(DIR0_ADDR,
                                 addr_read_buffer,
                                 1,
                                 SERIAL_TIMEOUT_MS * (NUM_BQ79616_DEVICES + 1));
        if (status != BMS_OK) {
            Serial.println("Startup Error (Daisy S3f): Broadcast read for DIR0_ADDR verification failed.");
            addressing_ok = false;
            g_bmsData.communicationFault = true;
        } else {
                      /* frame[0] = ToS (addr = NUM_BQ79616_DEVICES), so flip index */
            for (uint8_t idx = 0; idx < NUM_BQ79616_DEVICES; ++idx) {
                uint8_t frame      = addr_read_buffer[idx];
                uint8_t exp_addr   = NUM_BQ79616_DEVICES - idx;  /* ToS, …, 1 */
                if (frame != exp_addr) {
                   BMS_DEBUG_PRINTF("ERROR (S3f): device exp %d got 0x%02X\n",
                                      exp_addr, frame);
                   addressing_ok = false;
               } else 
               {                    BMS_DEBUG_PRINTF("S3f OK: device addr %d present\n", frame);
               }
            }
    }
}
    // Verify BQ79600 address
    uint8_t bridge_dir0_addr;
    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, DIR0_ADDR, &bridge_dir0_addr, 1, FRMWRT_SGL_R);
    if (status == BMS_OK) {
        if (bridge_dir0_addr == BQ79600_BRIDGE_DEVICE_ID) {
            BMS_DEBUG_PRINTF("Daisy S3f: Bridge Addr %d DIR0_ADDR OK: 0x%02X\n", BQ79600_BRIDGE_DEVICE_ID, bridge_dir0_addr);
        } else {
            BMS_DEBUG_PRINTF("ERROR (Daisy S3f): Bridge Addr %d reported DIR0_ADDR 0x%02X\n", BQ79600_BRIDGE_DEVICE_ID, bridge_dir0_addr);
            addressing_ok = false; g_bmsData.communicationFault = true;
        }
    } else {
        Serial.println("Startup Error (Daisy S3f): Failed to read Bridge DIR0_ADDR.");
        addressing_ok = false; g_bmsData.communicationFault = true;
    }
    if (!addressing_ok) return;


    // 3g. Broadcast write to FAULT_RST1 & FAULT_RST2 to clear communication faults.
    // (This is done by resetAllBQFaults later, but example did it here too)
    Serial.println("  Resetting stack comm faults (FAULT_RST1/2)...");
    //Serial.println("Daisy Step 3: Auto-Addressing Complete.");
    //if (g_bmsData.communicationFault) return;
    

    // -----------------------------------------------------------------------------------------------------------
    // all the stuff above this, I think there is already a function for. I need to compare the two but will do it later
    // -----------------------------------------------------------------------------------------------------------

    // 4. Set Registers (Detailed BQ79616 Configurations)
    // TODO: I think this should end up either refactored or rewritten to be more readable. 
    // This is one of those things I want to be able to change one day (and probably evaluate how our .h files are set up)

    Serial.println("[4] Configuring Stack...");
    // status = bqConfigureStackForHeartbeat(); 
    // if (status != BMS_OK) { g_bmsData.communicationFault = true; Serial.println("Startup Error (Daisy S4): Stack heartbeat config failed."); return; }

    // this replaced the pile of gemini code that was here before
    bool retFlag;
    configure_stack(reg_val_8bit, status, retFlag);
    if (retFlag)
        return;

    // 5. Final Fault Reset
    Serial.println("[5] Resetting all BQ device faults post-configuration...");
    status = resetAllBQFaults(); 
    if (status != BMS_OK) {
        Serial.println("Startup Warning (Daisy S5): Failed to reset BQ faults post-configuration.");
    }
    
    Serial.println("Startup sequence complete. ");
    g_bmsData.balancing_request = false; 
    g_bmsData.balancing_cycle_active = false;
    for(int i=0; i<NUM_BQ79616_DEVICES; ++i) g_bmsData.activeBalancingCells[i] = 0;
    
    // Print some final states for BQ79600
    // read_and_print_bq_control_register("End of Daisy Startup", BQ79600_BRIDGE_DEVICE_ID, BQ79600_CONTROL1, "BQ79600.CONTROL1");
    // read_and_print_bq_control_register("End of Daisy Startup", BQ79600_BRIDGE_DEVICE_ID, BQ79600_DEV_CONF1, "BQ79600.DEV_CONF1");
    // For Daisy Chain, BQ79600 CONTROL2 is not critical for mode, but can print for info

    // The function verify_stack_communication_after_startup() in transition_startup will do a final check.
}

FSM_State_t transition_startup() {
    if (g_bmsData.communicationFault) { // If action_startup set this due to a failure
        return FSM_STATE_FAULT_STARTUP_ERROR;
    }
    // If processBMSFaults (called after action_startup) confirms a fault from this startup attempt
    if (g_faultState.bridge_comm_fault_confirmed || g_faultState.comm_fault_confirmed) {
        return FSM_STATE_FAULT_STARTUP_ERROR;
    }
    Serial.println("Startup successful, transitioning to Normal Operation.");
    return FSM_STATE_NORMAL_OPERATION;
}

void action_normal_operation() {
    BMSErrorCode_t status;
    bool current_cycle_comm_ok = true;

    status = bqGetAllCellVoltages(&g_bmsData);
    if (status != BMS_OK) current_cycle_comm_ok = false;

    status = bqGetAllTemperatures(&g_bmsData);
    if (status != BMS_OK) current_cycle_comm_ok = false;
    
    status = bqGetStackFaultStatus(&g_bmsData); 
    if (status != BMS_OK) current_cycle_comm_ok = false;

    status = bqGetBridgeFaultStatus(&g_bmsData); 
    if (status != BMS_OK) current_cycle_comm_ok = false; 
    
    g_bmsData.communicationFault = !current_cycle_comm_ok; // Update based on this cycle's operations

    updatePackStatistics(&g_bmsData); 
    g_bmsData.stateOfCharge_pct = estimateSOC(&g_bmsData); 

    bool any_temp_high = false;
    for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        for (int j = 0; j < TEMP_SENSORS_PER_SLAVE; ++j) {
            if (g_bmsData.modules[i].cellTemperatures[j] > 350) { 
                any_temp_high = true; break;
            }
        }
        if (any_temp_high) break;
    }
    digitalWrite(FAN_PIN, any_temp_high ? HIGH : LOW);
    digitalWrite(IMD_ERROR_PIN, g_bmsData.imd_status_ok ? LOW : HIGH);
    digitalWrite(AMS_ERROR_PIN, g_bmsData.overallFaultStatus ? HIGH : LOW); // AMS_ERROR if overall fault


    float min_v_overall = 5000.0f, max_v_overall = 0.0f;
    bool cells_in_bal_range = false;
    if (NUM_BQ79616_DEVICES > 0) { // Prevent access if no devices
        min_v_overall = g_bmsData.minCellVoltage_mV;
        max_v_overall = g_bmsData.maxCellVoltage_mV;
        // Check if max_v_overall is within balancing range to request balancing
        if (max_v_overall > MIN_VOLTAGE_FOR_BALANCING_MV && max_v_overall < MAX_VOLTAGE_FOR_BALANCING_MV) {
            cells_in_bal_range = true;
        }
    }

    if (cells_in_bal_range && (max_v_overall - min_v_overall > CELL_BALANCE_VOLTAGE_DIFF_THRESHOLD_MV)) {
        if (!g_bmsData.balancing_cycle_active) { // Only request if not already in a cycle
            g_bmsData.balancing_request = true;
        }
    } else {
        g_bmsData.balancing_request = false; 
    }
}

FSM_State_t transition_normal_operation() {
    if (g_faultState.comm_fault_confirmed || g_faultState.bridge_comm_fault_confirmed || g_faultState.stack_heartbeat_fault_confirmed) {
        return FSM_STATE_FAULT_COMMUNICATION;
    }
    if (g_faultState.ovuv_fault_confirmed || g_faultState.otut_fault_confirmed) {
        return FSM_STATE_FAULT_MEASUREMENT;
    }
    if (g_faultState.unspecified_n_fault_confirmed) {
        Serial.println("Transitioning to FAULT_CRITICAL due to unspecified NFAULT assertion.");
        return FSM_STATE_FAULT_CRITICAL; 
    }
    if (g_bmsData.balancing_request && !g_bmsData.balancing_cycle_active) { // Request and not already in a cycle from last time
        return FSM_STATE_CELL_BALANCING;
    }
    // no fucking clue why this is here but whatev
    // if (g_bmsData.reset_pin_active) { 
    //     Serial.println("Reset pin detected, initiating shutdown.");
    //     return FSM_STATE_SHUTDOWN_PROCEDURE;
    // }
    return FSM_STATE_NORMAL_OPERATION;
}

void action_cell_balancing() {
    // Serial.println("Action: CELL_BALANCING");
    BMSErrorCode_t comm_status;
    bool current_cycle_comm_ok = true;

    comm_status = bqGetAllCellVoltages(&g_bmsData); if (comm_status != BMS_OK) current_cycle_comm_ok = false;
    comm_status = bqGetAllTemperatures(&g_bmsData); if (comm_status != BMS_OK) current_cycle_comm_ok = false;
    comm_status = bqGetStackFaultStatus(&g_bmsData); if (comm_status != BMS_OK) current_cycle_comm_ok = false;
    comm_status = bqGetBridgeFaultStatus(&g_bmsData); if (comm_status != BMS_OK) current_cycle_comm_ok = false;
    g_bmsData.communicationFault = !current_cycle_comm_ok;
    
    updatePackStatistics(&g_bmsData);
    g_bmsData.stateOfCharge_pct = estimateSOC(&g_bmsData);
    bool new_balancing_started_this_action = false;

    for (uint8_t mod_idx = 0; mod_idx < NUM_BQ79616_DEVICES; ++mod_idx) {
        // Check status of currently balancing cells
        if (g_bmsData.activeBalancingCells[mod_idx] != 0) {
            new_balancing_started_this_action = true; // Still actively managing balancing
            bqCheckBalancingStatus(mod_idx, &g_bmsData);
            
            uint16_t completed_mask = g_bmsData.modules[mod_idx].cb_complete1;
            uint8_t previously_active = g_bmsData.activeBalancingCells[mod_idx];
            uint8_t still_active_after_completion_check = previously_active & ~completed_mask;
            
            if ((previously_active & completed_mask) != 0) { // Some cells finished
                 BMS_DEBUG_PRINTF("Mod %d: Cells with mask 0x%X completed balancing.\n", mod_idx, (previously_active & completed_mask));
            }
            g_bmsData.activeBalancingCells[mod_idx] = still_active_after_completion_check;
        }

        // Decide if new balancing needs to start on this module (only if no cells currently active on it)
        if (g_bmsData.activeBalancingCells[mod_idx] == 0) { 
            uint16_t min_v_module = 5000, max_v_module = 0;
            for (int c = 0; c < CELLS_PER_SLAVE; ++c) {
                uint16_t v = g_bmsData.modules[mod_idx].cellVoltages[c];
                if (v < 1000 || v > 5000) continue;
                if (v < min_v_module) min_v_module = v;
                if (v > max_v_module) max_v_module = v;
            }

            if ((max_v_module - min_v_module) > CELL_BALANCE_VOLTAGE_DIFF_THRESHOLD_MV && 
                 max_v_module < MAX_VOLTAGE_FOR_BALANCING_MV && 
                 max_v_module > MIN_VOLTAGE_FOR_BALANCING_MV) { // Check max_v_module against min range too
                
                uint8_t cells_to_balance_mask_this_module = 0;
                uint16_t sorted_voltages[CELLS_PER_SLAVE];
                uint8_t sorted_indices[CELLS_PER_SLAVE];

                for(int c=0; c<CELLS_PER_SLAVE; ++c) {
                    sorted_voltages[c] = g_bmsData.modules[mod_idx].cellVoltages[c];
                    sorted_indices[c] = c;
                }
                // Simple bubble sort to find top N cells
                for(int k=0; k < CELLS_PER_SLAVE-1; ++k) {
                    for(int m=0; m < CELLS_PER_SLAVE-k-1; ++m) {
                        if(sorted_voltages[m] < sorted_voltages[m+1]) {
                            uint16_t temp_v = sorted_voltages[m]; sorted_voltages[m] = sorted_voltages[m+1]; sorted_voltages[m+1] = temp_v;
                            uint8_t temp_i = sorted_indices[m]; sorted_indices[m] = sorted_indices[m+1]; sorted_indices[m+1] = temp_i;
                        }
                    }
                }
                
                for(int k=0; k < NUM_CELLS_TO_BALANCE_SIMULTANEOUSLY_PER_MODULE; ++k) {
                    if (k >= CELLS_PER_SLAVE) break; // Should not happen
                    uint16_t cell_v = sorted_voltages[k];
                    uint8_t cell_idx = sorted_indices[k];
                    if (cell_v > MIN_VOLTAGE_FOR_BALANCING_MV && cell_v < MAX_VOLTAGE_FOR_BALANCING_MV &&
                        cell_v > (min_v_module + (CELL_BALANCE_VOLTAGE_DIFF_THRESHOLD_MV / 2)) ) { // Ensure it's significantly higher than min
                        cells_to_balance_mask_this_module |= (1 << cell_idx);
                    } else {
                        break; // Stop if cells are not eligible or not high enough
                    }
                }
                
                if (cells_to_balance_mask_this_module != 0) {
                    BMS_DEBUG_PRINTF("Module %d: Starting balancing. MinV: %dmV, MaxV: %dmV. Mask: 0x%X\n", mod_idx, min_v_module, max_v_module, cells_to_balance_mask_this_module);
                    for(int c=0; c < CELLS_PER_SLAVE; ++c) { // Start one by one
                        if((cells_to_balance_mask_this_module >> c) & 0x01) {
                            comm_status = bqStartCellBalancing(&g_bmsData, mod_idx, c, BALANCING_DURATION_CODE);
                            if (comm_status != BMS_OK) {
                                g_bmsData.communicationFault = true;
                                BMS_DEBUG_PRINTF("Error starting balance on Mod %d Cell %d\n", mod_idx, c);
                            } else {
                                new_balancing_started_this_action = true;
                            }
                        }
                    }
                }
            }
        }
    } 
    g_bmsData.balancing_cycle_active = new_balancing_started_this_action; // Update global flag
}

FSM_State_t transition_cell_balancing() {
    if (g_faultState.comm_fault_confirmed || g_faultState.bridge_comm_fault_confirmed || g_faultState.stack_heartbeat_fault_confirmed) {
        for (uint8_t mod_idx = 0; mod_idx < NUM_BQ79616_DEVICES; ++mod_idx) {
            if (g_bmsData.activeBalancingCells[mod_idx] != 0) {
                for(int c=0; c < CELLS_PER_SLAVE; ++c) if((g_bmsData.activeBalancingCells[mod_idx] >> c) & 0x01) bqStopCellBalancing(mod_idx, c);
                g_bmsData.activeBalancingCells[mod_idx] = 0;
            }
        }
        g_bmsData.balancing_cycle_active = false; g_bmsData.balancing_request = false;
        return FSM_STATE_FAULT_COMMUNICATION;
    }
    if (g_faultState.ovuv_fault_confirmed || g_faultState.otut_fault_confirmed) {
        for (uint8_t mod_idx = 0; mod_idx < NUM_BQ79616_DEVICES; ++mod_idx) { /* Stop balancing */ }
        g_bmsData.balancing_cycle_active = false; g_bmsData.balancing_request = false;
        return FSM_STATE_FAULT_MEASUREMENT;
    }
    if (g_faultState.unspecified_n_fault_confirmed) {
        for (uint8_t mod_idx = 0; mod_idx < NUM_BQ79616_DEVICES; ++mod_idx) { /* Stop balancing */ }
        g_bmsData.balancing_cycle_active = false; g_bmsData.balancing_request = false;
        return FSM_STATE_FAULT_CRITICAL;
    }

    float min_v_overall = 5000.0f, max_v_overall = 0.0f;
     if (NUM_BQ79616_DEVICES > 0) {
        min_v_overall = g_bmsData.minCellVoltage_mV;
        max_v_overall = g_bmsData.maxCellVoltage_mV;
    }

    bool any_cell_still_active = false;
    for(int i=0; i<NUM_BQ79616_DEVICES; ++i) {
        if (g_bmsData.activeBalancingCells[i] != 0) {
            any_cell_still_active = true; 
            break;
        }
    }

    if (!any_cell_still_active && (max_v_overall - min_v_overall <= TARGET_BALANCED_VOLTAGE_DIFF_MV)) {
        Serial.println("Cell balancing complete. Pack delta <= target.");
        g_bmsData.balancing_request = false;
        g_bmsData.balancing_cycle_active = false;
        return FSM_STATE_NORMAL_OPERATION;
    }
    
    if (!any_cell_still_active && !g_bmsData.balancing_cycle_active) { 
        if ((max_v_overall - min_v_overall) <= CELL_BALANCE_VOLTAGE_DIFF_THRESHOLD_MV) { // Not much more to do
             Serial.println("Cell balancing paused (delta small or no eligible cells).");
             g_bmsData.balancing_request = false; 
             return FSM_STATE_NORMAL_OPERATION;
        } else if (max_v_overall >= MAX_VOLTAGE_FOR_BALANCING_MV || min_v_overall <= MIN_VOLTAGE_FOR_BALANCING_MV){
            Serial.println("Cell balancing paused (voltages out of safe balancing range).");
            g_bmsData.balancing_request = false;
            return FSM_STATE_NORMAL_OPERATION;
        }
    }
    // If still requested (e.g. from normal_op) or a cycle is active, stay.
    // action_cell_balancing will decide if new cells can be started.
    if(g_bmsData.balancing_request || g_bmsData.balancing_cycle_active) {
        return FSM_STATE_CELL_BALANCING; 
    }
    // Default to normal operation if no longer requested and no cycle active
    Serial.println("Exiting cell balancing as no longer requested or active.");
    g_bmsData.balancing_request = false;
    g_bmsData.balancing_cycle_active = false;
    return FSM_STATE_NORMAL_OPERATION;
}

void action_fault_communication() {
    Serial.println("Action: FAULT_COMMUNICATION");
    
    if (g_faultState.comm_fault_retries < 3) {
        BMS_DEBUG_PRINTF("Attempting communication fault recovery (Retry %d)...\n", g_faultState.comm_fault_retries + 1);
        
        Serial.println("Performing full stack re-initialization for recovery...");
        BMSErrorCode_t status_recovery = BMS_OK;
        
        if(bqWakeUpBridge() != BMS_OK) status_recovery = BMS_ERROR_WAKEUP_FAILED;
        if(status_recovery == BMS_OK && bqWakeUpStack() != BMS_OK) status_recovery = BMS_ERROR_WAKEUP_FAILED;
        if(status_recovery == BMS_OK && bqConfigureBridgeForRing() != BMS_OK) status_recovery = BMS_ERROR_CONFIG_FAILED;
        if(status_recovery == BMS_OK && bqAutoAddressStack() != BMS_OK) status_recovery = BMS_ERROR_AUTOADDRESS_FAILED;
        if(status_recovery == BMS_OK && applyEssentialStackConfigsForRecovery() != BMS_OK) status_recovery = BMS_ERROR_CONFIG_FAILED;
        if(status_recovery == BMS_OK && bqConfigureStackForHeartbeat() != BMS_OK) status_recovery = BMS_ERROR_CONFIG_FAILED;

        if (status_recovery == BMS_OK) {
            BMSErrorCode_t reset_status = resetAllBQFaults(); 
            if(reset_status == BMS_OK){
                Serial.println("Recovery: BQ faults cleared. Software confirmed faults also cleared.");
                // resetAllBQFaults now also clears software confirmed flags and timers.
                // We need to re-evaluate if underlying comms issue is resolved.
                // Forcing a clear of immediate bmsData.communicationFault to allow next cycle check.
                g_bmsData.communicationFault = false; 
            } else {
                Serial.println("Recovery: Failed to reset BQ faults.");
            }
        } else {
             BMS_DEBUG_PRINTF("Recovery: Re-initialization failed with code %d.\n", status_recovery);
        }
        g_faultState.comm_fault_retries++;
    } else {
         Serial.println("Communication fault persists after max retries during FSM fault state.");
    }
}

FSM_State_t transition_fault_communication() {
    // processBMSFaults will update confirmed flags based on current state of g_bmsData.communicationFault etc.
    // If recovery was successful, g_bmsData.communicationFault would be false, leading to g_faultState flags clearing.
    if (!g_faultState.comm_fault_confirmed && !g_faultState.bridge_comm_fault_confirmed && !g_faultState.stack_heartbeat_fault_confirmed) {
        Serial.println("Comm fault condition appears resolved after recovery attempt. Transitioning to STARTUP.");
        g_faultState.comm_fault_retries = 0; 
        return FSM_STATE_STARTUP; 
    }
    if(g_faultState.comm_fault_retries >= 3 && 
       (g_faultState.comm_fault_confirmed || g_faultState.bridge_comm_fault_confirmed || g_faultState.stack_heartbeat_fault_confirmed) ) { 
        Serial.println("Max comm recovery retries reached, escalating to CRITICAL FAULT.");
        return FSM_STATE_FAULT_CRITICAL; 
    }
    return FSM_STATE_FAULT_COMMUNICATION; 
}

void action_fault_measurement() {
    Serial.println("Action: FAULT_MEASUREMENT (OV/UV/OT/UT)");
    if(g_faultState.ovuv_fault_confirmed) Serial.println("Confirmed OV/UV Fault Active!");
    if(g_faultState.otut_fault_confirmed) Serial.println("Confirmed OT/UT Fault Active!");
}

FSM_State_t transition_fault_measurement() {
    if (!g_faultState.ovuv_fault_confirmed && !g_faultState.otut_fault_confirmed) {
        Serial.println("Measurement fault condition cleared.");
        return FSM_STATE_NORMAL_OPERATION;
    }
    return FSM_STATE_FAULT_MEASUREMENT; 
}

void action_fault_startup_error() {
    Serial.println("Action: FAULT_STARTUP_ERROR - System initialization failed.");
    delay(2000);
    // Log specific startup error. Limited functionality.
}

FSM_State_t transition_fault_startup_error() {
    unsigned long currentTime = millis();
    if (startup_attempt_timestamp == 0) startup_attempt_timestamp = currentTime; // Should have been set by action_startup

    // Retry startup after a longer delay, or go to critical
    if (currentTime - startup_attempt_timestamp > 5000) { // Retry every 30 seconds
        Serial.println("Attempting to re-enter STARTUP from FAULT_STARTUP_ERROR.");
        // Clear relevant fault flags to allow startup to proceed
        g_faultState.comm_fault_retries = 0; 
        g_faultState.comm_fault_confirmed = false; g_faultState.comm_fault_first_detected_ts = 0;
        g_faultState.bridge_comm_fault_confirmed = false; g_faultState.bridge_comm_first_detected_ts = 0;
        g_bmsData.communicationFault = false; // Reset immediate trigger
        if (g_bmsData.bridge.fault_summary_bridge == 0xFF) g_bmsData.bridge.fault_summary_bridge = 0; // Reset bridge read error flag
        return FSM_STATE_STARTUP;
    }
    return FSM_STATE_FAULT_STARTUP_ERROR; 
}

void action_fault_critical() {
    Serial.println("Action: FAULT_CRITICAL - System Halted. Manual intervention required.");
    printBQDump(); 
    static bool dump_done = false;
    if(!dump_done) {
        printBQDump();
        dump_done = true;
    }
    delay(10000);
}

FSM_State_t transition_fault_critical() {
    return FSM_STATE_FAULT_CRITICAL; 
}

void action_shutdown_procedure() {
    Serial.println("Action: SHUTDOWN_PROCEDURE");
    // 1. Command contactors to open (not implemented here, assumes external)
    // 2. Save critical data (not implemented)
    // 3. Command BQ devices to SHUTDOWN
    BMSErrorCode_t status = bqShutdownDevices();
    if (status == BMS_OK) {
        Serial.println("Shutdown command sent successfully to BQ devices.");
    } else {
        Serial.println("Error sending shutdown command to BQ devices.");
    }
}

FSM_State_t transition_shutdown_procedure() {
    static unsigned long shutdown_initiated_time = 0;
    if (shutdown_initiated_time == 0) shutdown_initiated_time = millis();

    if (millis() - shutdown_initiated_time > 1000) { 
        shutdown_initiated_time = 0; // Reset for next potential shutdown
        return FSM_STATE_SYSTEM_OFF;
    }
    return FSM_STATE_SHUTDOWN_PROCEDURE;
}

void action_system_off() {
    Serial.println("Action: SYSTEM_OFF. MCU can enter low power or wait for reset.");
    digitalWrite(AMS_FAULT_PIN, LOW); // Clear fault pin if system is intentionally off
    // Optional: Enter MCU deep sleep mode
    while(true) {
        delayMicroseconds(5000); // Stay here, do nothing actively
    }
}

FSM_State_t transition_system_off(){
    return FSM_STATE_SYSTEM_OFF;
}

