/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.            *
 *                                                                          *
 * The information contained herein is property of Nordic Semiconductor ASA.*
 * Terms and conditions of usage are described in detail in NORDIC          *
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.                       *
 *                                                                          *
 * Licensees are granted free, non-transferable use of the information. NO  *
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from  *
 * the file.                                                                *
 *                                                                          */

/*                                                                          *
 * This file contents defines for the position of all the fields of ACI     *
 * command or event messages                                                *
 *                                                                          */

#ifndef ACI_OFFSET_H__
#define ACI_OFFSET_H__


	#define OFFSET_ACI_LL_CONN_PARAMS_T_MIN_CONN_INTERVAL_LSB 0
	#define OFFSET_ACI_LL_CONN_PARAMS_T_MIN_CONN_INTERVAL_MSB 1
	#define OFFSET_ACI_LL_CONN_PARAMS_T_MAX_CONN_INTERVAL_LSB 2
	#define OFFSET_ACI_LL_CONN_PARAMS_T_MAX_CONN_INTERVAL_MSB 3
	#define OFFSET_ACI_LL_CONN_PARAMS_T_SLAVE_LATENCY_LSB 4
	#define OFFSET_ACI_LL_CONN_PARAMS_T_SLAVE_LATENCY_MSB 5
	#define OFFSET_ACI_LL_CONN_PARAMS_T_TIMEOUT_MULT_LSB 6
	#define OFFSET_ACI_LL_CONN_PARAMS_T_TIMEOUT_MULT_MSB 7
	#define OFFSET_ACI_TX_DATA_T_PIPE_NUMBER 0
	#define OFFSET_ACI_TX_DATA_T_ACI_DATA 1
	#define OFFSET_ACI_RX_DATA_T_PIPE_NUMBER 0
	#define OFFSET_ACI_RX_DATA_T_ACI_DATA 1
	#define OFFSET_ACI_CMD_PARAMS_TEST_T_TEST_MODE_CHANGE 0
	#define OFFSET_ACI_CMD_PARAMS_ECHO_T_ECHO_DATA 0
	#define OFFSET_ACI_CMD_PARAMS_DTM_CMD_T_CMD_MSB 0
	#define OFFSET_ACI_CMD_PARAMS_DTM_CMD_T_CMD_LSB 1
	#define OFFSET_ACI_CMD_PARAMS_SETUP_T_SETUP_DATA 0
	#define OFFSET_ACI_CMD_PARAMS_WRITE_DYNAMIC_DATA_T_SEQ_NO 0
	#define OFFSET_ACI_CMD_PARAMS_WRITE_DYNAMIC_DATA_T_DYNAMIC_DATA 1
	#define OFFSET_ACI_CMD_PARAMS_SET_LOCAL_DATA_T_TX_DATA 0
	#define OFFSET_ACI_CMD_PARAMS_CONNECT_T_TIMEOUT_LSB 0
	#define OFFSET_ACI_CMD_PARAMS_CONNECT_T_TIMEOUT_MSB 1
	#define OFFSET_ACI_CMD_PARAMS_CONNECT_T_ADV_INTERVAL_LSB 2
	#define OFFSET_ACI_CMD_PARAMS_CONNECT_T_ADV_INTERVAL_MSB 3
	#define OFFSET_ACI_CMD_PARAMS_BOND_T_TIMEOUT_LSB 0
	#define OFFSET_ACI_CMD_PARAMS_BOND_T_TIMEOUT_MSB 1
	#define OFFSET_ACI_CMD_PARAMS_BOND_T_ADV_INTERVAL_LSB 2
	#define OFFSET_ACI_CMD_PARAMS_BOND_T_ADV_INTERVAL_MSB 3
	#define OFFSET_ACI_CMD_PARAMS_DISCONNECT_T_REASON 0
	#define OFFSET_ACI_CMD_PARAMS_SET_TX_POWER_T_DEVICE_POWER 0
	#define OFFSET_ACI_CMD_PARAMS_CHANGE_TIMING_T_CONN_PARAMS 0
	#define OFFSET_ACI_CMD_PARAMS_OPEN_REMOTE_PIPE_T_PIPE_NUMBER 0
	#define OFFSET_ACI_CMD_PARAMS_SEND_DATA_T_TX_DATA 0
	#define OFFSET_ACI_CMD_PARAMS_SEND_DATA_ACK_T_PIPE_NUMBER 0
	#define OFFSET_ACI_CMD_PARAMS_REQUEST_DATA_T_PIPE_NUMBER 0
	#define OFFSET_ACI_CMD_PARAMS_SEND_DATA_NACK_T_PIPE_NUMBER 0
	#define OFFSET_ACI_CMD_PARAMS_SEND_DATA_NACK_T_ERROR_CODE 1
	#define OFFSET_ACI_CMD_PARAMS_SET_APP_LATENCY_T_MODE 0
	#define OFFSET_ACI_CMD_PARAMS_SET_APP_LATENCY_T_LATENCY_LSB 0
	#define OFFSET_ACI_CMD_PARAMS_SET_APP_LATENCY_T_LATENCY_MSB 1
	#define OFFSET_ACI_CMD_PARAMS_SET_KEY_T_KEY_TYPE 0
	#define OFFSET_ACI_CMD_PARAMS_SET_KEY_T_PASSKEY 1
	#define OFFSET_ACI_CMD_PARAMS_SET_KEY_T_OOB_KEY 1
	#define OFFSET_ACI_CMD_PARAMS_OPEN_ADV_PIPE_T_PIPES 0
	#define OFFSET_ACI_CMD_PARAMS_BROADCAST_T_TIMEOUT_LSB 0
	#define OFFSET_ACI_CMD_PARAMS_BROADCAST_T_TIMEOUT_MSB 1
	#define OFFSET_ACI_CMD_PARAMS_BROADCAST_T_ADV_INTERVAL_LSB 2
	#define OFFSET_ACI_CMD_PARAMS_BROADCAST_T_ADV_INTERVAL_MSB 3
	#define OFFSET_ACI_CMD_PARAMS_CLOSE_REMOTE_PIPE_T_PIPE_NUMBER 0
	#define OFFSET_ACI_CMD_T_LEN 0
	#define OFFSET_ACI_CMD_T_CMD_OPCODE 1
	#define OFFSET_ACI_CMD_T_TEST 2
	#define OFFSET_ACI_CMD_T_ECHO 2
	#define OFFSET_ACI_CMD_T_DTM_CMD 2
	#define OFFSET_ACI_CMD_T_SETUP 2
	#define OFFSET_ACI_CMD_T_WRITE_DYNAMIC_DATA 2
	#define OFFSET_ACI_CMD_T_SET_LOCAL_DATA 2
	#define OFFSET_ACI_CMD_T_CONNECT 2
	#define OFFSET_ACI_CMD_T_BOND 2
	#define OFFSET_ACI_CMD_T_DISCONNECT 2
	#define OFFSET_ACI_CMD_T_SET_TX_POWER 2
	#define OFFSET_ACI_CMD_T_CHANGE_TIMING 2
	#define OFFSET_ACI_CMD_T_OPEN_REMOTE_PIPE 2
	#define OFFSET_ACI_CMD_T_SEND_DATA 2
	#define OFFSET_ACI_CMD_T_SEND_DATA_ACK 2
	#define OFFSET_ACI_CMD_T_REQUEST_DATA 2
	#define OFFSET_ACI_CMD_T_SEND_DATA_NACK 2
	#define OFFSET_ACI_CMD_T_SET_APP_LATENCY 2
	#define OFFSET_ACI_CMD_T_SET_KEY 2
	#define OFFSET_ACI_CMD_T_OPEN_ADV_PIPE 2
	#define OFFSET_ACI_CMD_T_BROADCAST 2
	#define OFFSET_ACI_CMD_T_CLOSE_REMOTE_PIPE 2
	#define OFFSET_ACI_EVT_PARAMS_DEVICE_STARTED_T_DEVICE_MODE 0
	#define OFFSET_ACI_EVT_PARAMS_DEVICE_STARTED_T_HW_ERROR 1
	#define OFFSET_ACI_EVT_PARAMS_DEVICE_STARTED_T_CREDIT_AVAILABLE 2
	#define OFFSET_ACI_EVT_PARAMS_HW_ERROR_T_LINE_NUM_LSB 0
	#define OFFSET_ACI_EVT_PARAMS_HW_ERROR_T_LINE_NUM_MSB 1
	#define OFFSET_ACI_EVT_PARAMS_HW_ERROR_T_FILE_NAME 2
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_DTM_CMD_T_EVT_MSB 0
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_DTM_CMD_T_EVT_LSB 1
	#define OFFSET_ACI_EVT_CMD_RSP_READ_DYNAMIC_DATA_T_SEQ_NO 0
	#define OFFSET_ACI_EVT_CMD_RSP_READ_DYNAMIC_DATA_T_DYNAMIC_DATA 1
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_DEVICE_VERSION_T_CONFIGURATION_ID_LSB 0
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_DEVICE_VERSION_T_CONFIGURATION_ID_MSB 1
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_DEVICE_VERSION_T_ACI_VERSION 2
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_DEVICE_VERSION_T_SETUP_FORMAT 3
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_DEVICE_VERSION_T_SETUP_ID_LSB0 4
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_DEVICE_VERSION_T_SETUP_ID_LSB1 5
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_DEVICE_VERSION_T_SETUP_ID_MSB0 6
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_DEVICE_VERSION_T_SETUP_ID_MSB1 7
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_DEVICE_VERSION_T_SETUP_STATUS 8
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_DEVICE_ADDRESS_T_BD_ADDR_OWN 0
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_DEVICE_ADDRESS_T_BD_ADDR_TYPE 6
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_BATTERY_LEVEL_T_BATTERY_LEVEL_LSB 0
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_BATTERY_LEVEL_T_BATTERY_LEVEL_MSB 1
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_TEMPERATURE_T_TEMPERATURE_VALUE_LSB 0
	#define OFFSET_ACI_EVT_CMD_RSP_PARAMS_GET_TEMPERATURE_T_TEMPERATURE_VALUE_MSB 1
	#define OFFSET_ACI_EVT_PARAMS_CMD_RSP_T_CMD_OPCODE 0
	#define OFFSET_ACI_EVT_PARAMS_CMD_RSP_T_CMD_STATUS 1
	#define OFFSET_ACI_EVT_PARAMS_CMD_RSP_T_DTM_CMD 2
	#define OFFSET_ACI_EVT_PARAMS_CMD_RSP_T_READ_DYNAMIC_DATA 2
	#define OFFSET_ACI_EVT_PARAMS_CMD_RSP_T_GET_DEVICE_VERSION 2
	#define OFFSET_ACI_EVT_PARAMS_CMD_RSP_T_GET_DEVICE_ADDRESS 2
	#define OFFSET_ACI_EVT_PARAMS_CMD_RSP_T_GET_BATTERY_LEVEL 2
	#define OFFSET_ACI_EVT_PARAMS_CMD_RSP_T_GET_TEMPERATURE 2
	#define OFFSET_ACI_EVT_PARAMS_CONNECTED_T_DEV_ADDR_TYPE 0
	#define OFFSET_ACI_EVT_PARAMS_CONNECTED_T_DEV_ADDR 1
	#define OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_RF_INTERVAL_LSB 7
	#define OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_RF_INTERVAL_MSB 8
	#define OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_SLAVE_RF_LATENCY_LSB 9
	#define OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_SLAVE_RF_LATENCY_MSB 10
	#define OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_RF_TIMEOUT_LSB 11
	#define OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_RF_TIMEOUT_MSB 12
	#define OFFSET_ACI_EVT_PARAMS_CONNECTED_T_MASTER_CLOCK_ACCURACY 13
	#define OFFSET_ACI_EVT_PARAMS_DISCONNECTED_T_ACI_STATUS 0
	#define OFFSET_ACI_EVT_PARAMS_DISCONNECTED_T_BTLE_STATUS 1
	#define OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_STATUS_CODE 0
	#define OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_STATUS_SOURCE 1
	#define OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_SECMODE1_BITMAP 2
	#define OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_SECMODE2_BITMAP 3
	#define OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_KEYS_EXCHANGED_SLAVE 4
	#define OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_KEYS_EXCHANGED_MASTER 5
	#define OFFSET_ACI_EVT_PARAMS_PIPE_STATUS_T_PIPES_OPEN_BITMAP 0
	#define OFFSET_ACI_EVT_PARAMS_PIPE_STATUS_T_PIPES_CLOSED_BITMAP 8
	#define OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_RF_INTERVAL_LSB 0
	#define OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_RF_INTERVAL_MSB 1
	#define OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_SLAVE_RF_LATENCY_LSB 2
	#define OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_SLAVE_RF_LATENCY_MSB 3
	#define OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_RF_TIMEOUT_LSB 4
	#define OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_RF_TIMEOUT_MSB 5
	#define OFFSET_ACI_EVT_PARAMS_DATA_CREDIT_T_CREDIT 0
	#define OFFSET_ACI_EVT_PARAMS_DATA_ACK_T_PIPE_NUMBER 0
	#define OFFSET_ACI_EVT_PARAMS_DATA_RECEIVED_T_RX_DATA 0
	#define OFFSET_ERROR_DATA_T_CONTENT 0
	#define OFFSET_ACI_EVT_PARAMS_PIPE_ERROR_T_PIPE_NUMBER 0
	#define OFFSET_ACI_EVT_PARAMS_PIPE_ERROR_T_ERROR_CODE 1
	#define OFFSET_ACI_EVT_PARAMS_PIPE_ERROR_T_ERROR_DATA 2
	#define OFFSET_ACI_EVT_PARAMS_DISPLAY_PASSKEY_T_PASSKEY 0
	#define OFFSET_ACI_EVT_PARAMS_KEY_REQUEST_T_KEY_TYPE 0
	#define OFFSET_ACI_EVT_T_LEN 0
	#define OFFSET_ACI_EVT_T_EVT_OPCODE 1
	#define OFFSET_ACI_EVT_T_DEVICE_STARTED 2
	#define OFFSET_ACI_EVT_T_HW_ERROR 2
	#define OFFSET_ACI_EVT_T_CMD_RSP 2
	#define OFFSET_ACI_EVT_T_CONNECTED 2
	#define OFFSET_ACI_EVT_T_DISCONNECTED 2
	#define OFFSET_ACI_EVT_T_BOND_STATUS 2
	#define OFFSET_ACI_EVT_T_PIPE_STATUS 2
	#define OFFSET_ACI_EVT_T_TIMING 2
	#define OFFSET_ACI_EVT_T_DATA_CREDIT 2
	#define OFFSET_ACI_EVT_T_DATA_ACK 2
	#define OFFSET_ACI_EVT_T_DATA_RECEIVED 2
	#define OFFSET_ACI_EVT_T_PIPE_ERROR 2
	#define OFFSET_ACI_EVT_T_DISPLAY_PASSKEY 2
	#define OFFSET_ACI_EVT_T_KEY_REQUEST 2

#endif //ACI_OFFSET_H__

