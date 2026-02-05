/*
 * can_security_utils.h
 * CAN Message Authentication Code (MAC) Utility Header
 */

#ifndef CAN_SECURITY_UTILS_H
#define CAN_SECURITY_UTILS_H

#include <stdint.h>

// 매직 넘버 (알고리즘 시작 값)
#define MAC_MAGIC_NUMBER 0x5A

/**
 * @brief MAC 값을 계산합니다.
 * @param data    MAC을 제외한 전체 데이터 배열 (Payload + Timestamp + Counter)
 * @param len     데이터 길이 (보통 7바이트)
 * @param counter 현재 메시지의 카운터 값
 * @return        계산된 1바이트 MAC
 */
uint8_t compute_mac(const uint8_t *data, uint8_t len, uint8_t counter);

/**
 * @brief 수신된 패킷의 MAC을 검증합니다.
 * @param data             수신된 데이터 배열 (MAC 제외)
 * @param len              데이터 길이 (보통 7바이트)
 * @param received_counter 수신된 카운터 값
 * @param received_mac     수신된 MAC 값
 * @return                 1: 검증 성공, 0: 검증 실패
 */
uint8_t verify_mac(const uint8_t *data, uint8_t len, uint8_t received_counter, uint8_t received_mac);

#endif // CAN_SECURITY_UTILS_H