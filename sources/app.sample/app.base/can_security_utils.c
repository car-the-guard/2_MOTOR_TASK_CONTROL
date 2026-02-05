/*
 * can_security_utils.c
 * CAN Message Authentication Code (MAC) Implementation
 */

#include "can_security_utils.h"

// =========================================================
// [중요] 비밀키 설정 (모든 노드가 동일한 키를 가져야 함)
// =========================================================
static const uint8_t SECRET_KEY[4] = {0x19, 0x99, 0x06, 0x15};

// 내부 유틸리티: 1비트 좌측 회전 (Rotate Left)
static inline uint8_t rotate_left_1(uint8_t value) {
    return (value << 1) | (value >> 7);
}

// MAC 계산 함수 구현
uint8_t compute_mac(const uint8_t *data, uint8_t len, uint8_t counter) {
    uint8_t mac;
    uint8_t i;
    
    // 1단계: 초기화 (매직 넘버 XOR 카운터)
    // 카운터 값이 변할 때마다 초기 시드값이 달라지는 효과
    mac = MAC_MAGIC_NUMBER ^ counter;
    
    // 2단계: 데이터 믹싱 (XOR -> ADD -> ROTATE)
    for (i = 0; i < len; i++) {
        mac ^= data[i];                  // 데이터 섞기
        mac += SECRET_KEY[i % 4];        // 비밀키 주입
        mac = rotate_left_1(mac);        // 비트 확산 (Diffusion)
    }
    
    return mac;
}

// MAC 검증 함수 구현
uint8_t verify_mac(const uint8_t *data, uint8_t len, uint8_t received_counter, uint8_t received_mac) {
    uint8_t calculated_mac;
    
    // 내가 가진 키로 다시 계산
    calculated_mac = compute_mac(data, len, received_counter);
    
    // 수신된 MAC과 비교
    if (calculated_mac == received_mac) {
        return 1;  // 인증 성공
    } else {
        return 0;  // 인증 실패 (데이터 변조 또는 키 불일치)
    }
}