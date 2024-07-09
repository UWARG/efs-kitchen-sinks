
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>




#define COM_TMOTOR_ESC_PUSHSCI_MAX_SIZE 260
#define COM_TMOTOR_ESC_PUSHSCI_SIGNATURE (0xCE2B6D6B6BDC0AE8ULL)

#define COM_TMOTOR_ESC_PUSHSCI_ID 1038





#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class com_tmotor_esc_PUSHSCI_cxx_iface;
#endif


struct com_tmotor_esc_PUSHSCI {

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = com_tmotor_esc_PUSHSCI_cxx_iface;
#endif




    uint32_t data_sequence;



    struct { uint8_t len; uint8_t data[255]; }data;



};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t com_tmotor_esc_PUSHSCI_encode(struct com_tmotor_esc_PUSHSCI* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool com_tmotor_esc_PUSHSCI_decode(const CanardRxTransfer* transfer, struct com_tmotor_esc_PUSHSCI* msg);

#if defined(CANARD_DSDLC_INTERNAL)

static inline void _com_tmotor_esc_PUSHSCI_encode(uint8_t* buffer, uint32_t* bit_ofs, struct com_tmotor_esc_PUSHSCI* msg, bool tao);
static inline bool _com_tmotor_esc_PUSHSCI_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct com_tmotor_esc_PUSHSCI* msg, bool tao);
void _com_tmotor_esc_PUSHSCI_encode(uint8_t* buffer, uint32_t* bit_ofs, struct com_tmotor_esc_PUSHSCI* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->data_sequence);

    *bit_ofs += 32;







    if (!tao) {


        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->data.len);
        *bit_ofs += 8;


    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const size_t data_len = msg->data.len > 255 ? 255 : msg->data.len;
#pragma GCC diagnostic pop
    for (size_t i=0; i < data_len; i++) {




        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->data.data[i]);

        *bit_ofs += 8;


    }





}

/*
 decode com_tmotor_esc_PUSHSCI, return true on failure, false on success
*/
bool _com_tmotor_esc_PUSHSCI_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct com_tmotor_esc_PUSHSCI* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;





    canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->data_sequence);

    *bit_ofs += 32;








    if (!tao) {


        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->data.len);
        *bit_ofs += 8;



    } else {

        msg->data.len = ((transfer->payload_len*8)-*bit_ofs)/8;


    }



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->data.len > 255) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->data.len; i++) {




        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->data.data[i]);

        *bit_ofs += 8;


    }






    return false; /* success */

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct com_tmotor_esc_PUSHSCI sample_com_tmotor_esc_PUSHSCI_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>


BROADCAST_MESSAGE_CXX_IFACE(com_tmotor_esc_PUSHSCI, COM_TMOTOR_ESC_PUSHSCI_ID, COM_TMOTOR_ESC_PUSHSCI_SIGNATURE, COM_TMOTOR_ESC_PUSHSCI_MAX_SIZE);


#endif
#endif
