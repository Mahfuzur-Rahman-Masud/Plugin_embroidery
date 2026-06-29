#ifndef _EMBROIDERY_SPI_H_
#define _EMBROIDERY_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "embroidery.h"

typedef enum {
    NONE,
    EMB_SPI_OK,
    SLAVE_OK,
    POLL, // 3
    START, // 4
    PAUSE,
    RESUME,
    ABORT,
    FORWARD,
    BACKWARD,
    PEL,
    MC_LINE, //11
    SYNC,
    BUF_CLEAR,
    RESET,
    RPM,
    STITCH, // 16
} action_t;

typedef enum {
    STATUS_IDLE = 0xAA, 
    STATUS_ACK = 0xBB, 
    STATUS_NACK = 0xCC
} spi_status_t;

// 1. Define the base structures first
typedef union {
    uint8_t data[84];
    struct {
        uint8_t id;
        action_t action;
        stich_type_t type;
        int32_t stitch_number;
        embroidery_thread_color_t color;
        float coord[3];
        uint32_t feed;
        uint32_t accel;
        int16_t rpm;
        bool last_stitch;
        bool wait_trigger;
    };
} req_t;

typedef union {
    uint8_t data[84];
    struct {
        uint8_t id;
        uint8_t action;
        uint32_t stitch_loaded;
        uint32_t stitch_done;
        float mpos[3];
        float wpos[3];
        uint32_t feed;
        uint8_t servo;
        uint8_t coolant;
        uint8_t fan;
        uint8_t temp[2];
        uint16_t state; // machine state such as IDLE | HOLD | ALARM
        uint8_t action_needed;
        int16_t rpm;
        uint8_t rx_free;
        uint8_t status;
    };
} resp_t;

#define EMB_SPI_HOST SPI3_HOST
#define EMB_MOSI GPIO_NUM_4
#define EMB_MISO GPIO_NUM_5
#define EMB_SCK GPIO_NUM_6
#define EMB_CS GPIO_NUM_7

#define EMB_SLAVE_SPI_HOST SPI3_HOST
#define EMB_SLAVE_MOSI GPIO_NUM_17
#define EMB_SLAVE_MISO GPIO_NUM_3
#define EMB_SLAVE_SCK GPIO_NUM_19
#define EMB_SLAVE_CS GPIO_NUM_14
#define EMB_SLAVE_SPI_READY GPIO_NUM_2 // Dedicated handshake line

bool emb_spi_master_init();
bool emb_spi_slave_init();
void emb_spi_slave_read_loop();

resp_t emb_spi_master_exchange(const req_t* req);
req_t emb_spi_slave_exchange(const resp_t* resp_to_send);

#ifdef __cplusplus
}
#endif
#endif // _EMBROIDERY_SPI_H_