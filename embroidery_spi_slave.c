#include "driver/spi_slave.h"
#include "embroidery.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// --- Hardware Pins Configuration ---
#define SLAVE_SPI_HOST SPI3_HOST

#define EMB_SLAVE_MOSI GPIO_NUM_19
#define EMB_SLAVE_MISO GPIO_NUM_14
#define EMB_SLAVE_SCK GPIO_NUM_17
#define EMB_SLAVE_CS GPIO_NUM_3
#define EMB_SLAVE_SPI_READY GPIO_NUM_2 // Dedicated handshake line

// --- Types & Enums Definitions ---
typedef enum {
    PAUSE,
    RESUME,
    ABORT,
    FORWARD,
    BACKWARD,
    PEL,
    MC_LINE,
    SYNC,
    BUF_CLEAR,
    RESET,
    RPM,
    STITCH,
} action_t;

typedef struct {
    action_t action;   //1
    stich_type_t type; //1
    embroidery_thread_color_t color; //1
    float coord[3]; // 1
    uint32_t feed; //1
    uint32_t accel;
    int16_t rpm;
    bool wait_trigger;
    char msg[64];
} req_t;

typedef struct {
    uint8_t flag; 
    float mpos[3];
    float wpos[3]; 
    uint32_t feed; 
    uint8_t servo; 
    uint8_t coolant; 
    uint8_t fan;
    uint8_t temp[2];
    uint16_t state;
    int16_t rpm;
    uint8_t status;
    uint8_t rx_size;
    char msg[64];
} resp_t;

// --- Safe Unified Memory Layout ---
// Determine the absolute largest packet size at compile time
#define SPI_EXCHANGE_SIZE (sizeof(req_t) > sizeof(resp_t) ? sizeof(req_t) : sizeof(resp_t))

typedef union {
    uint8_t buf[SPI_EXCHANGE_SIZE];
    req_t data;
} req_packet_t;

typedef union {
    uint8_t buf[SPI_EXCHANGE_SIZE];
    resp_t data;
} resp_packet_t;

// // --- Global DMA Buffers ---
static DMA_ATTR req_packet_t spi_req_pkt;
static DMA_ATTR resp_packet_t spi_resp_pkt;

// Global placeholders for application access
resp_t current_response;
static TaskHandle_t spi_task_handle = NULL;

// --- Core SPI Exchange Logic ---
bool emb_spi_slave_exchange(req_t* received_request, resp_t* response_to_send)
{
    spi_resp_pkt.data = *response_to_send;

    spi_slave_transaction_t t = {
        .length = SPI_EXCHANGE_SIZE * 8,
        .rx_buffer = spi_req_pkt.buf,
        .tx_buffer = spi_resp_pkt.buf
    };

    gpio_set_level(EMB_SLAVE_SPI_READY, 0);

    // 2. Block until Master acts on the signal and clocks data
    esp_err_t ret = spi_slave_transmit(SLAVE_SPI_HOST, &t, portMAX_DELAY);

    // 3. Instantly tell Master to STOP sending more data while we process
    gpio_set_level(EMB_SLAVE_SPI_READY, 1);

    if (ret == ESP_OK) {
        *received_request = spi_req_pkt.data;
        return true;
    }
    
    return false;
}

// --- Action Processor ---
static void process_received_action(req_t* req)
{
    switch (req->action) {
    case PEL:
        // Safely stream string commands directly into grblHAL parser loop
        protocol_execute_line(req->msg);
        break;

    case MC_LINE:
        // Handle coordinate or motion operations
        break;

    case PAUSE:
        // Call grblHAL internal real-time suspend mapping
        break;

    default:
        break;
    }
}

// --- Background Task ---
static void spi_slave_background_task(void* pvParameters)
{
    req_t inbound_request;

    while (1) {
        // Blocks here naturally until the Master initiates a clock transfer
        if (emb_spi_slave_exchange(&inbound_request, &current_response)) {

            // Route data to parsing mechanics immediately upon receipt
            process_received_action(&inbound_request);

            // Example: dynamically update response tracking buffers for next cycle
            current_response.flag = 1;
        }
    }
}

void init_handshake_pin(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << EMB_SLAVE_SPI_READY),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    gpio_set_level(EMB_SLAVE_SPI_READY, 1); // Default to Busy/Not Ready
}

// --- Initialization Entry Point ---
bool init_spi_slave_background_stream(void)
{
    memset(&spi_req_pkt, 0, sizeof(spi_req_pkt));
    memset(&spi_resp_pkt, 0, sizeof(spi_resp_pkt));
    memset(&current_response, 0, sizeof(resp_t));

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = EMB_SLAVE_MOSI,
        .miso_io_num = EMB_SLAVE_MISO,
        .sclk_io_num = EMB_SLAVE_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slv_cfg = {
        .mode = 0,
        .spics_io_num = EMB_SLAVE_CS,
        .queue_size = 1,
    };

    if (spi_slave_initialize(SLAVE_SPI_HOST, &bus_cfg, &slv_cfg, SPI_DMA_CH_AUTO) != ESP_OK) {
        return false;
    }

    // Spawn the background communication task pinned to Core 1
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        spi_slave_background_task, // Task function
        "SPI_Slave_Task", // Name
        4096, // Stack size in words
        NULL, // Parameters
        12, // Priority (Adjust relative to step generators)
        &spi_task_handle, // Task handle tracking
        1 // Run explicitly on Core 1
    );

    return (xReturned == pdPASS);
}