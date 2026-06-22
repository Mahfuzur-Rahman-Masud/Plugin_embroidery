
#include "embroidery_spi.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include <string.h>

// ---------------------------------------- SPI --------------------------

static spi_device_handle_t spi_master_handle = NULL;
// // --- Global DMA Buffers ---
static DMA_ATTR req_t spi_req_pkt;
static DMA_ATTR resp_t spi_resp_pkt;

void emb_spi_global_init()
{
    memset(&spi_req_pkt, 0, sizeof(spi_req_pkt));
    memset(&spi_resp_pkt, 0, sizeof(spi_resp_pkt));
}

bool emb_spi_master_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = EMB_MOSI,
        .miso_io_num = EMB_MISO,
        .sclk_io_num = EMB_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1000000, // 1 MHz
        .mode = 0,
        .spics_io_num = EMB_CS,
        .queue_size = 1,
    };

    // initialize bus
    if (spi_bus_initialize(EMB_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO) != ESP_OK) {
        return false;
    }

    // register to bus
    return (spi_bus_add_device(EMB_SPI_HOST, &dev_cfg, &spi_master_handle) == ESP_OK);
}

bool emb_spi_slave_init()
{

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

    return (spi_slave_initialize(EMB_SLAVE_SPI_HOST, &bus_cfg, &slv_cfg, SPI_DMA_CH_AUTO) == ESP_OK);
}

// bool run_receive(uint8_t* rx_buffer, uint8_t* tx_data_to_send, uint16_t length)
// {

//     spi_transaction_t t = {
//         .length = length * 8,
//         .rx_buffer = rx_buffer, // Where data from master will be saved
//         .tx_buffer = tx_data_to_send // Data pushed back to master over MISO simultaneously
//     };

//     // This blocks the CPU/Task until the external Master initiates a clock cycle
//     esp_err_t ret = spi_transmit(EMB_SPI_HOST, &t, portMAX_DELAY);

//     return (ret == ESP_OK);
// }


/**
 * @brief Exchanges a request packet for a response packet over the SPI bus.
 * * @param req Pointer to the populated local request data you want to send.
 * @retval true  The transaction completed successfully and data was copied into 'current_response'.
 * @retval false The SPI transaction failed or timed out.
 */
resp_t emb_spi_master_exchange(const req_t *req)
{


    resp_t resp = {0};
    // 1. Safety check to ensure the SPI master driver has been initialize
    if (spi_master_handle == NULL || req == NULL) {
        return resp;
    }


    memcpy(&spi_req_pkt, req, sizeof(req_t));
    memset(&spi_resp_pkt, 0, sizeof(resp_t)); // Clear the response DMA buffer to ensure we aren't looking at stale data on failure


    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans)); // Zero out the structure entirely first

    trans.length = sizeof(req_t) * 8;   // Length must be specified in BITS
    trans.rxlength = sizeof(resp_t) * 8; // Expected receive length in BITS
    trans.tx_buffer = &spi_req_pkt;            // Pointer to DMA-safe TX buffer
    trans.rx_buffer = &spi_resp_pkt;            // Pointer to DMA-safe RX buffer

    // 4. Execute a synchronous SPI polling transaction
    //    Because our payload is small and clock is 1MHz, polling is much faster 
    //    and has lower CPU overhead than queueing a background task interrupt.
    esp_err_t ret = spi_device_polling_transmit(spi_master_handle, &trans);
    
    if (ret != ESP_OK) {
        // Log or handle SPI communication errors here if needed
        return resp;
    }

    // 5. Transfer the safely received data from the raw DMA buffer 
    //    into your processable, structured 'current_response' variable.
    //    Note: If resp_t matches resp_packet_t, adjust the type/size parameters accordingly.
    memcpy(&resp, &spi_resp_pkt, sizeof(resp_t));

    return resp;
}


/**
 * @brief Synchronously blocks and exchanges data on the SPI slave node.
 * * @param resp_to_send Pointer to the local response data the slave wants to hand to the master.
 * @return req_t Returns the populated request packet received from the Master, or an all-zero packet on failure.
 */
req_t emb_spi_slave_exchange(const resp_t *resp_to_send)
{
    req_t received_req = {0}; // Zero-initialize local return structure

    // 1. Safety check for pointers
    if (resp_to_send == NULL) {
        return received_req;
    }

    // 2. Clear buffers and copy input data safely into the DMA alignment structure
    memcpy(&spi_resp_pkt, resp_to_send, sizeof(resp_t));
    memset(&spi_req_pkt, 0, sizeof(req_t)); 

    // 3. Configure the ESP-IDF Slave Transaction
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    // CRITICAL: We use 84 * 8 bits (the absolute union size) to guarantee 
    // that both master and slave buffers have matching internal hardware bit expectations.
    t.length = sizeof(req_t) * 8; 
    t.tx_buffer = spi_resp_pkt.data; // Pointer to DMA safe TX data
    t.rx_buffer = spi_req_pkt.data; // Pointer to DMA safe RX storage

    // 4. Signal the Master node via your handshaking line (Active-Low Ready)
    // gpio_set_level(EMB_SLAVE_SPI_READY, 0);
    
    // Block until the Master asserts CS and finishes clocking all 84 bytes
    esp_err_t ret = spi_slave_transmit(EMB_SLAVE_SPI_HOST, &t, portMAX_DELAY);
    
    // De-assert your ready handshaking line immediately after the transaction
    // gpio_set_level(EMB_SLAVE_SPI_READY, 1);

    // 5. If transmission was successful, return the data by value
    if (ret == ESP_OK) {
        memcpy(&received_req, &spi_req_pkt, sizeof(req_t));
    }else{
        printf("SPI Driver Error: 0x%X\n", ret);
    }

    return received_req;
}


// ---------------------------------------- SPI --------------------------