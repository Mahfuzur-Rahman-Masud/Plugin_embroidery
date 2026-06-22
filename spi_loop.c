#define EMBROIDERY_SPI_SPEED_HZ 1000000 // 1 MHz is perfect for standard diagnostics
#define SPI_MODE_0 0 // CPOL=0, CPHA=0 (Standard for Tajima/industrial tests)

#include "driver/spi_slave.h"
#include "driver/spi_master.h"
#include <string.h>

#define SLAVE_MOSI_PIN  GPIO_NUM_19  // Choose free GPIOs on your board
#define SLAVE_MISO_PIN  GPIO_NUM_14
#define SLAVE_SCK_PIN   GPIO_NUM_4             
#define SLAVE_CS_PIN    GPIO_NUM_5

#define TEST_MASTER_MOSI_PIN  SLAVE_MOSI_PIN
#define TEST_MASTER_MISO_PIN  SLAVE_MISO_PIN
#define TEST_MASTER_SCK_PIN   SLAVE_SCK_PIN
#define TEST_MASTER_CS_PIN    SLAVE_CS_PIN

static spi_device_handle_t master_handle = NULL;

bool embroidery_spi_slave_init(void)
{

        report_message("SPI: slave init", Message_Info);

    // 1. Configure the physical bus IO pins for the slave
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SLAVE_MOSI_PIN,
        .miso_io_num = SLAVE_MISO_PIN,
        .sclk_io_num = SLAVE_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // 2. Configure specific slave interface properties
    spi_slave_interface_config_t slv_cfg = {
        .mode = 0,                  // Must match Master SPI Mode (0)
        .spics_io_num = SLAVE_CS_PIN,
        .queue_size = 3,
        .flags = 0,
    };

    // 3. Initialize the SPI3 host as a Slave device using DMA Channel 2
    esp_err_t ret = spi_slave_initialize(SPI3_HOST, &bus_cfg, &slv_cfg, SPI_DMA_CH_AUTO); // GPIO_NUM_4
    
     report_message("SPI: slave ok", Message_Info);
    return (ret == ESP_OK);
}



/**
 * @brief Initialize the Master Subsystem (on SPI2_HOST)
 */
bool embroidery_spi_master_init(void)
{
    report_message("SPI: master init", Message_Info);
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = TEST_MASTER_MOSI_PIN,
        .miso_io_num = TEST_MASTER_MISO_PIN,
        .sclk_io_num = TEST_MASTER_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };

    // Initialize SPI2_HOST as the master bus
    if (spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO) != ESP_OK) {
        return false;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = EMBROIDERY_SPI_SPEED_HZ,
        .mode = SPI_MODE_0,
        .spics_io_num = TEST_MASTER_CS_PIN,
        .queue_size = 3,
    };

    // Attach our mock master device to the bus
    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &master_handle);
    report_message("SPI: master ok", Message_Info);
    return (ret == ESP_OK);
}

/**
 * @brief Slave data listening execution block
 */
bool embroidery_slave_receive(uint8_t *rx_buffer, uint16_t size)
{
    memset(rx_buffer, 0, size);

    spi_slave_transaction_t t = {
        .length = size * 8, 
        .rx_buffer = rx_buffer,
        .tx_buffer = NULL   
    };

    // Blocks until master begins transmission
    esp_err_t ret = spi_slave_transmit(SPI3_HOST, &t, portMAX_DELAY);
    return (ret == ESP_OK);
}

/**
 * @brief Master data transmission execution block
 */
bool embroidery_master_send(uint8_t *tx_buffer, uint16_t size)
{
    spi_transaction_t t = {
        .length = size * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = NULL
    };

    esp_err_t ret = spi_device_polling_transmit(master_handle, &t);
    return (ret == ESP_OK);
}



#include "driver/spi_slave.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// Global buffers to inspect test execution state
static uint8_t slave_rx_pool[64] = {0};
static uint8_t slave_tx_pool[64] = {0};
static TaskHandle_t slave_task_handle = NULL;

/**
 * @brief Background worker task that keeps the SPI Slave alert and listening.
 */
static void spi_slave_background_task(void *pvParameters)
{
    while (1) {
        spi_slave_transaction_t t = {
            .length = sizeof(slave_rx_pool) * 8, // Listen for up to 64 bytes
            .rx_buffer = slave_rx_pool,
            .tx_buffer = slave_tx_pool
        };

        // This blocks efficiently until the Master activates CS and clocks the line
        if (spi_slave_transmit(SPI3_HOST, &t, portMAX_DELAY) == ESP_OK) {
            // Optional: Process incoming data or log event here if needed
        }
    }
}

/**
 * @brief Spawns the background listener task for the Slave subsystem.
 */
void start_embroidery_slave_engine(void)
{
    // Populate dummy default response data into the Slave's outbound buffer
    memset(slave_tx_pool, 0xAA, sizeof(slave_tx_pool)); 
    
    xTaskCreate(
        spi_slave_background_task, 
        "spi_slave_task", 
        4096, 
        NULL, 
        10, // Give it a high priority so it never misses a master clock trigger
        &slave_task_handle
    );
}

/**
 * @brief Test Function 1: Master transmits a specific payload to the Slave.
 * @note Verifies that the physical MOSI wire pathway is intact.
 */
bool test_master_send(uint8_t *payload, uint16_t length)
{
    if (length > sizeof(slave_rx_pool)) return false;

    // Reset the target validation area so we don't read old data
    memset(slave_rx_pool, 0, sizeof(slave_rx_pool));

    spi_transaction_t t = {
        .length = length * 8,
        .tx_buffer = payload,
        .rx_buffer = NULL // Pure outbound transmission
    };

    // Execute transmission
    if (spi_device_polling_transmit(master_handle, &t) != ESP_OK) {
        return false; 
    }

    // Give the background task a tiny window to complete context processing
    vTaskDelay(pdMS_TO_TICKS(10));

    // VALIDATION: Compare Slave's RX buffer against what the Master sent
    return (memcmp(slave_rx_pool, payload, length) == 0);
}

/**
 * @brief Test Function 2: Master reads a block of data back from the Slave.
 * @note Verifies that the physical MISO wire pathway is intact.
 */
bool test_master_read(uint8_t *destination_buffer, uint16_t length)
{
    if (length > sizeof(slave_tx_pool)) return false;

    // Set up unique mock signature bytes inside the slave's queue
    for (int i = 0; i < length; i++) {
        slave_tx_pool[i] = 0x30 + i; // Generates ASCII sequence '0', '1', '2'...
    }

    // SPI requires the master to send dummy bytes to generate the clock pulses needed to read
    uint8_t *dummy_tx = malloc(length);
    if (!dummy_tx) return false;
    memset(dummy_tx, 0xFF, length);

    spi_transaction_t t = {
        .length = length * 8,
        .tx_buffer = dummy_tx,
        .rx_buffer = destination_buffer // Captures data streaming from Slave MISO
    };

    esp_err_t ret = spi_device_polling_transmit(master_handle, &t);
    free(dummy_tx);

    if (ret != ESP_OK) return false;

    // VALIDATION: Verify if received string matches the Slave's prepared pool data
    return (memcmp(destination_buffer, slave_tx_pool, length) == 0);
}


void run_embroidery_hardware_diagnostic(void)
{
    // 1. Initialize physical Master and Slave buses
    if (!embroidery_spi_slave_init() || !embroidery_spi_master_init()) {
        // Initialization failure (Check allocation flags or DMA channels)
        report_message("spi initialized", Message_Info);
        return;
    }

    // 2. Start the background Slave engine 
    start_embroidery_slave_engine();

     vTaskDelay(pdMS_TO_TICKS(10));

    // 3. Test Master Write (MOSI check)
    uint8_t test_msg[] = {'L', 'A', ':', '0'};
    if (test_master_send(test_msg, 4)) {
        // MOSI Pipeline Working Perfectly!
        report_message("Mosi Pipeline working", Message_Info);

    } else {
        // Line Dead (Verify physical Master Pin 13 to Slave Pin 19)
        report_message("Mosi pipeline dead", Message_Info);

    }

    // 4. Test Master Read (MISO check)
    uint8_t receive_packet[4] = {0};
    if (test_master_read(receive_packet, 4)) {
        // MISO Pipeline Working Perfectly!
        report_message("Master read is working", Message_Info);

    } else {
        // Line Dead (Verify physical Master Pin 12 to Slave Pin 14)
        report_message("Master read fail", Message_Info);

    }
}

