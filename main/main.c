#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define RF_RECEIVER_GPIO 3       // XIAO D1 / RF receiver DATA
#define RF_TRANSMITTER_GPIO 4    // XIAO D2 / RF transmitter DATA
#define RMT_RESOLUTION_HZ 1000000
#define RMT_SYMBOL_BUFFER_SIZE 128

// Approximate timings measured from the captured remote signals.
#define RF_SHORT_US 450
#define RF_LONG_US  1150
#define RF_FRAME_GAP_US 9000
#define RF_COMMAND_BITS 25
#define RF_TRANSMIT_REPEATS 20   // Increase this to improve reception reliability.

static const char *TAG = "RF_RECEIVER";

static rmt_channel_handle_t rx_channel;
static rmt_channel_handle_t tx_channel;
static rmt_encoder_handle_t copy_encoder;
static QueueHandle_t receive_queue;

static rmt_symbol_word_t raw_symbols[RMT_SYMBOL_BUFFER_SIZE];
static rmt_symbol_word_t tx_symbols[RF_COMMAND_BITS + 1];

// The first 16 bits are common to all captures. The final 9 bits identify
// the button. These are pulse-width symbols, with '1' meaning long/short
// and '0' meaning short/long.
// Exact 25-symbol power-on packet from the clean single-press capture.
static const char *POWER_ON_BITS   = "1111111111111111000010000";
static const char *POWER_OFF_BITS  = "1111111111111111000001110";
static const char *FLASH_BITS      = "1111111111111111000100000";
static const char *STROBE_BITS     = "1111111111111111000101000";
static const char *FADE_BITS       = "1111111111111111000110000";
static const char *SMOOTH_BITS     = "1111111111111111000111000";
static const char *BRIGHT_UP_BITS  = "1111111111111111000001010";
static const char *BRIGHT_DOWN_BITS = "1111111111111111000001100";
static const char *WHITE_BITS      = "1111111111111111000011000";

// Color-grid commands, named by row-column position on the remote.
static const char *COLOR_1_1_BITS = "1111111111111111000010010"; // Red
static const char *COLOR_1_2_BITS = "1111111111111111000010100"; // Green
static const char *COLOR_1_3_BITS = "1111111111111111000010110"; // Blue
static const char *COLOR_2_1_BITS = "1111111111111111000011010";
static const char *COLOR_2_2_BITS = "1111111111111111000011100";
static const char *COLOR_2_3_BITS = "1111111111111111000011110";
static const char *COLOR_3_1_BITS = "1111111111111111000100010";
static const char *COLOR_3_2_BITS = "1111111111111111000100100";
static const char *COLOR_3_3_BITS = "1111111111111111000100110";
static const char *COLOR_4_1_BITS = "1111111111111111000101010";
static const char *COLOR_4_2_BITS = "1111111111111111000101100";
static const char *COLOR_4_3_BITS = "1111111111111111000101110";
static const char *COLOR_5_1_BITS = "1111111111111111000110010";
static const char *COLOR_5_2_BITS = "1111111111111111000110100";
static const char *COLOR_5_3_BITS = "1111111111111111000110110";

typedef struct {
    rmt_rx_done_event_data_t received;
} receive_event_t;

static bool IRAM_ATTR rmt_rx_done_callback(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *edata,
    void *user_data)
{
    BaseType_t high_task_woken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;

    receive_event_t event = {
        .received = *edata,
    };

    xQueueSendFromISR(queue, &event, &high_task_woken);

    return high_task_woken == pdTRUE;
}

static void print_received_symbols(
    const rmt_symbol_word_t *symbols,
    size_t symbol_count)
{
    printf("\nCaptured %u RMT symbols:\n", (unsigned)symbol_count);

    for (size_t i = 0; i < symbol_count; i++) {
        printf(
            "%03u: level=%u duration=%" PRIu32
            " us, level=%u duration=%" PRIu32 " us\n",
            (unsigned)i,
            symbols[i].level0,
            (uint32_t)symbols[i].duration0,
            symbols[i].level1,
            (uint32_t)symbols[i].duration1
        );
    }

    printf("End frame\n\n");
}

static void build_tx_frame(const char *bits)
{
    for (size_t i = 0; i < RF_COMMAND_BITS; i++) {
        bool long_first = bits[i] == '1';

        // The SYN480R receiver output is inverted relative to the
        // transmitter DATA waveform. Swap the levels so the receiver sees
        // the same long/short polarity as the original remote.
        tx_symbols[i].level0 = 1;
        tx_symbols[i].duration0 = long_first ? RF_LONG_US : RF_SHORT_US;
        tx_symbols[i].level1 = 0;
        tx_symbols[i].duration1 = long_first ? RF_SHORT_US : RF_LONG_US;
    }

    // Leave the transmitter low between repeated packets.
    tx_symbols[RF_COMMAND_BITS].level0 = 0;
    tx_symbols[RF_COMMAND_BITS].duration0 = RF_FRAME_GAP_US;
    tx_symbols[RF_COMMAND_BITS].level1 = 0;
    tx_symbols[RF_COMMAND_BITS].duration1 = 0;
}

static void transmit_command(const char *name, const char *bits)
{
    build_tx_frame(bits);

    rmt_transmit_config_t tx_config = {
        // Send multiple copies. Each copy already includes the measured
        // approximately 12 ms inter-packet gap.
        .loop_count = RF_TRANSMIT_REPEATS,
    };

    ESP_ERROR_CHECK(
        rmt_transmit(
            tx_channel,
            copy_encoder,
            tx_symbols,
            sizeof(tx_symbols),
            &tx_config
        )
    );

    ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_channel, portMAX_DELAY));
    ESP_LOGI(TAG, "Transmitted %s (%d repeats)",
             name, RF_TRANSMIT_REPEATS);
}

static void transmitter_task(void *arg)
{
    (void)arg;

    bool power_on = false;

    while (true) {
        if (power_on) {
            transmit_command("power on", POWER_ON_BITS);
        } else {
            transmit_command("power off", POWER_OFF_BITS);
        }

        power_on = !power_on;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    receive_queue = xQueueCreate(1, sizeof(receive_event_t));

    if (receive_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create receive queue");
        return;
    }

    rmt_rx_channel_config_t rx_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .gpio_num = RF_RECEIVER_GPIO,
        .flags.invert_in = false,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_config, &rx_channel));

    rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = rmt_rx_done_callback,
    };

    ESP_ERROR_CHECK(
        rmt_rx_register_event_callbacks(
            rx_channel,
            &callbacks,
            receive_queue
        )
    );

    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    rmt_tx_channel_config_t tx_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .gpio_num = RF_TRANSMITTER_GPIO,
        .trans_queue_depth = 2,
        .flags.invert_out = false,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &tx_channel));

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_ERROR_CHECK(
        rmt_new_copy_encoder(&copy_encoder_config, &copy_encoder)
    );
    ESP_ERROR_CHECK(rmt_enable(tx_channel));

    rmt_receive_config_t receive_config = {
        // Ignore short glitches commonly produced by inexpensive RF receivers.
        .signal_range_min_ns = 3000,

        // End a capture when an RF signal remains inactive for 15 ms.
        .signal_range_max_ns = 15000000,
    };

    ESP_ERROR_CHECK(
        rmt_receive(
            rx_channel,
            raw_symbols,
            sizeof(raw_symbols),
            &receive_config
        )
    );

    ESP_LOGI(TAG, "RF receiver ready on GPIO%d", RF_RECEIVER_GPIO);
    ESP_LOGI(TAG, "RF transmitter ready on GPIO%d", RF_TRANSMITTER_GPIO);
    ESP_LOGI(TAG, "Alternating power commands every 5 seconds");
    ESP_LOGI(TAG, "Commands loaded: on=%s off=%s flash=%s strobe=%s fade=%s smooth=%s",
             POWER_ON_BITS, POWER_OFF_BITS, FLASH_BITS, STROBE_BITS,
             FADE_BITS, SMOOTH_BITS);
    ESP_LOGI(TAG, "Additional commands: bright_up=%s bright_down=%s white=%s",
             BRIGHT_UP_BITS, BRIGHT_DOWN_BITS, WHITE_BITS);
    ESP_LOGI(TAG, "Color commands loaded: %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s",
             COLOR_1_1_BITS, COLOR_1_2_BITS, COLOR_1_3_BITS,
             COLOR_2_1_BITS, COLOR_2_2_BITS, COLOR_2_3_BITS,
             COLOR_3_1_BITS, COLOR_3_2_BITS, COLOR_3_3_BITS,
             COLOR_4_1_BITS, COLOR_4_2_BITS, COLOR_4_3_BITS,
             COLOR_5_1_BITS, COLOR_5_2_BITS, COLOR_5_3_BITS);

    BaseType_t task_created = xTaskCreate(
        transmitter_task,
        "rf_transmitter",
        4096,
        NULL,
        5,
        NULL
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create transmitter task");
        return;
    }

    while (true) {
        receive_event_t event;

        if (xQueueReceive(receive_queue, &event, portMAX_DELAY) == pdTRUE) {
            print_received_symbols(
                event.received.received_symbols,
                event.received.num_symbols
            );

            // Arm the receiver for the next frame.
            ESP_ERROR_CHECK(
                rmt_receive(
                    rx_channel,
                    raw_symbols,
                    sizeof(raw_symbols),
                    &receive_config
                )
            );
        }
    }
}
