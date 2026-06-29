/*

  embroidery.c - plugin for reading and executing embroidery files from SD card.

  Part of grblHAL

  Copyright (c) 2023-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more programmed.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "embroidery.h"
#include "embroidery_spi.h"

#if EMBROIDERY_ENABLE

static const char* TAG = "EMBROIDERY";

#ifndef IOPORT_UNASSIGNED
#define IOPORT_UNASSIGNED 255
#endif

#if !(FS_ENABLE & FS_SDCARD)
#error "Embroidery plugin requires SD card plugin enabled!"
#endif

#include "custom/linear_motor.h"
#include "embroidery_spi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "grbl/grbl.h"
#include "grbl/motion_control.h"
#include "grbl/nvs_buffer.h"
#include "grbl/protocol.h"
#include "grbl/task.h"
#include "motor_control.h"
#include <math.h>


#ifdef MACHINE_FRONT
    
    #define STITCH_QUEUE_SIZE 32  // must be a power of 2

#else 
    #define STITCH_QUEUE_SIZE 16 // must be a power of 2

#endif    

extern bool brother_open_file(vfs_file_t* file, embroidery_t* api);
extern bool tajima_open_file(vfs_file_t* file, embroidery_t* api);

static float feed = 0;
static float accel = 0;

typedef enum {
    EmbroideryTrig_Falling = 0,
    EmbroideryTrig_Rising,
    EmbroideryTrig_ZLimit
} embroidery_trig_t;

typedef struct
{
    float feedrate;
    float z_travel;
    uint8_t port;
    bool sync_mode;
    uint16_t stop_delay;
    embroidery_trig_t edge;
    uint8_t debug_port;
    uint8_t break_port;
    uint8_t jump_port;

    float travel_window;
    float rpm_min;
    float rpm_max;
    float rpm_ramp_up;
    float rpm_ramp_down;
    float jump_feed_rate;

} embroidery_settings_t;

typedef struct
{
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
    stitch_t stitch[STITCH_QUEUE_SIZE];
} stitch_queue_t;

typedef struct
{
    uint32_t jumps;
    uint32_t stitches;
    uint32_t trims;
    uint32_t thread_changes;
    uint32_t sequin_ejects;
} embroidery_job_details_t;

typedef union {
    uint32_t value;
    struct
    {
        uint32_t pause : 1,
            trigger : 1,
            jump : 1;
    };
} embroidery_await_t;

typedef struct
{
    bool started;
    bool enqueued;
    bool completed;
    bool paused;
    bool stitching;
    bool first;
    volatile embroidery_await_t await;
    uint32_t trigger_interval, trigger_interval_min;
    uint32_t last_trigger;
    uint32_t stitch_interval;
    embroidery_job_details_t programmed;
    embroidery_job_details_t executed;
    uint32_t errs, exced, breaks;
    uint32_t spindle_stop;
    spindle_state_t spindle;
    volatile sys_state_t machine_state;
    vfs_file_t* file;
    plan_line_data_t plan_data;
    coord_data_t position;
    embroidery_thread_color_t color;
    stitch_queue_t queue;
    uint32_t stitch_loaded;
    uint32_t stitch_done;
} embroidery_job_t;

static uint8_t port, break_port, jump_port, debug_port;
static io_port_cfg_t d_in, d_out;
static uint32_t nvs_address;

static io_stream_t active_stream;
static embroidery_t api;
static embroidery_settings_t embroidery;
static on_report_options_ptr on_report_options;
static on_state_change_ptr on_state_change;
static on_execute_realtime_ptr on_execute_realtime;
static on_file_open_ptr on_file_open;
static driver_reset_ptr driver_reset;
static limit_interrupt_callback_ptr limits_interrupt_callback;
static embroidery_job_t job = { 0 };

TaskHandle_t xSDReadTaskHandle = NULL;


#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

static inline float __attribute__((always_inline)) _abs(float x){
    return x < 0 ? -x : x;
}

float IRAM_ATTR calculate_move_time(float dx, float dy, float f, float a)
{

    if (f == 0 || a == 0) {
        float fx = settings.axis[X_AXIS].max_rate / 60.0f;
        float fy = settings.axis[Y_AXIS].max_rate / 60.0f;
        float ax = settings.axis[X_AXIS].acceleration / 3600.0f;
        float ay = settings.axis[Y_AXIS].acceleration / 3600.0f;

        feed = fx < fy ? fx : fy;
        accel = ax < ay ? ax : ay;

        f = feed;
        a = accel;
    }

    float x = _abs(dx); // x motor travel
    float y = _abs(dy); // y motor travel

#ifdef COREXY // update how much each motor needs to travel for coreXY kinematics
    x = _abs(dx + dy);
    y = _abs(dx - dy);

    // may need to scale f and a
#endif

    float d = max(x, y); // assuming both motor has same speed, the maximum move determines the time required

    if (d < .001f) {
        return 0;
    }

    float ta = f / a; // time accelerating to reach max feed
    float da = 0.5f * a * ta * ta; // distance covered during acceleration to reach max feed

    float t = 0.0f;

    if (2.0f * da >= d) {
        // t = sqrtf(4 * d/ a);

        // Triangular profile (Short stitch: never reaches full feed rate)
        float v_peak = sqrtf(d * a);
        t = (2.0f * v_peak) / a;

    } else {

        float df = d - (2.0f * da); // distance traveled at max feed
        float tf = df / f; // time spent traveling at max speed
        t = (2.0f * ta) + tf;
    }

    return t * 1000;
}

// If current RPM is low then reduce all other stitch RPM in the queue to prepare the machine for a stop
static void IRAM_ATTR ramp_rpm(uint_fast8_t head)
{
    uint_fast8_t current_idx = head;
    uint_fast8_t prev_idx;

    float target_rpm = max(job.queue.stitch[current_idx].rpm, embroidery.rpm_ramp_down);

    while (current_idx != job.queue.tail) {
        prev_idx = (current_idx - 1) & (STITCH_QUEUE_SIZE - 1);

        if (prev_idx == job.queue.tail) {
            break;
        }

        if (job.queue.stitch[prev_idx].type == Stitch_Normal || job.queue.stitch[prev_idx].type == Stitch_SequinEject) {

            // job.queue.stitch[prev_idx].rpm = min(
            //     job.queue.stitch[prev_idx].rpm,
            //     job.queue.stitch[current_idx].rpm + RPM_RAMP_DOWN);

            job.queue.stitch[prev_idx].rpm = min(job.queue.stitch[prev_idx].rpm, target_rpm);

        } else {
            job.queue.stitch[prev_idx].rpm = 0.0f;
        }

        current_idx = prev_idx;
    }
}

float IRAM_ATTR calculate_spindle_rpm_max(float dx, float dy, float feed_max_mms, float accel_mms2)
{

    float tms = calculate_move_time(dx, dy, feed_max_mms, accel_mms2); // time to move xy
    tms = tms / embroidery.travel_window;

    float rpm = 60000.0f / tms;
    if (rpm > embroidery.rpm_max) {
        rpm = embroidery.rpm_max;
    } else if (rpm < embroidery.rpm_min) {
        rpm = embroidery.rpm_min;
    }

    return rpm;
}

void report_rpm(){
    #if USE_LINEAR_MOTOR_AS_SPINDLE
        linear_motor_report_rpm();
    #else
        reportRPM();
    #endif
}

int16_t get_rpm(){
      #if USE_LINEAR_MOTOR_AS_SPINDLE
        return linear_motor_get_rpm();
    #else
        return (int16_t) getRPM(); // motor control
    #endif
}


static bool  spindle_control(float rpm)
{
 
    // printf("spindle_control()\n");
    bool on = rpm > .001f;
    job.spindle.on = on;

    if (embroidery.sync_mode) {
        #if defined(USE_LINEAR_MOTOR_AS_SPINDLE)

                linear_motor_set_rpm_ramp((int16_t)rpm, (int16_t)embroidery.rpm_ramp_up);

        #elif defined(MOTOR_CONTROL_ENABLE)
                // hal.stream.write("spindle_control()\n");
                set_adrc_spindle_speed_ramp((float)rpm, (float)embroidery.rpm_ramp_up);

        #else
            if (spindle.set_speed) {
                spindle.set_speed((float)rpm);
            }
        #endif
    }

    return on;
}

static int32_t sdcard_read(void)
{

    // printf("sd_read()\n");

    // hal.stream.write("sdcard read()\n");
    // Guard 1: If job is already enqueued, nothing to process right now
    if (job.enqueued) {
        return SERIAL_NO_DATA;
    }

    uint_fast8_t bptr = (job.queue.head + 1) & (STITCH_QUEUE_SIZE - 1);

    // Guard 2: If the buffer is completely full, drop out early
    if (bptr == job.queue.tail) {
        return SERIAL_NO_DATA;
    }

    // Guard 3: Attempt to fetch the next stitch from the API.
    // If it fails (file end or read error), flag it, report it, and drop out.
    if(job.file == NULL){
        printf("Cannot read-- file is null!!! Attention!!!");
    }

    job.enqueued = !api.get_stitch(&job.queue.stitch[job.queue.head], job.file);
    if (job.enqueued) {

        // ensure previous stitch is slowed down to get to a full halt
        // may be this is not necessary as normally the last stitch is expected to be a jump or a stop
        uint_fast8_t prev_idx = (job.queue.head - 1) & (STITCH_QUEUE_SIZE - 1);
        if (prev_idx == job.queue.tail) {
            return SERIAL_NO_DATA;
        }

        job.queue.stitch[prev_idx].is_last = true;
        job.queue.stitch[prev_idx].rpm = min(job.queue.stitch[prev_idx].rpm, embroidery.rpm_ramp_down);
        ramp_rpm(prev_idx);
        return SERIAL_NO_DATA;
    }

    // -------------------------------------------------------------------------
    // CORE EXECUTION PIPELINE
    // -------------------------------------------------------------------------
    stitch_t stitch = job.queue.stitch[job.queue.head];
    float rpm = 0.0f;

    // Track statistics and handle raw RPM baselines
    switch (stitch.type) {
    case Stitch_Normal:
        rpm = calculate_spindle_rpm_max(stitch.target.x, stitch.target.y, feed, accel);
        job.programmed.stitches++;
        break;

    case Stitch_Jump:
        job.programmed.jumps++;
        break;

    case Stitch_Trim:
        job.programmed.trims++;
        break;

    case Stitch_Stop:
        job.programmed.thread_changes++;
        break;

    case Stitch_SequinEject:
        rpm = calculate_spindle_rpm_max(stitch.target.x, stitch.target.y, feed, accel);
        rpm = rpm / 2.0f;
        job.programmed.sequin_ejects++;
        break;
    }


    
    if(stitch.is_last){
        rpm = 0;
    }

    // set rpm and set any ramp for previous stitches
    job.queue.stitch[job.queue.head].rpm = rpm;
    ramp_rpm(job.queue.head);
    job.queue.head = bptr;

    #ifndef MACHINE_FRONT
        job.stitch_loaded = stitch.number;
    #endif

    // job.stitch_loaded

    return SERIAL_NO_DATA;
}

// Force clean termination of the file scanning thread upon any master reset sequence
static void  embroidery_abort_job(void)
{

    printf("EMB ABORT!!\n");


    // Close and release the tracking state flags instantly
    job.started = false;
    job.stitching = false;

    // 1. Terminate the background SD Card file pointer processing thread safely
    if (xSDReadTaskHandle != NULL) {
        vTaskDelete(xSDReadTaskHandle);
        xSDReadTaskHandle = NULL;
    }

    // 2. Safely close physical files if they are trapped open in memory tables
    if (job.file != NULL) {
        // Assuming standard grblHAL vfs wrappers
        vfs_close(job.file);
        job.file = NULL;
    }

    // 3. Immediately pull your custom linear motor voltage back to safe levels
    spindle_control(0);
}

static void end_job(void)
{

    // printf("end_job()\n");

    spindle_control(0);

    job.completed = job.enqueued = true;


    if (active_stream.type != StreamType_Null) {
        memcpy(&hal.stream, &active_stream, sizeof(io_stream_t));
        active_stream.type = StreamType_Null;
    }

    
    if (job.file) {
        vfs_close(job.file);
        job.file = NULL;
    }

    char log_buffer[64];
    printf("\nJob completed!: %s\n", api.name);
    printf("----------------------------------------\n");
    printf("Stitches: %d\n", api.stitches);
    snprintf(log_buffer, sizeof(log_buffer), "EMB Done: N:%d  J:%d T:%d C:%d S:%d\n",
    (int)job.executed.stitches,
    (int)job.executed.jumps,
    (int)job.executed.trims,
    (int)job.executed.thread_changes,
    (int)job.executed.sequin_ejects);
    
    printf(log_buffer);
    
    snprintf(log_buffer, sizeof(log_buffer), "ST        Interval: %d\n", job.stitch_interval);
    printf(log_buffer);
    
    snprintf(log_buffer, sizeof(log_buffer), "TRIG      Interval: %d\n", job.trigger_interval);
    printf(log_buffer);
    
    snprintf(log_buffer, sizeof(log_buffer), "TRIG MIN  Interval: %d\n", job.trigger_interval_min);
    printf(log_buffer);
    
    snprintf(log_buffer, sizeof(log_buffer), "Errors            : %d\n", job.errs);
    printf(log_buffer);
    printf("----------------------------------------\n\n");
}

// Start a tool change sequence. Called by gcode.c on a M6 command (via HAL).
static void thread_change(embroidery_thread_color_t color)
{
    const char* thread_color = api.get_thread_color(color);

    spindle_control(0);

    hal.stream.write_all("Color change: ");
    hal.stream.write_all(thread_color);
    hal.stream.write_all("\n");

    protocol_buffer_synchronize(); // Sync and finish all remaining buffered motions before moving on.
    system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
    protocol_execute_realtime(); // Execute suspend.

    job.executed.thread_changes++;
}

static void jump_out(bool on)
{
    hal.stream.write_all("#JumpO\n");
    report_rpm();

    if (jump_port == IOPORT_UNASSIGNED)
        hal.coolant.set_state((coolant_state_t) { .mist = on });
    else
        ioport_digital_out(jump_port, on);
}

static void exec_thread_change(void* data)
{
    api.thread_change(job.color);
}

static void thread_trim(void)
{
    spindle_control(0);
    report_message("#Thread Trim", Message_Info);
    report_rpm();

    protocol_buffer_synchronize(); // Sync and finish all remaining buffered motions before moving on.
    system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
    protocol_execute_realtime(); // Execute suspend.

    job.executed.trims++;
}

static void exec_thread_trim(void* data)
{
    api.thread_trim();
}

static void exec_hold(void* data)
{
    spindle_control(0);

    report_message((char*)data, Message_Info);
    protocol_buffer_synchronize(); // Sync and finish all remaining buffered motions before moving on.
    system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
    protocol_execute_realtime(); // Execute suspend.
    mc_line(job.position.values, &job.plan_data);
}

static void onStateChanged(sys_state_t state)
{

    // printf("onStateChange()\n");

    static uint32_t last_ms;

    if (job.machine_state == state)
        return;


    // if (state == STATE_RESET || state == STATE_ALARM) {
    if (state == STATE_ALARM) {
        embroidery_abort_job();
    }    

    switch (state) {

    case STATE_IDLE:
        if (job.await.jump) {
            jump_out(false);
            job.await.jump = false;

        } else if (job.stitching && job.machine_state == STATE_CYCLE) {

            uint32_t ms = hal.get_elapsed_ticks() - last_ms; // job.last_trigger;
            job.stitch_interval = max(job.stitch_interval, ms);
        }

        // machine is awaiting trigger so the spindle should be ramped up
        if (job.machine_state == STATE_HOLD && job.await.trigger) {
            spindle_control(embroidery.rpm_ramp_up);
        }

        break;

    case STATE_CYCLE:
        last_ms = hal.get_elapsed_ticks();
        break;
    }

    if (job.machine_state == STATE_HOLD) {
        job.await.pause = false;
    }

    job.machine_state = state;

    if (state == STATE_HOLD) {
        spindle_control(0);
    }

    if (on_state_change) {
        on_state_change(state);
    }

    if (debug_port != IOPORT_UNASSIGNED) {
        ioport_digital_out(debug_port, state == STATE_CYCLE);
    }
}



ISR_CODE void ISR_FUNC(z_limit_trigger)(limit_signals_t state)
{

}

ISR_CODE static void ISR_FUNC(needle_trigger)(uint8_t port, bool state)
{
    // printf("ndl()\n"); // this will surely crush the machine

    // uint32_t ms = hal.get_elapsed_ticks();
    uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    if (job.await.trigger /*&& ms - job.last_trigger > 25*/) {

        job.trigger_interval = ms - job.last_trigger;
        job.trigger_interval_min = job.trigger_interval_min < job.trigger_interval
            ? job.trigger_interval_min
            : job.trigger_interval;

        if (job.machine_state == STATE_CYCLE) {
            job.errs++;
            return;
        }

        job.await.trigger = false;
    }

    job.executed.stitches++;
    job.last_trigger = ms;
}

ISR_CODE static void ISR_FUNC(thread_break)(uint8_t port, bool state)
{
    if (job.file && !job.await.pause) {
        job.breaks++;
        if (job.executed.stitches > job.breaks + 10) {
            task_add_immediate(exec_hold, "Thread break!");
        }
    }
}

static void onTransferRealTime(){
    static bool busy = false;
    static bool was_busy = false;
    if (busy) {
        if (!was_busy) {
            // printf(".");
            was_busy = true;
        }

        return;
    }

    busy = true;
    was_busy = false;



    bool empty_q = job.queue.tail == job.queue.head || job.completed;

    if (empty_q) {
        if (job.enqueued && !job.completed) {
            end_job();
        }
    }


    static uint8_t tx_free = 5;
    static req_t req = { .id = 1 };
    static action_t pending_action = NONE;
    static action_t x_action = NONE;

    stitch_t* stitch = &job.queue.stitch[job.queue.tail];

    if(pending_action == NONE){


        if(!job.file){
            req.action = POLL;

        }else if(job.file && !job.started){
            req.action = START;
            req.stitch_number = api.stitches;

        }else if(empty_q 
            || tx_free < 1  
            || job.completed 
            || stitch->number < 1
            || stitch->number != job.stitch_loaded + 1
        ){
            req.action = POLL;
        }else {
            req.action = STITCH;
        }


        if(req.action == STITCH){
            req.stitch_number = stitch->number;
            req.type = stitch->type;
            req.color = stitch->color;
            req.coord[0] = stitch->target.x;
            req.coord[1] = stitch->target.y;
            req.rpm = stitch->rpm;
            req.feed = 10000;  //@Todo
            req.wait_trigger = stitch->type == Stitch_Normal || stitch->type == Stitch_SequinEject;
            req.last_stitch = stitch->is_last;
        }

  
        pending_action = req.action;

        if(x_action != POLL && pending_action != POLL){
            req.id++;

            printf("SPI: %s %u %d\n",
                (req.action == STITCH       ? "STITCH" 
                    : req.action == POLL    ? "POLL  "
                    : req.action == START   ? "START "
                                            : "ACT   "),
                req.id,                            
                req.stitch_number);
        }
    }



    if(req.last_stitch){
        // printf("sending last stitch %d\n", req.stitch_number);
    }

    resp_t resp = emb_spi_master_exchange(&req);
    if (resp.status == SLAVE_OK) {
        tx_free = resp.rx_free;
        job.stitch_loaded = resp.stitch_loaded == 0 ? job.stitch_loaded : resp.stitch_loaded;
        job.stitch_done = resp.stitch_done == 0 ? job.stitch_done : resp.stitch_done;

        if(job.stitch_loaded == stitch->number){
           job.queue.tail = (++job.queue.tail) & (STITCH_QUEUE_SIZE - 1);
           job.exced++;

            if(stitch->type == Stitch_Jump){
                job.programmed.jumps++;
            }else if (stitch->type == Stitch_Normal){
                job.programmed.stitches++;
            }else if (stitch->type == Stitch_SequinEject){
                job.programmed.sequin_ejects++;
            }else if (stitch->type == Stitch_Trim){
                job.programmed.trims++;
            }else {
                job.programmed.stitches ++;
            }

            if(stitch->is_last){
                printf("Last stitch sent!\n");
            }
        }

        if(resp.id == req.id){
            x_action = pending_action;
            pending_action = NONE;
        }

        switch (resp.action) {
        case START:
            printf("Job started\n");
            job.started = true;
            break;

        default:
            break;
        }

    } else {
        tx_free = tx_free - 1;
    }


    vTaskDelay(pdMS_TO_TICKS(5));
    busy = false;
}

void emb_spi_slave_read_loop(void* pvParameters)
{

    static resp_t resp = { .stitch_loaded = 0, .stitch_done = 0 };
    static uint8_t last_request;

    resp.action = SLAVE_OK;
    resp.status = SLAVE_OK;

    while (1) {
        uint8_t rx_free = (job.queue.tail - job.queue.head - 1 + STITCH_QUEUE_SIZE) % STITCH_QUEUE_SIZE;
        resp.rx_free = rx_free;
        uint_fast8_t bptr = (job.queue.head + 1) & (STITCH_QUEUE_SIZE - 1);

        resp.state = job.machine_state;
        resp.mpos[0] = sys.position[X_AXIS];
        resp.mpos[1] = sys.position[Y_AXIS];
        resp.mpos[2] = sys.position[Z_AXIS];
        resp.action_needed = 0;
        resp.rpm = get_rpm();
        resp.stitch_done = job.stitch_done;

        req_t req = emb_spi_slave_exchange(&resp);

        resp.action = req.action;
        resp.id = req.id;

        if (req.id != last_request) {
                // printf("SPI R: %s %u %d\t | %u\n",
                // (req.action == STITCH       ? "STITCH" 
                //     : req.action == POLL    ? "POLL  "
                //     : req.action == START   ? "START "
                //                             : "ACT   "),
                // req.id,                            
                // req.stitch_number, resp.rx_free);

            switch (req.action) {
            case PAUSE:
                grbl.enqueue_realtime_command('!');
                resp.status = SLAVE_OK;
                break;

            case RESUME:
                grbl.enqueue_realtime_command('~');
                resp.status = SLAVE_OK;
                break;

            case ABORT:
                grbl.enqueue_realtime_command(0x18);
                resp.status = SLAVE_OK;
                break;

            case MC_LINE:
                break;

            case START: {
                plan_data_init(&job.plan_data);

                // job.file = file;
                job.started = job.completed = job.enqueued = job.stitching = false;
                job.queue.head = job.queue.tail = job.stitch_interval = job.trigger_interval = job.await.value = job.breaks = 0;
                job.plan_data.feed_rate = embroidery.feedrate;
                job.plan_data.condition.rapid_motion = On;
                job.plan_data.spindle.hal->cap.at_speed = On,

                system_convert_array_steps_to_mpos(job.position.values, sys.position);

                memset(&job.programmed, 0, sizeof(embroidery_job_details_t));
                memset(&job.executed, 0, sizeof(embroidery_job_details_t));

                job.trigger_interval_min = 10000;
                job.stitch_interval = 0;
                job.trigger_interval = 0;
                job.errs = job.exced = 0;
                job.stitch_done = 0;
                job.stitch_loaded = 0;
                resp.status = SLAVE_OK;
                api.stitches = req.stitch_number;
                printf("Job started\n");
            }

            break;

            default:
                resp.status = SLAVE_OK;
                break;
            }

            last_request = req.id;
        }


        if (req.action != STITCH) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        
        stitch_t stitch = {
            .type = req.type,
            .color = req.color,

            .target.x = req.coord[0],
            .target.y = req.coord[1],
            .target.z = req.coord[2],
            .rpm = req.rpm,
            .number = req.stitch_number,
            .is_last = req.last_stitch
        };

        // Guard 2: If the buffer is completely full, drop out early
        // if (bptr == job.queue.tail) {
        //     return SERIAL_NO_DATA;
        // }

        if (rx_free > 0 && bptr != job.queue.tail && req.stitch_number != job.stitch_loaded) {
            resp.stitch_loaded = req.stitch_number;
            job.queue.stitch[job.queue.head] = stitch;
            job.stitch_loaded = resp.stitch_loaded;
            
            if(stitch.is_last){
                job.enqueued = true;
                ramp_rpm(job.queue.head);
                printf("last stitch loaded!\n");
            }
            
            job.queue.head = bptr;
            // printf("Stitch loaded: %ld\n", (long)req.stitch_number);
        } 
    
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void onExecuteRealtime(sys_state_t state)
{
    // printf("onExecRT()\n");

    static bool busy = false;

    on_execute_realtime(state);

    if (emb_machine_mode == Web_Front) {
        onTransferRealTime();
        return;
    }

    if (busy || job.completed) {
        return;
    }

    if (state == STATE_HOLD && job.spindle.on) {
        spindle_control(0);
    }

    if (state == STATE_HOLD) {
        return;
    }

    // it is time to stop the spindle as the delay is over
    if (job.spindle_stop && hal.get_elapsed_ticks() - job.last_trigger >= job.spindle_stop) {
        spindle_control(0);
        job.spindle_stop = 0;
    }

    // waiting for needle trigger or pause or jump
    if (job.await.value) {
        return;
    }

    if (job.enqueued && job.queue.tail == job.queue.head && !job.completed) {

        end_job();
        hal.stream.cancel_read_buffer();

        if (grbl.on_program_completed)
            grbl.on_program_completed(ProgramFlow_CompletedM30, false);

        grbl.report.feedback_message(Message_ProgramEnd);
        return;

    } else if (job.queue.tail == job.queue.head) {

        if (job.file && !job.enqueued && hal.stream.type != StreamType_File) {
            // printf("set emb file stream\n");
            memcpy(&active_stream, &hal.stream, sizeof(io_stream_t));
            hal.stream.type = StreamType_File; // then redirect to read from SD card instead
            hal.stream.read = sdcard_read;
        }

        return;
    }

    if (plan_get_block_buffer_available() < 3) {
        return;
    }

    stitch_t* stitch = &job.queue.stitch[job.queue.tail];

    // Wait for non-stitching moves to complete before starting stitching
    if (!job.stitching && stitch->type == Stitch_Normal && job.machine_state != STATE_IDLE) {
        return;
    }

    bool was_stitching = job.stitching;
    busy = true;
    job.queue.tail = ++job.queue.tail & (STITCH_QUEUE_SIZE - 1);
    stitch_t* next_stitch = &job.queue.stitch[job.queue.tail];

    // If stitching look-ahead to next command to see if we should stop the motor early to avoid overshoot.
    if (job.stitching) {
        if (job.queue.tail != job.queue.head
            && (next_stitch->type != Stitch_Normal || next_stitch->type != Stitch_SequinEject)) {

            if (next_stitch->type == Stitch_Jump) {
                jump_out(true);

            } else if (embroidery.stop_delay) {
                job.spindle_stop = embroidery.stop_delay;
            }
        }
    }

    if (!(job.stitching = (stitch->type == Stitch_Normal || stitch->type == Stitch_SequinEject)) && embroidery.stop_delay == 0) {
        spindle_control(0);
        job.spindle_stop = 0;
    }

    float f = 0.0f;
    switch (stitch->type) {

    case Stitch_Normal:
        job.exced++;
        job.plan_data.condition.rapid_motion = Off;

        job.position.x += stitch->target.x;
        job.position.y += stitch->target.y;
        spindle_control(stitch->rpm);

        // hal.stream.write_all("#SN\n");
        mc_line(job.position.values, &job.plan_data);

        if (!(job.await.trigger = embroidery.sync_mode)) {
            job.position.z = -embroidery.z_travel;
            mc_line(job.position.values, &job.plan_data);
            job.position.z = embroidery.z_travel;
            mc_line(job.position.values, &job.plan_data);
        }

        
        break;

    case Stitch_Jump:
        hal.stream.write_all("#STJ\n");
        spindle_control(0);
        report_rpm();

        job.executed.jumps++;
        job.plan_data.condition.rapid_motion = Off;

        job.position.x += stitch->target.x;
        job.position.y += stitch->target.y;

        f = job.plan_data.feed_rate;
        job.plan_data.feed_rate = embroidery.jump_feed_rate;
        job.await.jump = (stitch->target.x != 0.0f || stitch->target.y != 0.0f) && mc_line(job.position.values, &job.plan_data);
        job.plan_data.feed_rate = f;
        break;

    case Stitch_Trim:
        hal.stream.write_all("#STT\n");
        spindle_control(0);
        report_rpm();
        job.await.pause = true;
        job.plan_data.condition.rapid_motion = Off;
        job.spindle_stop = embroidery.stop_delay;

        job.position.x += stitch->target.x;
        job.position.y += stitch->target.y;

        f = job.plan_data.feed_rate;
        job.plan_data.feed_rate = embroidery.jump_feed_rate;
        mc_line(job.position.values, &job.plan_data);
        job.plan_data.feed_rate = f;
        task_add_immediate(exec_thread_trim, NULL);
        break;

    case Stitch_Stop:
        hal.stream.write_all("#STS\n");
        report_rpm();
        spindle_control(0);
        job.await.pause = !stitch->is_last;
        job.plan_data.condition.rapid_motion = On;

        job.position.x += stitch->target.x;
        job.position.y += stitch->target.y;

        f = job.plan_data.feed_rate;
        job.plan_data.feed_rate = embroidery.jump_feed_rate;
        mc_line(job.position.values, &job.plan_data);
        job.plan_data.feed_rate = f;

        job.color = stitch->color;
        
        if(!stitch->is_last){
            task_add_immediate(exec_thread_change, NULL);
        }

        job.spindle_stop = embroidery.stop_delay;
        break;

    case Stitch_SequinEject:
        hal.stream.write_all("#STE\n");

        job.exced++;
        job.executed.sequin_ejects++;
        job.plan_data.condition.rapid_motion = Off;

        job.position.x += stitch->target.x;
        job.position.y += stitch->target.y;
        spindle_control(stitch->rpm > embroidery.rpm_min * 2 ? stitch->rpm / 2 : stitch->rpm);

        mc_line(job.position.values, &job.plan_data);

        if (!(job.await.trigger = embroidery.sync_mode)) {
            job.position.z = -embroidery.z_travel;
            mc_line(job.position.values, &job.plan_data);
            job.position.z = embroidery.z_travel;
            mc_line(job.position.values, &job.plan_data);
        }

        break;
    }

    job.stitch_done = stitch->number;
    // printf("stitch: [%d/%d] / %d\n", job.stitch_done, job.stitch_loaded, api.stitches);

    busy = false;
}

static const char* trim(const char* s)
{
    char* s1 = strrchr(s, '\0');

    while (*(--s1) == '0')
        *s1 = '\0';

    if (*s1 == '.')
        *s1 = '\0';

    return s;
}

static spindle_data_t* spindleGetData(spindle_data_request_t request)
{
    static spindle_data_t spindle_data = { 0 };

    if (request == SpindleData_RPM)
        spindle_data.rpm = job.spindle.on ? 60000.0f / (float)job.trigger_interval : 0.0f;

    return &spindle_data;
}

static status_code_t onFileOpen(const char* fname, vfs_file_t* file, bool stream)
{
    bool ok = false;

    if (brother_open_file(file, &api) || tajima_open_file(file, &api)) {

        if (stream) {

            if (break_port == IOPORT_UNASSIGNED || ioport_wait_on_input(true, break_port, WaitMode_Immediate, 0.0f) == 0) {

                memcpy(&active_stream, &hal.stream, sizeof(io_stream_t)); // Save current stream pointers
                hal.stream.type = StreamType_File; // then redirect to read from SD card instead
                hal.stream.read = sdcard_read;

                plan_data_init(&job.plan_data);

                job.file = file;
                job.completed = job.enqueued = job.stitching = false;
                job.queue.head = job.queue.tail = job.stitch_interval = job.trigger_interval = job.await.value = job.breaks = 0;
                job.plan_data.feed_rate = embroidery.feedrate;
                job.plan_data.condition.rapid_motion = On;
                if (embroidery.sync_mode)
                    job.plan_data.spindle.hal->get_data = spindleGetData;
                job.plan_data.spindle.hal->cap.at_speed = On,
                system_convert_array_steps_to_mpos(job.position.values, sys.position);

                memset(&job.programmed, 0, sizeof(embroidery_job_details_t));
                memset(&job.executed, 0, sizeof(embroidery_job_details_t));

                job.trigger_interval_min = 10000;
                job.errs = job.exced = 0;
                job.stitch_done = 0;
                job.stitch_loaded = 0;
                job.started = false;

                // start_embroidery_task();
                hal.stream.write("ret onfileopen()\n");
            
            } else {
                vfs_close(file);
                report_message("No thread detected", Message_Error);
            }
        } else {

            bool no_move = false;
            coord_data_t target = { 0 };
            stich_type_t mode = Stitch_Stop;
            stitch_t stitch;

            stitch.target.x = stitch.target.y = 0.0f;

            hal.stream.write("G17G21G91" ASCII_EOL);
            hal.stream.write("F");
            hal.stream.write(uitoa((uint32_t)embroidery.feedrate));
            hal.stream.write(ASCII_EOL);

            while (api.get_stitch(&stitch, file)) {

                if (stitch.type == Stitch_Stop) {
                    hal.stream.write("T");
                    hal.stream.write(uitoa(stitch.color));
                    hal.stream.write(" (MSG,");
                    hal.stream.write(api.get_thread_color(stitch.color));
                    hal.stream.write(")" ASCII_EOL);
                } else if (stitch.type != Stitch_SequinEject) {

                    no_move = target.x == 0.0f && target.y == 0.0f;

                    if (no_move || mode != stitch.type) {
                        hal.stream.write(stitch.type == Stitch_Jump ? "G0" : "G1");
                        mode = stitch.type;
                    }
                    if (stitch.target.x != 0.0f) {
                        hal.stream.write("X");
                        hal.stream.write(trim(ftoa(stitch.target.x, N_DECIMAL_COORDVALUE_MM)));
                        target.x = stitch.target.x;
                    }
                    if (stitch.target.y != 0.0f) {
                        hal.stream.write("Y");
                        hal.stream.write(trim(ftoa(stitch.target.y, N_DECIMAL_COORDVALUE_MM)));
                        target.y = stitch.target.y;
                    }
                    hal.stream.write(ASCII_EOL);

                    if (stitch.type == Stitch_Trim)
                        hal.stream.write("M0 (MSG,Trim thread)" ASCII_EOL);
                }
            }

            hal.stream.write("M30" ASCII_EOL);
            end_job();
        }

        ok = true;
    }

    return ok ? Status_OK : (on_file_open ? on_file_open(fname, file, stream) : Status_Unhandled);
}

static void emb_driver_reset(void)
{


    report_message("#SD RST", Message_Info);
    embroidery_abort_job();
    driver_reset();
}

static const setting_group_detail_t embroidery_groups[] = {
    { Group_Root, Group_Embroidery, "Embroidery" }
};

static bool is_setting_available(const setting_detail_t* setting, uint_fast16_t offset)
{
    bool ok = false;

    switch (setting->id) {

    case Setting_Embroidery_Trigger_Port:
    case Setting_Embroidery_Edge:
    case Setting_Embroidery_Thread_Break_Port:
        ok = d_in.n_ports > 0;
        break;

    case Setting_Embroidery_Debug_Port:
    case Setting_Embroidery_Jump_Port:
        ok = d_out.n_ports > 0;
        break;

    default:
        break;
    }

    return ok;
}

static status_code_t set_port(setting_id_t setting, float value)
{
    status_code_t status = Status_SettingDisabled;

    switch (setting) {

    case Setting_Embroidery_Trigger_Port:
        status = d_in.set_value(&d_in, &embroidery.port, (pin_cap_t) { .irq_mode = IRQ_Mode_RisingFalling }, value);
        break;

    case Setting_Embroidery_Debug_Port:
        status = d_out.set_value(&d_out, &embroidery.debug_port, (pin_cap_t) { }, value);
        break;

    case Setting_Embroidery_Thread_Break_Port:
        status = d_in.set_value(&d_in, &embroidery.break_port, (pin_cap_t) { .irq_mode = IRQ_Mode_RisingFalling }, value);
        break;

    case Setting_Embroidery_Jump_Port:
        status = d_out.set_value(&d_out, &embroidery.jump_port, (pin_cap_t) { }, value);
        break;

    default:
        break;
    }

    return status;
}

static float get_port(setting_id_t setting)
{
    float value = -1.0f;

    switch (setting) {

    case Setting_Embroidery_Trigger_Port:
        value = d_in.get_value(&d_in, embroidery.port);
        break;

    case Setting_Embroidery_Debug_Port:
        value = d_out.get_value(&d_out, embroidery.debug_port);
        break;

    case Setting_Embroidery_Thread_Break_Port:
        value = d_in.get_value(&d_in, embroidery.break_port);
        break;

    case Setting_Embroidery_Jump_Port:
        value = d_out.get_value(&d_out, embroidery.jump_port);
        break;

    default:
        break;
    }

    return value;
}

static const setting_detail_t embroidery_settings[] = {
    { Setting_Embroidery_Feedrate, Group_Embroidery, "Feedrate", "mm/min", Format_Decimal, "####0.0", NULL, NULL, Setting_NonCore, &embroidery.feedrate, NULL, NULL, { 0 } },
    { Setting_Embroidery_Z_Travel, Group_Embroidery, "Z travel", "mm", Format_Decimal, "##0.0", NULL, NULL, Setting_NonCore, &embroidery.z_travel, NULL, NULL, { 0 } },
    { Setting_Embroidery_Trigger_Port, Group_AuxPorts, "Trigger port", NULL, Format_Decimal, "-#0", "-1", d_in.port_maxs, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },
    { Setting_Embroidery_Sync_Mode, Group_Embroidery, "Sync mode", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCore, &embroidery.sync_mode, NULL, NULL, { 0 } },
    { Setting_Embroidery_Stop_Delay, Group_Embroidery, "Stop delay", "milliseconds", Format_Int16, "##0", NULL, NULL, Setting_NonCore, &embroidery.stop_delay, NULL, NULL, { 0 } },
    { Setting_Embroidery_Edge, Group_Embroidery, "Trigger edge/input", NULL, Format_RadioButtons, "Falling,Rising,Z limit", NULL, NULL, Setting_NonCore, &embroidery.edge, NULL, NULL, { .reboot_required = On } },
    { Setting_Embroidery_Debug_Port, Group_AuxPorts, "Debug port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },
    { Setting_Embroidery_Thread_Break_Port, Group_AuxPorts, "Thread break port", NULL, Format_Decimal, "-#0", "-1", d_in.port_maxs, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },
    { Setting_Embroidery_Jump_Port, Group_AuxPorts, "Jump port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },

    // --- ADDED NEW SETTINGS POINTERS HERE ---
    { Setting_Embroidery_Travel_Window, Group_Embroidery, "Travel window", "fraction", Format_Decimal, "0.00", "0.05", "0.95", Setting_NonCore, &embroidery.travel_window, NULL, NULL, { 0 } },
    { Setting_Embroidery_RPM_Min, Group_Embroidery, "RPM min", "RPM", Format_Decimal, "####0", "0.0", "10000.0", Setting_NonCore, &embroidery.rpm_min, NULL, NULL, { 0 } },
    { Setting_Embroidery_RPM_Max, Group_Embroidery, "RPM max", "RPM", Format_Decimal, "####0", "0.0", "10000.0", Setting_NonCore, &embroidery.rpm_max, NULL, NULL, { 0 } },
    { Setting_Embroidery_RPM_Ramp_Up, Group_Embroidery, "RPM ramp (up)", "RPM/s", Format_Decimal, "####0", "1.0", "5000.0", Setting_NonCore, &embroidery.rpm_ramp_up, NULL, NULL, { 0 } },
    { Setting_Embroidery_RPM_Ramp_Down, Group_Embroidery, "RPM ramp (down)", "RPM/s", Format_Decimal, "####0", "1.0", "5000.0", Setting_NonCore, &embroidery.rpm_ramp_down, NULL, NULL, { 0 } },
    { Setting_Embroidery_Jump_Feedrate, Group_Embroidery, "Jump feedrate", "mm/min", Format_Decimal, "####0", "1.0", "50000.0", Setting_NonCore, &embroidery.jump_feed_rate, NULL, NULL, { 0 } }
};

static const setting_descr_t embroidery_settings_descr[] = {
    { Setting_Embroidery_Feedrate, "Feedrate to be used when embroidering." },
    { Setting_Embroidery_Z_Travel, "Z travel per stitch when needle is controlled by a stepper (sync mode = 0)." },
    { Setting_Embroidery_Trigger_Port, "Aux input port to use for needle trigger (sync mode = 1, trigger edge <> Z limit input). Set to -1 to disable." },
    { Setting_Embroidery_Sync_Mode, "When sync mode is enabled XY motion is controlled by needle trigger, else the Z axis stepper runs the needle motor." },
    { Setting_Embroidery_Stop_Delay, "Delay after last needle trigger before stopping needle motor (sync mode = 1)." },
    { Setting_Embroidery_Edge, "Trigger edge for needle trigger, from aux input or Z limit input (sync mode = 1).\\n\\n"
                               "NOTE: When Z limit input is used hard limits has to be enabled!" },
    { Setting_Embroidery_Debug_Port, "Debug port, outputs high on aux port when XY motion is ongoing. Set to -1 to disable." },
    { Setting_Embroidery_Thread_Break_Port, "Thread break detection port. Set to -1 to disable." },
    { Setting_Embroidery_Jump_Port, "Jump output port. Set to -1 to disable." },

    // --- ADDED NEW HELP TEXT DESCRIPTIONS HERE ---
    { Setting_Embroidery_Travel_Window, "Percentage of 1 rotation window allocated for XY gantry travel movement." },
    { Setting_Embroidery_RPM_Min, "Minimum spindle/needle motor rotation RPM baseline speed." },
    { Setting_Embroidery_RPM_Max, "Maximum spindle/needle motor rotation speed limit." },
    { Setting_Embroidery_RPM_Ramp_Up, "Acceleration ramp rate for bringing needle motor up to speed." },
    { Setting_Embroidery_RPM_Ramp_Down, "Deceleration ramp rate for bringing needle motor down to a stop." },
    { Setting_Embroidery_Jump_Feedrate, "Rapid feedrate velocity used specifically during non-stitching jump movements." }
};

static void embroidery_settings_save(void)
{
    report_message("#SETTING Save", Message_Info);
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t*)&embroidery, sizeof(embroidery_settings_t), true);
}

static void embroidery_settings_print()
{
    char log_buffer[64];

    // Group 1: General Mechanical Variables
    snprintf(log_buffer, sizeof(log_buffer), "Config G1: Feed:%.1f Z_Trv:%.1f StopDly:%u Sync:%d",
        embroidery.feedrate,
        embroidery.z_travel,
        (unsigned int)embroidery.stop_delay,
        (int)embroidery.sync_mode);
    report_message(log_buffer, Message_Info);

    // Group 2: Hardware/IO Ports Assignments
    snprintf(log_buffer, sizeof(log_buffer), "Config G2: TrigP:%d BrkP:%d JmpP:%d DbgP:%d Edge:%d",
        (int)embroidery.port,
        (int)embroidery.break_port,
        (int)embroidery.jump_port,
        (int)embroidery.debug_port,
        (int)embroidery.edge);
    report_message(log_buffer, Message_Info);

    // Group 3: Timing and Dynamics (New items)
    snprintf(log_buffer, sizeof(log_buffer), "Config G3: Window:%.2f RPM_Min:%.1f RPM_Max:%.1f",
        embroidery.travel_window,
        embroidery.rpm_min,
        embroidery.rpm_max);
    report_message(log_buffer, Message_Info);

    // Group 4: Ramps and Jumps (New items)
    snprintf(log_buffer, sizeof(log_buffer), "Config G4: RampUp:%.1f RampDn:%.1f JmpFeed:%.1f",
        embroidery.rpm_ramp_up,
        embroidery.rpm_ramp_down,
        embroidery.jump_feed_rate);
    report_message(log_buffer, Message_Info);
}

static void embroidery_settings_restore(void)
{

    report_message("#SETTING Restore", Message_Info);
    embroidery.feedrate = 4000.0f;
    embroidery.z_travel = 10.0f;
    embroidery.stop_delay = 0;
    embroidery.sync_mode = On;

    embroidery.port = IOPORT_UNASSIGNED;
    embroidery.break_port = IOPORT_UNASSIGNED;
    embroidery.jump_port = IOPORT_UNASSIGNED;
    embroidery.debug_port = IOPORT_UNASSIGNED;
    embroidery.edge = EmbroideryTrig_ZLimit;

    embroidery.travel_window = TRAVEL_WINDOW_PER_REV;
    embroidery.rpm_min = RPM_MIN;
    embroidery.rpm_max = RPM_MAX;
    embroidery.rpm_ramp_up = RPM_RAMP_UP;
    embroidery.rpm_ramp_down = RPM_RAMP_DOWN;
    embroidery.jump_feed_rate = JUMP_FEED_RATE;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t*)&embroidery, sizeof(embroidery_settings_t), true);
}

static void embroidery_settings_load(void)
{

    report_message("#Emb settings load", Message_Info);
    bool ok;

    if (hal.nvs.memcpy_from_nvs((uint8_t*)&embroidery, nvs_address, sizeof(embroidery_settings_t), true) != NVS_TransferResult_OK) {
        embroidery_settings_restore();
    }

    if (embroidery.travel_window <= 0.0f || embroidery.travel_window > 1.0f)
        embroidery.travel_window = TRAVEL_WINDOW_PER_REV;

    if (embroidery.rpm_min < 1 || embroidery.rpm_min > 500) {
        embroidery.rpm_min = RPM_MIN;
    }

    if (embroidery.rpm_max < 1 || embroidery.rpm_max > 2000) {
        embroidery.rpm_max = RPM_MAX;
    }

    if (embroidery.jump_feed_rate <= 0.0f)
        embroidery.jump_feed_rate = JUMP_FEED_RATE;

    if (embroidery.rpm_ramp_up > 500 || embroidery.rpm_ramp_up < 10)
        embroidery.rpm_ramp_up = RPM_RAMP_UP;
    if (embroidery.rpm_ramp_down > 500 || embroidery.rpm_ramp_down < 10)
        embroidery.rpm_ramp_down = RPM_RAMP_DOWN;

    if (embroidery.port >= d_in.n_ports)
        embroidery.port = IOPORT_UNASSIGNED;

    if (embroidery.break_port >= d_in.n_ports)
        embroidery.break_port = IOPORT_UNASSIGNED;

    if (embroidery.debug_port >= d_out.n_ports)
        embroidery.debug_port = IOPORT_UNASSIGNED;

    if (embroidery.jump_port >= d_out.n_ports)
        embroidery.jump_port = IOPORT_UNASSIGNED;

    if ((ok = embroidery.edge == EmbroideryTrig_ZLimit)) {

        hal.driver_cap.software_debounce = Off;

        limits_interrupt_callback = hal.limits.interrupt_callback;
        hal.limits.interrupt_callback = z_limit_trigger;
    } else if ((port = embroidery.port) != IOPORT_UNASSIGNED) {

        if ((ok = !!d_in.claim(&d_in, &port, "Embroidery needle trigger", (pin_cap_t) { .irq_mode = IRQ_Mode_RisingFalling })))
            ok = ioport_enable_irq(port, embroidery.edge ? IRQ_Mode_Rising : IRQ_Mode_Falling, needle_trigger);
    }

    if (ok) {
        if ((break_port = embroidery.break_port) != IOPORT_UNASSIGNED) {

            xbar_t* portinfo;

            if (!((portinfo = d_in.claim(&d_in, &break_port, "Embroidery thread break", (pin_cap_t) { .irq_mode = IRQ_Mode_RisingFalling })) && ioport_enable_irq(break_port, portinfo->mode.inverted ? IRQ_Mode_Rising : IRQ_Mode_Falling, thread_break)))
                task_run_on_startup(report_warning, "Embroidery plugin failed to claim port for thread break detection!");
        }

        if ((jump_port = embroidery.jump_port) != IOPORT_UNASSIGNED)
            d_out.claim(&d_out, &jump_port, "Embroidery jump output", (pin_cap_t) { });

        if ((debug_port = embroidery.debug_port) != IOPORT_UNASSIGNED)
            d_out.claim(&d_out, &debug_port, "Embroidery debug output", (pin_cap_t) { });

    } else {
        task_run_on_startup(report_warning, "Embroidery plugin failed to initialize, no pin for needle trigger signal!");
    }

    embroidery_settings_print();
}

static void onReportOptions(bool newopt)
{

    report_message("#REPORT Options", Message_Info);
    on_report_options(newopt);

    if (!newopt)
        report_plugin("EMBROIDERY", "0.14");
}

const char* embroidery_get_thread_color(embroidery_thread_color_t color)
{
    return api.get_thread_color ? api.get_thread_color(color) : NULL;
}

void embroidery_set_thread_trim_handler(thread_trim_ptr handler)
{
    api.thread_trim = handler;
}

void embroidery_set_thread_change_handler(thread_change_ptr handler)
{
    api.thread_change = handler;
}

void embroidery_init(void)
{

    static setting_details_t setting_details = {
        .groups = embroidery_groups,
        .n_groups = sizeof(embroidery_groups) / sizeof(setting_group_detail_t),
        .settings = embroidery_settings,
        .n_settings = sizeof(embroidery_settings) / sizeof(setting_detail_t),
        .descriptions = embroidery_settings_descr,
        .n_descriptions = sizeof(embroidery_settings_descr) / sizeof(setting_descr_t),
        .load = embroidery_settings_load,
        .restore = embroidery_settings_restore,
        .save = embroidery_settings_save
    };

    if ((nvs_address = nvs_alloc(sizeof(embroidery_settings_t)))) {

        job.completed = true;

        active_stream.type = StreamType_Null;

        ioports_cfg(&d_in, Port_Digital, Port_Input);
        ioports_cfg(&d_out, Port_Digital, Port_Output);

        settings_register(&setting_details);

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        driver_reset = hal.driver_reset;
        hal.driver_reset = emb_driver_reset;

        on_file_open = grbl.on_file_open;
        grbl.on_file_open = onFileOpen;

        on_state_change = grbl.on_state_change;
        grbl.on_state_change = onStateChanged;

        api.thread_trim = thread_trim;
        api.thread_change = thread_change;

        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = onExecuteRealtime;

        if (emb_machine_mode == Core_Back) {
            emb_spi_slave_init();
            // emb_spi_slave_read_loop();

            // Spawns a dedicated task pinned to Core 0 (the network/Wi-Fi core)
            // We give it a priority of 4 (grblHAL stepper task usually sits around 15-20)
            BaseType_t xReturned = xTaskCreatePinnedToCore(
                emb_spi_slave_read_loop, // Task function wrapper
                "emb_spi_slave_task", // Text name for debugging
                4096, // Stack size in words (16KB total - plenty for VFS / logic)
                NULL, // Task input parameters
                4, // Priority level (Low/Medium background execution)
                NULL, // Task handle out
                0 // Pinned to Core 0 (Wi-Fi and system core)
            );

            if (xReturned != pdPASS) {
                hal.stream.write_all("Error: Failed to create SPI Slave Task\r\n");
            }

        } else if (emb_machine_mode == Web_Front) {
            emb_spi_master_init();
        }

        // spindle = spindle_get(0);

        // init_spi_loopback_test();
        // if (run_loopback_verification()) {
        //     report_message("Emb SPI: verification complete", Message_Info);
        // } else {
        //     report_message("Emb SPI: verification failed", Message_Info);
        // }

        // if (run_loopback_verification()) {
        //     report_message("Emb SPI: verification 2 complete", Message_Info);
        // } else {
        //     report_message("Emb SPI: verification 2 failed", Message_Info);
        // }

    } else
        task_run_on_startup(report_warning, "Embroidery plugin failed to initialize, no NVS storage for settings!");
}

#endif // EMBROIDERY_ENABLE