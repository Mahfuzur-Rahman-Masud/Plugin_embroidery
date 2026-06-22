/*

  tajima.c - plugin for parsing Tajima DST format file read from SD card.

  Part of grblHAL

  Copyright (c) 2023 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "embroidery.h"

#if EMBROIDERY_ENABLE

typedef struct {
    uint8_t b0;
    uint8_t b1;
    uint8_t b2;
} stitch_data_t;

#define TAJIMA_COLOR_COUNT_MAX 15
static uint8_t color_index[TAJIMA_COLOR_COUNT_MAX];

static const char* get_thread_color(embroidery_thread_color_t color)
{
    return color > THREAD_COUNT ? thread_list[0].name : thread_list[color].name;
}

static inline int16_t IRAM_ATTR get_x(uint8_t b2, uint8_t b1, uint8_t b0)
{
    int16_t x = 0;

    if (b2 & bit(2))
        x += 81;
    if (b2 & bit(3))
        x -= 81;
    if (b1 & bit(2))
        x += 27;
    if (b1 & bit(3))
        x -= 27;
    if (b0 & bit(2))
        x += 9;
    if (b0 & bit(3))
        x -= 9;
    if (b1 & bit(0))
        x += 3;
    if (b1 & bit(1))
        x -= 3;
    if (b0 & bit(0))
        x += 1;
    if (b0 & bit(1))
        x -= 1;

    return x;
}

static inline int16_t IRAM_ATTR get_y(uint8_t b2, uint8_t b1, uint8_t b0)
{
    int16_t y = 0;

    if (b2 & bit(5))
        y += 81;
    if (b2 & bit(4))
        y -= 81;
    if (b1 & bit(5))
        y += 27;
    if (b1 & bit(4))
        y -= 27;
    if (b0 & bit(5))
        y += 9;
    if (b0 & bit(4))
        y -= 9;
    if (b1 & bit(7))
        y += 3;
    if (b1 & bit(6))
        y -= 3;
    if (b0 & bit(7))
        y += 1;
    if (b0 & bit(6))
        y -= 1;

    return y;
}

static bool IRAM_ATTR get_stitch(stitch_t* stitch, vfs_file_t* file)
{
    bool sm = false;
    int16_t dx = 0, dy = 0;
    stitch_data_t sd;

    if (vfs_read(&sd, sizeof(stitch_data_t), 1, file) == sizeof(stitch_data_t)) {

        if ((sd.b2 & 0b11110011) == 0b11110011)
            return false;

        dx = get_x(sd.b2, sd.b1, sd.b0);
        dy = get_y(sd.b2, sd.b1, sd.b0);

        if ((sd.b2 & 0b11000011) == 0b11000011)
            stitch->type = Stitch_Stop;
        else if ((sd.b2 & 0b01000011) == 0b01000011)
            sm = !sm;
        else if ((sd.b2 & 0b10000011) == 0b10000011)
            stitch->type = sm ? Stitch_SequinEject : Stitch_Jump;
        else
            stitch->type = Stitch_Normal;

        stitch->target.x = (float)dx / 10.0f;
        stitch->target.y = (float)dy / 10.0f;

        long position = ((vfs_t *)(file->fs))->ftell(file);
        
        // char buf[40];
        // snprintf(buf, sizeof(buf), "position: %lu\n", (unsigned long)position); //
        // hal.stream.write_all(buf);
        stitch->stich_number = (int32_t)((position - 512) / 3);

        stitch->is_last = file->size < position + 2;

    } else {
        return false;
    }

    return true;
}

static bool read_meta(char* buf, vfs_file_t* file)
{
    char c;

    *buf = '\0';
    while (vfs_read(&c, 1, 1, file) == 1) {
        if (c == ASCII_EOF)
            return false;

        if (c == ASCII_CR || c == ASCII_LF) {
            *buf = '\0';
            break;
        }

        *buf++ = c;
    }

    return true;
}

bool tajima_open_file(vfs_file_t* file, embroidery_t* api)
{
    bool ok = false;
    static char buf[21];

    // check tajima file initial bytes from the meta data
    if (vfs_read(buf, 3, 1, file) != 3 || strncmp(buf, "LA:", 3) != 0) {
        // strncmp returns zero for exact match.
        // We could not read initial 3 bytes or the initial 3 bytes are not exact match to what is expected
        vfs_seek(file, 0);
        return false;
    }

    char meta[20];
    float value;
    uint_fast8_t idx;

    read_meta(buf, file); // Name

    while (read_meta(meta, file)) {

        idx = 3;
        strcaps(meta);

        if (read_float(meta, &idx, &value)) {

            if (!strncmp(meta, "ST:", 3))
                api->stitches = value;
            else if (!strncmp(meta, "CO:", 3))
                api->color_changes = value;
            else if (!strncmp(meta, "+X:", 3))
                api->max.x = value / 10.0f;
            else if (!strncmp(meta, "-X:", 3))
                api->min.x = -value / 10.0f;
            else if (!strncmp(meta, "+Y:", 3))
                api->max.y = value / 10.0f;
            else if (!strncmp(meta, "-Y:", 3))
                api->min.y = -value / 10.0f;
        }
    }

    vfs_seek(file, 120);
    char colors[TAJIMA_COLOR_COUNT_MAX * 4 + 4];
    if (read_meta(colors, file) && strncmp(colors, "CI:", 3) == 0) {
        uint_fast8_t idx = 3; // Start right after the "CI:" prefix
        float value;
        uint8_t count = 0;

        while (read_float(colors, &idx, &value)) {
            if (count < TAJIMA_COLOR_COUNT_MAX) {
                color_index[count++] = (uint8_t)value;
            } else {
                break; // Stop if the file contains more than 15 colors to prevent overflow
            }
        }
    } else {
        for (uint_fast8_t i = 0; i < TAJIMA_COLOR_COUNT_MAX; i++) {
            color_index[i] = i;
        }
    }

    api->name = buf;
    api->get_stitch = get_stitch;
    api->get_thread_color = get_thread_color;

    vfs_seek(file, 512);
    ok = true;

    return ok;
}

#endif // EMBROIDERY_ENABLE
