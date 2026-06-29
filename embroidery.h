/*

  embroidery.h - plugin for reading and executing embroidery file from SD card.

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

#ifndef _EMBROIDERY_H_
#define _EMBROIDERY_H_

#include "driver.h"
typedef enum {
    Web_Front,
    Core_Back,
    Complex_Machine
}emb_machine_mode_t;

#ifdef MACHINE_CORE 
    static emb_machine_mode_t emb_machine_mode = Core_Back;
#elif defined(MACHINE_FRONT)
    static emb_machine_mode_t emb_machine_mode = Web_Front;
#else
    static emb_machine_mode_t emb_machine_mode = Complex_Machine;
#endif    

typedef enum {
    Stitch_Normal,
    Stitch_Trim,
    Stitch_Jump,
    Stitch_Stop,
    Stitch_SequinEject
} stich_type_t;

struct rgb {
    int r; 
    int g; 
    int b; 
};

struct thread {
    const char *name;
    struct rgb rgb;
};


static const struct thread thread_list[] = {
    { "Undefined",         { 220, 220, 220 } },
    { "Prussian Blue",     {  26,  10, 148 } },
    { "Blue",              {  15, 117, 255 } },
    { "Teal Green",        {   0, 147,  76 } },
    { "Corn Flower Blue",  { 186, 189, 254 } },
    { "Red",               { 236,   0,   0 } },
    { "Reddish Brown",     { 228, 153,  90 } },
    { "Magenta",           { 204,  72, 171 } },
    { "Light Lilac",       { 253, 196, 250 } },
    { "Lilac",             { 221, 132, 205 } },
    { "Mint Green",        { 107, 211, 138 } },
    { "Deep Gold",         { 228, 169,  69 } },
    { "Orange",            { 255, 189,  66 } },
    { "Yellow",            { 255, 230,   0 } },
    { "Lime Green",        { 108, 217,   0 } },
    { "Brass",             { 193, 169,  65 } },
    { "Silver",            { 181, 173, 151 } },
    { "Russet Brown",      { 186, 156,  95 } },
    { "Cream Brown",       { 250, 245, 158 } },
    { "Pewter",            { 128, 128, 128 } },
    { "Black",             {   0,   0,   0 } },
    { "Ultramarine",       {   0,  28, 223 } },
    { "Royal Purple",      { 223,   0, 184 } },
    { "Dark Gray",         {  98,  98,  98 } },
    { "Dark Brown",        { 105,  38,  13 } },
    { "Deep Rose",         { 255,   0,  96 } },
    { "Light Brown",       { 191, 130,   0 } },
    { "Salmon Pink",       { 243, 145, 120 } },
    { "Vermillion",        { 255, 104,   5 } },
    { "White",             { 240, 240, 240 } },
    { "Violet",            { 200,  50, 205 } },
    { "Seacrest",          { 176, 191, 155 } },
    { "Sky Blue",          { 101, 191, 235 } },
    { "Pumpkin",           { 255, 186,   4 } },
    { "Cream Yellow",      { 255, 240, 108 } },
    { "Khaki",             { 254, 202,  21 } },
    { "Clay Brown",        { 243, 129,   1 } },
    { "Leaf Green",        {  55, 169,  35 } },
    { "Peacock Blue",      {  35,  70,  95 } },
    { "Gray",              { 166, 166, 149 } },
    { "Warm Gray",         { 206, 191, 166 } },
    { "Dark Olive",        { 150, 170,   2 } },
    { "Linen",             { 255, 227, 198 } },
    { "Pink",              { 255, 153, 215 } },
    { "Deep Green",        {   0, 112,   4 } },
    { "Lavender",          { 237, 204, 251 } },
    { "Wisteria Violet",   { 192, 137, 216 } },
    { "Beige",             { 231, 217, 180 } },
    { "Carmine",           { 233,  14, 134 } },
    { "Amber Red",         { 207, 104,  41 } },
    { "Olive Green",       {  64, 134,  21 } },
    { "Dark Fuschia",      { 219,  23, 151 } },
    { "Tangerine",         { 255, 167,   4 } },
    { "Light Blue",        { 185, 255, 255 } },
    { "Emerald Green",     {  34, 137,  39 } },
    { "Purple",            { 182,  18, 205 } },
    { "Moss Green",        {   0, 170,   0 } },
    { "Flesh Pink",        { 254, 169, 220 } },
    { "Harvest Gold",      { 254, 213,  16 } },
    { "Electric Blue",     {   0, 151, 223 } },
    { "Lemon Yellow",      { 255, 255, 132 } },
    { "Fresh Green",       { 207, 231, 116 } },
    { "Applique Material", { 255, 200, 100 } },
    { "Applique Position", { 255, 200, 200 } },
    { "Applique",          { 255, 200, 200 } }
};

#define THREAD_COUNT (sizeof(thread_list) / sizeof(thread_list[0]))
static const size_t thread_count = THREAD_COUNT;


typedef uint8_t embroidery_thread_color_t;

typedef struct {
    stich_type_t type;
    embroidery_thread_color_t color;
    coord_data_t target;
    int32_t number;
    bool is_last;
    float rpm;
} stitch_t;

typedef bool (*get_stitch_ptr)(stitch_t *stitch, vfs_file_t *file);
typedef const char *(*get_thread_color_ptr)(uint8_t color);
typedef void (*thread_trim_ptr)(void);
typedef void (*thread_change_ptr)(embroidery_thread_color_t color);

typedef struct {
    get_stitch_ptr get_stitch;
    get_thread_color_ptr get_thread_color;
    thread_trim_ptr thread_trim;
    thread_change_ptr thread_change;
    const char *name;
    uint32_t stitches;
    uint32_t threads;
    uint32_t trims;
    uint32_t color_changes;
    coord_data_t min;
    coord_data_t max;
    coord_data_t size;
} embroidery_t;

typedef bool (*open_file_ptr)(vfs_file_t *file, embroidery_t *api);

const char *embroidery_get_thread_color (embroidery_thread_color_t color);
void embroidery_set_thread_trim_handler (thread_trim_ptr handler);
void embroidery_set_thread_change_handler (thread_change_ptr handler);

#endif // _EMBROIDERY_H_