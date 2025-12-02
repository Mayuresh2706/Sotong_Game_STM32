
#ifndef OLED_ANIMATIONS_H
#define OLED_ANIMATIONS_H

#include <stdint.h>

#define MAX_ANIMATIONS 5

// Animation structure - stores all data needed for one animation
typedef struct {
    const unsigned char** frames;      // Pointer to array of frame bitmaps
    const uint16_t* durations;         // Pointer to array of frame durations (ms)
    uint8_t frame_count;               // Total number of frames
    uint8_t x, y;                      // Position on screen
    uint8_t width, height;             // Dimensions of each frame
    uint8_t loop;                      // 1 = loop, 0 = play once

    // Runtime state (don't set these manually)
    uint8_t current_frame;
    uint32_t frame_start_time;
    uint8_t is_active;
    uint8_t is_finished;
} OLEDAnimation;

// Layout structure - contains multiple animations
typedef struct {
    OLEDAnimation* animations[MAX_ANIMATIONS];  // Pointers to animations
    uint8_t count;                              // Number of active animations
    uint8_t is_active;                          // Is this layout running?
} OLEDLayout;

// Public API
void oled_anim_init(void);
void oled_anim_update(void);
int oled_start_layout(OLEDLayout* layout, uint8_t interrupt_current);
void oled_stop_current_layout(void);
int oled_is_layout_active(void);
#endif // OLED_ANIMATIONS_H
