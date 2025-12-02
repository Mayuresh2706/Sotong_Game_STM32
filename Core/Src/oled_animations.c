#include "oled_animations.h"
#include "../../Drivers/OLED/ssd1306.h"
#include "stm32l4xx_hal.h"
#include <string.h>

// Internal state
static OLEDLayout* current_layout = NULL;
static OLEDLayout* pending_layout = NULL;

// Initialize the animation system
void oled_anim_init(void) {
    current_layout = NULL;
    pending_layout = NULL;
}

// Start a single animation
static void start_animation(OLEDAnimation* anim) {
    if (anim == NULL) return;

    anim->current_frame = 0;
    anim->frame_start_time = HAL_GetTick();
    anim->is_active = 1;
    anim->is_finished = 0;

    // Draw first frame
    if (anim->frames != NULL && anim->frames[0] != NULL) {
        ssd1306_DrawBitmap(anim->x, anim->y, anim->frames[0],
                          anim->width, anim->height, White);
    }
}

// Update a single animation - returns 1 if frame changed
static uint8_t update_animation(OLEDAnimation* anim) {
    if (anim == NULL || !anim->is_active) return 0;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - anim->frame_start_time;

    // Check if it's time to advance to next frame
    if (elapsed >= anim->durations[anim->current_frame]) {
        anim->current_frame++;

        // Check if animation is complete
        if (anim->current_frame >= anim->frame_count) {
            if (anim->loop) {
                // Loop back to start
                anim->current_frame = 0;
                anim->frame_start_time = now;
            } else {
                // Animation finished
                anim->is_active = 0;
                anim->is_finished = 1;
                return 0;
            }
        } else {
            anim->frame_start_time = now;
        }

        return 1; // Frame changed
    }

    return 0; // No change
}

// Start a layout
// interrupt_current: 0 = queue if busy, 1 = interrupt current layout
int oled_start_layout(OLEDLayout* layout, uint8_t interrupt_current) {
    if (layout == NULL || layout->count == 0) {
        return -1; // Invalid layout
    }

    // If a layout is currently running
    if (current_layout != NULL && current_layout->is_active) {
        if (interrupt_current) {
            // Stop current layout immediately
            //current_layout->is_active = 0;
        	oled_stop_current_layout();
            current_layout = NULL;
            pending_layout = NULL;
        } else {
            // Queue the new layout
            pending_layout = layout;
            return 0;
        }
    }

    // Start the new layout
    current_layout = layout;
    current_layout->is_active = 1;

    // Clear screen
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
    // Start all animations in the layout
    for (uint8_t i = 0; i < current_layout->count; i++) {
        if (current_layout->animations[i] != NULL) {
            start_animation(current_layout->animations[i]);
        }
    }

    // Update screen with first frames
    ssd1306_UpdateScreen();

    return 1; // Success
}

// Stop the current layout
void oled_stop_current_layout(void) {
    if (current_layout != NULL) {
        current_layout->is_active = 0;

        // Deactivate all animations
        for (uint8_t i = 0; i < current_layout->count; i++) {
            if (current_layout->animations[i] != NULL) {
                current_layout->animations[i]->is_active = 0;
            }
        }

        current_layout = NULL;
    }

    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
}

// Main update function - call this repeatedly in your main loop
void oled_anim_update(void) {
    // If no layout is active, check for pending
    if (current_layout == NULL || !current_layout->is_active) {
        if (pending_layout != NULL) {
            oled_start_layout(pending_layout, 1);
            pending_layout = NULL;
        }
        return;
    }

    uint8_t all_finished = 1;
    uint8_t any_frame_changed = 0;

    // Check which animations need frame updates
    for (uint8_t i = 0; i < current_layout->count; i++) {
        OLEDAnimation* anim = current_layout->animations[i];

        if (anim != NULL) {
            if (anim->is_active) {
                all_finished = 0;

                // Check if this animation needs a frame update
                if (update_animation(anim)) {
                    any_frame_changed = 1;
                }
            }
        }
    }

    // Only redraw if at least one frame changed
    if (any_frame_changed) {
        // Clear the screen
        ssd1306_Fill(Black);
        //ssd1306_UpdateScreen();
        // Redraw all active animations at their current frame
        for (uint8_t i = 0; i < current_layout->count; i++) {
            OLEDAnimation* anim = current_layout->animations[i];

            if (anim != NULL && anim->is_active) {
                if (anim->frames != NULL && anim->frames[anim->current_frame] != NULL) {
                    ssd1306_DrawBitmap(anim->x, anim->y,
                                      anim->frames[anim->current_frame],
                                      anim->width, anim->height, White);
                }
            }
        }

        // Update the physical screen
        ssd1306_UpdateScreen();
    }

    // If all animations finished, deactivate layout
    if (all_finished) {
        current_layout->is_active = 0;
        // Fill screen with white when layout finishes
        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
        // Start pending layout if exists
        if (pending_layout != NULL) {
            oled_start_layout(pending_layout, 1);
            pending_layout = NULL;
        }
    }
}
// Check if any layout is currently active
// Returns: 1 if a layout is playing, 0 if not
int oled_is_layout_active(void) {
    return (current_layout != NULL && current_layout->is_active);
}
