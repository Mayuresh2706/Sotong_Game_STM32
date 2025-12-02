#ifndef OLED_ASSETS_H
#define OLED_ASSETS_H

#include "oled_animations.h"
#include <stdint.h>

//SOTONG GAME TEXT ANIMATION
#define SOTONG_GAME_NUM_FRAMES 30
extern const unsigned char* sotong_game_frames[SOTONG_GAME_NUM_FRAMES];

extern const uint16_t sotong_game_durations[SOTONG_GAME_NUM_FRAMES];
extern OLEDAnimation sotong_game_anim;
extern OLEDLayout sotong_game_layout;

//RED LIGHT GREEN LIGHT TEXT ANIMATION
#define RED_LIGHT_GREEN_LIGHT_NUM_FRAMES 21
extern const unsigned char* red_light_green_light_frames[RED_LIGHT_GREEN_LIGHT_NUM_FRAMES];

extern const uint16_t red_light_green_light_durations[RED_LIGHT_GREEN_LIGHT_NUM_FRAMES];
extern OLEDAnimation red_light_green_light_anim;
extern OLEDLayout red_light_green_light_layout;

//CATCH AND RUN TEXT ANIMATION
#define CATCH_AND_RUN_NUM_FRAMES 27
extern const unsigned char* catch_and_run_frames[CATCH_AND_RUN_NUM_FRAMES];

extern const uint16_t catch_and_run_durations[CATCH_AND_RUN_NUM_FRAMES];
extern OLEDAnimation catch_and_run_anim;
extern OLEDLayout catch_and_run_layout;

//COUNTDOWN 10 SEC ANIMATION
extern const unsigned char* countdown_10_sec_frames[10];

extern const uint16_t countdown_10_sec_durations[10];
extern OLEDAnimation countdown_10_sec_anim;

//COUNTDOWN 8 SEC ANIMATION
extern const unsigned char* countdown_8_sec_frames[8];

extern const uint16_t countdown_8_sec_durations[8];
extern OLEDAnimation countdown_8_sec_anim;

//COUNTDOWN 3 SEC ANIMATION
extern const unsigned char* countdown_3_sec_frames[3];

extern const uint16_t countdown_3_sec_durations[3];
extern OLEDAnimation countdown_3_sec_anim;

//HOPPING CAT GREEN LIGHT ANIMATION
#define HOPPING_CAT_NUM_FRAMES 2

extern const unsigned char* hopping_cat_frames[HOPPING_CAT_NUM_FRAMES];

extern const uint16_t hopping_cat_durations[HOPPING_CAT_NUM_FRAMES];
extern OLEDAnimation hopping_cat_anim;

//GREEN AND RED TEXT FOR RED LIGHT GREEN LIGHT
extern const unsigned char* text_green_frames[1];

extern const uint16_t text_green_durations[1];
extern OLEDAnimation text_green_anim;

extern const unsigned char* text_red_frames[1];

extern const uint16_t text_red_durations[1];
extern OLEDAnimation text_red_anim;

//ALERT CAT FOR RED LIGHT
#define ALERT_CAT_NUM_FRAMES 30
extern const unsigned char* alert_cat_frames[ALERT_CAT_NUM_FRAMES];

extern const uint16_t alert_cat_durations[ALERT_CAT_NUM_FRAMES];
extern OLEDAnimation alert_cat_anim;


//RED LIGHT AND GREEN LIGHT LAYOUTS
extern OLEDLayout green_light_layout_initial;
extern OLEDLayout green_light_layout;
extern OLEDLayout red_light_layout;

//RED LIGHT GREEN LIGHT GAME OVER
extern const unsigned char* red_light_green_light_game_over_frames[1];

extern const uint16_t red_light_green_light_game_over_durations[1];
extern OLEDAnimation red_light_green_light_game_over_anim;
extern OLEDLayout red_light_green_light_game_over_layout;

//CATCH AND RUN CAT
#define CATCH_AND_RUN_CAT_NUM_FRAMES 36

extern const unsigned char* catch_and_run_cat_frames[CATCH_AND_RUN_CAT_NUM_FRAMES];

extern const uint16_t catch_and_run_cat_durations[CATCH_AND_RUN_CAT_NUM_FRAMES];
extern OLEDAnimation catch_and_run_cat_anim;

//CATCH AND RUN BLINKING EYES
#define BLINKING_EYES_NUM_FRAMES 4

extern const unsigned char* blinking_eyes_frames[BLINKING_EYES_NUM_FRAMES];

extern const uint16_t blinking_eyes_durations[BLINKING_EYES_NUM_FRAMES];
extern OLEDAnimation blinking_eyes_anim;

//CATCH AND RUN LAYOUT
extern OLEDLayout catch_and_run_animation_layout;

//ENFORCER NEARBY TEXT
extern const unsigned char* enforcer_nearby_text_frames[1];

extern const uint16_t enforcer_nearby_text_durations[1];
extern OLEDAnimation enforcer_nearby_text_anim;

//ENFORCER NEARBY LAYOUT
extern OLEDLayout enforcer_nearby_layout;

//CATCH AND RUN GAME OVER
extern const unsigned char* catch_and_run_game_over_frames[2];

extern const uint16_t catch_and_run_game_over_durations[2];
extern OLEDAnimation catch_and_run_game_over_anim;
extern OLEDLayout catch_and_run_game_over_layout;

//ESCAPED
extern const unsigned char* escaped_frames[1];

extern const uint16_t escaped_durations[1];
extern OLEDAnimation escaped_anim;
extern OLEDLayout escaped_layout;

//DANGEROUS ENVIRONMENT WARNING
extern const unsigned char* dangerous_environment_frames[1];

extern const uint16_t dangerous_environment_durations[1];
extern OLEDAnimation dangerous_environment_anim;
extern OLEDLayout dangerous_environment_layout;

//PLAYER WINS TEXT ANIMATION
#define PLAYER_WINS_NUM_FRAMES 50
extern const unsigned char* player_wins_frames[PLAYER_WINS_NUM_FRAMES];

extern const uint16_t player_wins_durations[PLAYER_WINS_NUM_FRAMES];
extern OLEDAnimation player_wins_anim;
extern OLEDLayout player_wins_layout;

#endif
