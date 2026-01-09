/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(ARC_SUPPORT)

#include "../gcode.h"
#include "../queue.h"
#include "../../module/motion.h"
#include "../../module/planner.h"

/**
 * G2000: Clockwise Spiral
 * G3000: Counterclockwise Spiral
 * - Command uses G2 or G3 commands internally and enqueued.
 * 
 * Parameters:
 * - X, Y specifies the target position
 * - P specifies the number of full circles to do (default: 1)
 * - S specifies the spacing between each revolution (default: 1mm)
 * 
 * Example:
 * 1. G2000 X100 Y100 S20 P2
 *    - Clockwise spiral to (100,100) with 20mm spacing doing 2 full circles
 * 2. G3000 I50 J0 P1
 *    - Counterclockwise spiral centered at (currentX+50, currentY+0) with default spacing doing 1 full circle
 * 
 */
void GcodeSuite::G2000_G3000(const bool clockwise) {
  if (!MOTION_CONDITIONS) return;
  
  // 1. Parse Parameters
  const float x_pos  = parser.linearval('X', 0.0f);
  const float y_pos  = parser.linearval('Y', 0.0f);
  const float r_init = parser.linearval('R', 5.0f);
  const float step   = parser.linearval('S', 5.0f);
  const int   turns  = parser.intval('P', 1);
  const int   feedrate = parser.intval('F', 3000);
  
  float current_r = r_init;
  float cur_x = current_position.x;
  float cur_y = current_position.y;

  // 2. Initial Positioning (Move to start of first arc)
  // We move to X - r_init relative to center
  char cmd[64];
  sprintf_P(cmd, PSTR("G0 X%f Y%f F%d"), x_pos, y_pos, feedrate);
  queue.enqueue_one(cmd);
  // sprintf_P(cmd, PSTR("G1 X%f Y%f F%d"), cur_x - r_init, cur_y, feedrate);
  // queue.enqueue_one(cmd);

  // float track_x = cur_x - r_init;

  // 3. Loop to generate arcs
  // for (int i = 0; i < (turns * 2); i++) {
  //     float next_r = current_r + step;
  //     float diameter = current_r + next_r;
      
  //     float target_x, offset_i;

  //     if (i % 2 == 0) {
  //         target_x = track_x + diameter;
  //         offset_i = current_r;
  //     } else {
  //         target_x = track_x - diameter;
  //         offset_i = -current_r;
  //     }

  //     // Generate the G2 or G3 string
  //     // G2 = CW, G3 = CCW
  //     sprintf_P(cmd, PSTR("%s X%f Y%f I%f J0"), 
  //               (clockwise ? "G2" : "G3"), target_x, cur_y, offset_i);
      
  //     queue.enqueue_one(cmd);

  //     // Update tracking
  //     track_x = target_x;
  //     current_r = next_r;
  // }
}

#endif // ARC_SUPPORT
