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
 * G1002: Clockwise Spiral
 * G1003: Counterclockwise Spiral
 * - Command uses G2 or G3 commands internally and enqueued.
 * 
 * Parameters:
 * - X, Y specifies the target position
 * - P specifies the number of full circles to do (default: 1)
 * - S specifies the spacing between each revolution (default: 1mm)
 * - P specifies the number of full circles to do (default: 1)
 * - F specifies feedrate in mm/min (default: 3000mm/min)
 * - D specifies start delay in milliseconds (default: 0ms)
 * - C specifies end/cut delay in milliseconds (default: 0ms)
 * 
 * Example:
 * 1. G1002 X100 Y100 S20 P2
 *    - Clockwise spiral to (100,100) with 20mm spacing doing 2 full circles
 * 2. G1003 X100 Y100 P1
 *    - Counterclockwise spiral centered at (currentX+50, currentY+0) with default spacing doing 1 full circle
 * 
 */
void GcodeSuite::G1002_G1003(const bool clockwise) {
  if (!MOTION_CONDITIONS) return;
  
  // 1. Parse Parameters
  const float x_center  = parser.linearval('X', 0.0f);
  const float y_center  = parser.linearval('Y', 0.0f);
  const float r_init = parser.linearval('R', 5.0f);
  const float step   = parser.linearval('S', 5.0f);
  const int   turns  = parser.intval('P', 1);
  const int   feedrate = parser.intval('F', 3000);
  const int   start_delay = parser.intval('D', 0);
  const int   end_delay   = parser.intval('C', 0);

  // 2. Initial Positioning (Move to start of first arc)
  char cmd[64];

  // 2.1 Move to start point for lead in
  // Use G0, it is used for rapid moves such as travel moves.
  // Later can be moved to G1004 if needed.
  sprintf_P(cmd, PSTR("G1 X%f Y%f F%d"), x_center, y_center, feedrate);
  queue.enqueue_one(cmd);

  // wiat for any previous moves to complete
  sprintf_P(cmd, PSTR("M400"));
  queue.enqueue_one(cmd);

  // start feeder using D2 & D4 pins
  // d2 is forward
  // d4 is reverse
  // both off = stop
  sprintf_P(cmd, PSTR("M42 P2 S255"));
  queue.enqueue_one(cmd);

  // start delay with motor forward
  sprintf_P(cmd, PSTR("G4 P%d"), start_delay);
  queue.enqueue_one(cmd);

  // 2.2 Lead in as arc to starting point
  // from now onwards all moves are G1 since we are cutting material
  sprintf_P(cmd, PSTR("%s X%f Y%f I%f J0 F%d"),
    (clockwise ? "G2" : "G3"),  // command
    (clockwise ? (x_center - r_init) : (x_center + r_init)),  // target X
    y_center, // target Y
    (clockwise ? (-(r_init * 0.5)) : (r_init * 0.5)),  // offset I
    feedrate);
  queue.enqueue_one(cmd);

  float current_r = r_init;
  float cur_x = (clockwise ? (x_center - r_init) : (x_center + r_init));
  float cur_y = y_center;
  
  float track_x = cur_x;

  // 3. Loop to generate arcs
  for (int i = 0; i < (turns * 2); i++) {

    float diameter = current_r * (clockwise ? 2.0f : -2.0f);
    float target_x, offset_i;

    if (i % 2 == 0) {
      target_x = track_x + diameter;
      offset_i = clockwise ? current_r : -current_r;
    } else {
      target_x = track_x - diameter;
      offset_i = clockwise ? -current_r : current_r;
    }

    // Generate the G2 or G3 string
    // G2 = CW, G3 = CCW
    sprintf_P(cmd, PSTR("%s X%f Y%f I%f J0"), 
      (clockwise ? "G2" : "G3"), target_x, cur_y, offset_i);
    
    queue.enqueue_one(cmd);

    // Update tracking
    track_x = target_x;
    current_r += step;
  }

  // 4. End delay
  // stop feeder
  sprintf_P(cmd, PSTR("M42 P2 S0"));
  queue.enqueue_one(cmd);

  // reverse feeder to reduce back lash
  sprintf_P(cmd, PSTR("M42 P4 S255"));
  queue.enqueue_one(cmd);

  // end delay with motor reverse
  sprintf_P(cmd, PSTR("G4 P%d"), end_delay);
  queue.enqueue_one(cmd);

  // stop reverse feeder
  sprintf_P(cmd, PSTR("M42 P4 S0"));
  queue.enqueue_one(cmd);
}

#endif // ARC_SUPPORT
