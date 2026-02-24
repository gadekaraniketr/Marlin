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

#define QUAD_ARCS_PER_TURN 4

/**
 * G1002 & G1003 - Spiral / Circle (Clockwise & counter-clockwise)
 * Parameters
 * 1.	X - horizontal center location in mm (float)
 * 2.	Y - vertical center location in mm (float)
 * 3.	R - inner start radius of circle in mm (float), default 5mm
 * 4.	P - number of turns in step of 0.25 (float), default to 4 turns
 * 5.	S - spacing between two parallel arcs in mm (float), default to 1mm
 * 6.	F - feed-rate at which table is moved (int), default to 3000 mm/min
 * 7.	D - start delay in milliseconds (int), default to 0ms
 * 8.	C - end delay in milliseconds (int), default to 700ms
 * 9.	H - hole diameter in mm (float), default to 5mm
 * 
 */
void GcodeSuite::G1002_G1003(const bool clockwise) {
  if (!MOTION_CONDITIONS) return;
  
  // 1. Parse Parameters
  const float center_x    = parser.linearval('X', current_position.x);
  const float center_y    = parser.linearval('Y', current_position.y);
  const float init_r      = parser.linearval('R', 5.0f);
  const float turns       = parser.linearval('P', 4.0f);
  const float spacing     = parser.linearval('S', 1.0f);
  const int   feedrate    = parser.intval('F', 3000);
  const int   start_delay = parser.intval('D', 0);
  const int   end_delay   = parser.intval('C', 700);
  const float hole_dia    = parser.linearval('H', 5.0f);

  char cmd[64];

  const float step_size           = spacing + hole_dia;
  const float step_size_half      = step_size * 0.5f;
  
  // 1. Move to start of lead in arc, center of spiral
  sprintf_P(cmd, PSTR("G1 X%f Y%f F%d"), center_x, center_y, feedrate);
  queue.enqueue_one(cmd);
  
  // 2. Wait for any previous moves to complete
  // if feeder start during movement then keep. if not, remove this command and see if it works.
  sprintf_P(cmd, PSTR("M400"));
  queue.enqueue_one(cmd);
  
  // Feeder control using D2 & D4 pins
  // D2 => F, D4 => R
  // Both off = STOP
  // 3. Start feeder forward
  sprintf_P(cmd, PSTR("M42 P2 S255"));
  queue.enqueue_one(cmd);
  
  // 4. Start delay with feeder forward
  sprintf_P(cmd, PSTR("G4 P%d"), start_delay);
  queue.enqueue_one(cmd);

  // 5. Lead in starting point
  const float lead_in_arc_radius  = init_r * 0.5f;
  sprintf_P(cmd, PSTR("%s X%f Y%f I%f J0 F%d"),
    (clockwise ? "G2" : "G3"),  // command
    (clockwise ? (center_x - init_r) : (center_x + init_r)),  // target X
    center_y, // target Y
    (clockwise ? (-lead_in_arc_radius) : (lead_in_arc_radius)),  // offset I
    feedrate);
  queue.enqueue_one(cmd);

  // 6. Loop to generate arcs for the spiral,
  // Since controller memory is limited, we do TWO 180 degree arcs for 1 full turn
  // reminder arc will do 90 degree turns + final arc for lead out
  float track_radius = init_r;
  float track_x = clockwise ? (center_x - track_radius) : (center_x + track_radius);
  
  for (int i = 0; i < int(turns * 2.0f); i++)
  {
    float track_diameter = track_radius * (clockwise ? 2.0f : -2.0f);
    
    float target_x, offset_i;
    if ((i % 2) == 0) {
      target_x = track_x + track_diameter;
      offset_i = clockwise ? track_radius : -track_radius;
    } else {
      target_x = track_x - track_diameter;
      offset_i = clockwise ? -track_radius : track_radius;
    }
    
    // Generate the G2 or G3 string
    // G2 = CW, G3 = CCW
    sprintf_P(cmd, PSTR("%s X%f Y%f I%f J0"), 
    (clockwise ? "G2" : "G3"), target_x, center_y, offset_i);
    
    queue.enqueue_one(cmd);
    
    // Update tracking
    track_x = target_x;
    track_radius += step_size_half;
  }
  
  // 7. Remaining 90 degree arcs + lead out arc
  const int total_quad_arcs = int(turns * QUAD_ARCS_PER_TURN) + 1;
  const int completed_quad_arcs = int(turns * 2.0f) * 2;

  float track_y = center_y;

  for (int i = completed_quad_arcs; i < total_quad_arcs; i++)
  {
    //  At the end of the last arc, stop the feeder
    if (i > (total_quad_arcs - 2))
    {
      // wait for any previous moves to complete
      sprintf_P(cmd, PSTR("M400"));
      queue.enqueue_one(cmd);

      // stop feeder forward
      sprintf_P(cmd, PSTR("M42 P2 S0"));
      queue.enqueue_one(cmd);

      // start reverse feeder
      sprintf_P(cmd, PSTR("M42 P4 S255"));
      queue.enqueue_one(cmd);
    }
    const int arc_index = i % QUAD_ARCS_PER_TURN;
    float target_x, target_y, offset_i, offset_j;
    if (arc_index == 0) {
      target_x = center_x;
      target_y = track_y + track_radius;
      offset_i = clockwise ? track_radius : -track_radius;
      offset_j = 0;
    }
    else if (arc_index == 1) {
      target_x = center_x + (clockwise ? track_radius : -track_radius);
      target_y = center_y;
      offset_i = 0;
      offset_j = -track_radius;
      track_radius += step_size_half;
    }
    else if (arc_index == 2) {
      target_x = track_x + (clockwise ? -track_radius : track_radius);
      target_y = track_y - track_radius;
      offset_i = clockwise ? -track_radius : track_radius;
      offset_j = 0;
    }
    else if (arc_index == 3) {
      target_x = track_x + (clockwise ? -track_radius : track_radius);
      target_y = center_y;
      offset_i = 0;
      offset_j = track_radius;
    }

    sprintf_P(cmd, PSTR("%s X%f Y%f I%f J%f"), 
      (clockwise ? "G2" : "G3"), target_x, target_y, offset_i, offset_j);
    queue.enqueue_one(cmd);
    
    track_x = target_x;
    track_y = target_y;
  }

  // wait for any previous moves to complete
  sprintf_P(cmd, PSTR("M400"));
  queue.enqueue_one(cmd);

  // stop reverse feeder
  sprintf_P(cmd, PSTR("M42 P4 S0"));
  queue.enqueue_one(cmd);
}

#endif // ARC_SUPPORT
