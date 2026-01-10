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
 * G1004: Clockwise Spiral on Table
 * G1005: Counterclockwise Spiral on Table
 * - Command uses G1002 or G1003 commands internally and enqueued.
 * 
 * Parameters:
 * - R specifies the initial radius (default: 5mm)
 * - P specifies the number of full circles to do (default: 1)
 * - S specifies the spacing between each revolution (default: 1mm)
 * - D specifies start delay in milliseconds (default: 0ms)
 * - C specifies end/cut delay in milliseconds (default: 0ms)
 * - F specifies feedrate in mm/min (default: 3000mm/min)
 * - H specifies nozzle hole diameter mm (default: 0mm)
 * Example:
 * 1. G1004 R10 P3 S5 D0 C0 F3000 H4 X10 Y10
  *    - Clockwise spiral on table with initial radius 10mm, 5mm spacing doing 3 full circles, no start/end delay, feedrate 3000mm/min, nozzle hole diameter 4mm, horizontal spacing 10mm, vertical spacing 10mm
 * 2. G1005 R10 P3 S5 D2 C2 F3000 H4
 * 
 */
void GcodeSuite::G1004_G1005(const bool clockwise) {
  if (!MOTION_CONDITIONS) return;
  
  // 1. Parse Parameters
  const int r_init        = parser.intval('R', 5);
  const int turns         = parser.intval('P', 3);
  const int gap_size      = parser.intval('S', 5);
  const int start_delay   = parser.intval('D', 0);
  const int end_delay     = parser.intval('C', 0);
  const int feedrate      = parser.intval('F', 3000);
  const int hole_diameter = parser.intval('H', 0);
  const int horizontal_spacing  = parser.intval('X', 10);
  const int vertical_spacing    = parser.intval('Y', 10);

  // R+Dia+Gap

  int step_size = hole_diameter + gap_size;

  char cmd[64];

  // 2. bed size is 400x270mm
  // we can devide bed into a grid of 3x2 cells
  // each cell is approximately 133x135mm
  // check if a spiral can fit into the cell

  const int cols = 3;
  const int rows = 2;

  float cell_width  = 120.0f; // X_BED_SIZE / cols;
  float cell_height = 120.0f; // Y_BED_SIZE / rows;

  const int spiral_radius = r_init + (turns * step_size) + max(horizontal_spacing, vertical_spacing);
  if (spiral_radius > (cell_width/2) || spiral_radius > (cell_height/2)) {
    SERIAL_ERROR_MSG("Error: Spiral diameter too large to fit in the cell.");
    return;
  }

  // 3. for each cell in the grid
  // calculate center position for each cell

  for (int row = 0; row < rows; row++) {
    bool row_even = (row % 2) == 0;
    for (int col = (row_even) ? 0 : cols - 1;
                   (row_even) ? (col < cols) : (col >= 0);
                   (row_even) ? col++ : col--) {
    
      float target_x = (col * cell_width) + (cell_width / 2) + horizontal_spacing;
      float target_y = (row * cell_height) + (cell_height / 2) + vertical_spacing;

      // Generate the G1002 or G1003 string
      // G1002/G1003 X.. Y.. R.. S.. P.. F..
      sprintf_P(cmd, PSTR("%s X%f Y%f R%d S%d P%d F%d D%d C%d"), 
      (clockwise ? "G1002" : "G1003"), target_x, target_y, r_init, step_size, turns, feedrate, start_delay, end_delay);
      queue.enqueue_one(cmd);
    }
  }
}

#endif // ARC_SUPPORT
