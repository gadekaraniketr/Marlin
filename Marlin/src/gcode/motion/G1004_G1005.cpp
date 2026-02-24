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

float get_spiral_current_x_center(const int col, const float col_width, const float col_spacing)
{
  if (col == 0)
  {
    return col_width * 0.5f;
  }
  else
  {
    return (col * col_width) + (col * col_spacing) + (col_width * 0.5f);
  }
}

float get_spiral_current_y_center(const int row, const float row_height, const float row_spacing)
{
  if (row == 0)
  {
    return row_height * 0.5f;
  }
  else
  {
    return (row * row_height) + (row * row_spacing) + (row_height * 0.5f);
  }
}

float get_spiral_next_x_center(int col, const int max_cols, const float col_width, const float col_spacing)
{
  col += 1; // next column
  
  if (col >= max_cols)
  {
    return 0.0f; // will be reset to 0 for next row
  }
  else
  {
    return (col * col_width) + (col * col_spacing) + (col_width * 0.5f);
  }
}

float get_spiral_next_y_center(int row, const int max_rows, const float row_height, const float row_spacing)
{
  row += 1; // next row
  
  if (row >= max_rows)
  {
    return 0.0f; // will be reset to 0 for next column
  }
  else
  {
    return (row * row_height) + (row * row_spacing) + (row_height * 0.5f);
  }
}

void GcodeSuite::G1004_G1005(const bool clockwise) {
  if (!MOTION_CONDITIONS) return;
  
  // 1. Parse Parameters
  const int r_init        = parser.intval('R', 5);
  const float turns       = parser.floatval('P', 3.0f);
  const int gap_size      = parser.intval('S', 5);
  const int feedrate      = parser.intval('F', 3000);
  const int start_delay   = parser.intval('D', 0);
  const int end_delay     = parser.intval('C', 0);
  const int hole_diameter = parser.intval('H', 0);
  const int horizontal_spacing  = parser.intval('X', 10);
  const int vertical_spacing    = parser.intval('Y', 10);

  const int step_size = hole_diameter + gap_size;
  const float spiral_end_radius = r_init + (round(turns) * step_size);

  char cmd[64];

  // 2. bed size is 400x270mm
  // we can devide bed into a grid of cells depending on the spacing between spirals and the size of the spiral

  const float cell_width  = spiral_end_radius * 2.0f;
  const float cell_height = spiral_end_radius * 2.0f;

  const int cols = min(4, int(X_BED_SIZE / cell_width));
  const int rows = min(3, int(Y_BED_SIZE / cell_height));

  // 3. for each cell in the grid
  // calculate center position for each cell

  SERIAL_ECHO_MSG("STEP SIZE: ", step_size);
  SERIAL_ECHO_MSG("SPIRAL END RADIUS: ", spiral_end_radius);
  SERIAL_ECHO_MSG("CELL WIDTH: ", cell_width);
  SERIAL_ECHO_MSG("CELL HEIGHT: ", cell_height);
  SERIAL_ECHO_MSG("COLS: ", cols);
  SERIAL_ECHO_MSG("ROWS: ", rows);

  for (int row = 0; row < rows; row++)
  {
    bool row_even = (row % 2) == 0;
    for (int col = (row_even) ? 0 : cols - 1;
                   (row_even) ? (col < cols) : (col >= 0);
                   (row_even) ? col++ : col--)
    {
      float current_x = get_spiral_current_x_center(col, cell_width, horizontal_spacing);
      float current_y = get_spiral_current_y_center(row, cell_height, vertical_spacing);

      sprintf_P(cmd, PSTR("%s X%f Y%f R%d P%.2f S%d F%d D%d C%d H%d"), 
      (clockwise ? "G1002" : "G1003"), current_x, current_y, r_init, turns, gap_size, feedrate, start_delay, end_delay, hole_diameter);
      queue.enqueue_one(cmd);
    }
  }

  // Move to 0
  sprintf_P(cmd, PSTR("G1 X0.00 Y0.00 F%d"), feedrate);
  queue.enqueue_one(cmd);
}

#endif // ARC_SUPPORT
