#include "../../inc/MarlinConfig.h"

#if ENABLED(ARC_SUPPORT)

#include "../gcode.h"
#include "../queue.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../sd/cardreader.h"

/**
 * G1006: Lines on Table
 * 
 * Parameters:
 * - L specifies the length (default: 5mm)
 * - D specifies start delay in milliseconds (default: 0ms)
 * - C specifies end/cut delay in milliseconds (default: 0ms)
 * - F specifies feedrate in mm/min (default: 3000mm/min)
 * - H specifies nozzle hole diameter mm (default: 0mm)
 * - X specifies horizontal spacing
 * - Y specifies vertical spacing
 * Example:
 * 1. G1006 L100 D200 C100 F3000 H4
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

void GcodeSuite::G1006() {
  if (!MOTION_CONDITIONS) return;
  
  // 1. Parse Parameters
  const int length        = parser.intval('L', 5);
  const int start_delay   = parser.intval('D', 0);
  const int end_delay     = parser.intval('C', 0);
  const int feedrate      = parser.intval('F', 3000);
  const int hole_diameter = parser.intval('H', 0);
  const int horizontal_spacing  = parser.intval('X', 10);
  const int vertical_spacing    = parser.intval('Y', 10);

  char cmd[64];

  // 2. bed size is 400x270mm
  // we can devide bed into a grid of cells depending on the spacing between lines and the length of the line
  if (length > max(X_BED_SIZE, Y_BED_SIZE))
    return;

  const int cols = min(6, int(X_BED_SIZE / (length + horizontal_spacing)));
  const int rows = min(4, int(Y_BED_SIZE / (hole_diameter + vertical_spacing)));

  // 3. for each cell in the grid
  // calculate center position for each cell

  // SERIAL_ECHO_MSG("STEP SIZE: ", step_size);
  // SERIAL_ECHO_MSG("SPIRAL END RADIUS: ", spiral_end_radius);
  // SERIAL_ECHO_MSG("CELL WIDTH: ", cell_width);
  // SERIAL_ECHO_MSG("CELL HEIGHT: ", cell_height);
  // SERIAL_ECHO_MSG("COLS: ", cols);
  // SERIAL_ECHO_MSG("ROWS: ", rows);
  
  if (!card.isMounted())
  {
    card.mount();
  }
  card.openFileWrite("/test.gco");

  if (card.isFileOpen())
  {
    SERIAL_ECHOLNPGM("Generating Shape to SD...");

    for (int row = 0; row < rows; row++)
    {
      bool row_even = (row % 2) == 0;
      for (int col = (row_even) ? 0 : cols - 1;
                    (row_even) ? (col < cols) : (col >= 0);
                    (row_even) ? col++ : col--)
      {
        // --------------------------------------------------------------------------------------------------------------------
        const float center_x    = 0; //get_spiral_current_x_center(col, cell_width, horizontal_spacing);
        const float center_y    = 0; //get_spiral_current_y_center(row, cell_height, vertical_spacing);
        const float init_r      = 0; //r_init;
        const float spacing     = 0; //gap_size;
        const float hole_dia    = 0; //hole_diameter;

        const float step_size           = spacing + hole_dia;
        const float step_size_half      = step_size * 0.5f;

        // 1. Move to start of lead in arc, center of spiral
        sprintf_P(cmd, PSTR("G1 X%f Y%f F%d"), center_x, center_y, feedrate);
        // queue.enqueue_one(cmd);
        card.write_command(cmd);
        
        // 2. Wait for any previous moves to complete
        // if feeder start during movement then keep. if not, remove this command and see if it works.
        sprintf_P(cmd, PSTR("M400"));
        // queue.enqueue_one(cmd);
        card.write_command(cmd);
        
        // Feeder control using D2 & D4 pins
        // D2 => F, D4 => R
        // Both off = STOP
        // 3. Start feeder forward
        // #define FEEDER_FWD_PIN                         2
        // #define FEEDER_REV_PIN                         4
        sprintf_P(cmd, PSTR("M42 P%d S255"), FEEDER_FWD_PIN);
        // queue.enqueue_one(cmd);
        card.write_command(cmd);
        
        // 4. Start delay with feeder forward
        sprintf_P(cmd, PSTR("G4 P%d"), start_delay);
        // queue.enqueue_one(cmd);
        card.write_command(cmd);

        // 5. Lead in starting point
        const float lead_in_arc_radius  = init_r * 0.5f;
        sprintf_P(cmd, PSTR("%s X%f Y%f I%f J0 F%d"),
          (clockwise ? "G2" : "G3"),  // command
          (clockwise ? (center_x - init_r) : (center_x + init_r)),  // target X
          center_y, // target Y
          (clockwise ? (-lead_in_arc_radius) : (lead_in_arc_radius)),  // offset I
          feedrate);
        // queue.enqueue_one(cmd);
        card.write_command(cmd);

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
          
          // queue.enqueue_one(cmd);
          card.write_command(cmd);
          
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
              // queue.enqueue_one(cmd);
              card.write_command(cmd);

              // stop feeder forward
              sprintf_P(cmd, PSTR("M42 P%d S0"), FEEDER_FWD_PIN);
              // queue.enqueue_one(cmd);
              card.write_command(cmd);

              // start reverse feeder
              sprintf_P(cmd, PSTR("M42 P%d S255"), FEEDER_REV_PIN);
              // queue.enqueue_one(cmd);
              card.write_command(cmd);
            }
            const int arc_index = i % QUAD_ARCS_PER_TURN;
            float target_x, target_y, offset_i, offset_j = 0.0f;
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
            // queue.enqueue_one(cmd);
            card.write_command(cmd);
            
            track_x = target_x;
            track_y = target_y;
          }

          // wait for any previous moves to complete
          sprintf_P(cmd, PSTR("M400"));
          // queue.enqueue_one(cmd);
          card.write_command(cmd);

          // stop reverse feeder
          sprintf_P(cmd, PSTR("M42 P%d S0"), FEEDER_REV_PIN);
          // queue.enqueue_one(cmd);
          card.write_command(cmd);
        // --------------------------------------------------------------------------------------------------------------------
      }
    }

    // Move to 0
    sprintf_P(cmd, PSTR("G1 Y%d F%d"), Y_BED_SIZE, feedrate);
    card.write_command(cmd);

    sprintf_P(cmd, PSTR("G1 X0 F%d"), feedrate);
    card.write_command(cmd);

    sprintf_P(cmd, PSTR("G1 Y0 F%d"), feedrate);
    card.write_command(cmd);

    card.closefile();
    SERIAL_ECHOLNPGM("Success: SD Card Write Completed.");
  }
  else
  {
    SERIAL_ECHOLNPGM("Error: SD Card Write Failed.");
  }
}

#endif // ARC_SUPPORT
