# Tangram Utils

Contains all library functions for calculating geometry of the tangram pieces

The tangram includes 7 pieces:
 1. 2 X Large Triangle
 2. 1 X Medium Triangle
 3. 2 X Small Triangle
 4. 1 X Square
 5. 1 X Parallelogram

This library contains the following functions
 - `draw_axis`: Draw the principle axis on the image
 - `validate_pieces`: Validate if has correct shapes
 - `get_ccw_angle`: Get the angle between two vector in CCW direction
 - `is_triangle_flipped`: Determine if the triangle if flipped
 - `find_center`: Find the center of a contour
 - `find_closest_tangram_piece`: Find the index of the closest matching tangram piece
 - `closest_tangram_piece_name`: Get the name of the closest matching tangram piece
 - `closest_tangram_piece_shape`: Get the shape of the closest matching tangram piece
 - `get_tangram_piece_orientation`: Get the angle of rotation for a tangram piece