/*
  motion_control.h - cartesian robot controller.
  Part of Turntable

  Copyright (c) 2009 Simen Svale Skogsrud

  Turntable is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Turntable is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Turntable.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef nuts_bolts_h
#define nuts_bolts_h

#include <string.h>

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

#define FALSE 0
#define TRUE 1

// Decide the sign of a value
#define signof(a) (((a)>0) ? 1 : (((a)<0) ? -1 : 0))

#define clear_vector(a) memset(a, 0, sizeof(a))

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#endif
