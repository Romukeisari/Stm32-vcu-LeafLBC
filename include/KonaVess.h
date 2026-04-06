/*
 * This file is part of the ZombieVerter project.
 *
 * Based on original work by Eric Reuter
 * Copyright (C) 2025-  Johannes Niinikoski <johannes.niinikoski@iki.fi>
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KONAVESS_H
#define KONAVESS_H

#include <vess.h>
// #include <stdint.h>
// #include "canhardware.h"

class KonaVess : public Vess {
public:
  KonaVess();
  void SetCanInterface(CanHardware *c);
  void DecodeCAN(int id, uint32_t data[2]);
  void Task100Ms();

private:
  static void handlealive(uint32_t data[2]);
};

#endif // KONAVESS_H
