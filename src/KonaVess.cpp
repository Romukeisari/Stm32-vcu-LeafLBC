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

#include "KonaVess.h"

uint8_t speed = 0;
uint8_t vessAliveTimeout = 0;

KonaVess::KonaVess() { vessAliveTimeout = 0; }

void KonaVess::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x5E3); // VESS alive broadcast
}

void KonaVess::Task100Ms() {
  uint8_t bytes[8];

  /////////////////////////////////////////////////////////////////////////
  // CAN 0x524: Speed frame
  // Bytes 2-3 carry speed as a big-endian 16-bit value (km/h * 256)
  bytes[0] = 0x60;
  bytes[1] = 0x01;

  float kph = Param::GetInt(Param::speed) * Param::GetFloat(Param::GearRatio);
  Param::SetFloat(Param::Veh_Speed, kph);
  speed = ABS(kph * 256); // 16-bit fixed-point, 1 LSB = 1/256 km/h

  bytes[2] = static_cast<uint8_t>(speed >> 8);
  bytes[3] = static_cast<uint8_t>(speed & 0xFF);
  bytes[4] = 0x5A;
  bytes[5] = 0x01;
  bytes[6] = 0xC0;
  bytes[7] = 0x02;

  can->Send(0x524, (uint32_t *)bytes, 8);

  /////////////////////////////////////////////////////////////////////////
  // CAN 0x200: Gear/direction frame
  // Byte 1: Current Gear bits 5-3
  // P 000
  // D 101
  // N 110
  // R 111
  // dirs "-1=Reverse, 0=Neutral, 1=Drive, 2=Park"

  bytes[0] = 0x00;

  // update shitPos over CAN
  int selectedDir = Param::GetInt(Param::dir);

  if (selectedDir == 0) {
    // neutral
    bytes[1] = 0x30;
  } else if (selectedDir == -1) {
    // reverse
    bytes[1] = 0x35;
  } else if (selectedDir == 1) {
    // drive
    bytes[1] = 0x28;
  } else if (selectedDir == 2) {
    // park
    bytes[1] = 0x00;
  }

  bytes[2] = 0x00;
  bytes[3] = 0x10;
  bytes[4] = 0x00;
  bytes[5] = 0x3B;
  bytes[6] = 0xD0;
  bytes[7] = 0x00;

  can->Send(0x200, (uint32_t *)bytes, 8);

  if (vessAliveTimeout > 0) {
    vessAliveTimeout--;
    Param::SetInt(Param::VessAlive, 1);
  } else {
    Param::SetInt(Param::VessAlive, 0);
  }
}

void KonaVess::DecodeCAN(int id, uint32_t data[2]) {
  switch (id) {
  case 0x5E3:
    KonaVess::handlealive(data);
    break;
  }
}

void KonaVess::handlealive(uint32_t data[2]) {
  (void)data; // We don't use data, so let's tell the compiler that this is not
              // used
  vessAliveTimeout = 10; // 10 × 100ms = 1 second window
}
