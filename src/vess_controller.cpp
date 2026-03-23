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

#include "vess_controller.h"
#include "stm32_can.h"

void VESSController::SetCanInterface(CanHardware* c)
{
    can = c;
    can->RegisterUserMessage(0x5E3); // VESS alive broadcast
}

void VESSController::setSpeedKmH(int kmh)
{
    speed = kmh * 256; // 16-bit fixed-point, 1 LSB = 1/256 km/h
}

void VESSController::setReverse(bool rev)
{
    reverse = rev;
}

void VESSController::Task100Ms()
{
    uint8_t bytes[8];

    /////////////////////////////////////////////////////////////////////////
    // CAN 0x524: Speed frame
    // Bytes 2-3 carry speed as a big-endian 16-bit value (km/h * 256)
    bytes[0] = 0x60;
    bytes[1] = 0x01;
    bytes[2] = static_cast<uint8_t>(speed >> 8);
    bytes[3] = static_cast<uint8_t>(speed & 0xFF);
    bytes[4] = 0x5A;
    bytes[5] = 0x01;
    bytes[6] = 0xC0;
    bytes[7] = 0x02;

    can->Send(0x524, (uint32_t*)bytes, 8);

    /////////////////////////////////////////////////////////////////////////
    // CAN 0x200: Gear/direction frame
    // Byte 1: 0b00111000 = reverse, 0b00101000 = forward
    bytes[0] = 0x00;
    bytes[1] = reverse ? 0b00111000 : 0b00101000;
    bytes[2] = 0x00;
    bytes[3] = 0x10;
    bytes[4] = 0x00;
    bytes[5] = 0x3B;
    bytes[6] = 0xD0;
    bytes[7] = 0x00;

    can->Send(0x200, (uint32_t*)bytes, 8);
}
