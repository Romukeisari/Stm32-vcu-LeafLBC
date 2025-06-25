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
    VESSController::SetCanInterface(c);//set Kona VCM messages on same bus as VESS
    can = c;

    can->RegisterUserMessage(0x5E3);//vess alive broadcast
}

void VESSController::setSpeedKmH(int kmh) {
    speed = kmh * 256;
}

void VESSController::setReverse(bool rev) {
    reverse = rev;
}

void VESSController::Task100Ms {

    uint8_t bytes[8];
    int opmode = Param::GetInt(Param::opmode);

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // CAN Message 0x524: Speed data frame

        // Data taken from Eric Reuter's python script
            bytes[0] = 0x60;
            bytes[1] = 0x01;
            bytes[2] = static_cast<uint8_t>(speed >> 8);
            bytes[3] = static_cast<uint8_t>(speed & 0xFF);
            bytes[4] = 0x5A;
            bytes[5] = 0x01;
            bytes[6] = 0xC0;
            bytes[7] = 0x02;

            can->Send(0x5224, (uint32_t*)bytes, 8);//send on can

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // CAN Message 0x200: Speed data frame

        // Data taken from Eric Reuter's python script
            bytes[0] = 0x00;
            bytes[1] = reverse ? 0b00111000 : 0b00101000;
            bytes[2] = 0x00;
            bytes[3] = 0x10;
            bytes[4] = 0x00;
            bytes[5] = 0x3B;
            bytes[6] = 0xD0;
            bytes[7] = 0x00;

            can->Send(0x200, (uint32_t*)bytes, 8);//send on can

}

bool VESSController::sendSpeedMessage() {
    uint8_t data[8] = {
        0x60, 0x01,
        static_cast<uint8_t>(speed >> 8),
        static_cast<uint8_t>(speed & 0xFF),
        0x5A, 0x01, 0xC0, 0x02
    };
    return can->sendMessage(0x524, data, 8);
}

bool VESSController::sendGearMessage() {
    uint8_t data[8] = {
        0x00,
        reverse ? 0b00111000 : 0b00101000,
        0x00, 0x10, 0x00, 0x3B, 0xD0, 0x00
    };
    return can->sendMessage(0x200, data, 8);
}
