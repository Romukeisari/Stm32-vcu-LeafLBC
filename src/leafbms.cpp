/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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

#include "leafbms.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"

#define CRCKEY 0x185

int LeafBMS::bmsGrp = 2;
int LeafBMS::bmsGrpIndex = -1;
uint8_t LeafBMS::voltBytes[NUMCELLS * 2];
uint8_t LeafBMS::statusBits[NUMCELLS / 4];
int32_t LeafBMS::Amperes;
int32_t LeafBMS::SOC;
int32_t LeafBMS::KW;
int32_t LeafBMS::KWh;
float LeafBMS::Voltage=0;
float LeafBMS::Voltage2=0;
int32_t LeafBMS::Temperature;

void LeafBMS::RegisterCanMessages(CanHardware* can)
{
   can->RegisterUserMessage(0x1DB);//Leaf BMS message 10ms
   can->RegisterUserMessage(0x1DC);//Leaf BMS message 10ms
   can->RegisterUserMessage(0x55B);//Leaf BMS message 100ms
   can->RegisterUserMessage(0x5BC);//Leaf BMS message 100ms
   can->RegisterUserMessage(0x5C0);//Leaf BMS message 500ms
   can->RegisterUserMessage(0x59E);//Leaf BMS message 500ms
}

void LeafBMS::DecodeCAN(int id, uint32_t data[2])
{
   static s32fp chgLimFiltered = 0;
   uint8_t* bytes = (uint8_t*)data;

    if (id == 0x1DB)
   {
      s32fp cur = (int16_t)(bytes[0] << 8) + (bytes[1] & 0xE0);
      s32fp udc = ((bytes[2] << 8) + (bytes[3] & 0xC0)) >> 1;
      bool interlock = (bytes[3] & (1 << 3)) >> 3;
      bool full = (bytes[3] & (1 << 4)) >> 4;

      Amperes = cur / 2;
      Voltage2 = udc / 2;
      Param::SetFixed(Param::idc, cur / 2);
      Param::SetFixed(Param::udc2, udc / 2);
      //Param::SetInt(Param::din_bmslock, interlock);
      //Param::SetInt(Param::batfull, full);
   }
   else if (id == 0x1DC)
   {
      s32fp dislimit = ((bytes[0] << 8) + (bytes[1] & 0xC0)) >> 1;
      s32fp chglimit = ((bytes[1] & 0x3F) << 9) + ((bytes[2] & 0xF0) << 1);

      chgLimFiltered = IIRFILTER(chgLimFiltered, chglimit, 5);

      //Param::SetFixed(Param::dislim, dislimit / 4);
      Param::SetFixed(Param::BMS_ChargeLim, chgLimFiltered / 4);
      //lastRecv = time;
   }
   else if (id == 0x55B)
   {
      s32fp soc = ((bytes[0] << 8) + (bytes[1] & 0xC0)) >> 1;
      Param::SetFixed(Param::SOC, soc / 10);
   }
   else if (id == 0x5BC)
   {
      int soh = bytes[4] >> 1;
      int cond = (bytes[6] >> 5) + ((bytes[5] & 0x3) << 3);
      int limres = bytes[5] >> 5;

      //Param::SetInt(Param::limreason, limres);

      //Only acquire quick charge remaining time
      if (cond == 0)
      {
         int time = bytes[7] + ((bytes[6] & 0x1F) << 8);

         //Param::SetInt(Param::chgtime, time);
      }

      //Param::SetInt(Param::soh, soh);
   }
   else if (id == 0x5C0)
   {
      int dtc = bytes[7];

      //Param::SetInt(Param::lbcdtc, dtc);

      if ((bytes[0] >> 6) == 1) //maximum
      {
         int tmpbat = bytes[2] >> 1;
         tmpbat -= 40;
         Param::SetInt(Param::tmpaux, tmpbat);
         Temperature = tmpbat;
      }
   }
}
