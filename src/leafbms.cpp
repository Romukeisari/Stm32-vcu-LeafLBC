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
   //can->RegisterUserMessage(0x1DA);//Leaf Inv message 10ms
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
/*
   if (id == 0x7BB)
   {
        if (bmsGrp == 1)
      {         if (bmsGrpIndex == 0)
         {
						// Extracting relevant bytes for HV Bat Current 2 calculation
						Amperes = (bytes[3] << 24) | (bytes[4] << 16) | (bytes[5] << 8) | bytes[6];
						// Checking if the most significant bit is set (indicating a negative value)
						if (Amperes & 0x80000000) {
						// If it is set, perform two's complement conversion and divide by 1024
						Amperes = (Amperes | 0xFF000000);
                                }
								if (Amperes & 0x80000000) {
									Amperes = (Amperes | -0x100000000) / 1024;
								} else {
									Amperes = Amperes / 1024;
								}
         }
         else if (bmsGrpIndex == 1)
         {

         }
         else if (bmsGrpIndex == 2)
         {

         }

      }
      if (bmsGrp == 2)
      {
         if (bmsGrpIndex == 0)
         {
            for (int i = 4; i < 8; i++)
            {
               voltBytes[i - 4] = bytes[i];
            }
         }
         else if (bmsGrpIndex < 28)
         {
            for (int i = 1; i < 8; i++)
            {
               voltBytes[7 * bmsGrpIndex - 4 + i] = bytes[i];
            }
         }
      }
      else if (bmsGrp == 4)
      {
         if (bmsGrpIndex == 0)
         {
            Param::SetInt(Param::tmpbat1, (int)bytes[6]);
         }
         else if (bmsGrpIndex == 1)
         {
            Param::SetInt(Param::tmpbat2, (int)bytes[2]);
         }
         else if (bmsGrpIndex == 2)
         {
            Param::SetInt(Param::tmpbat3, (int)bytes[1]);
         }
      }
      else if (bmsGrp == 6)
      {
         if (bmsGrpIndex == 0)
         {
            for (int i = 0; i < 4; i++)
               statusBits[i] = bytes[4 + i];
         }
         else if (bmsGrpIndex == 1)
         {
            for (int i = 0; i < 7; i++)
               statusBits[4 + i] = bytes[1 + i];
         }
         else if (bmsGrpIndex == 2)
         {
            for (int i = 0; i < 7; i++)
               statusBits[11 + i] = bytes[1 + i];
         }
         else if (bmsGrpIndex == 3)
         {
            for (int i = 0; i < 6; i++)
               statusBits[18 + i] = bytes[1 + i];
         }
      }
   }
   else
    */
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

/*void LeafBMS::RequestNextFrame(CanHardware* can)
{
   uint32_t canData[2] = { 0, 0xffffffff };

      if (bmsGrp == 1)
   {
      if (bmsGrpIndex == -1)
      {
         bmsGrpIndex++;
         canData[0] = 0x2 | 0x21 << 8 | bmsGrp << 16 | 0xff << 24;
         can->Send(0x79B, canData);
      }
      else if (bmsGrpIndex < 6)
      {
         bmsGrpIndex++;
         canData[0] = 0x30 | 0x1 << 8 | 0x0 << 16 | 0xff << 24;
         can->Send(0x79B, canData);
      }
      else
      {
         bmsGrpIndex = -1;
         bmsGrp = 2;
      }
   }
   else if (bmsGrp == 2)
   {
      if (bmsGrpIndex == -1)
      {
         bmsGrpIndex++;
         canData[0] = 0x2 | 0x21 << 8 | bmsGrp << 16 | 0xff << 24;
         can->Send(0x79B, canData);
      }
      else if (bmsGrpIndex < 28)
      {
         bmsGrpIndex++;
         canData[0] = 0x30 | 0x1 << 8 | 0x0 << 16 | 0xff << 24;
         can->Send(0x79B, canData);
      }
      else
      {
         bmsGrpIndex = -1;
         bmsGrp = 4;
         int min = 4500, max = 0, avg = 0;

         for (int i = 0; i < NUMCELLS; i++)
         {
            uint16_t voltage = GetCellVoltage(i);
            avg += voltage;
            min = MIN(min, voltage);
            max = MAX(max, voltage);
         }

         Param::SetInt(Param::batmin, min);
         Param::SetInt(Param::batmax, max);
         Param::SetInt(Param::batavg, avg / NUMCELLS);
      }
   }
   else if (bmsGrp == 4)
   {
      if (bmsGrpIndex == -1)
      {
         bmsGrpIndex++;
         canData[0] = 0x2 | 0x21 << 8 | bmsGrp << 16 | 0xff << 24;
         can->Send(0x79B, canData);
      }
      else if (bmsGrpIndex < 2)
      {
         bmsGrpIndex++;
         canData[0] = 0x30 | 0x1 << 8 | 0x0 << 16 | 0xff << 24;
         can->Send(0x79B, canData);
      }
      else
      {
         bmsGrpIndex = -1;
         bmsGrp = 6;
      }
   }
   else if (bmsGrp == 6)
   {
      if (bmsGrpIndex == -1)
      {
         bmsGrpIndex++;
         canData[0] = 0x2 | 0x21 << 8 | bmsGrp << 16 | 0xff << 24;
         can->Send(0x79B, canData);
      }
      else if (bmsGrpIndex < 4)
      {
         bmsGrpIndex++;
         canData[0] = 0x30 | 0x1 << 8 | 0x0 << 16 | 0xff << 24;
         can->Send(0x79B, canData);
      }
      else
      {
         bmsGrpIndex = -1;
         bmsGrp = 1;
      }
   }
}

uint16_t LeafBMS::GetCellVoltage(int idx)
{
   if (idx < NUMCELLS)
   {
      return (voltBytes[2 * idx] << 8 | voltBytes[2 * idx + 1]) & 0x1FFF;
   }
   return -1;
}

int LeafBMS::GetCellStatus(int idx)
{
   if (idx < NUMCELLS)
   {
      int shift = (idx & 3) * 2;
      return (statusBits[idx >> 2] >> shift) & 0x3;
   }
   return -1;
}
*/
