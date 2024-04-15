/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
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

#include "NissanPDM.h"
#include "my_fp.h"
#include "my_math.h"
#include "stm32_can.h"
#include "params.h"
#include "utils.h"

static uint16_t Vbatt=0;
static uint16_t VbattSP=0;
static uint8_t counter_1db=0;
static uint8_t counter_1dc=0;
static uint8_t counter_1d4=0;
static uint8_t counter_11a_d6=0;
static uint8_t counter_1f2=0;
static uint8_t counter_55b=0;
static uint8_t OBCpwrSP=0;
static uint8_t OBCpwr=0;
static bool OBCwake = false;
static bool PPStat = false;
static uint8_t OBCVoltStat=0;
static uint8_t PlugStat=0;
static uint16_t calcBMSpwr=0;


/*Info on running Leaf Gen 2,3 PDM
IDs required :
0x1D4 VCM (10ms)
0x1DB LBC (10ms)
0x1DC LBC (10ms)
0x1F2 VCM (10ms)
0x50B VCM (100ms)
0x55B LBC (100ms)
0x59E LBC (500ms)
0x5BC LBC (100ms)

PDM sends:
0x390 (100ms)
0x393 (100ms)
0x679 on evse plug insert

For QC:

From PDM to QC

PDM EV CAN --------to-------QC CAN
0x3b9                       0x100
0x3bb                       0x101
0x3bc                       0x102
0x3be                       0x200
0x4ba                       0x110
0x4bb                       0x201
0x4bc                       0x700
0x4c0                       0x202

From QC to PDM

QC CAN----------------------PDM EV CAN
0x108                       0x3c8
0x109                       0x3c9
0x208                       0x3cd
0x209                       0x4be


*/

void NissanPDM::SetCanInterface(CanHardware* c)
{
   can = c;
   can->RegisterUserMessage(0x679);//Leaf obc msg
   can->RegisterUserMessage(0x390);//Leaf obc msg
}

void NissanPDM::DecodeCAN(int id, uint32_t data[2])
{
   uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)

   if (id == 0x679)// THIS MSG FIRES ONCE ON CHARGE PLUG INSERT
   {
      uint8_t dummyVar = bytes[0];
      dummyVar = dummyVar;
      OBCwake = true;             //0x679 is received once when we plug in if pdm is asleep so wake wakey...
   }

   else if (id == 0x390)// THIS MSG FROM PDM
   {
      OBCVoltStat = (bytes[3] >> 3) & 0x03;
      PlugStat = bytes[5] & 0x0F;
      if(PlugStat == 0x08) PPStat = true; //plug inserted
      if(PlugStat == 0x00) PPStat = false; //plug not inserted
      Param::SetInt(Param::PlugDet,PPStat);
   }
}

bool NissanPDM::ControlCharge(bool RunCh, bool ACReq)
{
   bool dummy=RunCh;
   dummy=dummy;

   int opmode = Param::GetInt(Param::opmode);
   if(opmode != MOD_CHARGE)
   {
      if(ACReq && OBCwake)
      {
         //OBCwake = false;//reset obc wake for next time
         return true;
      }
   }

   if(PPStat && ACReq) return true;
   if(!PPStat || !ACReq)
   {
      OBCwake = false;
      return false;
   }
   return false;
}


void NissanPDM::Task10Ms()
{
   int opmode = Param::GetInt(Param::opmode);
   int SHUNT = Param::GetInt(Param::Type);

   uint8_t bytes[8];

if (opmode == MOD_CHARGE) //ONLY send when charging else sent by leafinv.cpp
    {
         /////////////////////////////////////////////////////////////////////////////////////////////////
   // CAN Message 0x1D4: Target Motor Torque

   // Data taken from a gen1 inFrame where the car is starting to
   // move at about 10% throttle: F70700E0C74430D4

   // Usually F7, but can have values between 9A...F7 (gen1)
   bytes[0] = 0xF7;
   // 2016: 6E
   // outFrame.data.bytes[0] = 0x6E;

   // Usually 07, but can have values between 07...70 (gen1)
   bytes[1] = 0x07;
   // 2016: 6E
   //outFrame.data.bytes[1] = 0x6E;

   // override any torque commands if not in run mode.
  /* if (opmode != MOD_RUN)
   {
      final_torque_request = 0;
   }

   // Requested torque (signed 12-bit value + always 0x0 in low nibble)
   if(final_torque_request >= -2048 && final_torque_request <= 2047)
   {
      bytes[2] = ((final_torque_request < 0) ? 0x80 : 0) |((final_torque_request >> 4) & 0x7f);
      bytes[3] = (final_torque_request << 4) & 0xf0;
   }
   else
   {
      bytes[2] = 0x00;
      bytes[3] = 0x00;
   }*/

      bytes[2] = 0x00; //Because we're in charge mode, torque request is ALWAYS 0
      bytes[3] = 0x00; //Because we're in charge mode, torque request is ALWAYS 0
   // MSB nibble: Runs through the sequence 0, 4, 8, C
   // LSB nibble: Precharge report (precedes actual precharge
   //             control)
   //   0: Discharging (5%)
   //   2: Precharge not started (1.4%)
   //   3: Precharging (0.4%)
   //   5: Starting discharge (3x10ms) (2.0%)
   //   7: Precharged (93%)
   bytes[4] = 0x07 | (counter_1d4 << 6);
   //bytes[4] = 0x02 | (counter_1d4 << 6);
   //Bit 2 is HV status. 0x00 No HV, 0x01 HV On.

   counter_1d4++;
   if(counter_1d4 >= 4) counter_1d4 = 0;

   // MSB nibble:
   //   0: 35-40ms at startup when gear is 0, then at shutdown 40ms
   //      after the car has been shut off (6% total)
   //   4: Otherwise (94%)
   // LSB nibble:
   //   0: ~100ms when changing gear, along with 11A D0 b3:0 value
   //      D (0.3%)
   //   2: Reverse gear related (13%)
   //   4: Forward gear related (21%)
   //   6: Occurs always when gear 11A D0 is 01 or 11 (66%)
   //outFrame.data.bytes[5] = 0x44;
   //outFrame.data.bytes[5] = 0x46;

   // 2016 drive cycle: 06, 46, precharge, 44, drive, 46, discharge, 06
   // 0x46 requires ~25 torque to start
   //outFrame.data.bytes[5] = 0x46;
   // 0x44 requires ~8 torque to start
   bytes[5] = 0x44;
   //bit 6 is Main contactor status. 0x00 Not on, 0x01 on.

   // MSB nibble:
   //   In a drive cycle, this slowly changes between values (gen1):
   //     leaf_on_off.txt:
   //       5 7 3 2 0 1 3 7
   //     leaf_on_rev_off.txt:
   //       5 7 3 2 0 6
   //     leaf_on_Dx3.txt:
   //       5 7 3 2 0 2 3 2 0 2 3 2 0 2 3 7
   //     leaf_on_stat_DRDRDR.txt:
   //       0 1 3 7
   //     leaf_on_Driveincircle_off.txt:
   //       5 3 2 0 8 B 3 2 0 8 A B 3 2 0 8 A B A 8 0 2 3 7
   //     leaf_on_wotind_off.txt:
   //       3 2 0 8 A B 3 7
   //     leaf_on_wotinr_off.txt:
   //       5 7 3 2 0 8 A B 3 7
   //     leaf_ac_charge.txt:
   //       4 6 E 6
   //   Possibly some kind of control flags, try to figure out
   //   using:
   //     grep 000001D4 leaf_on_wotind_off.txt | cut -d' ' -f10 | uniq | ~/projects/leaf_tools/util/hex_to_ascii_binary.py
   //   2016:
   //     Has different values!
   // LSB nibble:
   //   0: Always (gen1)
   //   1:  (2016)

   // 2016 drive cycle:
   //   E0: to 0.15s
   //   E1: 2 messages
   //   61: to 2.06s (inverter is powered up and precharge
   //                 starts and completes during this)
   //   21: to 13.9s
   //   01: to 17.9s
   //   81: to 19.5s
   //   A1: to 26.8s
   //   21: to 31.0s
   //   01: to 33.9s
   //   81: to 48.8s
   //   A1: to 53.0s
   //   21: to 55.5s
   //   61: 2 messages
   //   60: to 55.9s
   //   E0: to end of capture (discharge starts during this)

   // This value has been chosen at the end of the hardest
   // acceleration in the wide-open-throttle pull, with full-ish
   // torque still being requested, in
   //   LeafLogs/leaf_on_wotind_off.txt
   //outFrame.data.bytes[6] = 0x00;

   // This value has been chosen for being seen most of the time
   // when, and before, applying throttle in the wide-open-throttle
   // pull, in
   //   LeafLogs/leaf_on_wotind_off.txt

   //if (opmode != MOD_CHARGE)  bytes[6] = 0x30;    //brake applied heavilly.
   //if (opmode == MOD_CHARGE)  bytes[6] = 0xE0;   //charging mode
    bytes[6] = 0xE0;   //charging mode
   //In Gen 2 byte 6 is Charge status.
   //0x8C Charging interrupted
   //0xE0 Charging

   // Value chosen from a 2016 log
   //outFrame.data.bytes[6] = 0x61;

   // Value chosen from a 2016 log
   // 2016-24kWh-ev-on-drive-park-off.pcap #12101 / 15.63s
   // outFrame.data.bytes[6] = 0x01;
   //byte 6 brake signal

   // Extra CRC
   nissan_crc(bytes, 0x85);

   can->Send(0x1D4, (uint32_t*)bytes, 8);//send on can1
}


    if(SHUNT!= 3) //If we run leaf BMS, no need to send these
    {
   /////////////////////////////////////////////////////////////////////////////////////////////////
   // CAN Message 0x1DB

   //We need to send 0x1db here with voltage measured by inverter
   //Zero seems to work also on my gen1
   ////////////////////////////////////////////////////////////////
   //Byte 1 bits 8-10 LB Failsafe Status
   //0x00 Normal start req. seems to stay on this value most of the time
   //0x01 Normal stop req
   //0x02 Charge stop req
   //0x03 Charge and normal stop req. Other values call for a caution lamp which we don't need
   //bits 11-12 LB relay cut req
   //0x00 no req
   //0x01,0x02,0x03 main relay off req
   s16fp TMP_battI = (Param::Get(Param::idc))*2;
   s16fp TMP_battV = (Param::Get(Param::udc))*4;
   bytes[0] = TMP_battI >> 8;     //MSB current. 11 bit signed MSBit first
   bytes[1] = TMP_battI & 0xE0;  //LSB current bits 7-5. Dont need to mess with bits 0-4 for now as 0 works.
   bytes[2] = TMP_battV >> 8;
   bytes[3] = ((TMP_battV & 0xC0) | (0x2b)); //0x2b should give no cut req, main rly on permission,normal p limit.
   bytes[4] = 0x40;  //SOC for dash in Leaf. fixed val.
   bytes[5] = 0x00;
   bytes[6] = counter_1db;

   // Extra CRC in byte 7
   nissan_crc(bytes, 0x85);


   counter_1db++;
   if(counter_1db >= 4) counter_1db = 0;

   can->Send(0x1DB, (uint32_t*)bytes, 8);

    }
   /////////////////////////////////////////////////////////////////////////////////////////////////
   // CAN Message 0x50B

   // Statistics from 2016 capture:
   //     10 00000000000000
   //     21 000002c0000000
   //    122 000000c0000000
   //    513 000006c0000000

   // Let's just send the most common one all the time
   // FIXME: This is a very sloppy implementation. Thanks. I try:)
   bytes[0] = 0x00;
   bytes[1] = 0x00;
   bytes[2] = 0x06;
   bytes[3] = 0xc0;
   bytes[4] = 0x00;
   bytes[5] = 0x00;
   bytes[6] = 0x00;

   //possible problem here as 0x50B is DLC 7....
   can->Send(0x50B, (uint32_t*)bytes, 7);

    if(SHUNT != 3) //If we run leaf BMS, no need to send these
    {
   /////////////////////////////////////////////////////////////////////////////////////////////////
   // CAN Message 0x1DC:

   // 0x1dc from lbc. Contains chg power lims and disch power lims.
   // Disch power lim in byte 0 and byte 1 bits 6-7. Just set to max for now.
   // Max charging power in bits 13-20. 10 bit unsigned scale 0.25.Byte 1 limit in kw.
   bytes[0]=0x6E;
   bytes[1]=0x0A;
   bytes[2]=0x05;
   bytes[3]=0xD5;
   bytes[4]=0x00;//may not need pairing code crap here...and we don't:)
   bytes[5]=0x00;
   bytes[6]=counter_1dc;
   // Extra CRC in byte 7
   nissan_crc(bytes, 0x85);

   counter_1dc++;
   if (counter_1dc >= 4)
      counter_1dc = 0;

   can->Send(0x1DC, (uint32_t*)bytes, 8);
    }
   /////////////////////////////////////////////////////////////////////////////////////////////////
   // CAN Message 0x1F2: Charge Power and DC/DC Converter Control

   // convert power setpoint to PDM format:
   //    0x70 = 3 amps ish
   //    0x6a = 1.4A
   //    0x66 = 0.5A
   //    0x65 = 0.3A
   //    0x64 = no chg
   //    so 0x64=100. 0xA0=160. so 60 decimal steps. 1 step=100W???
   /////////////////////////////////////////////////////////////////////////////////////////////////////


   // get actual voltage and voltage setpoints
   Vbatt = Param::GetInt(Param::udc);
   VbattSP = Param::GetInt(Param::Voltspnt);
   calcBMSpwr=(Vbatt * Param::GetInt(Param::BMS_ChargeLim));//BMS charge current limit but needs to be power for most AC charger types.


   OBCpwrSP = (MIN(Param::GetInt(Param::Pwrspnt),calcBMSpwr) / 100) + 0x64;

   if (opmode == MOD_CHARGE && Param::GetInt(Param::Chgctrl) == ChargeControl::Enable)
   {
      // clamp min and max values
      if (OBCpwrSP > 0xA0)
         OBCpwrSP = 0xA0;
      else if (OBCpwrSP < 0x64)
         OBCpwrSP = 0x64;

      // if measured vbatt is less than setpoint got to max power from web ui
      if (Vbatt < VbattSP)
         OBCpwr = OBCpwrSP;

      // decrement charger power if volt setpoint is reached
      if (Vbatt >= VbattSP)
         OBCpwr--;
   }
   else
   {
      // set power to 0 if charge control is set to off or not in charge mode
      OBCpwr = 0x64;
   }


   // Commanded chg power in byte 1 and byte 0 bits 0-1. 10 bit number.
   // byte 1=0x64 and byte 0=0x00 at 0 power.
   // 0x00 chg 0ff dcdc on.
   bytes[0] = 0x30;  // msg is muxed but pdm doesn't seem to care.
   bytes[1] = OBCpwr;
   bytes[2] = 0x20;//0x20=Normal Charge
   bytes[3] = 0xAC;
   bytes[4] = 0x00;
   bytes[5] = 0x3C;
   bytes[6] = counter_1f2;
   bytes[7] = 0x8F;  //may not need checksum here?

   counter_1f2++;
   if (counter_1f2 >= 4)
   {
      counter_1f2 = 0;
   }

   can->Send(0x1F2, (uint32_t*)bytes, 8);
}

void NissanPDM::Task100Ms()
{
	int SHUNT = Param::GetInt(Param::Type);
    if(SHUNT != 3) //If we run leaf BMS, no need to send these
    {
   // MSGS for charging with pdm
   uint8_t bytes[8];

   /////////////////////////////////////////////////////////////////////////////////////////////////
   // CAN Message 0x55B:

   bytes[0] = 0xA4;
   bytes[1] = 0x40;
   bytes[2] = 0xAA;
   bytes[3] = 0x00;
   bytes[4] = 0xDF;
   bytes[5] = 0xC0;
   bytes[6] = ((0x1 << 4) | (counter_55b));
   // Extra CRC in byte 7
   nissan_crc(bytes, 0x85);

   counter_55b++;
   if(counter_55b >= 4) counter_55b = 0;

   can->Send(0x55b, (uint32_t*)bytes, 8);

   /////////////////////////////////////////////////////////////////////////////////////////////////
   // CAN Message 0x59E:

   bytes[0] = 0x00;//Static msg works fine here
   bytes[1] = 0x00;//Batt capacity for chg and qc.
   bytes[2] = 0x0c;
   bytes[3] = 0x76;
   bytes[4] = 0x18;
   bytes[5] = 0x00;
   bytes[6] = 0x00;
   bytes[7] = 0x00;

   can->Send(0x59e, (uint32_t*)bytes, 8);

   /////////////////////////////////////////////////////////////////////////////////////////////////
   // CAN Message 0x5BC:

   // muxed msg with info for gids etc. Will try static for a test.
   bytes[0] = 0x3D;//Static msg works fine here
   bytes[1] = 0x80;
   bytes[2] = 0xF0;
   bytes[3] = 0x64;
   bytes[4] = 0xB0;
   bytes[5] = 0x01;
   bytes[6] = 0x00;
   bytes[7] = 0x32;

   can->Send(0x5bc, (uint32_t*)bytes, 8);
}
}

int8_t NissanPDM::fahrenheit_to_celsius(uint16_t fahrenheit)
{
   int16_t result = ((int16_t)fahrenheit - 32) * 5 / 9;
   if(result < -128)
      return -128;
   if(result > 127)
      return 127;
   return result;
}



void NissanPDM::nissan_crc(uint8_t *data, uint8_t polynomial)
{
   // We want to process 8 bytes with the 8th byte being zero
   data[7] = 0;
   uint8_t crc = 0;
   for(int b=0; b<8; b++)
   {
      for(int i=7; i>=0; i--)
      {
         uint8_t bit = ((data[b] &(1 << i)) > 0) ? 1 : 0;
         if(crc >= 0x80)
            crc = (uint8_t)(((crc << 1) + bit) ^ polynomial);
         else
            crc = (uint8_t)((crc << 1) + bit);
      }
   }
   data[7] = crc;
}

