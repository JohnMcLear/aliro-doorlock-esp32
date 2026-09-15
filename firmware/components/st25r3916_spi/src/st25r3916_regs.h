/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ST25R3916 / ST25R3916B register map, direct commands and bit fields used
 * by this driver. Values are taken from ST's RFAL (st25r3916_com.h,
 * st25r3916.h) and cross-checked against the Yondoor ESPHome driver that
 * was verified on the STEVAL-MB17149B board.
 *
 * Registers with ST25R3916_SPACE_B set live in register space B and must be
 * reached through the 0xFB space-B access prefix on SPI.
 */

#pragma once

#include <cstdint>

namespace st25r3916 {

constexpr uint8_t kSpaceB = 0x40;

// Space A
constexpr uint8_t kRegIoConf1 = 0x00;
constexpr uint8_t kRegIoConf2 = 0x01;
constexpr uint8_t kRegOpControl = 0x02;
constexpr uint8_t kRegMode = 0x03;
constexpr uint8_t kRegBitRate = 0x04;
constexpr uint8_t kRegIso14443aNfc = 0x05;
constexpr uint8_t kRegPassiveTarget = 0x08;
constexpr uint8_t kRegAux = 0x0A;
constexpr uint8_t kRegRxConf1 = 0x0B;
constexpr uint8_t kRegRxConf2 = 0x0C;
constexpr uint8_t kRegRxConf3 = 0x0D;
constexpr uint8_t kRegRxConf4 = 0x0E;
constexpr uint8_t kRegMaskRxTimer = 0x0F;
constexpr uint8_t kRegNoResponseTimer1 = 0x10;
constexpr uint8_t kRegNoResponseTimer2 = 0x11;
constexpr uint8_t kRegTimerEmvControl = 0x12;
constexpr uint8_t kRegIrqMaskMain = 0x16;
constexpr uint8_t kRegIrqMaskTimerNfc = 0x17;
constexpr uint8_t kRegIrqMaskErrorWup = 0x18;
constexpr uint8_t kRegIrqMaskTarget = 0x19;
constexpr uint8_t kRegIrqMain = 0x1A;
constexpr uint8_t kRegFifoStatus1 = 0x1E;
constexpr uint8_t kRegFifoStatus2 = 0x1F;
constexpr uint8_t kRegCollisionStatus = 0x20;
constexpr uint8_t kRegNumTxBytes1 = 0x22;
constexpr uint8_t kRegNumTxBytes2 = 0x23;
constexpr uint8_t kRegAdResult = 0x25;
constexpr uint8_t kRegAntTuneA = 0x26;
constexpr uint8_t kRegAntTuneB = 0x27;
constexpr uint8_t kRegTxDriver = 0x28;
constexpr uint8_t kRegPtMod = 0x29;
constexpr uint8_t kRegFieldThresholdActv = 0x2A;
constexpr uint8_t kRegFieldThresholdDeactv = 0x2B;
constexpr uint8_t kRegRegulatorControl = 0x2C;
constexpr uint8_t kRegAuxDisplay = 0x31;
constexpr uint8_t kRegIcIdentity = 0x3F;

// Space B
constexpr uint8_t kRegEmdSupConf = kSpaceB | 0x05;
constexpr uint8_t kRegCorrConf1 = kSpaceB | 0x0C;
constexpr uint8_t kRegCorrConf2 = kSpaceB | 0x0D;
constexpr uint8_t kRegAuxMod = kSpaceB | 0x28;
constexpr uint8_t kRegResAmMod = kSpaceB | 0x2A;
constexpr uint8_t kRegOvershootConf1 = kSpaceB | 0x30;
constexpr uint8_t kRegOvershootConf2 = kSpaceB | 0x31;
constexpr uint8_t kRegUndershootConf1 = kSpaceB | 0x32;
constexpr uint8_t kRegUndershootConf2 = kSpaceB | 0x33;

// SPI operation mode prefixes
constexpr uint8_t kSpiRead = 0x40;
constexpr uint8_t kSpiFifoLoad = 0x80;
constexpr uint8_t kSpiFifoRead = 0x9F;
constexpr uint8_t kSpiSpaceBAccess = 0xFB;

// Direct commands
constexpr uint8_t kCmdSetDefault = 0xC1;
constexpr uint8_t kCmdStop = 0xC2;
constexpr uint8_t kCmdTransmitWithCrc = 0xC4;
constexpr uint8_t kCmdTransmitWithoutCrc = 0xC5;
constexpr uint8_t kCmdTransmitReqa = 0xC6;
constexpr uint8_t kCmdTransmitWupa = 0xC7;
constexpr uint8_t kCmdMeasureAmplitude = 0xD3;
constexpr uint8_t kCmdResetRxGain = 0xD5;
constexpr uint8_t kCmdAdjustRegulators = 0xD6;
constexpr uint8_t kCmdClearFifo = 0xDB;
constexpr uint8_t kCmdMeasureVdd = 0xDF;

// IC identity
constexpr uint8_t kIcTypeMask = 0xF8;
constexpr uint8_t kIcTypeSt25r3916 = 0x28;
constexpr uint8_t kIcTypeSt25r3916b = 0x30;
constexpr uint8_t kIcRevMask = 0x07;

// IO_CONF2
constexpr uint8_t kIoConf2Sup3v = 0x80;
constexpr uint8_t kIoConf2AatEn = 0x20;
constexpr uint8_t kIoConf2MisoPd2 = 0x10;
constexpr uint8_t kIoConf2MisoPd1 = 0x08;
constexpr uint8_t kIoConf2IoDrvLvl = 0x04;

// OP_CONTROL
constexpr uint8_t kOpControlEn = 0x80;
constexpr uint8_t kOpControlRxEn = 0x40;
constexpr uint8_t kOpControlTxEn = 0x08;

// MODE
constexpr uint8_t kModeIso14443a = 0x08;  // om = 0001b, tr_am = OOK

// ISO14443A_NFC
constexpr uint8_t kIso14443aAntcl = 0x01;

// AUX
constexpr uint8_t kAuxNoCrcRx = 0x80;
constexpr uint8_t kAuxDisCorr = 0x04;

// TIMER_EMV_CONTROL
constexpr uint8_t kTimerEmvNrtStep4096 = 0x01;
constexpr uint8_t kTimerEmvNrtEmv = 0x02;
constexpr uint8_t kTimerEmvMrtStep512 = 0x08;

// REGULATOR_CONTROL
constexpr uint8_t kRegulatorControlRegS = 0x80;
constexpr uint8_t kRegulatorControlMpsvMask = 0x07;  // 0 = VDD

// AUX_DISPLAY
constexpr uint8_t kAuxDisplayTxOn = 0x20;
constexpr uint8_t kAuxDisplayOscOk = 0x10;
constexpr uint8_t kAuxDisplayRxOn = 0x08;

// FIFO_STATUS2
constexpr uint8_t kFifoStatus2BMask = 0xC0;
constexpr uint8_t kFifoStatus2Ovr = 0x10;
constexpr uint8_t kFifoStatus2LbMask = 0x0E;
constexpr uint8_t kFifoStatus2NpLb = 0x01;

constexpr uint16_t kFifoDepth = 512;

// 32-bit interrupt view: byte 0 = IRQ_MAIN (0x1A), byte 1 = IRQ_TIMER_NFC
// (0x1B), byte 2 = IRQ_ERROR_WUP (0x1C). Same layout as RFAL.
constexpr uint32_t kIrqOsc = 0x00000080;
constexpr uint32_t kIrqFwl = 0x00000040;
constexpr uint32_t kIrqRxs = 0x00000020;
constexpr uint32_t kIrqRxe = 0x00000010;
constexpr uint32_t kIrqTxe = 0x00000008;
constexpr uint32_t kIrqCol = 0x00000004;
constexpr uint32_t kIrqRxRest = 0x00000002;
constexpr uint32_t kIrqDct = 0x00008000;
constexpr uint32_t kIrqNre = 0x00004000;
constexpr uint32_t kIrqGpe = 0x00002000;
constexpr uint32_t kIrqCrc = 0x00800000;
constexpr uint32_t kIrqPar = 0x00400000;
constexpr uint32_t kIrqErr2 = 0x00200000;
constexpr uint32_t kIrqErr1 = 0x00100000;

constexpr uint32_t kIrqErrors = kIrqCrc | kIrqPar | kIrqErr2 | kIrqErr1;

} // namespace st25r3916
