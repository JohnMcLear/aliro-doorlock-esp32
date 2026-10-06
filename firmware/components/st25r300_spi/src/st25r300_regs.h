/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ST25R300 register map, the parts this NFC-A poller uses.
 *
 * The ST25R300 is NOT register compatible with the ST25R3916 (ST AN6313):
 * different addresses, a different SPI encoding, three IRQ registers instead
 * of four, a different OPERATION register, and no dedicated REQA/WUPA
 * commands. Values below are taken from the ST25R300 datasheet (DS14655 rev 2)
 * and from John McLear's ESPHome ST25R300 driver
 * (esphome_st25r/components/st25r300), which is verified on hardware for
 * ISO 14443A UID, NDEF and ISO-DEP.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace st25r300 {

// SPI framing (DS14655 table 5): register write = addr & 0x7F, register read =
// addr | 0x80, FIFO load = 0x5F, FIFO read = 0xDF, direct commands are sent as
// a single byte. SPI mode 1 (CPOL 0, CPHA 1), MSB first, CS active low.
constexpr uint8_t kSpiRead = 0x80;
constexpr uint8_t kSpiFifoLoad = 0x5F;
constexpr uint8_t kSpiFifoRead = 0xDF;

constexpr size_t kFifoDepth = 256;

// Registers
constexpr uint8_t kRegOperation = 0x00;
constexpr uint8_t kRegGeneralConf = 0x01;
constexpr uint8_t kRegTxDriver = 0x03;
constexpr uint8_t kRegTxMod1 = 0x04;
constexpr uint8_t kRegRxAnalog1 = 0x09;
constexpr uint8_t kRegRxAnalog2 = 0x0A;
constexpr uint8_t kRegRxAnalog3 = 0x0B;
constexpr uint8_t kRegRxDigital = 0x0D;
constexpr uint8_t kRegCorr1 = 0x0E;
constexpr uint8_t kRegCorr2 = 0x0F;
constexpr uint8_t kRegCorr3 = 0x10;
constexpr uint8_t kRegCorr4 = 0x11;
constexpr uint8_t kRegCorr5 = 0x12;
constexpr uint8_t kRegCorr6 = 0x13;
constexpr uint8_t kRegProtocol1 = 0x14;
constexpr uint8_t kRegTxProtocol1 = 0x15;
constexpr uint8_t kRegRxProtocol1 = 0x17;
constexpr uint8_t kRegNrtConf1 = 0x22;  // nrt_emv (b1), nrt_step (b0)
constexpr uint8_t kRegNrtConf2 = 0x23;  // nrt[15:8]
constexpr uint8_t kRegNrtConf3 = 0x24;  // nrt[7:0]
constexpr uint8_t kRegTxFrame1 = 0x34;  // ntx[12:5]
constexpr uint8_t kRegTxFrame2 = 0x35;  // ntx[4:0] in 7:3, nbtx in 2:0
constexpr uint8_t kRegFifoStatus1 = 0x36;
constexpr uint8_t kRegFifoStatus2 = 0x37;
constexpr uint8_t kRegCollisionDisplay = 0x38;
constexpr uint8_t kRegIrqMask1 = 0x39;
constexpr uint8_t kRegIrqMask2 = 0x3A;
constexpr uint8_t kRegIrqMask3 = 0x3B;
constexpr uint8_t kRegIrqStatus1 = 0x3C;
constexpr uint8_t kRegIrqStatus2 = 0x3D;
constexpr uint8_t kRegIrqStatus3 = 0x3E;
constexpr uint8_t kRegIcIdentity = 0x3F;
constexpr uint8_t kRegStatus1 = 0x40;
constexpr uint8_t kRegStaticStatus3 = 0x44;
constexpr uint8_t kRegSenseRf = 0x4C;

// OPERATION (0x00)
constexpr uint8_t kOpEn = 0x08;
constexpr uint8_t kOpVdddrEn = 0x10;
constexpr uint8_t kOpRxEn = 0x20;
constexpr uint8_t kOpTxEn = 0x40;
constexpr uint8_t kOpAllOn = kOpEn | kOpVdddrEn | kOpRxEn | kOpTxEn;  // 0x78

// STATUS1 (0x40)
constexpr uint8_t kStatus1OscOk = 0x10;

// TX_PROTOCOL1 (0x15)
constexpr uint8_t kTxProtShortFrame = 0x00;  // 7-bit WUPA/REQA: no CRC, no parity
constexpr uint8_t kTxProtAnticol = 0x40;     // a_tx_par
constexpr uint8_t kTxProtData = 0x60;        // a_tx_par | tx_crc

// RX_PROTOCOL1 (0x17). b_rx_sof and b_rx_eof must stay set or I_rxs never fires.
constexpr uint8_t kRxProtShortFrame = 0x38;  // sof | eof | a_rx_par
constexpr uint8_t kRxProtAnticol = 0x39;     // sof | eof | a_rx_par | antcl
constexpr uint8_t kRxProtRxNbtx = 0x02;      // received bits positioned per nbtx (split-byte anticollision)
constexpr uint8_t kRxProtData = 0x3C;        // sof | eof | a_rx_par | rx_crc

// FIFO_STATUS2 (0x37)
constexpr uint8_t kFifoStatus2B8 = 0x40;   // fifo_b<8>
constexpr uint8_t kFifoStatus2Unf = 0x20;
constexpr uint8_t kFifoStatus2Ovr = 0x10;
constexpr uint8_t kFifoStatus2NpLb = 0x01;

// STATIC_STATUS3 (0x44): cleared at the start of each reception
constexpr uint8_t kStatic3Crc = 0x80;
constexpr uint8_t kStatic3Par = 0x40;
constexpr uint8_t kStatic3Hfe = 0x20;
constexpr uint8_t kStatic3Sfe = 0x10;

// Interrupts, folded into one 24-bit word: IRQ_STATUS1 in bits 7:0,
// IRQ_STATUS2 in 15:8, IRQ_STATUS3 in 23:16. Same layout for the masks.
constexpr uint32_t kIrqRxErr = 1u << 0;
constexpr uint32_t kIrqTxe = 1u << 1;
constexpr uint32_t kIrqRxs = 1u << 2;
constexpr uint32_t kIrqRxe = 1u << 3;
constexpr uint32_t kIrqCol = 1u << 6;
constexpr uint32_t kIrqNre = 1u << (8 + 6);
constexpr uint32_t kIrqOsc = 1u << (16 + 0);
constexpr uint32_t kIrqDct = 1u << (16 + 4);

// IC identity: ic_type in bits 7:3 is 10110b for the ST25R300.
constexpr uint8_t kIcTypeMask = 0xF8;
constexpr uint8_t kIcTypeSt25r300 = 0xB0;
constexpr uint8_t kIcRevMask = 0x07;

// Direct commands (DS14655 table 13; odd codes are the SPI encodings)
constexpr uint8_t kCmdSetDefault = 0x61;
constexpr uint8_t kCmdStopAll = 0x63;
constexpr uint8_t kCmdClearFifo = 0x65;
constexpr uint8_t kCmdClearRxGain = 0x67;
constexpr uint8_t kCmdAdjustRegulators = 0x69;
constexpr uint8_t kCmdTransmitData = 0x6B;
constexpr uint8_t kCmdFieldOn = 0x6F;
constexpr uint8_t kCmdSenseRf = 0x7D;

} // namespace st25r300
