// Copyright 2009 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/CommonTypes.h"

#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP::JITIR::x64
{
// DAR $arD
// 0000 0000 0000 01dd
// Decrement address register $arD.
void DSPEmitterIR::dar(const UDSPInstruction opc)
{
  u8 reg = opc & 0x3;
  X64Reg tmp1 = m_gpr.GetFreeXReg();

  //	g_dsp.r[opc & 0x3] = dsp_decrement_addr_reg(opc & 0x3);
  m_gpr.ReadReg(DSP_REG_WR0 + reg, RDX, RegisterExtension::Zero);
  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
  MOVZX(32, 16, RAX, ar_reg);

  decrement_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0 + reg);

  m_gpr.PutXReg(tmp1);
}

// IAR $arD
// 0000 0000 0000 10dd
// Increment address register $arD.
void DSPEmitterIR::iar(const UDSPInstruction opc)
{
  u8 reg = opc & 0x3;
  X64Reg tmp1 = m_gpr.GetFreeXReg();

  //	g_dsp.r[opc & 0x3] = dsp_increment_addr_reg(opc & 0x3);
  m_gpr.ReadReg(DSP_REG_WR0 + reg, RDX, RegisterExtension::Zero);
  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
  MOVZX(32, 16, RAX, ar_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0 + reg);

  m_gpr.PutXReg(tmp1);
}

// SUBARN $arD
// 0000 0000 0000 11dd
// Subtract indexing register $ixD from an addressing register $arD.
// used only in IPL-NTSC ucode
void DSPEmitterIR::subarn(const UDSPInstruction opc)
{
  u8 reg = opc & 0x3;
  X64Reg tmp1 = m_gpr.GetFreeXReg();

  //	u8 dreg = opc & 0x3;
  //	g_dsp.r[dreg] = dsp_decrease_addr_reg(dreg, (s16)g_dsp.r[DSP_REG_IX0 + dreg]);

  m_gpr.ReadReg(DSP_REG_WR0 + reg, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0 + reg, RCX, RegisterExtension::Sign);
  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
  MOVZX(32, 16, RAX, ar_reg);

  decrease_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0 + reg);

  m_gpr.PutXReg(tmp1);
}

// ADDARN $arD, $ixS
// 0000 0000 0001 ssdd
// Adds indexing register $ixS to an addressing register $arD.
// It is critical for the Zelda ucode that this one wraps correctly.
void DSPEmitterIR::addarn(const UDSPInstruction opc)
{
  u8 reg = opc & 0x3;
  u8 _ix_reg = (opc >> 2) & 0x3;
  X64Reg tmp1 = m_gpr.GetFreeXReg();

  //	u8 dreg = opc & 0x3;
  //	u8 sreg = (opc >> 2) & 0x3;
  //	g_dsp.r[dreg] = dsp_increase_addr_reg(dreg, (s16)g_dsp.r[DSP_REG_IX0 + sreg]);

  // From looking around it is always called with the matching index register
  m_gpr.ReadReg(DSP_REG_WR0 + reg, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0 + _ix_reg, RCX, RegisterExtension::Sign);
  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
  MOVZX(32, 16, RAX, ar_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0 + reg);

  m_gpr.PutXReg(tmp1);
}

//----

void DSPEmitterIR::setCompileSR(u16 bit)
{
  //	g_dsp.r[DSP_REG_SR] |= bit
  const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
  OR(16, sr_reg, Imm16(bit));
  m_gpr.PutReg(DSP_REG_SR);

  m_compile_status_register |= bit;
}

void DSPEmitterIR::clrCompileSR(u16 bit)
{
  //	g_dsp.r[DSP_REG_SR] &= bit
  const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
  AND(16, sr_reg, Imm16(~bit));
  m_gpr.PutReg(DSP_REG_SR);

  m_compile_status_register &= ~bit;
}
// SBCLR #I
// 0001 0011 aaaa aiii
// bit of status register $sr. Bit number is calculated by adding 6 to
// immediate value I.
void DSPEmitterIR::sbclr(const UDSPInstruction opc)
{
  u8 bit = (opc & 0x7) + 6;

  clrCompileSR(1 << bit);
}

// SBSET #I
// 0001 0010 aaaa aiii
// Set bit of status register $sr. Bit number is calculated by adding 6 to
// immediate value I.
void DSPEmitterIR::sbset(const UDSPInstruction opc)
{
  u8 bit = (opc & 0x7) + 6;

  setCompileSR(1 << bit);
}

// 1000 bbbv xxxx xxxx, bbb >= 101
// This is a bunch of flag setters, flipping bits in SR. So far so good,
// but it's harder to know exactly what effect they have.
void DSPEmitterIR::srbith(const UDSPInstruction opc)
{
  switch ((opc >> 8) & 0xf)
  {
  // M0/M2 change the multiplier mode (it can multiply by 2 for free).
  case 0xa:  // M2
    clrCompileSR(SR_MUL_MODIFY);
    break;
  case 0xb:  // M0
    setCompileSR(SR_MUL_MODIFY);
    break;

  // If set, treat multiplicands as unsigned.
  // If clear, treat them as signed.
  case 0xc:  // CLR15
    clrCompileSR(SR_MUL_UNSIGNED);
    break;
  case 0xd:  // SET15
    setCompileSR(SR_MUL_UNSIGNED);
    break;

  // Automatic 40-bit sign extension when loading ACx.M.
  // SET40 changes something very important: see the LRI instruction above.
  case 0xe:  // SET16 (CLR40)
    clrCompileSR(SR_40_MODE_BIT);
    break;

  case 0xf:  // SET40
    setCompileSR(SR_40_MODE_BIT);
    break;

  default:
    break;
  }
}

}  // namespace DSP::JITIR::x64
