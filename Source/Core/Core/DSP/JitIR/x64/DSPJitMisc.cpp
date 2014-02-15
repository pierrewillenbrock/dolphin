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
}

void DSPEmitterIR::clrCompileSR(u16 bit)
{
  //	g_dsp.r[DSP_REG_SR] &= bit
  const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
  AND(16, sr_reg, Imm16(~bit));
  m_gpr.PutReg(DSP_REG_SR);
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

void DSPEmitterIR::iremit_AddAOp(IRInsn const& insn)
{
  // inputs: 1. value, 2. mask, 3. increment
  // output: value
  // for now, just assume input value and mask
  // and output value match.
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  u8 reg = insn.inputs[0].guest_reg;
  u8 ix_reg = insn.inputs[2].guest_reg & 0x3;

  ASSERT_MSG(DSPLLE, insn.inputs[1].guest_reg == insn.inputs[0].guest_reg + DSP_REG_WR0,
             "AddAOp register mismatch");
  if (insn.inputs[2].type == IROp::REG)
  {
    m_gpr.ReadReg(DSP_REG_WR0 + reg, RDX, RegisterExtension::Zero);
    m_gpr.ReadReg(DSP_REG_IX0 + ix_reg, RCX, RegisterExtension::Sign);
    const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
    MOVZX(32, 16, RAX, ar_reg);

    increase_addr_reg(RAX, RDX, RCX, tmp1);

    MOV(32, ar_reg, R(tmp1));
    m_gpr.PutReg(DSP_REG_AR0 + reg);
  }
  else if (insn.inputs[2].type == IROp::IMM && insn.inputs[2].imm == 1)
  {
    m_gpr.ReadReg(DSP_REG_WR0 + reg, RDX, RegisterExtension::Zero);
    const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
    MOVZX(32, 16, RAX, ar_reg);

    increment_addr_reg(RAX, RDX, tmp1, RCX);

    MOV(32, ar_reg, R(RAX));
    m_gpr.PutReg(DSP_REG_AR0 + reg);
  }
  else
    ASSERT_MSG(DSPLLE, 0, "unhandled AddAOp variant");
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AddAOp = {
    "AddAOp", &DSPEmitterIR::iremit_AddAOp, 0x0000, 0x0000, 0x0000, 0x0000,
};

void DSPEmitterIR::iremit_SubAOp(IRInsn const& insn)
{
  // inputs: 1. value, 2. mask, 3. increment
  // output: value
  // for now, just assume input value and mask
  // and output value match.
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  u8 reg = insn.inputs[0].guest_reg;

  ASSERT_MSG(DSPLLE,
             insn.inputs[1].guest_reg == insn.inputs[0].guest_reg + DSP_REG_WR0 &&
                 (insn.inputs[2].type != IROp::REG ||
                  insn.inputs[2].guest_reg == insn.inputs[0].guest_reg + DSP_REG_IX0),
             "SubAOp register mismatch");
  if (insn.inputs[2].type == IROp::REG)
  {
    m_gpr.ReadReg(DSP_REG_WR0 + reg, RDX, RegisterExtension::Zero);
    m_gpr.ReadReg(DSP_REG_IX0 + reg, RCX, RegisterExtension::Sign);
    const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
    MOVZX(32, 16, RAX, ar_reg);

    decrease_addr_reg(RAX, RDX, RCX, tmp1);

    MOV(32, ar_reg, R(tmp1));
    m_gpr.PutReg(DSP_REG_AR0 + reg);
  }
  else if (insn.inputs[2].type == IROp::IMM && insn.inputs[2].imm == 1)
  {
    m_gpr.ReadReg(DSP_REG_WR0 + reg, RDX, RegisterExtension::Zero);
    const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
    MOVZX(32, 16, RAX, ar_reg);

    decrement_addr_reg(RAX, RDX, tmp1, RCX);

    MOV(32, ar_reg, R(tmp1));
    m_gpr.PutReg(DSP_REG_AR0 + reg);
  }
  else
    ASSERT_MSG(DSPLLE, 0, "unhandled AddAOp variant");
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::SubAOp = {
    "SubAOp", &DSPEmitterIR::iremit_SubAOp, 0x0000, 0x0000, 0x0000, 0x0000,
};

void DSPEmitterIR::iremit_SBSetOp(IRInsn const& insn)
{
  setCompileSR(1 << insn.inputs[0].imm);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::SBSetOp = {
    "SBSetOp", &DSPEmitterIR::iremit_SBSetOp, 0x0000, 0x0000, 0x0000,
    0x0000,  // parser needs to fill these
};

void DSPEmitterIR::iremit_SBClrOp(IRInsn const& insn)
{
  clrCompileSR(1 << insn.inputs[0].imm);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::SBClrOp = {
    "SBClrOp", &DSPEmitterIR::iremit_SBClrOp, 0x0000, 0x0000, 0x0000,
    0x0000,  // parser needs to fill these
};

}  // namespace DSP::JITIR::x64
