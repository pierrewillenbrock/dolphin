// Copyright 2009 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/CommonTypes.h"

#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP::JITIR::x64
{
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

//----

// DAR $arD
// 0000 0000 0000 01dd
// Decrement address register $arD.
void DSPEmitterIR::ir_dar(const UDSPInstruction opc)
{
  u8 areg = opc & 0x3;
  IRInsn p = {&SubAOp, {IROp::R(areg), IROp::R(areg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(areg)};
  ir_add_op(p);
}

// IAR $arD
// 0000 0000 0000 10dd
// Increment address register $arD.
void DSPEmitterIR::ir_iar(const UDSPInstruction opc)
{
  u8 areg = opc & 0x3;
  IRInsn p = {&AddAOp, {IROp::R(areg), IROp::R(areg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(areg)};
  ir_add_op(p);
}

// SUBARN $arD
// 0000 0000 0000 11dd
// Subtract indexing register $ixD from an addressing register $arD.
// used only in IPL-NTSC ucode
void DSPEmitterIR::ir_subarn(const UDSPInstruction opc)
{
  u8 areg = opc & 0x3;
  IRInsn p = {&SubAOp,
              {IROp::R(areg), IROp::R(areg + DSP_REG_WR0), IROp::R(areg + DSP_REG_IX0)},
              IROp::R(areg)};
  ir_add_op(p);
}

// ADDARN $arD, $ixS
// 0000 0000 0001 ssdd
// Adds indexing register $ixS to an addressing register $arD.
// It is critical for the Zelda ucode that this one wraps correctly.
void DSPEmitterIR::ir_addarn(const UDSPInstruction opc)
{
  u8 areg = opc & 0x3;
  u8 ireg = (opc >> 2) & 0x3;
  IRInsn p = {&AddAOp,
              {IROp::R(areg), IROp::R(areg + DSP_REG_WR0), IROp::R(ireg + DSP_REG_IX0)},
              IROp::R(areg)};
  ir_add_op(p);
}

// SBCLR #I
// 0001 0011 aaaa aiii
// bit of status register $sr. Bit number is calculated by adding 6 to
// immediate value I.
void DSPEmitterIR::ir_sbclr(const UDSPInstruction opc)
{
  u8 bit = (opc & 0x7) + 6;
  IRInsn p = {
      &SBClrOp, {IROp::Imm(bit)}, IROp::None(),    {},
      0x0000,   (u16)(1 << bit),  (u16)(1 << bit), 0x0000,
  };
  ir_add_op(p);
}

// SBSET #I
// 0001 0010 aaaa aiii
// Set bit of status register $sr. Bit number is calculated by adding 6 to
// immediate value I.
void DSPEmitterIR::ir_sbset(const UDSPInstruction opc)
{
  u8 bit = (opc & 0x7) + 6;
  IRInsn p = {&SBSetOp, {IROp::Imm(bit)}, IROp::None(),    {},
              0x0000,   (u16)(1 << bit),  (u16)(1 << bit), (u16)(1 << bit)};
  ir_add_op(p);
}

// 1000 bbbv xxxx xxxx, bbb >= 101
// This is a bunch of flag setters, flipping bits in SR. So far so good,
// but it's harder to know exactly what effect they have.
void DSPEmitterIR::ir_srbith(const UDSPInstruction opc)
{
  u8 bit;

  switch ((opc >> 8) & 0xe)
  {
  case 0xa:  // M0/M2
    bit = 13;
    break;
  case 0xc:  // CLR15/SET15
    bit = 15;
    break;
  case 0xe:  // SET16/SET40
    bit = 14;
    break;
  default:
    ASSERT_MSG(DSPLLE, 0, "Bad opcode for srbith: %04x", opc);
    bit = -1;
    break;
  }
  IRInsn p = {&InvalidOp, {IROp::Imm(bit)}, IROp::None(),    {},
              0x0000,     (u16)(1 << bit),  (u16)(1 << bit), 0x0000};
  if (opc & 0x100)
  {
    p.constant_val_SR = 1 << bit;
    p.emitter = &SBSetOp;
  }
  else
  {
    p.emitter = &SBClrOp;
  }
  ir_add_op(p);
}

void DSPEmitterIR::iremit_AddAOp(IRInsn const& insn)
{
  // inputs: 1. value, 2. mask, 3. increment
  // output: value
  // for now, just assume input value and mask
  // and output value match.
  u8 reg = insn.inputs[0].guest_reg;
  u8 ix_reg = insn.inputs[2].guest_reg & 0x3;
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  _assert_msg_(DSPLLE, insn.inputs[1].guest_reg == insn.inputs[0].guest_reg + DSP_REG_WR0,
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
    _assert_msg_(DSPLLE, 0, "unhandled AddAOp variant");
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AddAOp = {
    "AddAOp",    &DSPEmitterIR::iremit_AddAOp, 0x0000, 0x0000, 0x0000, 0x0000, false, {}, {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_SubAOp(IRInsn const& insn)
{
  // inputs: 1. value, 2. mask, 3. increment
  // output: value
  // for now, just assume input value and mask
  // and output value match.
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  u8 reg = insn.inputs[0].guest_reg;

  _assert_msg_(DSPLLE,
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
    _assert_msg_(DSPLLE, 0, "unhandled AddAOp variant");
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::SubAOp = {
    "SubAOp",    &DSPEmitterIR::iremit_SubAOp, 0x0000, 0x0000, 0x0000, 0x0000, false, {}, {},
    {{OpAnyReg}}};

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
