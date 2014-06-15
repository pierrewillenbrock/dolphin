// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/CommonTypes.h"

#include "Common/Logging/Log.h"
#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP::JITIR::x64
{
u16 DSPEmitterIR::ReadIFXRegisterHelper(DSPEmitterIR& emitter, u16 address)
{
  return emitter.m_dsp_core.DSPState().ReadIFX(address);
}

void DSPEmitterIR::WriteIFXRegisterHelper(DSPEmitterIR& emitter, u16 address, u16 value)
{
  emitter.m_dsp_core.DSPState().WriteIFX(address, value);
}

void DSPEmitterIR::dsp_reg_stack_push(StackRegister stack_reg, Gen::X64Reg tmp1, Gen::X64Reg tmp2,
                                      Gen::X64Reg tmp3)
{
  const auto reg_index = static_cast<size_t>(stack_reg);

  // g_dsp.reg_stack_ptrs[reg_index]++;
  // g_dsp.reg_stack_ptrs[reg_index] &= DSP_STACK_MASK;
  MOV(8, R(tmp3), M_SDSP_reg_stack_ptrs(reg_index));
  ADD(8, R(tmp3), Imm8(1));
  AND(8, R(tmp3), Imm8(DSP_STACK_MASK));
  MOV(8, M_SDSP_reg_stack_ptrs(reg_index), R(tmp3));

  // g_dsp.reg_stacks[reg_index][g_dsp.reg_stack_ptrs[reg_index]] = g_dsp.r[DSP_REG_ST0 + reg_index];
  MOV(16, R(tmp1), M_SDSP_r_st(reg_index));
  MOVZX(64, 8, tmp3, R(tmp3));
  MOV(64, R(tmp2), ImmPtr(m_dsp_core.DSPState().reg_stacks[reg_index]));
  MOV(16, MComplex(tmp2, tmp3, SCALE_2, 0), R(tmp1));
}

void DSPEmitterIR::dsp_reg_stack_pop(StackRegister stack_reg, Gen::X64Reg tmp1, Gen::X64Reg tmp2,
                                     Gen::X64Reg tmp3)
{
  const auto reg_index = static_cast<size_t>(stack_reg);

  // g_dsp.r[DSP_REG_ST0 + reg_index] = g_dsp.reg_stacks[reg_index][g_dsp.reg_stack_ptrs[reg_index]];
  MOV(8, R(tmp3), M_SDSP_reg_stack_ptrs(reg_index));
  MOVZX(64, 8, tmp3, R(tmp3));
  MOV(64, R(tmp2), ImmPtr(m_dsp_core.DSPState().reg_stacks[reg_index]));
  MOV(16, R(tmp1), MComplex(tmp2, tmp3, SCALE_2, 0));
  MOV(16, M_SDSP_r_st(reg_index), R(tmp1));

  // g_dsp.reg_stack_ptrs[reg_index]--;
  // g_dsp.reg_stack_ptrs[reg_index] &= DSP_STACK_MASK;
  SUB(8, R(tmp3), Imm8(1));
  AND(8, R(tmp3), Imm8(DSP_STACK_MASK));
  MOV(8, M_SDSP_reg_stack_ptrs(reg_index), R(tmp3));
}

void DSPEmitterIR::dsp_reg_store_stack(StackRegister stack_reg, Gen::OpArg const& source,
                                       Gen::X64Reg tmp1, Gen::X64Reg tmp2, Gen::X64Reg tmp3)
{
  dsp_reg_stack_push(stack_reg, tmp1, tmp2, tmp3);
  // g_dsp.r[DSP_REG_ST0 + stack_reg] = val;
  if (source.IsImm())
    MOV(16, M_SDSP_r_st(static_cast<size_t>(stack_reg)), source.AsImm16());
  else
    MOV(16, M_SDSP_r_st(static_cast<size_t>(stack_reg)), source);
}

void DSPEmitterIR::dsp_reg_load_stack(StackRegister stack_reg, Gen::X64Reg host_dreg,
                                      Gen::X64Reg tmp1, Gen::X64Reg tmp2, Gen::X64Reg tmp3)
{
  ASSERT_MSG(DSPLLE, host_dreg != tmp1 && host_dreg != tmp2 && host_dreg != tmp3,
               "bad register allocation");
  // u16 val = g_dsp.r[DSP_REG_ST0 + stack_reg];
  MOV(16, R(host_dreg), M_SDSP_r_st(static_cast<size_t>(stack_reg)));

  dsp_reg_stack_pop(stack_reg, tmp1, tmp2, tmp3);
}

// addr math
//
// These functions detect overflow by checking if
// the bit past the top of the mask(WR) has changed in AR.
// They detect values under the minimum for a mask by adding wr + 1
// and checking if the bit past the top of the mask doesn't change.
// Both are done while ignoring changes due to values/holes in IX
// above the mask.

void DSPEmitterIR::increment_addr_reg(Gen::X64Reg ar, Gen::OpArg wr, Gen::X64Reg tmp1,
                                      Gen::X64Reg tmp4)
{
  if (wr.IsImm() && (wr.AsImm16().Imm16() & (wr.AsImm16().Imm16() + 1)) == 0)
  {
    // this is just add+mask. still need to keep the
    // upper bits somewhere
    if (wr.AsImm16().Imm16() == 0xffff)
      LEA(16, ar, MDisp(ar, 1));
    else
    {
      LEA(16, tmp1, MDisp(ar, 1));
      AND(16, R(ar), Imm16(~wr.AsImm16().Imm16()));
      AND(16, R(tmp1), wr.AsImm16());
      OR(16, R(ar), R(tmp1));
    }
    return;
  }
  // u32 nar = ar + 1;
  MOV(32, R(tmp1), R(ar));
  LEA(32, ar, MDisp(ar, 1));

  // if ((nar ^ ar) > ((wr | 1) << 1))
  //		nar -= wr + 1;
  XOR(32, R(tmp1), R(ar));
  if (wr.IsImm())
    CMP(32, R(tmp1), Imm32((wr.AsImm16().Imm16() | 1) << 1));
  else
  {
    LEA(32, tmp4, MRegSum(wr.GetSimpleReg(), wr.GetSimpleReg()));
    OR(32, R(tmp4), Imm8(2));
    CMP(32, R(tmp1), R(tmp4));
  }

  FixupBranch nowrap = J_CC(CC_BE);
  if (wr.IsImm())
    SUB(16, R(ar), Imm16(wr.AsImm16().Imm16() + 1));
  else
  {
    SUB(16, R(ar), wr);
    SUB(16, R(ar), Imm8(1));
  }
  SetJumpTarget(nowrap);
  // g_dsp.r.ar[reg] = nar;
}

void DSPEmitterIR::decrement_addr_reg(Gen::X64Reg ar_in, Gen::OpArg wr, Gen::X64Reg ar_out,
                                      Gen::X64Reg tmp4)
{
  // u32 nar = ar + wr;
  // edi = nar
  if (wr.IsImm())
    LEA(32, ar_out, MDisp(ar_in, wr.AsImm16().Imm16()));
  else
    LEA(32, ar_out, MRegSum(ar_in, wr.GetSimpleReg()));

  if (wr.IsImm() && (wr.AsImm16().Imm16() & (wr.AsImm16().Imm16() + 1)) == 0)
  {
    // this is just add+mask. still need to keep the
    // high bits somewhere
    if (wr.AsImm16().Imm16() != 0xffff)
    {
      AND(16, R(ar_in), Imm16(~wr.AsImm16().Imm16()));
      AND(16, R(ar_out), wr.AsImm16());
      OR(16, R(ar_out), R(ar_in));
    }
    return;
  }

  // if (((nar ^ ar) & ((wr | 1) << 1)) > wr)
  //		nar -= wr + 1;
  XOR(32, R(ar_in), R(ar_out));
  if (wr.IsImm())
  {
    AND(32, R(ar_in), Imm32((wr.AsImm16().Imm16() | 1) << 1));
    CMP(32, R(ar_in), Imm32(wr.AsImm16().Imm16()));
  }
  else
  {
    LEA(32, tmp4, MRegSum(wr.GetSimpleReg(), wr.GetSimpleReg()));
    OR(32, R(tmp4), Imm8(2));
    AND(32, R(ar_in), R(tmp4));
    CMP(32, R(ar_in), wr);
  }
  FixupBranch nowrap = J_CC(CC_BE);
  if (wr.IsImm())
    SUB(16, R(ar_out), Imm16(wr.AsImm16().Imm16() + 1));
  else
  {
    SUB(16, R(ar_out), wr);
    SUB(16, R(ar_out), Imm8(1));
  }
  SetJumpTarget(nowrap);
  // g_dsp.r.ar[reg] = nar;
}

// Increase addr register according to the correspond ix register
void DSPEmitterIR::increase_addr_reg(Gen::X64Reg ar_in, Gen::OpArg wr, Gen::X64Reg ix,
                                     Gen::X64Reg ar_out)
{
  // u32 nar = ar + ix;
  // edi = nar
  LEA(32, ar_out, MRegSum(ar_in, ix));

  if (wr.IsImm() && (wr.AsImm16().Imm16() & (wr.AsImm16().Imm16() + 1)) == 0)
  {
    // this is just add+mask. still need to keep the
    // high bits somewhere
    if (wr.AsImm16().Imm16() != 0xffff)
    {
      AND(16, R(ar_in), Imm16(~wr.AsImm16().Imm16()));
      AND(16, R(ar_out), Imm16(wr.AsImm16().Imm16()));
      OR(16, R(ar_out), R(ar_in));
    }
    return;
  }

  // u32 dar = (nar ^ ar ^ ix) & ((wr | 1) << 1);
  // eax = dar
  XOR(32, R(ar_in), R(ix));
  XOR(32, R(ar_in), R(ar_out));

  // if (ix >= 0)
  TEST(32, R(ix), R(ix));
  FixupBranch negative = J_CC(CC_S);
  if (wr.IsImm())
  {
    AND(32, R(ar_in), Imm32((wr.AsImm16().Imm16() | 1) << 1));
    // if (dar > wr)
    CMP(32, R(ar_in), wr.AsImm32());
  }
  else
  {
    LEA(32, ix, MRegSum(wr.GetSimpleReg(), wr.GetSimpleReg()));
    OR(32, R(ix), Imm8(2));
    AND(32, R(ar_in), R(ix));
    // if (dar > wr)
    CMP(32, R(ar_in), wr);
  }

  FixupBranch done = J_CC(CC_BE);
  // nar -= wr + 1;
  if (wr.IsImm())
    SUB(16, R(ar_out), Imm16(wr.AsImm16().Imm16() + 1));
  else
  {
    SUB(16, R(ar_out), wr);
    SUB(16, R(ar_out), Imm8(1));
  }
  FixupBranch done2 = J();

  // else
  SetJumpTarget(negative);
  if (wr.IsImm())
  {
    AND(32, R(ar_in), Imm32((wr.AsImm16().Imm16() | 1) << 1));
    // if ((((nar + wr + 1) ^ nar) & dar) <= wr)
    LEA(32, ix, MDisp(ar_out, wr.AsImm16().Imm16() + 1));
  }
  else
  {
    LEA(32, ix, MRegSum(wr.GetSimpleReg(), wr.GetSimpleReg()));
    OR(32, R(ix), Imm8(2));
    AND(32, R(ar_in), R(ix));
    // if ((((nar + wr + 1) ^ nar) & dar) <= wr)
    LEA(32, ix, MComplex(ar_out, wr.GetSimpleReg(), SCALE_1, 1));
  }

  XOR(32, R(ix), R(ar_out));
  AND(32, R(ix), R(ar_in));
  if (wr.IsImm())
    CMP(32, R(ix), wr.AsImm32());
  else
    CMP(32, R(ix), wr);
  FixupBranch done3 = J_CC(CC_A);
  // nar += wr + 1;
  if (wr.IsImm())
    LEA(32, ar_out, MDisp(ar_out, wr.AsImm16().Imm16() + 1));
  else
    LEA(32, ar_out, MComplex(ar_out, wr.GetSimpleReg(), SCALE_1, 1));

  SetJumpTarget(done);
  SetJumpTarget(done2);
  SetJumpTarget(done3);
  // g_dsp.r.ar[reg] = nar;
}

// Decrease addr register according to the correspond ix register
void DSPEmitterIR::decrease_addr_reg(Gen::X64Reg ar_in, Gen::OpArg wr, Gen::X64Reg ix,
                                     Gen::X64Reg ar_out)
{
  NOT(32, R(ix));  // esi = ~ix

  // u32 nar = ar - ix; (ar + ~ix + 1)
  LEA(32, ar_out, MComplex(ar_in, ix, SCALE_1, 1));

  if (wr.IsImm() && (wr.AsImm16().Imm16() & (wr.AsImm16().Imm16() + 1)) == 0)
  {
    // this is just add+mask. still need to keep the
    // upper bits somewhere
    if (wr.AsImm16().Imm16() != 0xffff)
    {
      AND(16, R(ar_in), Imm16(~wr.AsImm16().Imm16()));
      AND(16, R(ar_out), wr.AsImm16());
      OR(16, R(ar_out), R(ar_in));
    }
    return;
  }

  // u32 dar = (nar ^ ar ^ ~ix) & ((wr | 1) << 1);
  // eax = dar
  XOR(32, R(ar_in), R(ix));
  XOR(32, R(ar_in), R(ar_out));

  // if ((u32)ix > 0xFFFF8000)  ==> (~ix < 0x00007FFF)
  CMP(32, R(ix), Imm32(0x00007FFF));
  FixupBranch positive = J_CC(CC_AE);
  if (wr.IsImm())
  {
    AND(32, R(ar_in), Imm32((wr.AsImm16().Imm16() | 1) << 1));
    // if (dar > wr)
    CMP(32, R(ar_in), wr.AsImm32());
  }
  else
  {
    LEA(32, ix, MRegSum(wr.GetSimpleReg(), wr.GetSimpleReg()));
    OR(32, R(ix), Imm8(2));
    AND(32, R(ar_in), R(ix));
    // if (dar > wr)
    CMP(32, R(ar_in), wr);
  }

  FixupBranch done = J_CC(CC_BE);
  // nar -= wr + 1;
  if (wr.IsImm())
    SUB(16, R(ar_out), Imm16(wr.AsImm16().Imm16() + 1));
  else
  {
    SUB(16, R(ar_out), wr);
    SUB(16, R(ar_out), Imm8(1));
  }
  FixupBranch done2 = J();

  // else
  SetJumpTarget(positive);
  if (wr.IsImm())
  {
    AND(32, R(ar_in), Imm32((wr.AsImm16().Imm16() | 1) << 1));
    // if ((((nar + wr + 1) ^ nar) & dar) <= wr)
    LEA(32, ix, MDisp(ar_out, wr.AsImm16().Imm16() + 1));
  }
  else
  {
    LEA(32, ix, MRegSum(wr.GetSimpleReg(), wr.GetSimpleReg()));
    OR(32, R(ix), Imm8(2));
    AND(32, R(ar_in), R(ix));
    // if ((((nar + wr + 1) ^ nar) & dar) <= wr)
    LEA(32, ix, MComplex(ar_out, wr.GetSimpleReg(), SCALE_1, 1));
  }

  XOR(32, R(ix), R(ar_out));
  AND(32, R(ix), R(ar_in));
  if (wr.IsImm())
    CMP(32, R(ix), wr.AsImm32());
  else
    CMP(32, R(ix), wr);
  FixupBranch done3 = J_CC(CC_A);
  // nar += wr + 1;
  if (wr.IsImm())
    LEA(32, ar_out, MDisp(ar_out, wr.AsImm16().Imm16() + 1));
  else
    LEA(32, ar_out, MComplex(ar_out, wr.GetSimpleReg(), SCALE_1, 1));

  SetJumpTarget(done);
  SetJumpTarget(done2);
  SetJumpTarget(done3);
  // return nar
}

void DSPEmitterIR::dmem_write(Gen::X64Reg value, Gen::X64Reg destaddr, Gen::X64Reg tmp1,
                              IRInsn const& insn)
{
  //	if (saddr == 0)
  CMP(16, R(destaddr), Imm16(0x0fff));
  FixupBranch ifx = J_CC(CC_A);

  //  g_dsp.dram[addr & DSP_DRAM_MASK] = val;
  AND(16, R(destaddr), Imm16(DSP_DRAM_MASK));
  MOV(64, R(tmp1), ImmPtr(m_dsp_core.DSPState().dram));
  MOV(16, MComplex(tmp1, destaddr, SCALE_2, 0), R(value));

  FixupBranch end = J(true);
  //	else if (saddr == 0xf)
  SetJumpTarget(ifx);
  MOVZX(32, 16, value, R(value));
  preABICall(insn);
  ABI_CallFunctionPRR(WriteIFXRegisterHelper, this, destaddr, value);
  postABICall(insn);
  SetJumpTarget(end);
}

void DSPEmitterIR::dmem_write_imm(u16 address, X64Reg value, X64Reg tmp1, IRInsn const& insn)
{
  switch (address >> 12)
  {
  case 0x0:  // 0xxx DRAM
    MOV(64, R(tmp1), ImmPtr(m_dsp_core.DSPState().dram));
    MOV(16, MDisp(tmp1, (address & DSP_DRAM_MASK) * 2), R(value));
    break;

  case 0xf:  // Fxxx HW regs
  {
    preABICall(insn);
    MOV(16, R(EAX), Imm16(address));
    ABI_CallFunctionPRR(WriteIFXRegisterHelper, this, EAX, value);
    postABICall(insn);
    break;
  }
  default:  // Unmapped/non-existing memory
    ERROR_LOG_FMT(DSPLLE, "{:04x} DSP ERROR: Write to UNKNOWN ({:04x}) memory",
                  m_dsp_core.DSPState().pc, address);
    break;
  }
}

// In:  (address) - the address to read
// Out: (host_dreg) - the result of the read (used by caller)
void DSPEmitterIR::imem_read(X64Reg address, X64Reg host_dreg)
{
  //	if (addr == 0)
  CMP(16, R(address), Imm16(0x0fff));
  FixupBranch irom = J_CC(CC_A);
  //	return g_dsp.iram[addr & DSP_IRAM_MASK];
  AND(16, R(address), Imm16(DSP_IRAM_MASK));
  MOV(64, R(host_dreg), ImmPtr(m_dsp_core.DSPState().iram));
  MOV(16, R(host_dreg), MComplex(host_dreg, address, SCALE_2, 0));

  FixupBranch end = J();
  SetJumpTarget(irom);
  //	else if (addr == 0x8)
  //		return g_dsp.irom[addr & DSP_IROM_MASK];
  AND(16, R(address), Imm16(DSP_IROM_MASK));
  MOV(64, R(host_dreg), ImmPtr(m_dsp_core.DSPState().irom));
  MOV(16, R(host_dreg), MComplex(host_dreg, address, SCALE_2, 0));

  SetJumpTarget(end);
}

// In:  (address) - the address to read
// Out: (host_dreg) - the result of the read (used by caller)
// address must be abi safe
void DSPEmitterIR::dmem_read(X64Reg address, X64Reg host_dreg, IRInsn const& insn)
{
  //	if (saddr == 0)
  CMP(16, R(address), Imm16(0x0fff));
  FixupBranch dram = J_CC(CC_A);
  //	return g_dsp.dram[addr & DSP_DRAM_MASK];
  AND(32, R(address), Imm32(DSP_DRAM_MASK));
  MOV(64, R(host_dreg), ImmPtr(m_dsp_core.DSPState().dram));
  MOV(16, R(host_dreg), MComplex(host_dreg, address, SCALE_2, 0));

  FixupBranch end = J(true);
  SetJumpTarget(dram);
  //	else if (saddr == 0x1)
  CMP(16, R(address), Imm16(0x1fff));
  FixupBranch ifx = J_CC(CC_A);
  //		return g_dsp.coef[addr & DSP_COEF_MASK];
  AND(32, R(address), Imm32(DSP_COEF_MASK));
  MOV(64, R(host_dreg), ImmPtr(m_dsp_core.DSPState().coef));
  MOV(16, R(host_dreg), MComplex(host_dreg, address, SCALE_2, 0));

  FixupBranch end2 = J(true);
  SetJumpTarget(ifx);
  //	else if (saddr == 0xf)
  //		return gdsp_ifx_read(addr);
  preABICall(insn, host_dreg);
  ABI_CallFunctionPR(ReadIFXRegisterHelper, this, address);
  postABICall(insn, host_dreg);
  SetJumpTarget(end);
  SetJumpTarget(end2);
}

void DSPEmitterIR::dmem_read_imm(u16 address, X64Reg host_dreg, IRInsn const& insn)
{
  switch (address >> 12)
  {
  case 0x0:  // 0xxx DRAM
    MOV(64, R(host_dreg), ImmPtr(m_dsp_core.DSPState().dram));
    MOV(16, R(host_dreg), MDisp(host_dreg, (address & DSP_DRAM_MASK) * 2));
    break;

  case 0x1:  // 1xxx COEF
    MOV(64, R(host_dreg), ImmPtr(m_dsp_core.DSPState().coef));
    MOV(16, R(host_dreg), MDisp(host_dreg, (address & DSP_COEF_MASK) * 2));
    break;

  case 0xf:  // Fxxx HW regs
  {
    preABICall(insn, host_dreg);
    ABI_CallFunctionPC(ReadIFXRegisterHelper, this, address);
    postABICall(insn, host_dreg);
    break;
  }
  default:  // Unmapped/non-existing memory
    ERROR_LOG_FMT(DSPLLE, "{:04x} DSP ERROR: Read from UNKNOWN ({:04x}) memory",
                  m_dsp_core.DSPState().pc, address);
  }
}

// Returns s64 in (long_reg)
void DSPEmitterIR::round_long(X64Reg long_reg)
{
  // if (prod & 0x10000) prod = (prod + 0x8000) & ~0xffff;
  // else prod = (prod + 0x7fff) & ~0xffff;
  BT(32, R(long_reg), Imm8(16));
  ADC(64, R(long_reg), Imm32(0x7FFF));
  XOR(16, R(long_reg), R(long_reg));
  // return prod;
}

}  // namespace DSP::JITIR::x64
