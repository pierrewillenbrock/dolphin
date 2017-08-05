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

void DSPEmitterIR::dsp_reg_store_stack(StackRegister stack_reg, Gen::X64Reg host_sreg,
                                       Gen::X64Reg tmp1, Gen::X64Reg tmp2, Gen::X64Reg tmp3)
{
  ASSERT_MSG(DSPLLE, host_sreg != tmp1 && host_sreg != tmp2 && host_sreg != tmp3,
               "bad register allocation");
  dsp_reg_stack_push(stack_reg, tmp1, tmp2, tmp3);

  // g_dsp.r[DSP_REG_ST0 + stack_reg] = val;
  MOV(16, M_SDSP_r_st(static_cast<size_t>(stack_reg)), R(host_sreg));
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

void DSPEmitterIR::dsp_reg_store_stack_imm(StackRegister stack_reg, u16 val, Gen::X64Reg tmp1,
                                           Gen::X64Reg tmp2, Gen::X64Reg tmp3)
{
  dsp_reg_stack_push(stack_reg, tmp1, tmp2, tmp3);
  // g_dsp.r[DSP_REG_ST0 + stack_reg] = val;
  MOV(16, M_SDSP_r_st(static_cast<size_t>(stack_reg)), Imm16(val));
}

void DSPEmitterIR::dsp_op_write_reg(int reg, Gen::X64Reg host_sreg, Gen::X64Reg tmp1,
                                    Gen::X64Reg tmp2, Gen::X64Reg tmp3)
{
  switch (reg & 0x1f)
  {
  // 8-bit sign extended registers.
  case DSP_REG_ACH0:
  case DSP_REG_ACH1:
    m_gpr.WriteReg(reg, R(host_sreg));
    break;

  // Stack registers.
  case DSP_REG_ST0:
  case DSP_REG_ST1:
  case DSP_REG_ST2:
  case DSP_REG_ST3:
    dsp_reg_store_stack(static_cast<StackRegister>(reg - DSP_REG_ST0), host_sreg, tmp1, tmp2, tmp3);
    break;

  default:
    m_gpr.WriteReg(reg, R(host_sreg));
    break;
  }
}

void DSPEmitterIR::dsp_op_write_reg_imm(int reg, u16 val, Gen::X64Reg tmp1, Gen::X64Reg tmp2,
                                        Gen::X64Reg tmp3)
{
  switch (reg & 0x1f)
  {
  // 8-bit sign extended registers. Should look at prod.h too...
  case DSP_REG_ACH0:
  case DSP_REG_ACH1:
    m_gpr.WriteReg(reg, Imm16((u16)(s16)(s8)(u8)val));
    break;
  // Stack registers.
  case DSP_REG_ST0:
  case DSP_REG_ST1:
  case DSP_REG_ST2:
  case DSP_REG_ST3:
    dsp_reg_store_stack_imm(static_cast<StackRegister>(reg - DSP_REG_ST0), val, tmp1, tmp2, tmp3);
    break;

  default:
    m_gpr.WriteReg(reg, Imm16(val));
    break;
  }
}

void DSPEmitterIR::dsp_conditional_extend_accum(int reg, X64Reg tmp1)
{
  switch (reg)
  {
  case DSP_REG_ACM0:
  case DSP_REG_ACM1:
  {
    const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
    DSPJitIRRegCache c(m_gpr);
    TEST(16, sr_reg, Imm16(SR_40_MODE_BIT));
    FixupBranch not_40bit = J_CC(CC_Z, true);
    // if (g_dsp.r[DSP_REG_SR] & SR_40_MODE_BIT)
    //{
    // Sign extend into whole accum.
    // u16 val = g_dsp.r[reg];
    m_gpr.ReadReg(reg, tmp1, RegisterExtension::Sign);
    SHR(32, R(tmp1), Imm8(16));
    // g_dsp.r[reg - DSP_REG_ACM0 + DSP_REG_ACH0] = (val & 0x8000) ? 0xFFFF : 0x0000;
    // g_dsp.r[reg - DSP_REG_ACM0 + DSP_REG_ACL0] = 0;
    m_gpr.WriteReg(reg - DSP_REG_ACM0 + DSP_REG_ACH0, R(tmp1));
    m_gpr.WriteReg(reg - DSP_REG_ACM0 + DSP_REG_ACL0, Imm16(0));
    //}
    m_gpr.FlushRegs(c);
    SetJumpTarget(not_40bit);
    m_gpr.PutReg(DSP_REG_SR, false);
  }
  }
}

void DSPEmitterIR::dsp_conditional_extend_accum_imm(int reg, u16 val)
{
  switch (reg)
  {
  case DSP_REG_ACM0:
  case DSP_REG_ACM1:
  {
    const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
    DSPJitIRRegCache c(m_gpr);
    TEST(16, sr_reg, Imm16(SR_40_MODE_BIT));
    FixupBranch not_40bit = J_CC(CC_Z, true);
    // if (g_dsp.r[DSP_REG_SR] & SR_40_MODE_BIT)
    //{
    // Sign extend into whole accum.
    // g_dsp.r[reg - DSP_REG_ACM0 + DSP_REG_ACH0] = (val & 0x8000) ? 0xFFFF : 0x0000;
    // g_dsp.r[reg - DSP_REG_ACM0 + DSP_REG_ACL0] = 0;
    m_gpr.WriteReg(reg - DSP_REG_ACM0 + DSP_REG_ACH0, Imm16((val & 0x8000) ? 0xffff : 0x0000));

    m_gpr.WriteReg(reg - DSP_REG_ACM0 + DSP_REG_ACL0, Imm16(0));
    //}
    m_gpr.FlushRegs(c);
    SetJumpTarget(not_40bit);
    m_gpr.PutReg(DSP_REG_SR, false);
  }
  }
}

void DSPEmitterIR::dsp_op_read_reg_dont_saturate(int reg, Gen::X64Reg host_dreg,
                                                 RegisterExtension extend, Gen::X64Reg tmp1,
                                                 Gen::X64Reg tmp2, Gen::X64Reg tmp3)
{
  switch (reg & 0x1f)
  {
  case DSP_REG_ST0:
  case DSP_REG_ST1:
  case DSP_REG_ST2:
  case DSP_REG_ST3:
    dsp_reg_load_stack(static_cast<StackRegister>(reg - DSP_REG_ST0), host_dreg, tmp1, tmp2, tmp3);
    switch (extend)
    {
    case RegisterExtension::Sign:
      MOVSX(64, 16, host_dreg, R(host_dreg));
      break;
    case RegisterExtension::Zero:
      MOVZX(64, 16, host_dreg, R(host_dreg));
      break;
    case RegisterExtension::None:
    default:
      break;
    }
    return;
  default:
    m_gpr.ReadReg(reg, host_dreg, extend);
    return;
  }
}

void DSPEmitterIR::dsp_op_read_reg(int reg, Gen::X64Reg host_dreg, RegisterExtension extend,
                                   Gen::X64Reg tmp1, Gen::X64Reg tmp2, Gen::X64Reg tmp3)
{
  switch (reg & 0x1f)
  {
  case DSP_REG_ST0:
  case DSP_REG_ST1:
  case DSP_REG_ST2:
  case DSP_REG_ST3:
    dsp_reg_load_stack(static_cast<StackRegister>(reg - DSP_REG_ST0), host_dreg, tmp1, tmp2, tmp3);
    switch (extend)
    {
    case RegisterExtension::Sign:
      MOVSX(64, 16, host_dreg, R(host_dreg));
      break;
    case RegisterExtension::Zero:
      MOVZX(64, 16, host_dreg, R(host_dreg));
      break;
    case RegisterExtension::None:
    default:
      break;
    }
    return;
  case DSP_REG_ACM0:
  case DSP_REG_ACM1:
  {
    // we already know this is ACCM0 or ACCM1
    const OpArg acc_reg = m_gpr.GetReg(reg - DSP_REG_ACM0 + DSP_REG_ACC0_64);
    const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);

    DSPJitIRRegCache c(m_gpr);
    TEST(16, sr_reg, Imm16(SR_40_MODE_BIT));
    FixupBranch not_40bit = J_CC(CC_Z, true);

    MOVSX(64, 32, host_dreg, acc_reg);
    CMP(64, R(host_dreg), acc_reg);
    FixupBranch no_saturate = J_CC(CC_Z);

    TEST(64, acc_reg, acc_reg);
    FixupBranch negative = J_CC(CC_LE);

    MOV(64, R(host_dreg), Imm32(0x7fff));  // this works for all extend modes
    FixupBranch done_positive = J();

    SetJumpTarget(negative);
    if (extend == RegisterExtension::None || extend == RegisterExtension::Zero)
      MOV(64, R(host_dreg), Imm32(0x00008000));
    else
      MOV(64, R(host_dreg), Imm32(0xffff8000));
    FixupBranch done_negative = J();

    SetJumpTarget(no_saturate);
    SetJumpTarget(not_40bit);

    MOV(64, R(host_dreg), acc_reg);
    if (extend == RegisterExtension::None || extend == RegisterExtension::Zero)
      SHR(64, R(host_dreg), Imm8(16));
    else
      SAR(64, R(host_dreg), Imm8(16));
    SetJumpTarget(done_positive);
    SetJumpTarget(done_negative);
    m_gpr.FlushRegs(c);
    m_gpr.PutReg(reg - DSP_REG_ACM0 + DSP_REG_ACC0_64, false);

    m_gpr.PutReg(DSP_REG_SR, false);
  }
    return;
  default:
    m_gpr.ReadReg(reg, host_dreg, extend);
    return;
  }
}

// addr math
//
// These functions detect overflow by checking if
// the bit past the top of the mask(WR) has changed in AR.
// They detect values under the minimum for a mask by adding wr + 1
// and checking if the bit past the top of the mask doesn't change.
// Both are done while ignoring changes due to values/holes in IX
// above the mask.

void DSPEmitterIR::increment_addr_reg(int reg, Gen::X64Reg tmp1, Gen::X64Reg tmp2, Gen::X64Reg tmp3,
                                      Gen::X64Reg tmp4)
{
  const OpArg wr_reg = m_gpr.GetReg(DSP_REG_WR0 + reg);
  MOVZX(32, 16, tmp3, wr_reg);
  m_gpr.PutReg(DSP_REG_WR0 + reg, false);

  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
  MOVZX(32, 16, tmp1, ar_reg);

  // u32 nar = ar + 1;
  LEA(32, tmp2, MDisp(tmp1, 1));

  // if ((nar ^ ar) > ((wr | 1) << 1))
  //		nar -= wr + 1;
  XOR(32, R(tmp1), R(tmp2));
  LEA(32, tmp4, MRegSum(tmp3, tmp3));
  OR(32, R(tmp4), Imm8(2));
  CMP(32, R(tmp1), R(tmp4));
  FixupBranch nowrap = J_CC(CC_BE);
  SUB(16, R(tmp2), R(tmp3));
  SUB(16, R(tmp2), Imm8(1));
  SetJumpTarget(nowrap);
  // g_dsp.r.ar[reg] = nar;

  MOV(16, ar_reg, R(tmp2));
  m_gpr.PutReg(DSP_REG_AR0 + reg);
}

void DSPEmitterIR::decrement_addr_reg(int reg, Gen::X64Reg tmp1, Gen::X64Reg tmp2, Gen::X64Reg tmp3,
                                      Gen::X64Reg tmp4)
{
  const OpArg wr_reg = m_gpr.GetReg(DSP_REG_WR0 + reg);
  MOVZX(32, 16, tmp3, wr_reg);
  m_gpr.PutReg(DSP_REG_WR0 + reg, false);

  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
  MOVZX(32, 16, tmp2, ar_reg);

  // u32 nar = ar + wr;
  // edi = nar
  LEA(32, tmp1, MRegSum(tmp2, tmp3));

  // if (((nar ^ ar) & ((wr | 1) << 1)) > wr)
  //		nar -= wr + 1;
  XOR(32, R(tmp2), R(tmp1));
  LEA(32, tmp4, MRegSum(tmp3, tmp3));
  OR(32, R(tmp4), Imm8(2));
  AND(32, R(tmp2), R(tmp4));
  CMP(32, R(tmp2), R(tmp3));
  FixupBranch nowrap = J_CC(CC_BE);
  SUB(16, R(tmp1), R(tmp3));
  SUB(16, R(tmp1), Imm8(1));
  SetJumpTarget(nowrap);
  // g_dsp.r.ar[reg] = nar;

  MOV(16, ar_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0 + reg);
}

// Increase addr register according to the correspond ix register
void DSPEmitterIR::increase_addr_reg(int reg, int _ix_reg, Gen::X64Reg tmp1, Gen::X64Reg tmp2,
                                     Gen::X64Reg tmp3, Gen::X64Reg tmp4)
{
  const OpArg wr_reg = m_gpr.GetReg(DSP_REG_WR0 + reg);
  MOVZX(32, 16, tmp3, wr_reg);
  m_gpr.PutReg(DSP_REG_WR0 + reg, false);

  const OpArg ix_reg = m_gpr.GetReg(DSP_REG_IX0 + _ix_reg);
  MOVSX(32, 16, tmp4, ix_reg);
  m_gpr.PutReg(DSP_REG_IX0 + _ix_reg, false);

  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
  MOVZX(32, 16, tmp2, ar_reg);

  // u32 nar = ar + ix;
  // edi = nar
  LEA(32, tmp1, MRegSum(tmp2, tmp4));

  // u32 dar = (nar ^ ar ^ ix) & ((wr | 1) << 1);
  // eax = dar
  XOR(32, R(tmp2), R(tmp4));
  XOR(32, R(tmp2), R(tmp1));

  // if (ix >= 0)
  TEST(32, R(tmp4), R(tmp4));
  FixupBranch negative = J_CC(CC_S);
  LEA(32, tmp4, MRegSum(tmp3, tmp3));
  OR(32, R(tmp4), Imm8(2));
  AND(32, R(tmp2), R(tmp4));

  // if (dar > wr)
  CMP(32, R(tmp2), R(tmp3));
  FixupBranch done = J_CC(CC_BE);
  // nar -= wr + 1;
  SUB(16, R(tmp1), R(tmp3));
  SUB(16, R(tmp1), Imm8(1));
  FixupBranch done2 = J();

  // else
  SetJumpTarget(negative);
  LEA(32, tmp4, MRegSum(tmp3, tmp3));
  OR(32, R(tmp4), Imm8(2));
  AND(32, R(tmp2), R(tmp4));

  // if ((((nar + wr + 1) ^ nar) & dar) <= wr)
  LEA(32, tmp4, MComplex(tmp1, tmp3, SCALE_1, 1));
  XOR(32, R(tmp4), R(tmp1));
  AND(32, R(tmp4), R(tmp2));
  CMP(32, R(tmp4), R(tmp3));
  FixupBranch done3 = J_CC(CC_A);
  // nar += wr + 1;
  LEA(32, tmp1, MComplex(tmp1, tmp3, SCALE_1, 1));

  SetJumpTarget(done);
  SetJumpTarget(done2);
  SetJumpTarget(done3);
  // g_dsp.r.ar[reg] = nar;

  MOV(16, ar_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0 + reg);
}

// Decrease addr register according to the correspond ix register
void DSPEmitterIR::decrease_addr_reg(int reg, Gen::X64Reg tmp1, Gen::X64Reg tmp2, Gen::X64Reg tmp3,
                                     Gen::X64Reg tmp4)
{
  const OpArg wr_reg = m_gpr.GetReg(DSP_REG_WR0 + reg);
  MOVZX(32, 16, tmp3, wr_reg);
  m_gpr.PutReg(DSP_REG_WR0 + reg, false);

  const OpArg ix_reg = m_gpr.GetReg(DSP_REG_IX0 + reg);
  MOVSX(32, 16, tmp4, ix_reg);
  m_gpr.PutReg(DSP_REG_IX0 + reg, false);

  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
  MOVZX(32, 16, tmp2, ar_reg);

  NOT(32, R(tmp4));  // esi = ~ix

  // u32 nar = ar - ix; (ar + ~ix + 1)
  LEA(32, tmp1, MComplex(tmp2, tmp4, SCALE_1, 1));

  // u32 dar = (nar ^ ar ^ ~ix) & ((wr | 1) << 1);
  // eax = dar
  XOR(32, R(tmp2), R(tmp4));
  XOR(32, R(tmp2), R(tmp1));

  // if ((u32)ix > 0xFFFF8000)  ==> (~ix < 0x00007FFF)
  CMP(32, R(tmp4), Imm32(0x00007FFF));
  FixupBranch positive = J_CC(CC_AE);
  LEA(32, tmp4, MRegSum(tmp3, tmp3));
  OR(32, R(tmp4), Imm8(2));
  AND(32, R(tmp2), R(tmp4));

  // if (dar > wr)
  CMP(32, R(tmp2), R(tmp3));
  FixupBranch done = J_CC(CC_BE);
  // nar -= wr + 1;
  SUB(16, R(tmp1), R(tmp3));
  SUB(16, R(tmp1), Imm8(1));
  FixupBranch done2 = J();

  // else
  SetJumpTarget(positive);
  LEA(32, tmp4, MRegSum(tmp3, tmp3));
  OR(32, R(tmp4), Imm8(2));
  AND(32, R(tmp2), R(tmp4));

  // if ((((nar + wr + 1) ^ nar) & dar) <= wr)
  LEA(32, tmp4, MComplex(tmp1, tmp3, SCALE_1, 1));
  XOR(32, R(tmp4), R(tmp1));
  AND(32, R(tmp4), R(tmp2));
  CMP(32, R(tmp4), R(tmp3));
  FixupBranch done3 = J_CC(CC_A);
  // nar += wr + 1;
  LEA(32, tmp1, MComplex(tmp1, tmp3, SCALE_1, 1));

  SetJumpTarget(done);
  SetJumpTarget(done2);
  SetJumpTarget(done3);
  // return nar

  MOV(16, ar_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0 + reg);
}

void DSPEmitterIR::dmem_write(X64Reg value, Gen::X64Reg destaddr, Gen::X64Reg tmp1)
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
  DSPJitIRRegCache c(m_gpr);
  MOVZX(32, 16, value, R(value));
  m_gpr.PushRegs();
  ABI_CallFunctionPRR(WriteIFXRegisterHelper, this, destaddr, value);
  m_gpr.PopRegs();
  m_gpr.FlushRegs(c);
  SetJumpTarget(end);
}

void DSPEmitterIR::dmem_write_imm(u16 address, X64Reg value, X64Reg tmp1)
{
  switch (address >> 12)
  {
  case 0x0:  // 0xxx DRAM
    MOV(64, R(tmp1), ImmPtr(m_dsp_core.DSPState().dram));
    MOV(16, MDisp(tmp1, (address & DSP_DRAM_MASK) * 2), R(value));
    break;

  case 0xf:  // Fxxx HW regs
  {
    m_gpr.PushRegs();
    MOV(16, R(EAX), Imm16(address));
    ABI_CallFunctionPRR(WriteIFXRegisterHelper, this, EAX, value);
    m_gpr.PopRegs();
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
void DSPEmitterIR::dmem_read(X64Reg address, X64Reg host_dreg)
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
  DSPJitIRRegCache c(m_gpr);
  m_gpr.PushRegs();
  ABI_CallFunctionPR(ReadIFXRegisterHelper, this, address);
  m_gpr.PopRegs();  // todo: make gpr temporarily leave RAX alone
  if (host_dreg != RAX)
    MOV(16, R(host_dreg), R(RAX));
  m_gpr.FlushRegs(c);
  SetJumpTarget(end);
  SetJumpTarget(end2);
}

void DSPEmitterIR::dmem_read_imm(u16 address, X64Reg host_dreg)
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
    m_gpr.PushRegs();
    ABI_CallFunctionPC(ReadIFXRegisterHelper, this, address);
    m_gpr.PopRegs();  // todo: make gpr temporarily leave RAX alone
    if (host_dreg != RAX)
      MOV(16, R(host_dreg), R(RAX));
    break;
  }
  default:  // Unmapped/non-existing memory
    ERROR_LOG_FMT(DSPLLE, "{:04x} DSP ERROR: Read from UNKNOWN ({:04x}) memory",
                  m_dsp_core.DSPState().pc, address);
  }
}

// Returns s64 in (long_prod)
void DSPEmitterIR::get_long_prod(Gen::X64Reg long_prod, Gen::X64Reg tmp1)
{
  // s64 val   = (s8)(u8)g_dsp.r[DSP_REG_PRODH];
  const OpArg prod_reg = m_gpr.GetReg(DSP_REG_PROD_64);
  MOV(64, R(long_prod), prod_reg);
  m_gpr.PutReg(DSP_REG_PROD_64, false);
  // no use in keeping prod_reg any longer.
  MOV(64, R(tmp1), R(long_prod));
  SHL(64, R(long_prod), Imm8(64 - 40));  // sign extend
  SAR(64, R(long_prod), Imm8(64 - 40));
  SHR(64, R(tmp1), Imm8(48));
  SHL(64, R(tmp1), Imm8(16));
  ADD(64, R(long_prod), R(tmp1));
}

// For accurate emulation, this is wrong - but the real prod registers behave
// in completely bizarre ways. Probably not meaningful to emulate them accurately.
void DSPEmitterIR::set_long_prod(X64Reg host_sreg, X64Reg tmp1)
{
  MOV(64, R(tmp1), Imm64(0x000000ffffffffffULL));
  AND(64, R(host_sreg), R(tmp1));
  const OpArg prod_reg = m_gpr.GetReg(DSP_REG_PROD_64, false);
  //	g_dsp.r[DSP_REG_PRODL] = (u16)val;
  MOV(64, prod_reg, R(host_sreg));

  m_gpr.PutReg(DSP_REG_PROD_64, true);
}

void DSPEmitterIR::round_long(X64Reg long_acc)
{
  // if (prod & 0x10000) prod = (prod + 0x8000) & ~0xffff;
  // else prod = (prod + 0x7fff) & ~0xffff;
  BT(32, R(long_acc), Imm8(16));
  ADC(64, R(long_acc), Imm32(0x7FFF));
  XOR(16, R(long_acc), R(long_acc));
  // return prod;
}

}  // namespace DSP::JITIR::x64
