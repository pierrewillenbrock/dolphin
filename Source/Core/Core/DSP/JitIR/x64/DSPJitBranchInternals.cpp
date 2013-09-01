// Copyright (C) 2013 Dolphin Project.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 2.0.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License 2.0 for more details.

// A copy of the GPL 2.0 should have been included with the program.
// If not, see http://www.gnu.org/licenses/

// Official SVN repository and contact information can be found at
// http://code.google.com/p/dolphin-emu/

#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP
{
namespace JITIR
{
namespace x64
{

static void CheckExceptionsThunk(DSPCore& dsp)
{
  dsp.CheckExceptions();
}

// Must go out of block if exception is detected
void DSPEmitterIR::checkExceptions(u32 retval, u16 pc)
{
  // no need to check for SR_INT_EXT_ENABLE here. the check in DSPCore
  // should be enough. SR_INT_EXT_ENABLE is only relevant in conjunction
  // with EXP_INT, which can only be generated while DSP is not running
  //(in single core mode. if on thread, the latency still should
  // not hurt)
  const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);

  TEST(16, sr_reg, Imm16(SR_INT_ENABLE));
  FixupBranch int_disabled = J_CC(CC_Z, true);

  // Check for interrupts and exceptions
  TEST(8, M_SDSP_exceptions(), Imm8(0xff));
  FixupBranch skipCheck = J_CC(CC_Z, true);

  MOV(16, M_SDSP_pc(), Imm16(pc));

  DSPJitIRRegCache c(m_gpr);
  m_gpr.PutReg(DSP_REG_SR);
  m_gpr.SaveRegs();
  ABI_CallFunctionP(CheckExceptionsThunk, &m_dsp_core);
  MOV(32, R(EAX), Imm32(retval));
  JMP(m_return_dispatcher, true);
  m_gpr.LoadRegs(false);
  m_gpr.FlushRegs(c, false);

  SetJumpTarget(skipCheck);

  SetJumpTarget(int_disabled);
  m_gpr.PutReg(DSP_REG_SR);
}

// LOOP handling: Loop stack is used to control execution of repeated blocks of
// instructions. Whenever there is value on stack $st2 and current PC is equal
// value at $st2, then value at stack $st3 is decremented. If value is not zero
// then PC is modified with value from call stack $st0. Otherwise values from
// call stack $st0 and both loop stacks $st2 and $st3 are popped and execution
// continues at next opcode.
void DSPEmitterIR::HandleLoop()
{
  MOVZX(32, 16, EAX, M_SDSP_r_st(2));
  MOVZX(32, 16, ECX, M_SDSP_r_st(3));

  TEST(32, R(RCX), R(RCX));
  FixupBranch rLoopCntG = J_CC(CC_E, true);
  CMP(16, R(RAX), Imm16(m_compile_pc - 1));
  FixupBranch rLoopAddrG = J_CC(CC_NE, true);

  SUB(16, M_SDSP_r_st(3), Imm16(1));
  CMP(16, M_SDSP_r_st(3), Imm16(0));

  FixupBranch loadStack = J_CC(CC_LE, true);
  MOVZX(32, 16, ECX, M_SDSP_r_st(0));
  MOV(16, M_SDSP_pc(), R(RCX));
  FixupBranch loopUpdated = J(true);

  SetJumpTarget(loadStack);
  DSPJitIRRegCache c(m_gpr);
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  dsp_reg_stack_pop(StackRegister::Call, tmp1, tmp2, RAX);
  dsp_reg_stack_pop(StackRegister::LoopAddress, tmp1, tmp2, RAX);
  dsp_reg_stack_pop(StackRegister::LoopCounter, tmp1, tmp2, RAX);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
  m_gpr.FlushRegs(c);

  SetJumpTarget(loopUpdated);
  SetJumpTarget(rLoopAddrG);
  SetJumpTarget(rLoopCntG);
}

void DSPEmitterIR::iremit_CheckExceptionsOp(IRInsn const& insn)
{
  checkExceptions(insn.inputs[0].imm, insn.inputs[1].imm);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::CheckExceptionsOp = {
    "CheckExceptionsOp",
    &DSPEmitterIR::iremit_CheckExceptionsOp,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    true};

}  // namespace x64
}  // namespace JITIR
}  // namespace DSP
