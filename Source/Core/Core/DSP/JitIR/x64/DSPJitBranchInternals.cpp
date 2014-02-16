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

#include "Core/DSP/DSPAnalyzer.h"
#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP
{
namespace JITIR
{
namespace x64
{

static constexpr size_t DSP_IDLE_SKIP_CYCLES = 0x1000;

void DSPEmitterIR::WriteBranchExit(u16 execd_cycles, bool keepGpr)
{
  m_gpr.SaveRegs();
  if (m_dsp_core.DSPState().GetAnalyzer().IsIdleSkip(m_start_address))
  {
    MOV(16, R(EAX), Imm16(DSP_IDLE_SKIP_CYCLES));
  }
  else
  {
    MOV(16, R(EAX), Imm16(execd_cycles));
  }
  JMP(m_return_dispatcher, true);
  if (keepGpr)
    m_gpr.LoadRegs(false);
}

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
void DSPEmitterIR::iremit_HandleLoopOp(IRInsn const& insn)
{
  u16 pc = insn.inputs[0].imm;
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  MOV(16, M_SDSP_pc(), Imm16(pc));

  MOVZX(32, 16, EAX, M_SDSP_r_st(2));
  TEST(32, R(EAX), R(EAX));
  FixupBranch rLoopAddressExit = J_CC(CC_LE, true);

  MOVZX(32, 16, EAX, M_SDSP_r_st(3));
  TEST(32, R(EAX), R(EAX));
  FixupBranch rLoopCounterExit = J_CC(CC_LE, true);

  MOVZX(32, 16, EAX, M_SDSP_r_st(2));
  MOVZX(32, 16, ECX, M_SDSP_r_st(3));

  TEST(32, R(RCX), R(RCX));
  FixupBranch rLoopCntG = J_CC(CC_E, true);
  CMP(16, R(RAX), Imm16(pc - 1));
  FixupBranch rLoopAddrG = J_CC(CC_NE, true);

  SUB(16, M_SDSP_r_st(3), Imm16(1));
  CMP(16, M_SDSP_r_st(3), Imm16(0));

  FixupBranch loadStack = J_CC(CC_LE, true);
  MOVZX(32, 16, ECX, M_SDSP_r_st(0));
  MOV(16, M_SDSP_pc(), R(RCX));
  FixupBranch loopUpdated = J(true);

  SetJumpTarget(loadStack);
  DSPJitIRRegCache c1(m_gpr);
  dsp_reg_stack_pop(StackRegister::Call, tmp1, tmp2, RAX);
  dsp_reg_stack_pop(StackRegister::LoopAddress, tmp1, tmp2, RAX);
  dsp_reg_stack_pop(StackRegister::LoopCounter, tmp1, tmp2, RAX);
  m_gpr.FlushRegs(c1);

  SetJumpTarget(loopUpdated);
  SetJumpTarget(rLoopAddrG);
  SetJumpTarget(rLoopCntG);

  DSPJitIRRegCache c2(m_gpr);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
  WriteBranchExit(insn.cycle_count);
  m_gpr.FlushRegs(c2, false);

  SetJumpTarget(rLoopAddressExit);
  SetJumpTarget(rLoopCounterExit);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::HandleLoopOp = {
    "HandleLoopOp",
    &DSPEmitterIR::iremit_HandleLoopOp,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    true,
    {},
    {},
    {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_UpdatePCOp(IRInsn const& insn)
{
  MOV(16, M_SDSP_pc(), Imm16(insn.inputs[0].imm));
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::UpdatePCOp = {
    "UpdatePCOp", &DSPEmitterIR::iremit_UpdatePCOp, 0x0000, 0x0000, 0x0000, 0x0000, true};

void DSPEmitterIR::iremit_CheckExceptionsOp(IRInsn const& insn)
{
  checkExceptions(insn.cycle_count, insn.inputs[0].imm);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::CheckExceptionsOp = {
    "CheckExceptionsOp",
    &DSPEmitterIR::iremit_CheckExceptionsOp,
    SR_INT_ENABLE,
    0x0000,
    0x0000,
    0x0000,
    true,
    {{OpImmAny}}};

}  // namespace x64
}  // namespace JITIR
}  // namespace DSP
