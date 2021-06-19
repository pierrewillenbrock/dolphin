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

#include <set>
#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP
{
namespace JITIR
{
namespace x64
{

void DSPEmitterIR::WriteBranchExit(bool keepGpr)
{
  leaveJitCode();

  // Check the result of the sub
  MOV(64, R(RCX), ImmPtr(&m_cycles_left));
  CMP(16, MatR(RCX), Imm16(0));
  J_CC(CC_G, m_reenter_dispatcher);  // was CC_A

  JMP(m_return_dispatcher, true);
}

void DSPEmitterIR::dropAllRegs(IRInsn const& insn)
{
  if (insn.SR.IsSimpleReg())
  {
    if (insn.modifies_SR)
      MOV(16, M_SDSP_r_sr(), insn.SR);
  }
}

void DSPEmitterIR::enterJitCode()
{
}

void DSPEmitterIR::leaveJitCode()
{
}

void DSPEmitterIR::preABICall(IRInsn const& insn, X64Reg returnreg)
{
  std::set<X64Reg> regs;
  if (insn.SR.IsSimpleReg())
    regs.insert(insn.SR.GetSimpleReg());
  // save the registers being marked active
  for (unsigned int i = 0; i < m_cregs.size(); i++)
  {
    if (m_cregs[i].active && m_cregs[i].oparg.GetSimpleReg() != returnreg)
      regs.insert(m_cregs[i].oparg.GetSimpleReg());
  }

  // hardcoding alignment to 16 bytes
  if (regs.size() & 1)
    SUB(64, R(RSP), Imm32(8));

  // std::set is sorted
  for (auto it = regs.begin(); it != regs.end(); it++)
    PUSH(*it);

  leaveJitCode();
}

void DSPEmitterIR::postABICall(IRInsn const& insn, X64Reg returnreg)
{
  enterJitCode();

  if (returnreg != INVALID_REG && returnreg != RAX)
    MOV(64, R(returnreg), R(RAX));

  std::set<X64Reg> regs;
  if (insn.SR.IsSimpleReg())
    regs.insert(insn.SR.GetSimpleReg());
  // restore the registers being marked active
  for (unsigned int i = 0; i < m_cregs.size(); i++)
  {
    if (m_cregs[i].active && m_cregs[i].oparg.GetSimpleReg() != returnreg)
      regs.insert(m_cregs[i].oparg.GetSimpleReg());
  }

  // std::set is sorted
  for (auto it = regs.rbegin(); it != regs.rend(); it++)
    POP(*it);

  // hardcoding alignment to 16 bytes
  if (regs.size() & 1)
    ADD(64, R(RSP), Imm32(8));
}

// LOOP handling: Loop stack is used to control execution of repeated m_blocks of
// instructions. Whenever there is value on stack $st2 and current PC is equal
// value at $st2, then value at stack $st3 is decremented. If value is not zero
// then PC is modified with value from call stack $st0. Otherwise values from
// call stack $st0 and both loop stacks $st2 and $st3 are popped and execution
// continues at next opcode.
void DSPEmitterIR::iremit_HandleLoopUnknownBeginOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  // this cannot be easily predicted. leave it here.
  // but this is the first one that we will try to predict, still
  // easier to do than rets, since all "callers" ([b]loop[i]) have static
  // addresses, and there should be a single loop op for each of the
  // HandleLoop pseudo ops
  MOVZX(32, 16, tmp1, M_SDSP_r_st(0));
  MOV(16, M_SDSP_pc(), R(tmp1));

  dropAllRegs(insn);
  WriteBranchExit();
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::HandleLoopUnknownBeginOp = {
    "HandleLoopUnknownBeginOp",
    &DSPEmitterIR::iremit_HandleLoopUnknownBeginOp,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    true,
    {},
    {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_HandleLoopJumpBeginOp(IRInsn const& insn)
{
  u16 loop_begin = insn.inputs[0].oparg.AsImm16().Imm16();

  CMP(16, M_SDSP_r_st(0), Imm16(loop_begin));
  FixupBranch branchTaken = J_CC(CC_NE, true);

  SetJumpTarget(branchTaken, m_int3_loop);
  insn.branchTaken = branchTaken;
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::HandleLoopJumpBeginOp = {
    "HandleLoopJumpBeginOp",
    &DSPEmitterIR::iremit_HandleLoopJumpBeginOp,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    true,
    {{OpImmAny}},
};

// LOOP handling: Loop stack is used to control execution of repeated m_blocks of
// instructions. Whenever there is value on stack $st2 and current PC is equal
// value at $st2, then value at stack $st3 is decremented. If value is not zero
// then PC is modified with value from call stack $st0. Otherwise values from
// call stack $st0 and both loop stacks $st2 and $st3 are popped and execution
// continues at next opcode.
void DSPEmitterIR::iremit_HandleLoopOp(IRInsn const& insn)
{
  u16 pc = insn.inputs[0].oparg.AsImm16().Imm16();
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();

  CMP(16, M_SDSP_r_st(2), Imm16(pc - 1));
  FixupBranch rLoopAddressExit = J_CC(CC_NE, true);

  MOV(16, R(tmp1), M_SDSP_r_st(3));
  TEST(16, R(tmp1), R(tmp1));
  FixupBranch rLoopCounterExit = J_CC(CC_E, true);

  SUB(16, R(tmp1), Imm16(1));
  MOV(16, M_SDSP_r_st(3), R(tmp1));
  FixupBranch branchTaken = J_CC(CC_G, true);

  SetJumpTarget(branchTaken, m_int3_loop);
  insn.branchTaken = branchTaken;

  // decremented counter, went 0.
  // pop stack, then continue as if nothing happened
  dsp_reg_stack_pop(StackRegister::Call, tmp1, tmp2, tmp3);
  dsp_reg_stack_pop(StackRegister::LoopAddress, tmp1, tmp2, tmp3);
  dsp_reg_stack_pop(StackRegister::LoopCounter, tmp1, tmp2, tmp3);

  // wrong address or counter already was 0
  SetJumpTarget(rLoopCounterExit);
  SetJumpTarget(rLoopAddressExit);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::HandleLoopOp = {
    "HandleLoopOp",
    &DSPEmitterIR::iremit_HandleLoopOp,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    true,
    {{OpImmAny}},
    {},
    {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

static void CheckExceptionsThunk(DSPCore& dsp)
{
  dsp.CheckExceptions();
}

void DSPEmitterIR::iremit_CheckExceptionsUncondOp(IRInsn const& insn)
{
  // pc behaviour here cannot be predicted at compile time.
  MOV(16, M_SDSP_pc(), insn.inputs[0].oparg.AsImm16());

  preABICall(insn);
  ABI_CallFunctionP(CheckExceptionsThunk, &m_dsp_core);
  postABICall(insn);

  WriteBranchExit();
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::CheckExceptionsUncondOp = {
    "CheckExceptionsUncondOp",
    &DSPEmitterIR::iremit_CheckExceptionsUncondOp,
    // the called irq cannot need any SR parts, and when we resume, the
    // old analysis still holds.
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    true,
    {{OpImmAny}}};

void DSPEmitterIR::iremit_CheckExceptionsOp(IRInsn const& insn)
{
  // no need to check for SR_INT_EXT_ENABLE here. the check in DSPCore
  // should be enough. SR_INT_EXT_ENABLE is only relevant in conjunction
  // with EXP_INT, which can only be generated while DSP is not running
  //(in single core mode. if on thread, the latency still should
  // not hurt)
  if ((insn.const_SR & SR_INT_ENABLE) != 0 && (insn.value_SR & SR_INT_ENABLE) == 0)
  {
    // interrupts disabled.
    // dummy jump
    insn.branchTaken = m_unused_jump;
    return;
  }

  FixupBranch int_disabled;
  if ((insn.const_SR & SR_INT_ENABLE) == 0)
  {
    TEST(16, insn.SR, Imm16(SR_INT_ENABLE));
    int_disabled = J_CC(CC_Z, false);
  }

  // Must go out of block if exception is detected
  // Check for interrupts and exceptions
  TEST(8, M_SDSP_exceptions(), Imm8(0xff));
  FixupBranch branchTaken = J_CC(CC_NZ, true);

  SetJumpTarget(branchTaken, m_int3_loop);
  insn.branchTaken = branchTaken;

  if ((insn.const_SR & SR_INT_ENABLE) == 0)
    SetJumpTarget(int_disabled);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::CheckExceptionsOp = {
    "CheckExceptionsOp",
    &DSPEmitterIR::iremit_CheckExceptionsOp,
    SR_INT_ENABLE,
    0x0000,
    0x0000,
    0x0000,
    true,
    {}};

void DSPEmitterIR::iremit_WriteBranchExitOp(IRInsn const& insn)
{
  if (insn.inputs[0].oparg.IsImm())
  {
    u16 dest = insn.inputs[0].oparg.AsImm16().Imm16();

    MOV(16, M_SDSP_pc(), Imm16(dest));
  }
  else
  {
    MOV(16, M_SDSP_pc(), insn.inputs[0].oparg);
  }
  WriteBranchExit();
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::WriteBranchExitOp = {
    "WriteBranchExitOp",
    &DSPEmitterIR::iremit_WriteBranchExitOp,
    0xffff,
    0x0000,
    0x0000,
    0x0000,
    true,
    {{OpImmAny | OpAnyReg}}};

void DSPEmitterIR::iremit_CycleCountExitOp(IRInsn const& insn)
{
  if (insn.inputs[0].oparg.IsImm())
  {
    MOV(16, M_SDSP_pc(), insn.inputs[0].oparg.AsImm16());  // dest
  }
  else
  {
    MOV(16, M_SDSP_pc(), insn.inputs[0].oparg);
  }
  dropAllRegs(insn);  // actually, this is a noop. we don't modify SR.
  WriteBranchExit();
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::CycleCountExitOp = {
    "CycleCountExitOp",
    &DSPEmitterIR::iremit_CycleCountExitOp,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    true,
    {{OpImmAny | OpAnyReg}}};

void DSPEmitterIR::iremit_CycleCountUpdateOp(IRInsn const& insn)
{
  // Decrement m_cycles_left
  SUB(16, M(&m_cycles_left), insn.inputs[0].oparg.AsImm16());  // cyclecount
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::CycleCountUpdateOp = {
    "CycleCountUpdateOp",
    &DSPEmitterIR::iremit_CycleCountUpdateOp,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    true,
    {{OpImmAny}}  // number of cycles done up to now
};

void DSPEmitterIR::iremit_CycleCountUpdateCheckOp(IRInsn const& insn)
{
  // Decrement m_cycles_left
  MOV(64, R(RCX), ImmPtr(&m_cycles_left));
  SUB(16, MatR(RCX), insn.inputs[0].oparg.AsImm16());  // cyclecount

  FixupBranch branchTaken = J_CC(CC_BE, true);

  SetJumpTarget(branchTaken, m_int3_loop);
  insn.branchTaken = branchTaken;
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::CycleCountUpdateCheckOp = {
    "CycleCountUpdateCheckOp",
    &DSPEmitterIR::iremit_CycleCountUpdateCheckOp,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    true,
    {{OpImmAny}}  // number of cycles done up to now
};

}  // namespace x64
}  // namespace JITIR
}  // namespace DSP
