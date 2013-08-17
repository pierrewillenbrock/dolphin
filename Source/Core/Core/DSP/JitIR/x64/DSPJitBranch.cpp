// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/CommonTypes.h"

#include "Core/DSP/DSPAnalyzer.h"
#include "Core/DSP/DSPCore.h"
#include "Core/DSP/DSPTables.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP::JITIR::x64
{
void DSPEmitterIR::ReJitConditional(const UDSPInstruction opc,
                                    void (DSPEmitterIR::*conditional_fn)(UDSPInstruction,
                                                                         X64Reg tmp1, X64Reg tmp2),
                                    X64Reg tmp1, X64Reg tmp2)
{
  u8 cond = opc & 0xf;
  if (cond == 0xf)  // Always true.
  {
    (this->*conditional_fn)(opc, tmp1, tmp2);
    return;
  }

  m_gpr.ReadReg(DSP_REG_SR, EAX, RegisterExtension::None);

  switch (cond)
  {
  case 0x0:  // GE - Greater Equal
  case 0x1:  // L - Less
    LEA(16, EDX, MScaled(EAX, SCALE_4, 0));
    XOR(16, R(EAX), R(EDX));
    TEST(16, R(EAX), Imm16(8));
    break;
  case 0x2:  // G - Greater
  case 0x3:  // LE - Less Equal
    LEA(16, EDX, MScaled(EAX, SCALE_4, 0));
    XOR(16, R(EAX), R(EDX));
    ADD(16, R(EAX), R(EAX));
    OR(16, R(EAX), R(EDX));
    TEST(16, R(EAX), Imm16(0x10));
    break;
  case 0x4:  // NZ - Not Zero
  case 0x5:  // Z - Zero
    TEST(16, R(EAX), Imm16(SR_ARITH_ZERO));
    break;
  case 0x6:  // NC - Not carry
  case 0x7:  // C - Carry
    TEST(16, R(EAX), Imm16(SR_CARRY));
    break;
  case 0x8:  // ? - Not over s32
  case 0x9:  // ? - Over s32
    TEST(16, R(EAX), Imm16(SR_OVER_S32));
    break;
  case 0xa:  // ?
  case 0xb:  // ?
    LEA(16, EDX, MRegSum(EAX, EAX));
    OR(16, R(EAX), R(EDX));
    SHL(16, R(EDX), Imm8(3));
    NOT(16, R(EAX));
    OR(16, R(EAX), R(EDX));
    TEST(16, R(EAX), Imm16(0x20));
    break;
  case 0xc:  // LNZ  - Logic Not Zero
  case 0xd:  // LZ - Logic Zero
    TEST(16, R(EAX), Imm16(SR_LOGIC_ZERO));
    break;
  case 0xe:  // 0 - Overflow
    TEST(16, R(EAX), Imm16(SR_OVERFLOW));
    break;
  }
  DSPJitIRRegCache c1(m_gpr);
  FixupBranch skip_code =
      cond == 0xe ? J_CC(CC_E, true) : J_CC((CCFlags)(CC_NE - (cond & 1)), true);
  (this->*conditional_fn)(opc, tmp1, tmp2);
  m_gpr.FlushRegs(c1);
  SetJumpTarget(skip_code);
}

void DSPEmitterIR::WriteBranchExit()
{
  DSPJitIRRegCache c(m_gpr);
  m_gpr.SaveRegs();
  if (m_dsp_core.DSPState().GetAnalyzer().IsIdleSkip(m_start_address))
  {
    MOV(16, R(EAX), Imm16(0x1000));
  }
  else
  {
    MOV(16, R(EAX), Imm16(m_block_size[m_start_address]));
  }
  JMP(m_return_dispatcher, true);
  m_gpr.LoadRegs(false);
  m_gpr.FlushRegs(c, false);
}

void DSPEmitterIR::r_jcc(const UDSPInstruction opc, X64Reg tmp1, X64Reg tmp2)
{
  u16 dest = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  MOV(16, M_SDSP_pc(), Imm16(dest));
  DSPJitIRRegCache c(m_gpr);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
  WriteBranchExit();
  m_gpr.FlushRegs(c, false);
}
// Generic jmp implementation
// Jcc addressA
// 0000 0010 1001 cccc
// aaaa aaaa aaaa aaaa
// Jump to addressA if condition cc has been met. Set program counter to
// address represented by value that follows this "jmp" instruction.
// NOTE: Cannot use FallBackToInterpreter(opc) here because of the need to write branch exit
void DSPEmitterIR::jcc(const UDSPInstruction opc)
{
  MOV(16, M_SDSP_pc(), Imm16(m_compile_pc + 2));
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  ReJitConditional(opc, &DSPEmitterIR::r_jcc, tmp1, tmp2);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

void DSPEmitterIR::r_jmprcc(const UDSPInstruction opc, X64Reg tmp1, X64Reg tmp2)
{
  u8 reg = (opc >> 5) & 0x7;
  // reg can only be DSP_REG_ARx and DSP_REG_IXx now,
  // no need to handle DSP_REG_STx.
  m_gpr.ReadReg(reg, RAX, RegisterExtension::None);
  MOV(16, M_SDSP_pc(), R(EAX));
  DSPJitIRRegCache c(m_gpr);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
  WriteBranchExit();
  m_gpr.FlushRegs(c, false);
}
// Generic jmpr implementation
// JMPcc $R
// 0001 0111 rrr0 cccc
// Jump to address; set program counter to a value from register $R.
// NOTE: Cannot use FallBackToInterpreter(opc) here because of the need to write branch exit
void DSPEmitterIR::jmprcc(const UDSPInstruction opc)
{
  MOV(16, M_SDSP_pc(), Imm16(m_compile_pc + 1));
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  ReJitConditional(opc, &DSPEmitterIR::r_jmprcc, tmp1, tmp2);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

void DSPEmitterIR::r_call(const UDSPInstruction opc, X64Reg tmp1, X64Reg tmp2)
{
  MOV(16, R(DX), Imm16(m_compile_pc + 2));
  dsp_reg_store_stack(StackRegister::Call, RDX, tmp1, tmp2, RAX);
  u16 dest = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  MOV(16, M_SDSP_pc(), Imm16(dest));
  DSPJitIRRegCache c(m_gpr);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
  WriteBranchExit();
  m_gpr.FlushRegs(c, false);
}
// Generic call implementation
// CALLcc addressA
// 0000 0010 1011 cccc
// aaaa aaaa aaaa aaaa
// Call function if condition cc has been met. Push program counter of
// instruction following "call" to $st0. Set program counter to address
// represented by value that follows this "call" instruction.
// NOTE: Cannot use FallBackToInterpreter(opc) here because of the need to write branch exit
void DSPEmitterIR::call(const UDSPInstruction opc)
{
  MOV(16, M_SDSP_pc(), Imm16(m_compile_pc + 2));
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  ReJitConditional(opc, &DSPEmitterIR::r_call, tmp1, tmp2);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

void DSPEmitterIR::r_callr(const UDSPInstruction opc, X64Reg tmp1, X64Reg tmp2)
{
  u8 reg = (opc >> 5) & 0x7;
  MOV(16, R(DX), Imm16(m_compile_pc + 1));
  dsp_reg_store_stack(StackRegister::Call, RDX, tmp1, tmp2, RAX);
  m_gpr.ReadReg(reg, RAX, RegisterExtension::None);
  MOV(16, M_SDSP_pc(), R(EAX));
  DSPJitIRRegCache c(m_gpr);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
  WriteBranchExit();
  m_gpr.FlushRegs(c, false);
}
// Generic callr implementation
// CALLRcc $R
// 0001 0111 rrr1 cccc
// Call function if condition cc has been met. Push program counter of
// instruction following "call" to call stack $st0. Set program counter to
// register $R.
// NOTE: Cannot use FallBackToInterpreter(opc) here because of the need to write branch exit
void DSPEmitterIR::callr(const UDSPInstruction opc)
{
  MOV(16, M_SDSP_pc(), Imm16(m_compile_pc + 1));
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  ReJitConditional(opc, &DSPEmitterIR::r_callr, tmp1, tmp2);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

void DSPEmitterIR::r_ifcc(const UDSPInstruction opc, X64Reg tmp1, X64Reg tmp2)
{
  MOV(16, M_SDSP_pc(), Imm16(m_compile_pc + 1));
}
// Generic if implementation
// IFcc
// 0000 0010 0111 cccc
// Execute following opcode if the condition has been met.
// NOTE: Cannot use FallBackToInterpreter(opc) here because of the need to write branch exit
void DSPEmitterIR::ifcc(const UDSPInstruction opc)
{
  const auto& state = m_dsp_core.DSPState();
  const u16 address = m_compile_pc + 1;
  const DSPOPCTemplate* const op_template = GetOpTemplate(state.ReadIMEM(address));

  MOV(16, M_SDSP_pc(), Imm16(address + op_template->size));
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  ReJitConditional(opc, &DSPEmitterIR::r_ifcc, tmp1, tmp2);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
  WriteBranchExit();
}

void DSPEmitterIR::r_ret(const UDSPInstruction opc, X64Reg tmp1, X64Reg tmp2)
{
  dsp_reg_load_stack(StackRegister::Call, RDX, tmp1, tmp2, RAX);
  MOV(16, M_SDSP_pc(), R(DX));
  DSPJitIRRegCache c(m_gpr);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
  WriteBranchExit();
  m_gpr.FlushRegs(c, false);
}

// Generic ret implementation
// RETcc
// 0000 0010 1101 cccc
// Return from subroutine if condition cc has been met. Pops stored PC
// from call stack $st0 and sets $pc to this location.
// NOTE: Cannot use FallBackToInterpreter(opc) here because of the need to write branch exit
void DSPEmitterIR::ret(const UDSPInstruction opc)
{
  MOV(16, M_SDSP_pc(), Imm16(m_compile_pc + 1));
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  ReJitConditional(opc, &DSPEmitterIR::r_ret, tmp1, tmp2);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// RTI
// 0000 0010 1111 1111
// Return from exception. Pops stored status register $sr from data stack
// $st1 and program counter PC from call stack $st0 and sets $pc to this
// location.
void DSPEmitterIR::rti(const UDSPInstruction opc)
{
  //	g_dsp.r[DSP_REG_SR] = dsp_reg_load_stack(StackRegister::Data);
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  dsp_reg_load_stack(StackRegister::Data, RDX, tmp1, tmp2, RAX);
  m_gpr.WriteReg(DSP_REG_SR, R(RDX));
  //	g_dsp.pc = dsp_reg_load_stack(StackRegister::Call);
  dsp_reg_load_stack(StackRegister::Call, RDX, tmp1, tmp2, RAX);
  MOV(16, M_SDSP_pc(), R(DX));
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// HALT
// 0000 0000 0020 0001
// Stops execution of DSP code. Sets bit DSP_CR_HALT in register DREG_CR.
void DSPEmitterIR::halt(const UDSPInstruction)
{
  OR(16, M_SDSP_cr(), Imm16(CR_HALT));
  SUB(16, M_SDSP_pc(), Imm16(1));
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

// LOOP $R
// 0000 0000 010r rrrr
// Repeatedly execute following opcode until counter specified by value
// from register $R reaches zero. Each execution decrement counter. Register
// $R remains unchanged. If register $R is set to zero at the beginning of loop
// then looped instruction will not get executed.
// Actually, this instruction simply prepares the loop stacks for the above.
// The looping hardware takes care of the rest.
void DSPEmitterIR::loop(const UDSPInstruction opc)
{
  u16 reg = opc & 0x1f;
  //	u16 cnt = g_dsp.r[reg];
  // todo: check if we can use normal variant here
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  dsp_op_read_reg_dont_saturate(reg, RDX, RegisterExtension::Zero, tmp1, tmp2, RAX);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
  u16 loop_pc = m_compile_pc + 1;

  TEST(16, R(EDX), R(EDX));
  DSPJitIRRegCache c(m_gpr);
  FixupBranch cnt = J_CC(CC_Z, true);
  X64Reg tmp3 = m_gpr.GetFreeXReg();
  X64Reg tmp4 = m_gpr.GetFreeXReg();
  dsp_reg_store_stack(StackRegister::LoopCounter, RDX, tmp3, tmp4, RAX);
  MOV(16, R(RDX), Imm16(m_compile_pc + 1));
  dsp_reg_store_stack(StackRegister::Call, RDX, tmp3, tmp4, RAX);
  MOV(16, R(RDX), Imm16(loop_pc));
  dsp_reg_store_stack(StackRegister::LoopAddress, RDX, tmp3, tmp4, RAX);
  m_gpr.PutXReg(tmp4);
  m_gpr.PutXReg(tmp3);
  m_gpr.FlushRegs(c);
  MOV(16, M_SDSP_pc(), Imm16(m_compile_pc + 1));
  FixupBranch exit = J(true);

  SetJumpTarget(cnt);
  // dsp_skip_inst();
  const auto& state = m_dsp_core.DSPState();
  MOV(16, M_SDSP_pc(), Imm16(loop_pc + GetOpTemplate(state.ReadIMEM(loop_pc))->size));
  WriteBranchExit();
  m_gpr.FlushRegs(c, false);
  SetJumpTarget(exit);
}

// LOOPI #I
// 0001 0000 iiii iiii
// Repeatedly execute following opcode until counter specified by
// immediate value I reaches zero. Each execution decrement counter. If
// immediate value I is set to zero at the beginning of loop then looped
// instruction will not get executed.
// Actually, this instruction simply prepares the loop stacks for the above.
// The looping hardware takes care of the rest.
void DSPEmitterIR::loopi(const UDSPInstruction opc)
{
  u16 cnt = opc & 0xff;
  u16 loop_pc = m_compile_pc + 1;

  if (cnt)
  {
    X64Reg tmp1 = m_gpr.GetFreeXReg();
    X64Reg tmp2 = m_gpr.GetFreeXReg();
    MOV(16, R(RDX), Imm16(m_compile_pc + 1));
    dsp_reg_store_stack(StackRegister::Call, RDX, tmp1, tmp2, RAX);
    MOV(16, R(RDX), Imm16(loop_pc));
    dsp_reg_store_stack(StackRegister::LoopAddress, RDX, tmp1, tmp2, RAX);
    MOV(16, R(RDX), Imm16(cnt));
    dsp_reg_store_stack(StackRegister::LoopCounter, RDX, tmp1, tmp2, RAX);
    m_gpr.PutXReg(tmp2);
    m_gpr.PutXReg(tmp1);

    MOV(16, M_SDSP_pc(), Imm16(m_compile_pc + 1));
  }
  else
  {
    // dsp_skip_inst();
    const auto& state = m_dsp_core.DSPState();
    MOV(16, M_SDSP_pc(), Imm16(loop_pc + GetOpTemplate(state.ReadIMEM(loop_pc))->size));
    WriteBranchExit();
  }
}

// BLOOP $R, addrA
// 0000 0000 011r rrrr
// aaaa aaaa aaaa aaaa
// Repeatedly execute block of code starting at following opcode until
// counter specified by value from register $R reaches zero. Block ends at
// specified address addrA inclusive, ie. opcode at addrA is the last opcode
// included in loop. Counter is pushed on loop stack $st3, end of block address
// is pushed on loop stack $st2 and repeat address is pushed on call stack $st0.
// Up to 4 nested loops are allowed.
void DSPEmitterIR::bloop(const UDSPInstruction opc)
{
  const u16 reg = opc & 0x1f;
  //	u16 cnt = g_dsp.r[reg];
  // todo: check if we can use normal variant here
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  dsp_op_read_reg_dont_saturate(reg, RDX, RegisterExtension::Zero, tmp1, tmp2, RAX);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
  u16 loop_pc = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);

  TEST(16, R(EDX), R(EDX));
  DSPJitIRRegCache c(m_gpr);
  FixupBranch cnt = J_CC(CC_Z, true);
  X64Reg tmp3 = m_gpr.GetFreeXReg();
  X64Reg tmp4 = m_gpr.GetFreeXReg();
  dsp_reg_store_stack(StackRegister::LoopCounter, RDX, tmp3, tmp4, RAX);
  MOV(16, R(RDX), Imm16(m_compile_pc + 2));
  dsp_reg_store_stack(StackRegister::Call, RDX, tmp3, tmp4, RAX);
  MOV(16, R(RDX), Imm16(loop_pc));
  dsp_reg_store_stack(StackRegister::LoopAddress, RDX, tmp3, tmp4, RAX);
  MOV(16, M_SDSP_pc(), Imm16(m_compile_pc + 2));
  m_gpr.PutXReg(tmp4);
  m_gpr.PutXReg(tmp3);
  m_gpr.FlushRegs(c, true);
  FixupBranch exit = J(true);

  SetJumpTarget(cnt);
  // g_dsp.pc = loop_pc;
  // dsp_skip_inst();
  const auto& state = m_dsp_core.DSPState();
  MOV(16, M_SDSP_pc(), Imm16(loop_pc + GetOpTemplate(state.ReadIMEM(loop_pc))->size));
  WriteBranchExit();
  m_gpr.FlushRegs(c, false);
  SetJumpTarget(exit);
}

// BLOOPI #I, addrA
// 0001 0001 iiii iiii
// aaaa aaaa aaaa aaaa
// Repeatedly execute block of code starting at following opcode until
// counter specified by immediate value I reaches zero. Block ends at specified
// address addrA inclusive, ie. opcode at addrA is the last opcode included in
// loop. Counter is pushed on loop stack $st3, end of block address is pushed
// on loop stack $st2 and repeat address is pushed on call stack $st0. Up to 4
// nested loops are allowed.
void DSPEmitterIR::bloopi(const UDSPInstruction opc)
{
  const auto& state = m_dsp_core.DSPState();
  const u16 cnt = opc & 0xff;
  //	u16 loop_pc = dsp_fetch_code();
  const u16 loop_pc = state.ReadIMEM(m_compile_pc + 1);

  if (cnt != 0)
  {
    X64Reg tmp1 = m_gpr.GetFreeXReg();
    X64Reg tmp2 = m_gpr.GetFreeXReg();
    MOV(16, R(RDX), Imm16(m_compile_pc + 2));
    dsp_reg_store_stack(StackRegister::Call, RDX, tmp1, tmp2, RAX);
    MOV(16, R(RDX), Imm16(loop_pc));
    dsp_reg_store_stack(StackRegister::LoopAddress, RDX, tmp1, tmp2, RAX);
    MOV(16, R(RDX), Imm16(cnt));
    dsp_reg_store_stack(StackRegister::LoopCounter, RDX, tmp1, tmp2, RAX);
    m_gpr.PutXReg(tmp2);
    m_gpr.PutXReg(tmp1);

    MOV(16, M_SDSP_pc(), Imm16(m_compile_pc + 2));
  }
  else
  {
    // g_dsp.pc = loop_pc;
    // dsp_skip_inst();
    MOV(16, M_SDSP_pc(), Imm16(loop_pc + GetOpTemplate(state.ReadIMEM(loop_pc))->size));
    WriteBranchExit();
  }
}

}  // namespace DSP::JITIR::x64
