// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/CommonTypes.h"

#include "Core/DSP/DSPCore.h"
#include "Core/DSP/DSPTables.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP::JITIR::x64
{
static u16 branch_needs_SR[16] = {SR_SIGN | SR_OVERFLOW,
                                  SR_SIGN | SR_OVERFLOW,
                                  SR_SIGN | SR_ARITH_ZERO | SR_OVERFLOW,
                                  SR_SIGN | SR_ARITH_ZERO | SR_OVERFLOW,
                                  SR_ARITH_ZERO,
                                  SR_ARITH_ZERO,
                                  SR_CARRY,
                                  SR_CARRY,
                                  SR_OVER_S32,
                                  SR_OVER_S32,
                                  SR_TOP2BITS | SR_OVER_S32 | SR_OVERFLOW,
                                  SR_TOP2BITS | SR_OVER_S32 | SR_OVERFLOW,
                                  SR_LOGIC_ZERO,
                                  SR_LOGIC_ZERO,
                                  SR_OVERFLOW,
                                  0x0000};

void DSPEmitterIR::IRReJitConditional(
    u8 cond, DSPEmitterIR::IRInsn const& insn,
    void (DSPEmitterIR::*conditional_fn)(DSPEmitterIR::IRInsn const& insn), OpArg const& sr_reg,
    bool negate, X64Reg tmp1, X64Reg tmp2)
{
  if (cond == 0xf && !negate)
  {  // Always true.
    (this->*conditional_fn)(insn);
    return;
  }
  if (cond == 0xf && negate)
  {  // Never true
    return;
  }

  MOV(16, R(tmp2), sr_reg);

  switch (cond)
  {
  case 0x0:  // GE - Greater Equal
  case 0x1:  // L - Less
    LEA(16, tmp1, MScaled(tmp2, SCALE_4, 0));
    XOR(16, R(tmp2), R(tmp1));
    TEST(16, R(tmp2), Imm16(8));
    break;
  case 0x2:  // G - Greater
  case 0x3:  // LE - Less Equal
    LEA(16, tmp1, MScaled(tmp2, SCALE_4, 0));
    XOR(16, R(tmp2), R(tmp1));
    ADD(16, R(tmp2), R(tmp2));
    OR(16, R(tmp2), R(tmp1));
    TEST(16, R(tmp2), Imm16(0x10));
    break;
  case 0x4:  // NZ - Not Zero
  case 0x5:  // Z - Zero
    TEST(16, R(tmp2), Imm16(SR_ARITH_ZERO));
    break;
  case 0x6:  // NC - Not carry
  case 0x7:  // C - Carry
    TEST(16, R(tmp2), Imm16(SR_CARRY));
    break;
  case 0x8:  // ? - Not over s32
  case 0x9:  // ? - Over s32
    TEST(16, R(tmp2), Imm16(SR_OVER_S32));
    break;
  case 0xa:  // ?
  case 0xb:  // ?
    LEA(16, tmp1, MRegSum(tmp2, tmp2));
    OR(16, R(tmp2), R(tmp1));
    SHL(16, R(tmp1), Imm8(3));
    NOT(16, R(tmp2));
    OR(16, R(tmp2), R(tmp1));
    TEST(16, R(tmp2), Imm16(0x20));
    break;
  case 0xc:  // LNZ  - Logic Not Zero
  case 0xd:  // LZ - Logic Zero
    TEST(16, R(tmp2), Imm16(SR_LOGIC_ZERO));
    break;
  case 0xe:  // 0 - Overflow
    TEST(16, R(tmp2), Imm16(SR_OVERFLOW));
    break;
  }
  bool equal = (cond == 0xe) || (cond & 1);
  if (negate)
    equal = !equal;
  FixupBranch skip_code = equal ? J_CC(CC_E, true) : J_CC(CC_NE, true);
  // actually, this is guaranteed to emit code that does not return
  (this->*conditional_fn)(insn);
  SetJumpTarget(skip_code);
}

// Generic jmp implementation
// Jcc addressA
// 0000 0010 1001 cccc
// aaaa aaaa aaaa aaaa
// Jump to addressA if condition cc has been met. Set program counter to
// address represented by value that follows this "jmp" instruction.
void DSPEmitterIR::ir_jcc(const UDSPInstruction opc)
{
  u8 cc = opc & 0xf;
  u16 dest = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&JmpOp,
              {IROp::Imm(cc),
               IROp::Imm(dest),               // address if true
               IROp::Imm(m_compile_pc + 2)},  // address if false
              IROp::None(),
              {},
              branch_needs_SR[cc],
              0x0000,
              0x0000,
              0x0000};
  ir_add_op(p);
}

// Generic jmpr implementation
// JMPcc $R
// 0001 0111 rrr0 cccc
// Jump to address; set program counter to a value from register $R.
void DSPEmitterIR::ir_jmprcc(const UDSPInstruction opc)
{
  u8 cc = opc & 0xf;
  u8 reg = (opc >> 5) & 0x7;
  IRInsn p = {&JmpOp,
              {IROp::Imm(cc),
               IROp::R(reg),                  // address if true
               IROp::Imm(m_compile_pc + 2)},  // address if false
              IROp::None(),
              {},
              branch_needs_SR[cc],
              0x0000,
              0x0000,
              0x0000};
  ir_add_op(p);
}

// Generic call implementation
// CALLcc addressA
// 0000 0010 1011 cccc
// aaaa aaaa aaaa aaaa
// Call function if condition cc has been met. Push program counter of
// instruction following "call" to $st0. Set program counter to address
// represented by value that follows this "call" instruction.
void DSPEmitterIR::ir_call(const UDSPInstruction opc)
{
  u8 cc = opc & 0xf;
  u16 dest = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&CallOp,
              {IROp::Imm(cc),
               IROp::Imm(dest),               // call if true
               IROp::Imm(m_compile_pc + 2)},  // return addr, addr if false
              IROp::None(),
              {},
              branch_needs_SR[cc],
              0x0000,
              0x0000,
              0x0000};
  ir_add_op(p);
}

// Generic callr implementation
// CALLRcc $R
// 0001 0111 rrr1 cccc
// Call function if condition cc has been met. Push program counter of
// instruction following "call" to call stack $st0. Set program counter to
// register $R.
void DSPEmitterIR::ir_callr(const UDSPInstruction opc)
{
  u8 cc = opc & 0xf;
  u8 reg = (opc >> 5) & 0x7;
  IRInsn p = {&CallOp,
              {IROp::Imm(cc),
               IROp::R(reg),                  // call if true
               IROp::Imm(m_compile_pc + 1)},  // return addr, addr if false
              IROp::None(),
              {},
              branch_needs_SR[cc],
              0x0000,
              0x0000,
              0x0000};
  ir_add_op(p);
}

// Generic if implementation
// IFcc
// 0000 0010 0111 cccc
// Execute following opcode if the condition has been met.
void DSPEmitterIR::ir_ifcc(const UDSPInstruction opc)
{
  u8 cc = opc & 0xf;
  if (cc == 0xf)
    return;
  u16 dest = m_compile_pc + 1;
  IRInsn p = {&JmpOp,
              {IROp::Imm(cc | 0x80),  // invert logic
               IROp::Imm(dest +       // address if false
                         GetOpTemplate(m_dsp_core.DSPState().ReadIMEM(dest))->size),
               IROp::Imm(dest)},  // address if true
              IROp::None(),
              {},
              branch_needs_SR[cc],
              0x0000,
              0x0000,
              0x0000};
  ir_add_op(p);
}

// Generic ret implementation
// RETcc
// 0000 0010 1101 cccc
// Return from subroutine if condition cc has been met. Pops stored PC
// from call stack $st0 and sets $pc to this location.
void DSPEmitterIR::ir_ret(const UDSPInstruction opc)
{
  u8 cc = opc & 0xf;
  IRInsn p = {&RetOp,
              {IROp::Imm(cc), IROp::Imm(m_compile_pc + 1)},
              IROp::None(),
              {},
              branch_needs_SR[cc],
              0x0000,
              0x0000,
              0x0000};
  ir_add_op(p);
}

// RTI
// 0000 0010 1111 1111
// Return from exception. Pops stored status register $sr from data stack
// $st1 and program counter PC from call stack $st0 and sets $pc to this
// location.
void DSPEmitterIR::ir_rti(const UDSPInstruction opc)
{
  IRInsn p = {
      &RtiOp,
  };
  ir_add_op(p);
}

// HALT
// 0000 0000 0020 0001
// Stops execution of DSP code. Sets bit DSP_CR_HALT in register DREG_CR.
void DSPEmitterIR::ir_halt(const UDSPInstruction opc)
{
  IRInsn p = {
      &HaltOp,
  };
  ir_add_op(p);
}

// LOOP $R
// 0000 0000 010r rrrr
// Repeatedly execute following opcode until counter specified by value
// from register $R reaches zero. Each execution decrement counter. Register
// $R remains unchanged. If register $R is set to zero at the beginning of loop
// then looped instruction will not get executed.
// Actually, this instruction simply prepares the loop stacks for the above.
// The looping hardware takes care of the rest.
void DSPEmitterIR::ir_loop(const UDSPInstruction opc)
{
  u16 reg = opc & 0x1f;
  u16 loop_pc = m_compile_pc + 1;
  IRInsn p = {
      &LoopOp, {IROp::R(reg), IROp::Imm(m_compile_pc + 1), IROp::Imm(loop_pc)}, IROp::None()};
  ir_add_op(p);
}

// LOOPI #I
// 0001 0000 iiii iiii
// Repeatedly execute following opcode until counter specified by
// immediate value I reaches zero. Each execution decrement counter. If
// immediate value I is set to zero at the beginning of loop then looped
// instruction will not get executed.
// Actually, this instruction simply prepares the loop stacks for the above.
// The looping hardware takes care of the rest.
void DSPEmitterIR::ir_loopi(const UDSPInstruction opc)
{
  u16 cnt = opc & 0xff;
  u16 loop_pc = m_compile_pc + 1;
  IRInsn p = {
      &LoopOp, {IROp::Imm(cnt), IROp::Imm(m_compile_pc + 1), IROp::Imm(loop_pc)}, IROp::None()};
  ir_add_op(p);
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
void DSPEmitterIR::ir_bloop(const UDSPInstruction opc)
{
  u16 reg = opc & 0x1f;
  u16 loop_pc = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {
      &LoopOp, {IROp::R(reg), IROp::Imm(m_compile_pc + 2), IROp::Imm(loop_pc)}, IROp::None()};
  ir_add_op(p);
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
void DSPEmitterIR::ir_bloopi(const UDSPInstruction opc)
{
  const auto &state = m_dsp_core.DSPState();
  u16 cnt = opc & 0xff;
  u16 loop_pc = state.ReadIMEM(m_compile_pc + 1);
  IRInsn p = {
      &LoopOp, {IROp::Imm(cnt), IROp::Imm(m_compile_pc + 2), IROp::Imm(loop_pc)}, IROp::None()};
  ir_add_op(p);
}

void DSPEmitterIR::iremit_LoopOp(IRInsn const& insn)
{
  const auto& state = m_dsp_core.DSPState();
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();

  u16 loop_start = insn.inputs[1].oparg.AsImm16().Imm16();
  u16 loop_end = insn.inputs[2].oparg.AsImm16().Imm16();
  if (insn.inputs[0].oparg.IsImm())
  {
    u16 cnt = insn.inputs[0].oparg.AsImm16().Imm16();
    if (cnt)
    {
      dsp_reg_store_stack(StackRegister::Call, Imm16(loop_start), tmp1, tmp2, tmp3);
      dsp_reg_store_stack(StackRegister::LoopAddress, Imm16(loop_end), tmp1, tmp2, tmp3);
      dsp_reg_store_stack(StackRegister::LoopCounter, Imm16(cnt), tmp1, tmp2, tmp3);
    }
    else
    {
      MOV(16, M_SDSP_pc(), Imm16(loop_end + GetOpTemplate(state.ReadIMEM(loop_end))->size));
      DSPJitIRRegCache c(m_gpr);
      dropAllRegs(insn);
      WriteBranchExit(insn.cycle_count);
      m_gpr.FlushRegs(c, false);
    }
  }
  else
  {
    TEST(16, insn.inputs[0].oparg, insn.inputs[0].oparg);
    FixupBranch cnt = J_CC(CC_Z, true);
    dsp_reg_store_stack(StackRegister::LoopCounter, insn.inputs[0].oparg, tmp1, tmp2, tmp3);
    dsp_reg_store_stack(StackRegister::Call, Imm16(loop_start), tmp1, tmp2, tmp3);
    dsp_reg_store_stack(StackRegister::LoopAddress, Imm16(loop_end), tmp1, tmp2, tmp3);
    FixupBranch exit = J(true);

    SetJumpTarget(cnt);
    MOV(16, M_SDSP_pc(), Imm16(loop_end + GetOpTemplate(dsp_imem_read(loop_end))->size));
    DSPJitIRRegCache c1(m_gpr);
    dropAllRegs(insn);
    WriteBranchExit(insn.cycle_count);
    m_gpr.FlushRegs(c1, false);
    SetJumpTarget(exit);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LoopOp = {
    "LoopOp", &DSPEmitterIR::iremit_LoopOp,
    0x0000,   0x0000,
    0x0000,   0x0000,
    true,     {{OpAnyReg | OpImm | ExtendZero16 | Clobbered}, {OpImmAny}, {OpImmAny}},
    {},       {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_HaltOp(IRInsn const& insn)
{
  OR(16, M_SDSP_cr(), Imm16(CR_HALT));
  SUB(16, M_SDSP_pc(), Imm16(1));
  DSPJitIRRegCache c(m_gpr);
  dropAllRegs(insn);
  WriteBranchExit(insn.cycle_count);
  m_gpr.FlushRegs(c, false);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::HaltOp = {
    "HaltOp", &DSPEmitterIR::iremit_HaltOp, 0x0000, 0x0000, 0x0000, 0x0000, true};

void DSPEmitterIR::irr_ret(DSPEmitterIR::IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();
  X64Reg tmp4 = insn.temps[3].oparg.GetSimpleReg();
  dsp_reg_load_stack(StackRegister::Call, tmp4, tmp1, tmp2, tmp3);
  MOV(16, M_SDSP_pc(), R(tmp4));
  DSPJitIRRegCache c(m_gpr);
  dropAllRegs(insn);
  WriteBranchExit(insn.cycle_count);
  m_gpr.FlushRegs(c, false);
}

void DSPEmitterIR::iremit_RetOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  uint8_t cc = insn.inputs[0].oparg.AsImm8().Imm8();
  IRReJitConditional(cc, insn, &DSPEmitterIR::irr_ret, insn.SR, false, tmp1, tmp2);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::RetOp = {
    "RetOp", &DSPEmitterIR::iremit_RetOp,
    0x0000,  0x0000,
    0x0000,  0x0000,
    true,    {{OpImmAny}, {OpImmAny}},
    {},      {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_RtiOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();
  X64Reg tmp4 = insn.temps[3].oparg.GetSimpleReg();

  dsp_reg_load_stack(StackRegister::Data, tmp4, tmp1, tmp2, tmp3);
  MOV(16, insn.SR, R(tmp4));
  dsp_reg_load_stack(StackRegister::Call, tmp4, tmp1, tmp2, tmp3);
  MOV(16, M_SDSP_pc(), R(tmp4));
  DSPJitIRRegCache c(m_gpr);
  dropAllRegs(insn);
  WriteBranchExit(insn.cycle_count);
  m_gpr.FlushRegs(c, false);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::RtiOp = {
    "RtiOp", &DSPEmitterIR::iremit_RtiOp,
    0x0000,  0xffff,
    0x0000,  0x0000,  // restores SR from stack 1
    true,    {},
    {},      {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::irr_jmp(DSPEmitterIR::IRInsn const& insn)
{
  if (insn.inputs[1].oparg.IsImm())
  {
    MOV(16, M_SDSP_pc(), insn.inputs[1].oparg.AsImm16());
  }
  else
  {
    MOV(16, M_SDSP_pc(), insn.inputs[1].oparg);
  }
  DSPJitIRRegCache c(m_gpr);
  dropAllRegs(insn);
  WriteBranchExit(insn.cycle_count);
  m_gpr.FlushRegs(c, false);
}

void DSPEmitterIR::iremit_JmpOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  uint8_t cc = insn.inputs[0].oparg.AsImm8().Imm8();
  IRReJitConditional(cc & 0xf, insn, &DSPEmitterIR::irr_jmp, insn.SR, cc & 0x80, tmp1, tmp2);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::JmpOp = {
    "JmpOp", &DSPEmitterIR::iremit_JmpOp,
    0x0000,  0x0000,
    0x0000,  0x0000,
    true,    {{OpImmAny}, {OpAnyReg | OpImmAny}, {OpImmAny}},
    {},      {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::irr_call(DSPEmitterIR::IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();

  dsp_reg_store_stack(StackRegister::Call, insn.inputs[2].oparg, tmp1, tmp2, tmp3);
  if (insn.inputs[1].oparg.IsImm())
  {
    MOV(16, M_SDSP_pc(), insn.inputs[1].oparg.AsImm16());
  }
  else
  {
    MOV(16, M_SDSP_pc(), insn.inputs[1].oparg);
  }
  DSPJitIRRegCache c(m_gpr);
  dropAllRegs(insn);
  WriteBranchExit(insn.cycle_count);
  m_gpr.FlushRegs(c, false);
}

void DSPEmitterIR::iremit_CallOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  uint8_t cc = insn.inputs[0].oparg.AsImm8().Imm8();
  IRReJitConditional(cc, insn, &DSPEmitterIR::irr_call, insn.SR, false, tmp1, tmp2);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::CallOp = {
    "CallOp", &DSPEmitterIR::iremit_CallOp,
    0x0000,   0x0000,
    0x0000,   0x0000,
    true,     {{OpImmAny}, {OpAnyReg | OpImmAny}, {OpImmAny}},
    {},       {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

}  // namespace DSP::JITIR::x64
