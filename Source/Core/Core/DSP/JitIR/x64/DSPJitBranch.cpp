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

void DSPEmitterIR::IRReJitConditional(u8 cond, DSPEmitterIR::IRInsn const& insn,
                                      OpArg const& sr_reg, bool negate, X64Reg tmp1, X64Reg tmp2)
{
  if (cond == 0xf && !negate)
  {  // Always true.
    FixupBranch branchTaken = J(true);
    SetJumpTarget(branchTaken, m_int3_loop);
    insn.branchTaken = branchTaken;

    return;
  }
  if (cond == 0xf && negate)
  {  // Never true
    // put in our dummy data
    insn.branchTaken = m_unused_jump;
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
  bool equal = (cond != 0xe) && !(cond & 1);
  if (negate)
    equal = !equal;
  FixupBranch branchTaken = equal ? J_CC(CC_E, true) : J_CC(CC_NE, true);
  SetJumpTarget(branchTaken, m_int3_loop);
  insn.branchTaken = branchTaken;
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

  if (cc == 0xf)
  {
    IRInsn p2 = {
        &WriteBranchExitOp, {IROp::Imm(dest)}, IROp::None(), {}, 0x0000, 0x0000, 0x0000, 0x0000};

    ir_add_op(p2);
  }
  else
  {
    IRInsn p = {&JmpOp, {IROp::Imm(cc)}, IROp::None(), {}, branch_needs_SR[cc],
                0x0000, 0x0000,          0x0000};

    IRInsn p2 = {&WriteBranchExitOp,
                 {IROp::Imm(dest)},  // address if true
                 IROp::None(),
                 {},
                 0x0000,
                 0x0000,
                 0x0000,
                 0x0000};

    IRInsnNode* in = makeIRInsnNode(p2);

    ir_add_branch(p, in, in);
  }
}

// Generic jmpr implementation
// JMPcc $R
// 0001 0111 rrr0 cccc
// Jump to address; set program counter to a value from register $R.
void DSPEmitterIR::ir_jmprcc(const UDSPInstruction opc)
{
  u8 cc = opc & 0xf;
  u8 reg = (opc >> 5) & 0x7;

  if (cc == 0xf)
  {
    IRInsn p2 = {
        &WriteBranchExitOp, {IROp::R(reg)}, IROp::None(), {}, 0x0000, 0x0000, 0x0000, 0x0000};

    ir_add_op(p2);
  }
  else
  {
    IRInsn p = {&JmpOp, {IROp::Imm(cc)}, IROp::None(), {}, branch_needs_SR[cc],
                0x0000, 0x0000,          0x0000};

    IRInsn p2 = {&WriteBranchExitOp,
                 {IROp::R(reg)},  // address if true
                 IROp::None(),
                 {},
                 0x0000,
                 0x0000,
                 0x0000,
                 0x0000};

    IRInsnNode* in = makeIRInsnNode(p2);

    ir_add_branch(p, in, in);
  }
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
  if (cc == 0xf)
  {
    IRInsn p2 = {&CallUncondOp, {IROp::Imm(m_compile_pc + 2)},  // return addr
                 IROp::None(),  {},
                 0x0000,        0x0000,
                 0x0000,        0x0000};
    IRInsnNode* in = makeIRInsnNode(p2);

    IRInsn p3 = {&WriteBranchExitOp,
                 {IROp::Imm(dest)},  // callee addr
                 IROp::None(),
                 {},
                 0x0000,
                 0x0000,
                 0x0000,
                 0x0000};
    IRInsnNode* in2 = makeIRInsnNode(p3);
    in->addNext(in2);

    ir_add_irnodes(in, in2);
  }
  else
  {
    IRInsn p = {&CallOp, {IROp::Imm(cc)}, IROp::None(), {}, branch_needs_SR[cc],
                0x0000,  0x0000,          0x0000};

    IRInsn p2 = {&CallUncondOp, {IROp::Imm(m_compile_pc + 2)},  // return addr
                 IROp::None(),  {},
                 0x0000,        0x0000,
                 0x0000,        0x0000};
    IRInsnNode* in = makeIRInsnNode(p2);

    IRInsn p3 = {&WriteBranchExitOp,
                 {IROp::Imm(dest)},  // callee addr
                 IROp::None(),
                 {},
                 0x0000,
                 0x0000,
                 0x0000,
                 0x0000};
    IRInsnNode* in2 = makeIRInsnNode(p3);
    in->addNext(in2);

    ir_add_branch(p, in, in2);
  }
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
  if (cc == 0xf)
  {
    IRInsn p2 = {&CallUncondOp, {IROp::Imm(m_compile_pc + 1)},  // return addr
                 IROp::None(),  {},
                 0x0000,        0x0000,
                 0x0000,        0x0000};
    IRInsnNode* in = makeIRInsnNode(p2);

    IRInsn p3 = {&WriteBranchExitOp,
                 {IROp::R(reg)},  // callee addr
                 IROp::None(),
                 {},
                 0x0000,
                 0x0000,
                 0x0000,
                 0x0000};
    IRInsnNode* in2 = makeIRInsnNode(p3);
    in->addNext(in2);

    ir_add_irnodes(in, in2);
  }
  else
  {
    IRInsn p = {&CallOp, {IROp::Imm(cc)}, IROp::None(), {}, branch_needs_SR[cc],
                0x0000,  0x0000,          0x0000};

    IRInsn p2 = {&CallUncondOp, {IROp::Imm(m_compile_pc + 1)},  // return addr
                 IROp::None(),  {},
                 0x0000,        0x0000,
                 0x0000,        0x0000};
    IRInsnNode* in = makeIRInsnNode(p2);

    IRInsn p3 = {&WriteBranchExitOp,
                 {IROp::R(reg)},  // callee addr
                 IROp::None(),
                 {},
                 0x0000,
                 0x0000,
                 0x0000,
                 0x0000};
    IRInsnNode* in2 = makeIRInsnNode(p3);
    in->addNext(in2);

    ir_add_branch(p, in, in2);
  }
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
              {IROp::Imm(cc | 0x80)},  // invert logic
              IROp::None(),
              {},
              branch_needs_SR[cc],
              0x0000,
              0x0000,
              0x0000};

  IRInsn p2 = {&WriteBranchExitOp,
               {IROp::Imm(dest + GetOpTemplate(m_dsp_core.DSPState().ReadIMEM(dest))->size)},
               IROp::None(),
               {},
               0x0000,
               0x0000,
               0x0000,
               0x0000};
  IRInsnNode* in = makeIRInsnNode(p2);

  ir_add_branch(p, in, in);
}

// Generic ret implementation
// RETcc
// 0000 0010 1101 cccc
// Return from subroutine if condition cc has been met. Pops stored PC
// from call stack $st0 and sets $pc to this location.
void DSPEmitterIR::ir_ret(const UDSPInstruction opc)
{
  u8 cc = opc & 0xf;
  if (cc == 0xf)
  {
    IRInsn p2 = {&RetUncondOp, {}, IROp::None(), {}, 0x0000, 0x0000, 0x0000, 0x0000};
    ir_add_op(p2);
  }
  else
  {
    IRInsn p = {&RetOp, {IROp::Imm(cc)}, IROp::None(), {}, branch_needs_SR[cc],
                0x0000, 0x0000,          0x0000};

    IRInsn p2 = {&RetUncondOp, {}, IROp::None(), {}, 0x0000, 0x0000, 0x0000, 0x0000};
    IRInsnNode* in = makeIRInsnNode(p2);

    ir_add_branch(p, in, in);
  }
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
  IRInsnNode* in = makeIRInsnNode(p);

  IRInsn p2 = {&WriteBranchExitOp, {IROp::Imm(m_compile_pc)}};
  IRInsnNode* in2 = new IRInsnNode();
  m_node_storage.push_back(in2);
  in2->insn = p2;
  in->addNext(in2);

  ir_add_irnodes(in, in2);
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

  IRInsn p2 = {&WriteBranchExitOp,
               {IROp::Imm(loop_pc + GetOpTemplate(m_dsp_core.DSPState().ReadIMEM(loop_pc))->size)},
               IROp::None()};
  IRInsnNode* in = makeIRInsnNode(p2);

  ir_add_branch(p, in, in);

  if (m_addr_info[loop_pc + 1].loop_begin != 0xffff)
    m_addr_info[loop_pc + 1].loop_begin = 0xfffe;
  else
    m_addr_info[loop_pc + 1].loop_begin = m_compile_pc + 1;
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

  IRInsn p2 = {&WriteBranchExitOp,
               {IROp::Imm(loop_pc + GetOpTemplate(m_dsp_core.DSPState().ReadIMEM(loop_pc))->size)},
               IROp::None()};
  IRInsnNode* in = makeIRInsnNode(p2);

  ir_add_branch(p, in, in);

  if (m_addr_info[loop_pc + 1].loop_begin != 0xffff)
    m_addr_info[loop_pc + 1].loop_begin = 0xfffe;
  else
    m_addr_info[loop_pc + 1].loop_begin = m_compile_pc + 1;
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

  IRInsn p2 = {&WriteBranchExitOp,
               {IROp::Imm(loop_pc + GetOpTemplate(m_dsp_core.DSPState().ReadIMEM(loop_pc))->size)},
               IROp::None()};
  IRInsnNode* in = makeIRInsnNode(p2);

  ir_add_branch(p, in, in);

  if (m_addr_info[loop_pc + 1].loop_begin != 0xffff)
    m_addr_info[loop_pc + 1].loop_begin = 0xfffe;
  else
    m_addr_info[loop_pc + 1].loop_begin = m_compile_pc + 2;
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

  IRInsn p2 = {&WriteBranchExitOp,
               {IROp::Imm(loop_pc + GetOpTemplate(state.ReadIMEM(loop_pc))->size)},
               IROp::None()};
  IRInsnNode* in = makeIRInsnNode(p2);

  ir_add_branch(p, in, in);

  if (m_addr_info[loop_pc + 1].loop_begin != 0xffff)
    m_addr_info[loop_pc + 1].loop_begin = 0xfffe;
  else
    m_addr_info[loop_pc + 1].loop_begin = m_compile_pc + 2;
}

void DSPEmitterIR::iremit_LoopOp(IRInsn const& insn)
{
  // check if the loop is going to be entered and skip over if not.
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
      // dummy jump
      insn.branchTaken = m_unused_jump;

      dsp_reg_store_stack(StackRegister::Call, Imm16(loop_start), tmp1, tmp2, tmp3);
      dsp_reg_store_stack(StackRegister::LoopAddress, Imm16(loop_end), tmp1, tmp2, tmp3);
      dsp_reg_store_stack(StackRegister::LoopCounter, Imm16(cnt), tmp1, tmp2, tmp3);
    }
    else
    {
      // when we know this, it would be better
      // to completely skip over the loop body
      FixupBranch branchTaken = J(true);
      SetJumpTarget(branchTaken, m_int3_loop);
      insn.branchTaken = branchTaken;
    }
  }
  else
  {
    TEST(16, insn.inputs[0].oparg, insn.inputs[0].oparg);
    FixupBranch branchTaken = J_CC(CC_Z, true);

    SetJumpTarget(branchTaken, m_int3_loop);
    insn.branchTaken = branchTaken;

    dsp_reg_store_stack(StackRegister::LoopCounter, insn.inputs[0].oparg, tmp1, tmp2, tmp3);
    dsp_reg_store_stack(StackRegister::Call, Imm16(loop_start), tmp1, tmp2, tmp3);
    dsp_reg_store_stack(StackRegister::LoopAddress, Imm16(loop_end), tmp1, tmp2, tmp3);
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
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::HaltOp = {
    "HaltOp", &DSPEmitterIR::iremit_HaltOp, 0x0000, 0x0000, 0x0000, 0x0000, true};

void DSPEmitterIR::iremit_RetUncondOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();
  X64Reg tmp4 = insn.temps[3].oparg.GetSimpleReg();
  // this cannot be (easily) predicted. leave the BranchExit here.
  dsp_reg_load_stack(StackRegister::Call, tmp4, tmp1, tmp2, tmp3);
  MOV(16, M_SDSP_pc(), R(tmp4));
  dropAllRegs(insn);
  WriteBranchExit(insn.cycle_count);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::RetUncondOp = {
    "RetUncondOp",
    &DSPEmitterIR::iremit_RetUncondOp,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    true,
    {},
    {},
    {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_RetOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  uint8_t cc = insn.inputs[0].oparg.AsImm8().Imm8();
  IRReJitConditional(cc, insn, insn.SR, false, tmp1, tmp2);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::RetOp = {
    "RetOp", &DSPEmitterIR::iremit_RetOp, 0x0000, 0x0000, 0x0000, 0x0000, true, {{OpImmAny}},
    {},      {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_RtiOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();
  X64Reg tmp4 = insn.temps[3].oparg.GetSimpleReg();

  // this cannot be (easily) predicted. leave the BranchExit here.
  dsp_reg_load_stack(StackRegister::Data, tmp4, tmp1, tmp2, tmp3);
  MOV(16, insn.SR, R(tmp4));
  dsp_reg_load_stack(StackRegister::Call, tmp4, tmp1, tmp2, tmp3);
  MOV(16, M_SDSP_pc(), R(tmp4));
  dropAllRegs(insn);
  WriteBranchExit(insn.cycle_count);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::RtiOp = {
    "RtiOp", &DSPEmitterIR::iremit_RtiOp,
    0x0000,  0xffff,
    0x0000,  0x0000,  // restores SR from stack 1
    true,    {},
    {},      {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_JmpOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  uint8_t cc = insn.inputs[0].oparg.AsImm8().Imm8();
  IRReJitConditional(cc & 0xf, insn, insn.SR, cc & 0x80, tmp1, tmp2);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::JmpOp = {
    "JmpOp", &DSPEmitterIR::iremit_JmpOp, 0x0000, 0x0000, 0x0000, 0x0000, true, {{OpImmAny}},
    {},      {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_CallUncondOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();

  dsp_reg_store_stack(StackRegister::Call, insn.inputs[0].oparg, tmp1, tmp2, tmp3);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::CallUncondOp = {
    "CallUncondOp",
    &DSPEmitterIR::iremit_CallUncondOp,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    true,
    {{OpImmAny}},
    {},
    {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_CallOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  uint8_t cc = insn.inputs[0].oparg.AsImm8().Imm8();
  IRReJitConditional(cc, insn, insn.SR, false, tmp1, tmp2);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::CallOp = {
    "CallOp", &DSPEmitterIR::iremit_CallOp, 0x0000, 0x0000, 0x0000, 0x0000, true, {{OpImmAny}},
    {},       {{OpAnyReg}, {OpAnyReg}}};

}  // namespace DSP::JITIR::x64
