// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

// Additional copyrights go to Duddie and Tratax (c) 2004

#include "Common/CommonTypes.h"

#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP::JITIR::x64
{
// MRR $D, $S
// 0001 11dd ddds ssss
// Move value from register $S to register $D.
void DSPEmitterIR::ir_mrr(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x1f;
  u8 dreg = (opc >> 5) & 0x1f;
  IRInsn p = {&Mov16Op, {IROp::R(sreg)}, IROp::R(dreg)};
  ir_add_op(p);
}

// LRI $D, #I
// 0000 0000 100d dddd
// iiii iiii iiii iiii
// Load immediate value I to register $D.
//
// DSPSpy discovery: This, and possibly other instructions that load a
// register, has a different behaviour in S40 mode if loaded to AC0.M: The
// value gets sign extended to the whole accumulator! This does not happen in
// S16 mode.
void DSPEmitterIR::ir_lri(const UDSPInstruction opc)
{
  u8 reg = opc & 0x1F;
  u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {
      &Mov16Op,
      {IROp::Imm(imm)},
      IROp::R(reg),
      {},
      0x0000,
      (reg == DSP_REG_SR) ? (u16)0xffff : (u16)0x0000,
      (reg == DSP_REG_SR) ? (u16)0xffff : (u16)0x0000,
      (reg == DSP_REG_SR) ? imm : (u16)0x0000,
  };
  ir_add_op(p);
}

// LRIS $(0x18+D), #I
// 0000 1ddd iiii iiii
// Load immediate value I (8-bit sign extended) to accumulator register.
void DSPEmitterIR::ir_lris(const UDSPInstruction opc)
{
  u8 reg = ((opc >> 8) & 0x7) + DSP_REG_AXL0;
  u16 imm = (s8)opc;
  IRInsn p = {&Mov16Op, {IROp::Imm(imm)}, IROp::R(reg)};
  ir_add_op(p);
}

// SRS @M, $(0x18+S)
// 0010 1sss mmmm mmmm
// Move value from register $(0x18+D) to data memory pointed by address
// CR[0-7] | M. That is, the upper 8 bits of the address are the
// bottom 8 bits from CR, and the lower 8 bits are from the 8-bit immediate.
// Note: pc+=2 in duddie's doc seems wrong
void DSPEmitterIR::ir_srs(const UDSPInstruction opc)
{
  u8 reg = ((opc >> 8) & 0x7) + 0x18;
  u8 saddr = opc & 0xFF;
  IRInsn p = {&Store16SOp, {IROp::Imm(saddr), IROp::R(DSP_REG_CR), IROp::R(reg)}, IROp::None()};
  ir_add_op(p);
}

// LRS $(0x18+D), @M
// 0010 0ddd mmmm mmmm
// Move value from data memory pointed by address CR[0-7] | M to register
// $(0x18+D).  That is, the upper 8 bits of the address are the bottom 8 bits
// from CR, and the lower 8 bits are from the 8-bit immediate.
void DSPEmitterIR::ir_lrs(const UDSPInstruction opc)
{
  u8 reg = ((opc >> 8) & 0x7) + 0x18;
  u8 saddr = opc & 0xFF;
  IRInsn p = {&Load16SOp, {IROp::Imm(saddr), IROp::R(DSP_REG_CR)}, IROp::R(reg)};
  ir_add_op(p);
}

// LR $D, @M
// 0000 0000 110d dddd
// mmmm mmmm mmmm mmmm
// Move value from data memory pointed by address M to register $D.
void DSPEmitterIR::ir_lr(const UDSPInstruction opc)
{
  int reg = opc & 0x1F;
  u16 address = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&Load16Op, {IROp::Imm(address)}, IROp::R(reg)};
  ir_add_op(p);
}

// SR @M, $S
// 0000 0000 111s ssss
// mmmm mmmm mmmm mmmm
// Store value from register $S to a memory pointed by address M.
void DSPEmitterIR::ir_sr(const UDSPInstruction opc)
{
  u8 reg = opc & 0x1F;
  u16 address = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&Store16Op, {IROp::Imm(address), IROp::R(reg)}, IROp::None()};
  ir_add_op(p);
}

// SI @M, #I
// 0001 0110 mmmm mmmm
// iiii iiii iiii iiii
// Store 16-bit immediate value I to a memory location pointed by address
// M (M is 8-bit value sign extended).
void DSPEmitterIR::ir_si(const UDSPInstruction opc)
{
  u16 address = (s8)opc;
  u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&Store16Op, {IROp::Imm(address), IROp::Imm(imm)}, IROp::None()};
  ir_add_op(p);
}

// LRR $D, @$S
// 0001 1000 0ssd dddd
// Move value from data memory pointed by addressing register $S to register $D.
void DSPEmitterIR::ir_lrr(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x3;
  u8 dreg = opc & 0x1f;
  IRInsn p = {&Load16Op, {IROp::R(sreg)}, IROp::R(dreg)};
  ir_add_op(p);
}

// LRRD $D, @$S
// 0001 1000 1ssd dddd
// Move value from data memory pointed by addressing register $S toregister $D.
// Decrement register $S.
void DSPEmitterIR::ir_lrrd(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x3;
  u8 dreg = opc & 0x1f;
  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R(dreg)};
  ir_add_op(p1);
  IRInsn p2 = {&SubAOp, {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(sreg)};
  ir_add_op(p2);
}

// LRRI $D, @$S
// 0001 1001 0ssd dddd
// Move value from data memory pointed by addressing register $S to register $D.
// Increment register $S.
void DSPEmitterIR::ir_lrri(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x3;
  u8 dreg = opc & 0x1f;
  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R(dreg)};
  ir_add_op(p1);
  IRInsn p2 = {&AddAOp, {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(sreg)};
  ir_add_op(p2);
}

// LRRN $D, @$S
// 0001 1001 1ssd dddd
// Move value from data memory pointed by addressing register $S to register $D.
// Add indexing register $(0x4+S) to register $S.
void DSPEmitterIR::ir_lrrn(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x3;
  u8 dreg = opc & 0x1f;
  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R(dreg)};
  ir_add_op(p1);
  IRInsn p2 = {&AddAOp,
               {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::R(sreg + DSP_REG_IX0)},
               IROp::R(sreg)};
  ir_add_op(p2);
}

// SRR @$D, $S
// 0001 1010 0dds ssss
// Store value from source register $S to a memory location pointed by
// addressing register $D.
void DSPEmitterIR::ir_srr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x3;
  u8 sreg = opc & 0x1f;
  IRInsn p = {&Store16Op, {IROp::R(dreg), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p);
}

// SRRD @$D, $S
// 0001 1010 1dds ssss
// Store value from source register $S to a memory location pointed by
// addressing register $D. Decrement register $D.
void DSPEmitterIR::ir_srrd(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x3;
  u8 sreg = opc & 0x1f;
  IRInsn p1 = {&Store16Op, {IROp::R(dreg), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p1);
  IRInsn p2 = {&SubAOp, {IROp::R(dreg), IROp::R(dreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(dreg)};
  ir_add_op(p2);
}

// SRRI @$D, $S
// 0001 1011 0dds ssss
// Store value from source register $S to a memory location pointed by
// addressing register $D. Increment register $D.
void DSPEmitterIR::ir_srri(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x3;
  u8 sreg = opc & 0x1f;
  IRInsn p1 = {&Store16Op, {IROp::R(dreg), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p1);
  IRInsn p2 = {&AddAOp, {IROp::R(dreg), IROp::R(dreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(dreg)};
  ir_add_op(p2);
}

// SRRN @$D, $S
// 0001 1011 1dds ssss
// Store value from source register $S to a memory location pointed by
// addressing register $D. Add DSP_REG_IX0 register to register $D.
void DSPEmitterIR::ir_srrn(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x3;
  u8 sreg = opc & 0x1f;
  IRInsn p1 = {&Store16Op, {IROp::R(dreg), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p1);
  IRInsn p2 = {&AddAOp,
               {IROp::R(dreg), IROp::R(dreg + DSP_REG_WR0), IROp::R(dreg + DSP_REG_IX0)},
               IROp::R(dreg)};
  ir_add_op(p2);
}

// ILRR $acD.m, @$arS
// 0000 001d 0001 00ss
// Move value from instruction memory pointed by addressing register
// $arS to mid accumulator register $acD.m.
void DSPEmitterIR::ir_ilrr(const UDSPInstruction opc)
{
  u16 sreg = opc & 0x3;
  u16 dreg = (opc >> 8) & 1;
  IRInsn p = {&ILoad16Op, {IROp::R(sreg)}, IROp::R(dreg + DSP_REG_ACM0)};
  ir_add_op(p);
}

// ILRRD $acD.m, @$arS
// 0000 001d 0001 01ss
// Move value from instruction memory pointed by addressing register
// $arS to mid accumulator register $acD.m. Decrement addressing register $arS.
void DSPEmitterIR::ir_ilrrd(const UDSPInstruction opc)
{
  u16 sreg = opc & 0x3;
  u16 dreg = (opc >> 8) & 1;
  IRInsn p1 = {&ILoad16Op, {IROp::R(sreg)}, IROp::R(dreg + DSP_REG_ACM0)};
  ir_add_op(p1);
  IRInsn p2 = {&SubAOp, {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(sreg)};
  ir_add_op(p2);
}

// ILRRI $acD.m, @$S
// 0000 001d 0001 10ss
// Move value from instruction memory pointed by addressing register
// $arS to mid accumulator register $acD.m. Increment addressing register $arS.
void DSPEmitterIR::ir_ilrri(const UDSPInstruction opc)
{
  u16 sreg = opc & 0x3;
  u16 dreg = (opc >> 8) & 1;
  IRInsn p1 = {&ILoad16Op, {IROp::R(sreg)}, IROp::R(dreg + DSP_REG_ACM0)};
  ir_add_op(p1);
  IRInsn p2 = {&AddAOp, {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(sreg)};
  ir_add_op(p2);
}

// ILRRN $acD.m, @$arS
// 0000 001d 0001 11ss
// Move value from instruction memory pointed by addressing register
// $arS to mid accumulator register $acD.m. Add corresponding indexing
// register $ixS to addressing register $arS.
void DSPEmitterIR::ir_ilrrn(const UDSPInstruction opc)
{
  u16 sreg = opc & 0x3;
  u16 dreg = (opc >> 8) & 1;
  IRInsn p1 = {&ILoad16Op, {IROp::R(sreg)}, IROp::R(dreg + DSP_REG_ACM0)};
  ir_add_op(p1);
  IRInsn p2 = {&AddAOp,
               {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::R(sreg + DSP_REG_IX0)},
               IROp::R(sreg)};
  ir_add_op(p2);
}

void DSPEmitterIR::iremit_Mov16Op(IRInsn const& insn)
{
  _assert_msg_(DSPLLE, insn.output.type == IROp::REG, "unhandled Mov16Op variant");
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  if (insn.inputs[0].type == IROp::REG)
  {
    int in_reg = ir_to_regcache_reg(insn.inputs[0].guest_reg);
    dsp_op_read_reg(in_reg, tmp1, RegisterExtension::None, RDX, RAX, tmp2);
    dsp_op_write_reg(out_reg, tmp1, RDX, RAX, tmp2);
    dsp_conditional_extend_accum(out_reg, RAX);
  }
  else if (insn.inputs[0].type == IROp::IMM)
  {
    dsp_op_write_reg_imm(out_reg, insn.inputs[0].imm, tmp1, RAX, tmp2);
    dsp_conditional_extend_accum_imm(out_reg, insn.inputs[0].imm);
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled Mov16Op variant");
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Mov16Op = {
    "Mov16Op", &DSPEmitterIR::iremit_Mov16Op, 0x0000, 0x0000, 0x0000, 0x0000, false, {},
    {},        {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_Load16Op(IRInsn const& insn)
{
  _assert_msg_(DSPLLE, insn.output.type == IROp::REG, "unhandled Load16Op variant");
  // input: address.
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  if (insn.inputs[0].type == IROp::IMM)
  {
    dmem_read_imm(insn.inputs[0].imm, RAX);
    dsp_op_write_reg(out_reg, RAX, tmp1, tmp2, RAX);  // RAX+RAX is broken for ST accesses
    dsp_conditional_extend_accum(out_reg, RAX);
  }
  else if (insn.inputs[0].type == IROp::REG)
  {
    int in_reg = ir_to_regcache_reg(insn.inputs[0].guest_reg);
    m_gpr.ReadReg(in_reg, RDX, RegisterExtension::Zero);
    dmem_read(RDX, RAX);
    dsp_op_write_reg(out_reg, RAX, tmp1, tmp2, RAX);  // RAX+RAX is broken for ST accesses
    dsp_conditional_extend_accum(out_reg, RAX);
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled Load16Op variant");
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Load16Op = {
    "Load16Op", &DSPEmitterIR::iremit_Load16Op, 0x0000, 0x0000, 0x0000, 0x0000, false, {},
    {},         {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_ILoad16Op(IRInsn const& insn)
{
  _assert_msg_(DSPLLE, insn.inputs[0].type == IROp::REG && insn.output.type == IROp::REG,
               "unhandled ILoad16Op variant");
  // input: address.
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  int in_reg = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  m_gpr.ReadReg(in_reg, tmp1, RegisterExtension::Zero);
  imem_read(tmp1, RAX);
  dsp_op_write_reg(out_reg, RAX, tmp1, tmp2, RAX);  // RAX+RAX is broken for ST accesses
  dsp_conditional_extend_accum(out_reg, RAX);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::ILoad16Op = {
    "ILoad16Op", &DSPEmitterIR::iremit_ILoad16Op, 0x0000, 0x0000, 0x0000, 0x0000, false, {},
    {},          {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_Store16Op(IRInsn const& insn)
{
  // inputs: 0: address, 1: value
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();
  if (insn.inputs[0].type == IROp::IMM && insn.inputs[1].type == IROp::REG)
  {
    int in_reg = ir_to_regcache_reg(insn.inputs[1].guest_reg);
    dsp_op_read_reg(in_reg, tmp1, RegisterExtension::None, tmp2, tmp3, RAX);
    dmem_write_imm(insn.inputs[0].imm, tmp1, tmp2);
  }
  else if (insn.inputs[0].type == IROp::IMM && insn.inputs[1].type == IROp::IMM)
  {
    MOV(16, R(tmp1), Imm16(insn.inputs[1].imm));
    dmem_write_imm(insn.inputs[0].imm, tmp1, tmp2);
  }
  else if (insn.inputs[0].type == IROp::REG && insn.inputs[1].type == IROp::REG)
  {
    int out_reg = ir_to_regcache_reg(insn.inputs[0].guest_reg);
    int in_reg = ir_to_regcache_reg(insn.inputs[1].guest_reg);
    dsp_op_read_reg(in_reg, tmp1, RegisterExtension::None, tmp2, tmp3, RAX);
    m_gpr.ReadReg(out_reg, RAX, RegisterExtension::Zero);
    dmem_write(tmp1, RAX, RCX);
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled Store16Op variant");
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Store16Op = {
    "Store16Op", &DSPEmitterIR::iremit_Store16Op,     0x0000, 0x0000, 0x0000, 0x0000, false, {},
    {},          {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_Load16SOp(IRInsn const& insn)
{
  // input: 0: address, 1: CR
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  m_gpr.ReadReg(DSP_REG_CR, RAX, RegisterExtension::Zero);
  MOV(8, R(AH), R(AL));
  MOV(8, R(AL), Imm8(insn.inputs[0].imm));

  MOV(64, R(tmp1), R(RAX));

  dmem_read(tmp1, RAX);

  dsp_op_write_reg(out_reg, RAX, tmp1, tmp2, RAX);  // RAX+RAX is broken for ST accesses
  dsp_conditional_extend_accum(out_reg, RAX);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Load16SOp = {
    "Load16SOp", &DSPEmitterIR::iremit_Load16SOp, 0x0000, 0x0000, 0x0000, 0x0000, false, {},
    {},          {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_Store16SOp(IRInsn const& insn)
{
  // inputs: 0: address, 1: CR, 2: value
  int in_reg = ir_to_regcache_reg(insn.inputs[2].guest_reg);
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();

  dsp_op_read_reg(in_reg, tmp1, RegisterExtension::None, tmp2, tmp3, RAX);

  m_gpr.ReadReg(DSP_REG_CR, RAX, RegisterExtension::Zero);
  MOV(8, R(AH), R(AL));
  MOV(8, R(AL), Imm8(insn.inputs[0].imm));
  dmem_write(tmp1, RAX, RCX);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Store16SOp = {
    "Store16SOp", &DSPEmitterIR::iremit_Store16SOp,    0x0000, 0x0000, 0x0000, 0x0000, false, {},
    {},           {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

}  // namespace DSP::JITIR::x64
