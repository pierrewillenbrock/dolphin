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
void DSPEmitterIR::mrr(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x1f;
  u8 dreg = (opc >> 5) & 0x1f;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  dsp_op_read_reg(sreg, EDX, RegisterExtension::None, tmp1, tmp2, RAX);
  dsp_op_write_reg(dreg, EDX, tmp1, tmp2, RAX);
  dsp_conditional_extend_accum(dreg, RAX);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
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
void DSPEmitterIR::lri(const UDSPInstruction opc)
{
  u8 reg = opc & 0x1F;
  u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  dsp_op_write_reg_imm(reg, imm, tmp1, tmp2, RAX);
  dsp_conditional_extend_accum_imm(reg, imm);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LRIS $(0x18+D), #I
// 0000 1ddd iiii iiii
// Load immediate value I (8-bit sign extended) to accumulator register.
void DSPEmitterIR::lris(const UDSPInstruction opc)
{
  u8 reg = ((opc >> 8) & 0x7) + DSP_REG_AXL0;
  u16 imm = (s8)opc;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  dsp_op_write_reg_imm(reg, imm, tmp1, tmp2, RAX);
  dsp_conditional_extend_accum_imm(reg, imm);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SRS @M, $(0x18+S)
// 0010 1sss mmmm mmmm
// Move value from register $(0x18+S) to data memory pointed by address
// CR[0-7] | M. That is, the upper 8 bits of the address are the
// bottom 8 bits from CR, and the lower 8 bits are from the 8-bit immediate.
// Note: pc+=2 in duddie's doc seems wrong
void DSPEmitterIR::srs(const UDSPInstruction opc)
{
  u8 reg = ((opc >> 8) & 0x7) + 0x18;
  // u16 addr = (g_dsp.r.cr << 8) | (opc & 0xFF);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  dsp_op_read_reg(reg, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  m_gpr.ReadReg(DSP_REG_CR, RAX, RegisterExtension::Zero);
  SHL(16, R(EAX), Imm8(8));
  OR(16, R(EAX), Imm16(opc & 0xFF));
  dmem_write(tmp1, RAX, RCX);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LRS $(0x18+D), @M
// 0010 0ddd mmmm mmmm
// Move value from data memory pointed by address CR[0-7] | M to register
// $(0x18+D).  That is, the upper 8 bits of the address are the bottom 8 bits
// from CR, and the lower 8 bits are from the 8-bit immediate.
void DSPEmitterIR::lrs(const UDSPInstruction opc)
{
  u8 reg = ((opc >> 8) & 0x7) + 0x18;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  // u16 addr = (g_dsp.r[DSP_REG_CR] << 8) | (opc & 0xFF);
  m_gpr.ReadReg(DSP_REG_CR, tmp1, RegisterExtension::Zero);
  SHL(16, R(tmp1), Imm8(8));
  OR(16, R(tmp1), Imm16(opc & 0xFF));
  dmem_read(tmp1, RAX);

  dsp_op_write_reg(reg, RAX, tmp2, tmp3, RAX);
  dsp_conditional_extend_accum(reg, RAX);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LR $D, @M
// 0000 0000 110d dddd
// mmmm mmmm mmmm mmmm
// Move value from data memory pointed by address M to register $D.
void DSPEmitterIR::lr(const UDSPInstruction opc)
{
  int reg = opc & 0x1F;
  u16 address = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  dmem_read_imm(address, RAX);
  dsp_op_write_reg(reg, EAX, tmp1, tmp2, RAX);
  dsp_conditional_extend_accum(reg, RAX);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SR @M, $S
// 0000 0000 111s ssss
// mmmm mmmm mmmm mmmm
// Store value from register $S to a memory pointed by address M.
void DSPEmitterIR::sr(const UDSPInstruction opc)
{
  const u8 reg = opc & 0x1F;
  const u16 address = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  dsp_op_read_reg(reg, tmp1, RegisterExtension::None, tmp2, tmp3, RAX);
  dmem_write_imm(address, tmp1, RDX);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SI @M, #I
// 0001 0110 mmmm mmmm
// iiii iiii iiii iiii
// Store 16-bit immediate value I to a memory location pointed by address
// M (M is 8-bit value sign extended).
void DSPEmitterIR::si(const UDSPInstruction opc)
{
  const u16 address = static_cast<s8>(opc);
  const u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);

  const X64Reg tmp1 = m_gpr.GetFreeXReg();

  MOV(32, R(tmp1), Imm32((u32)imm));
  dmem_write_imm(address, tmp1, RDX);

  m_gpr.PutXReg(tmp1);
}

// LRR $D, @$S
// 0001 1000 0ssd dddd
// Move value from data memory pointed by addressing register $S to register $D.
void DSPEmitterIR::lrr(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x3;
  u8 dreg = opc & 0x1f;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(sreg, tmp1, RegisterExtension::None);
  dmem_read(tmp1, RAX);

  dsp_op_write_reg(dreg, EAX, tmp2, tmp3, RAX);
  dsp_conditional_extend_accum(dreg, RAX);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LRRD $D, @$S
// 0001 1000 1ssd dddd
// Move value from data memory pointed by addressing register $S to register $D.
// Decrement register $S.
void DSPEmitterIR::lrrd(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x3;
  u8 dreg = opc & 0x1f;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(sreg, tmp1, RegisterExtension::None);
  dmem_read(tmp1, RAX);

  dsp_op_write_reg(dreg, EAX, tmp2, tmp3, RAX);

  dsp_conditional_extend_accum(dreg, RAX);
  decrement_addr_reg(sreg, tmp2, RAX, RDX, RCX);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LRRI $D, @$S
// 0001 1001 0ssd dddd
// Move value from data memory pointed by addressing register $S to register $D.
// Increment register $S.
void DSPEmitterIR::lrri(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x3;
  u8 dreg = opc & 0x1f;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(sreg, tmp1, RegisterExtension::None);
  dmem_read(tmp1, RAX);

  dsp_op_write_reg(dreg, EAX, tmp2, tmp3, RAX);

  dsp_conditional_extend_accum(dreg, RAX);
  increment_addr_reg(sreg, tmp2, RAX, RDX, RCX);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LRRN $D, @$S
// 0001 1001 1ssd dddd
// Move value from data memory pointed by addressing register $S to register $D.
// Add indexing register $(0x4+S) to register $S.
void DSPEmitterIR::lrrn(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x3;
  u8 dreg = opc & 0x1f;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(sreg, tmp1, RegisterExtension::None);
  dmem_read(tmp1, RAX);

  dsp_op_write_reg(dreg, EAX, tmp2, tmp3, RAX);

  dsp_conditional_extend_accum(dreg, RAX);
  increase_addr_reg(sreg, sreg, tmp2, RAX, RDX, RCX);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SRR @$D, $S
// 0001 1010 0dds ssss
// Store value from source register $S to a memory location pointed by
// addressing register $D.
void DSPEmitterIR::srr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x3;
  u8 sreg = opc & 0x1f;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  dsp_op_read_reg(sreg, tmp1, RegisterExtension::None, tmp2, tmp3, RAX);
  m_gpr.ReadReg(dreg, RAX, RegisterExtension::Zero);
  dmem_write(tmp1, RAX, RCX);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SRRD @$D, $S
// 0001 1010 1dds ssss
// Store value from source register $S to a memory location pointed by
// addressing register $D. Decrement register $D.
void DSPEmitterIR::srrd(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x3;
  u8 sreg = opc & 0x1f;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  dsp_op_read_reg(sreg, tmp1, RegisterExtension::None, tmp2, tmp3, RAX);
  m_gpr.ReadReg(dreg, RAX, RegisterExtension::Zero);
  dmem_write(tmp1, RAX, RCX);

  decrement_addr_reg(dreg, tmp2, RAX, RDX, RCX);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SRRI @$D, $S
// 0001 1011 0dds ssss
// Store value from source register $S to a memory location pointed by
// addressing register $D. Increment register $D.
void DSPEmitterIR::srri(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x3;
  u8 sreg = opc & 0x1f;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  dsp_op_read_reg(sreg, tmp1, RegisterExtension::None, tmp2, tmp3, RAX);
  m_gpr.ReadReg(dreg, RAX, RegisterExtension::Zero);
  dmem_write(tmp1, RAX, RCX);

  increment_addr_reg(dreg, tmp2, RAX, RDX, RCX);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SRRN @$D, $S
// 0001 1011 1dds ssss
// Store value from source register $S to a memory location pointed by
// addressing register $D. Add DSP_REG_IX0 register to register $D.
void DSPEmitterIR::srrn(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x3;
  u8 sreg = opc & 0x1f;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  dsp_op_read_reg(sreg, tmp1, RegisterExtension::None, tmp2, tmp3, RAX);
  m_gpr.ReadReg(dreg, RAX, RegisterExtension::Zero);
  dmem_write(tmp1, RAX, RCX);

  increase_addr_reg(dreg, dreg, tmp2, RAX, RDX, RCX);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// ILRR $acD.m, @$arS
// 0000 001d 0001 00ss
// Move value from instruction memory pointed by addressing register
// $arS to mid accumulator register $acD.m.
void DSPEmitterIR::ilrr(const UDSPInstruction opc)
{
  u16 reg = opc & 0x3;
  u16 dreg = (opc >> 8) & 1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(reg, tmp1, RegisterExtension::Zero);
  imem_read(tmp1, RAX);

  m_gpr.WriteReg(dreg + DSP_REG_ACM0, R(RAX));
  dsp_conditional_extend_accum(dreg + DSP_REG_ACM0, RAX);

  m_gpr.PutXReg(tmp1);
}

// ILRRD $acD.m, @$arS
// 0000 001d 0001 01ss
// Move value from instruction memory pointed by addressing register
// $arS to mid accumulator register $acD.m. Decrement addressing register $arS.
void DSPEmitterIR::ilrrd(const UDSPInstruction opc)
{
  u16 reg = opc & 0x3;
  u16 dreg = (opc >> 8) & 1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(reg, tmp1, RegisterExtension::Zero);
  imem_read(tmp1, RAX);

  m_gpr.WriteReg(dreg + DSP_REG_ACM0, R(RAX));
  dsp_conditional_extend_accum(dreg + DSP_REG_ACM0, RAX);
  decrement_addr_reg(reg, tmp2, RAX, RDX, RCX);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// ILRRI $acD.m, @$S
// 0000 001d 0001 10ss
// Move value from instruction memory pointed by addressing register
// $arS to mid accumulator register $acD.m. Increment addressing register $arS.
void DSPEmitterIR::ilrri(const UDSPInstruction opc)
{
  u16 reg = opc & 0x3;
  u16 dreg = (opc >> 8) & 1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(reg, tmp1, RegisterExtension::Zero);
  imem_read(tmp1, RAX);

  m_gpr.WriteReg(dreg + DSP_REG_ACM0, R(RAX));
  dsp_conditional_extend_accum(dreg + DSP_REG_ACM0, RAX);
  increment_addr_reg(reg, tmp2, RAX, RDX, RCX);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// ILRRN $acD.m, @$arS
// 0000 001d 0001 11ss
// Move value from instruction memory pointed by addressing register
// $arS to mid accumulator register $acD.m. Add corresponding indexing
// register $ixS to addressing register $arS.
void DSPEmitterIR::ilrrn(const UDSPInstruction opc)
{
  u16 reg = opc & 0x3;
  u16 dreg = (opc >> 8) & 1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(reg, tmp1, RegisterExtension::Zero);
  imem_read(tmp1, RAX);

  m_gpr.WriteReg(dreg + DSP_REG_ACM0, R(RAX));
  dsp_conditional_extend_accum(dreg + DSP_REG_ACM0, RAX);
  increase_addr_reg(reg, reg, tmp2, RAX, RDX, RCX);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

}  // namespace DSP::JITIR::x64
