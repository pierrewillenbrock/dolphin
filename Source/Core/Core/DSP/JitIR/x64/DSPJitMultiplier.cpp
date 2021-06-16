// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

// Additional copyrights go to Duddie and Tratax (c) 2004

// Multiplier and product register control

#include <cstddef>

#include "Common/CommonTypes.h"

#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP::JITIR::x64
{
// Returns s64 in (dst)
// In: (mul) = s16 a, (dst) = s16 b
void DSPEmitterIR::multiply(X64Reg dst, X64Reg mul)
{
  //	prod = (s16)a * (s16)b; //signed
  IMUL(64, dst, R(mul));

  //	Conditionally multiply by 2.
  //	if ((g_dsp.r.sr & SR_MUL_MODIFY) == 0)
  const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
  TEST(16, sr_reg, Imm16(SR_MUL_MODIFY));
  FixupBranch noMult2 = J_CC(CC_NZ);
  //		prod <<= 1;
  ADD(64, R(dst), R(dst));
  SetJumpTarget(noMult2);
  m_gpr.PutReg(DSP_REG_SR, false);
  //	return prod;
}

// Only MULX family instructions have unsigned/mixed support.
// Returns s64 in (dst)
// In: (mul) = s16 a, (dst) = s16 b
void DSPEmitterIR::multiply_uu(X64Reg dst, X64Reg mul)
{
  //	if ((sign == 1) && (g_dsp.r.sr & SR_MUL_UNSIGNED)) //unsigned
  OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
  TEST(16, sr_reg, Imm16(SR_MUL_UNSIGNED));
  FixupBranch unsignedMul = J_CC(CC_NZ);
  //		prod = (s16)a * (s16)b; //signed
  MOVSX(64, 16, dst, R(dst));
  MOVSX(64, 16, mul, R(mul));
  IMUL(64, dst, R(mul));
  FixupBranch signedMul = J(true);

  SetJumpTarget(unsignedMul);
  DSPJitIRRegCache c(m_gpr);
  m_gpr.PutReg(DSP_REG_SR, false);
  // unsigned support ON if both ax?.l regs are used
  //	prod = (u32)(a * b);
  MOVZX(64, 16, mul, R(mul));
  MOVZX(64, 16, dst, R(dst));
  // the result is the same as for MUL, but we get to avoid clobbering RDX
  IMUL(64, dst, R(mul));

  m_gpr.FlushRegs(c);
  SetJumpTarget(signedMul);

  //	Conditionally multiply by 2.
  //	if ((g_dsp.r.sr & SR_MUL_MODIFY) == 0)
  TEST(16, sr_reg, Imm16(SR_MUL_MODIFY));
  FixupBranch noMult2 = J_CC(CC_NZ);
  //		prod <<= 1;
  ADD(64, R(dst), R(dst));
  SetJumpTarget(noMult2);
  m_gpr.PutReg(DSP_REG_SR, false);
  //	return prod;
}

// Only MULX family instructions have unsigned/mixed support.
// Returns s64 in (dst)
// In: (mul) = s16 a, (dst) = s16 b
void DSPEmitterIR::multiply_us(X64Reg dst, X64Reg mul)
{
  //	if ((sign == 1) && (g_dsp.r.sr & SR_MUL_UNSIGNED)) //unsigned
  const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
  TEST(16, sr_reg, Imm16(SR_MUL_UNSIGNED));
  FixupBranch unsignedMul = J_CC(CC_NZ);
  //		prod = (s16)a * (s16)b; //signed
  MOVSX(64, 16, dst, R(dst));
  IMUL(64, dst, R(mul));
  FixupBranch signedMul = J(true);

  SetJumpTarget(unsignedMul);
  DSPJitIRRegCache c(m_gpr);
  m_gpr.PutReg(DSP_REG_SR, false);
  // mixed support ON (u16)axl.1  * (s16)axh.0
  //	prod = (s16)a * b;
  MOVZX(64, 16, dst, R(dst));
  IMUL(64, dst, R(mul));

  m_gpr.FlushRegs(c);
  SetJumpTarget(signedMul);

  //	Conditionally multiply by 2.
  //	if ((g_dsp.r.sr & SR_MUL_MODIFY) == 0)
  TEST(16, sr_reg, Imm16(SR_MUL_MODIFY));
  FixupBranch noMult2 = J_CC(CC_NZ);
  //		prod <<= 1;
  ADD(64, R(dst), R(dst));
  SetJumpTarget(noMult2);
  m_gpr.PutReg(DSP_REG_SR, false);
  //	return prod;
}

//----

// CLRP
// 1000 0100 xxxx xxxx
// Clears product register $prod.
// Magic numbers taken from duddie's doc

// 00ff_(fff0 + 0010)_0000 = 0100_0000_0000, conveniently, lower 40bits = 0

// It's not ok, to just zero all of them, correct values should be set because of
// direct use of prod regs by AX/AXWII (look @that part of ucode).
void DSPEmitterIR::ir_clrp(const UDSPInstruction opc)
{
  IRInsn p = {&ClrPOp, {}, IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p);
}

// TSTPROD
// 1000 0101 xxxx xxxx
// Test prod regs value.

// flags out: --xx xx0x
void DSPEmitterIR::ir_tstprod(const UDSPInstruction opc)
{
  IRInsn p = {&TstPOp, {IROp::R(IROp::DSP_REG_PROD_ALL)}, IROp::None()};
  ir_add_op(p);
}

//----

// MOVP $acD
// 0110 111d xxxx xxxx
// Moves multiply product from $prod register to accumulator $acD register.

// flags out: --xx xx0x
void DSPEmitterIR::ir_movp(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&MovPOp, {IROp::R(IROp::DSP_REG_PROD_ALL)}, IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// MOVNP $acD
// 0111 111d xxxx xxxx
// Moves negative of multiply product from $prod register to accumulator
// $acD register.

// flags out: --xx xx0x
void DSPEmitterIR::ir_movnp(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&MovNPOp, {IROp::R(IROp::DSP_REG_PROD_ALL)}, IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// MOVPZ $acD
// 1111 111d xxxx xxxx
// Moves multiply product from $prod register to accumulator $acD
// register and sets (rounds) $acD.l to 0

// flags out: --xx xx0x
void DSPEmitterIR::ir_movpz(const UDSPInstruction opc)
{
  // this actually looks like a variant of movaxh
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&MovPZOp, {IROp::R(IROp::DSP_REG_PROD_ALL)}, IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ADDPAXZ $acD, $axS
// 1111 10sd xxxx xxxx
// Adds secondary accumulator $axS to product register and stores result
// in accumulator register. Low 16-bits of $acD ($acD.l) are set (round) to 0.

// flags out: --xx xx0x
void DSPEmitterIR::ir_addpaxz(const UDSPInstruction opc)
{
  // the store here actually looks like a variant of movaxh
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {&AddPAxZOp,
              {IROp::R(IROp::DSP_REG_PROD_ALL), IROp::R(sreg + IROp::DSP_REG_AX0_ALL)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

//----

// MULAXH
// 1000 0011 xxxx xxxx
// Multiply $ax0.h by $ax0.h
void DSPEmitterIR::ir_mulaxh(const UDSPInstruction opc)
{
  IRInsn p = {
      &MulOp, {IROp::R(DSP_REG_AXH0), IROp::R(DSP_REG_AXH0)}, IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p);
}

//----

// MUL $axS.l, $axS.h
// 1001 s000 xxxx xxxx
// Multiply low part $axS.l of secondary accumulator $axS by high part
// $axS.h of secondary accumulator $axS (treat them both as signed).
void DSPEmitterIR::ir_mul(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 11) & 0x1;
  IRInsn p = {&MulOp,
              {IROp::R(sreg + DSP_REG_AXL0), IROp::R(sreg + DSP_REG_AXH0)},
              IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p);
}

// MULAC $axS.l, $axS.h, $acR
// 1001 s10r xxxx xxxx
// Add product register to accumulator register $acR. Multiply low part
// $axS.l of secondary accumulator $axS by high part $axS.h of secondary
// accumulator $axS (treat them both as signed).

// flags out: --xx xx0x
void DSPEmitterIR::ir_mulac(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 11) & 0x1;
  IRInsn p1 = {&MulOp,
               {IROp::R(sreg + DSP_REG_AXL0), IROp::R(sreg + DSP_REG_AXH0)},
               IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p1);
  IRInsn p2 = {&AddPOp,  // actually a variant of Add40Op, but prod
               // is not available in a preadded format.
               {IROp::R(rreg + IROp::DSP_REG_ACC0_ALL), IROp::R(IROp::DSP_REG_PROD_ALL)},
               IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p2);
}

// MULMV $axS.l, $axS.h, $acR
// 1001 s11r xxxx xxxx
// Move product register to accumulator register $acR. Multiply low part
// $axS.l of secondary accumulator $axS by high part $axS.h of secondary
// accumulator $axS (treat them both as signed).

// flags out: --xx xx0x
void DSPEmitterIR::ir_mulmv(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 11) & 0x1;
  IRInsn p1 = {&MulOp,
               {IROp::R(sreg + DSP_REG_AXL0), IROp::R(sreg + DSP_REG_AXH0)},
               IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p1);
  IRInsn p2 = {&MovPOp,  // actually a variant of Add40Op, but prod
               // is not available in a preadded format.
               {IROp::R(IROp::DSP_REG_PROD_ALL)},
               IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p2);
}

// MULMVZ $axS.l, $axS.h, $acR
// 1001 s01r xxxx xxxx
// Move product register to accumulator register $acR and clear (round) low part
// of accumulator register $acR.l. Multiply low part $axS.l of secondary
// accumulator $axS by high part $axS.h of secondary accumulator $axS (treat
// them both as signed).

// flags out: --xx xx0x
void DSPEmitterIR::ir_mulmvz(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 11) & 0x1;
  IRInsn p1 = {&MulOp,
               {IROp::R(sreg + DSP_REG_AXL0), IROp::R(sreg + DSP_REG_AXH0)},
               IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p1);
  IRInsn p2 = {&MovPZOp, {IROp::R(IROp::DSP_REG_PROD_ALL)}, IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p2);
}

//----

// MULX $ax0.S, $ax1.T
// 101s t000 xxxx xxxx
// Multiply one part $ax0 by one part $ax1.
// Part is selected by S and T bits. Zero selects low part, one selects high part.
void DSPEmitterIR::ir_mulx(const UDSPInstruction opc)
{
  u8 treg = ((opc >> 11) & 0x1);
  u8 sreg = ((opc >> 12) & 0x1);
  IRInsn p = {&InvalidOp,
              {IROp::R((treg << 1) + DSP_REG_AXL1), IROp::R((sreg << 1) + DSP_REG_AXL0)},
              IROp::R(IROp::DSP_REG_PROD_ALL)};
  if (treg && sreg)
  {
    p.emitter = &MulOp;
  }
  else if (!treg && sreg)
  {
    p.emitter = &MulSUOp;
  }
  else if (treg && !sreg)
  {
    p.emitter = &MulUSOp;
  }
  else if (!treg && !sreg)
  {
    p.emitter = &MulUUOp;
  }
  ir_add_op(p);
}

// MULXAC $ax0.S, $ax1.T, $acR
// 101s t01r xxxx xxxx
// Add product register to accumulator register $acR. Multiply one part
// $ax0 by one part $ax1. Part is selected by S and
// T bits. Zero selects low part, one selects high part.

// flags out: --xx xx0x
void DSPEmitterIR::ir_mulxac(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;
  IRInsn p1 = {&InvalidOp,
               {IROp::R((treg << 1) + DSP_REG_AXL1), IROp::R((sreg << 1) + DSP_REG_AXL0)},
               IROp::R(IROp::DSP_REG_PROD_ALL)};
  if (treg && sreg)
  {
    p1.emitter = &MulOp;
  }
  else if (!treg && sreg)
  {
    p1.emitter = &MulSUOp;
  }
  else if (treg && !sreg)
  {
    p1.emitter = &MulUSOp;
  }
  else if (!treg && !sreg)
  {
    p1.emitter = &MulUUOp;
  }
  ir_add_op(p1);
  IRInsn p2 = {&AddPOp,  // actually a variant of Add40Op, but prod
               // is not available in a preadded format.
               {IROp::R(rreg + IROp::DSP_REG_ACC0_ALL), IROp::R(IROp::DSP_REG_PROD_ALL)},
               IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p2);
}

// MULXMV $ax0.S, $ax1.T, $acR
// 101s t11r xxxx xxxx
// Move product register to accumulator register $acR. Multiply one part
// $ax0 by one part $ax1. Part is selected by S and
// T bits. Zero selects low part, one selects high part.

// flags out: --xx xx0x
void DSPEmitterIR::ir_mulxmv(const UDSPInstruction opc)
{
  u8 rreg = ((opc >> 8) & 0x1);
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;
  IRInsn p1 = {&InvalidOp,
               {IROp::R((treg << 1) + DSP_REG_AXL1), IROp::R((sreg << 1) + DSP_REG_AXL0)},
               IROp::R(IROp::DSP_REG_PROD_ALL)};
  if (treg && sreg)
  {
    p1.emitter = &MulOp;
  }
  else if (!treg && sreg)
  {
    p1.emitter = &MulSUOp;
  }
  else if (treg && !sreg)
  {
    p1.emitter = &MulUSOp;
  }
  else if (!treg && !sreg)
  {
    p1.emitter = &MulUUOp;
  }
  ir_add_op(p1);
  IRInsn p2 = {&MovPOp,  // actually a variant of Add40Op, but prod
               // is not available in a preadded format.
               {IROp::R(IROp::DSP_REG_PROD_ALL)},
               IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p2);
}

// MULXMV $ax0.S, $ax1.T, $acR
// 101s t01r xxxx xxxx
// Move product register to accumulator register $acR and clear (round) low part
// of accumulator register $acR.l. Multiply one part $ax0 by one part $ax1
// Part is selected by S and T bits. Zero selects low part,
// one selects high part.

// flags out: --xx xx0x
void DSPEmitterIR::ir_mulxmvz(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;
  IRInsn p1 = {&InvalidOp,
               {IROp::R((treg << 1) + DSP_REG_AXL1), IROp::R((sreg << 1) + DSP_REG_AXL0)},
               IROp::R(IROp::DSP_REG_PROD_ALL)};
  if (treg && sreg)
  {
    p1.emitter = &MulOp;
  }
  else if (!treg && sreg)
  {
    p1.emitter = &MulSUOp;
  }
  else if (treg && !sreg)
  {
    p1.emitter = &MulUSOp;
  }
  else if (!treg && !sreg)
  {
    p1.emitter = &MulUUOp;
  }
  ir_add_op(p1);
  IRInsn p2 = {&MovPZOp, {IROp::R(IROp::DSP_REG_PROD_ALL)}, IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p2);
}

//----

// MULC $acS.m, $axT.h
// 110s t000 xxxx xxxx
// Multiply mid part of accumulator register $acS.m by high part $axS.h of
// secondary accumulator $axS (treat them both as signed).
void DSPEmitterIR::ir_mulc(const UDSPInstruction opc)
{
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;
  IRInsn p = {&MulOp,
              {IROp::R(sreg + DSP_REG_ACM0), IROp::R(treg + DSP_REG_AXH0)},
              IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p);
}

// MULCAC $acS.m, $axT.h, $acR
// 110s	t10r xxxx xxxx
// Multiply mid part of accumulator register $acS.m by high part $axS.h of
// secondary accumulator $axS  (treat them both as signed). Add product
// register before multiplication to accumulator $acR.

// flags out: --xx xx0x
void DSPEmitterIR::ir_mulcac(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;
  IRInsn p1 = {&MulOp,
               {IROp::R(sreg + DSP_REG_ACM0), IROp::R(treg + DSP_REG_AXH0)},
               IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p1);
  IRInsn p2 = {&AddPOp,  // actually a variant of Add40Op, but prod
               // is not available in a preadded format.
               {IROp::R(rreg + IROp::DSP_REG_ACC0_ALL), IROp::R(IROp::DSP_REG_PROD_ALL)},
               IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p2);
}

// MULCMV $acS.m, $axT.h, $acR
// 110s t11r xxxx xxxx
// Multiply mid part of accumulator register $acS.m by high part $axT.h of
// secondary accumulator $axT  (treat them both as signed). Move product
// register before multiplication to accumulator $acR.
// possible mistake in duddie's doc axT.h rather than axS.h

// flags out: --xx xx0x
void DSPEmitterIR::ir_mulcmv(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;
  IRInsn p1 = {&MulOp,
               {IROp::R(sreg + DSP_REG_ACM0), IROp::R(treg + DSP_REG_AXH0)},
               IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p1);
  IRInsn p2 = {&MovPOp,  // actually a variant of Add40Op, but prod
               // is not available in a preadded format.
               {IROp::R(IROp::DSP_REG_PROD_ALL)},
               IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p2);
}

// MULCMVZ $acS.m, $axT.h, $acR
// 110s	t01r xxxx xxxx
// (fixed possible bug in duddie's description, s->t)
// Multiply mid part of accumulator register $acS.m by high part $axT.h of
// secondary accumulator $axT  (treat them both as signed). Move product
// register before multiplication to accumulator $acR, set (round) low part of
// accumulator $acR.l to zero.

// flags out: --xx xx0x
void DSPEmitterIR::ir_mulcmvz(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;
  IRInsn p1 = {&MulOp,
               {IROp::R(sreg + DSP_REG_ACM0), IROp::R(treg + DSP_REG_AXH0)},
               IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p1);
  IRInsn p2 = {&MovPZOp, {IROp::R(IROp::DSP_REG_PROD_ALL)}, IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p2);
}

//----

// MADDX ax0.S ax1.T
// 1110 00st xxxx xxxx
// Multiply one part of secondary accumulator $ax0 (selected by S) by
// one part of secondary accumulator $ax1 (selected by T) (treat them both as
// signed) and add result to product register.
void DSPEmitterIR::ir_maddx(const UDSPInstruction opc)
{
  u8 treg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {&MAddOp,
              {IROp::R(IROp::DSP_REG_PROD_ALL), IROp::R((sreg << 1) + DSP_REG_AXL0),
               IROp::R((treg << 1) + DSP_REG_AXL1)},
              IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p);
}

// MSUBX $(0x18+S*2), $(0x19+T*2)
// 1110 01st xxxx xxxx
// Multiply one part of secondary accumulator $ax0 (selected by S) by
// one part of secondary accumulator $ax1 (selected by T) (treat them both as
// signed) and subtract result from product register.
void DSPEmitterIR::ir_msubx(const UDSPInstruction opc)
{
  u8 treg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {&MSubOp,
              {IROp::R(IROp::DSP_REG_PROD_ALL), IROp::R((sreg << 1) + DSP_REG_AXL0),
               IROp::R((treg << 1) + DSP_REG_AXL1)},
              IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p);
}

// MADDC $acS.m, $axT.h
// 1110 10st xxxx xxxx
// Multiply middle part of accumulator $acS.m by high part of secondary
// accumulator $axT.h (treat them both as signed) and add result to product
// register.
void DSPEmitterIR::ir_maddc(const UDSPInstruction opc)
{
  u8 treg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {
      &MAddOp,
      {IROp::R(IROp::DSP_REG_PROD_ALL), IROp::R(sreg + DSP_REG_ACM0), IROp::R(treg + DSP_REG_AXH0)},
      IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p);
}

// MSUBC $acS.m, $axT.h
// 1110 11st xxxx xxxx
// Multiply middle part of accumulator $acS.m by high part of secondary
// accumulator $axT.h (treat them both as signed) and subtract result from
// product register.
void DSPEmitterIR::ir_msubc(const UDSPInstruction opc)
{
  u8 treg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {
      &MSubOp,
      {IROp::R(IROp::DSP_REG_PROD_ALL), IROp::R(sreg + DSP_REG_ACM0), IROp::R(treg + DSP_REG_AXH0)},
      IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p);
}

// MADD $axS.l, $axS.h
// 1111 001s xxxx xxxx
// Multiply low part $axS.l of secondary accumulator $axS by high part
// $axS.h of secondary accumulator $axS (treat them both as signed) and add
// result to product register.
void DSPEmitterIR::ir_madd(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 8) & 0x1;
  IRInsn p = {
      &MAddOp,
      {IROp::R(IROp::DSP_REG_PROD_ALL), IROp::R(sreg + DSP_REG_AXL0), IROp::R(sreg + DSP_REG_AXH0)},
      IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p);
}

// MSUB $axS.l, $axS.h
// 1111 011s xxxx xxxx
// Multiply low part $axS.l of secondary accumulator $axS by high part
// $axS.h of secondary accumulator $axS (treat them both as signed) and
// subtract result from product register.
void DSPEmitterIR::ir_msub(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 8) & 0x1;
  IRInsn p = {
      &MSubOp,
      {IROp::R(IROp::DSP_REG_PROD_ALL), IROp::R(sreg + DSP_REG_AXL0), IROp::R(sreg + DSP_REG_AXH0)},
      IROp::R(IROp::DSP_REG_PROD_ALL)};
  ir_add_op(p);
}

void DSPEmitterIR::iremit_ClrPOp(IRInsn const& insn)
{
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);  // PROD
  MOV(64, R(RAX), Imm64(0x001000fffff00000ULL));
  m_gpr.WriteReg(out_reg, R(RAX));
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::ClrPOp = {
    "ClrPOp", &DSPEmitterIR::iremit_ClrPOp, 0x0000, 0x0000, 0x0000, 0x0000};

void DSPEmitterIR::iremit_TstPOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);  // PROD
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  _assert_msg_(DSPLLE, in_reg0 == DSP_REG_PROD_64, "in_reg0 must be PROD for TstPOp");
  get_long_prod(RAX, tmp1);
  if (FlagsNeeded(insn.addr))
  {
    Update_SR_Register64(RAX, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::TstPOp = {
    "TstPOp",    &DSPEmitterIR::iremit_TstPOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000, false, {}, {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_MovPOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);  // PROD
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  _assert_msg_(DSPLLE, in_reg0 == DSP_REG_PROD_64, "in_reg0 must be PROD for MovPOp");
  get_long_prod(RAX, tmp1);

  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded(insn.addr))
  {
    Update_SR_Register64(RAX, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MovPOp = {
    "MovPOp",    &DSPEmitterIR::iremit_MovPOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000, false, {}, {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_MovNPOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);  // PROD
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  _assert_msg_(DSPLLE, in_reg0 == DSP_REG_PROD_64, "in_reg0 must be PROD for MovNPOp");
  get_long_prod(RAX, tmp1);
  NEG(64, R(RAX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded(insn.addr))
  {
    Update_SR_Register64(RAX, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MovNPOp = {
    "MovNPOp",   &DSPEmitterIR::iremit_MovNPOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000, false, {}, {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_MovPZOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);  // PROD
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  _assert_msg_(DSPLLE, in_reg0 == DSP_REG_PROD_64, "in_reg0 must be PROD for MovPZOp");
  get_long_prod(RAX, tmp1);
  round_long(RAX);

  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded(insn.addr))
  {
    Update_SR_Register64(RAX, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MovPZOp = {
    "MovPZOp",   &DSPEmitterIR::iremit_MovPZOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000, false, {}, {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_AddPAxZOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);  // PROD
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  m_gpr.ReadReg(in_reg1, tmp1, RegisterExtension::Sign);
  MOV(64, R(RDX), R(tmp1));
  AND(64, R(RDX), Imm32(~0xffff));
  ASSERT_MSG(DSPLLE, in_reg0 == DSP_REG_PROD_64, "in_reg0 must be PROD for AddPAxZOp");
  get_long_prod(RAX, tmp2);
  round_long(RAX);
  ADD(64, R(RAX), R(RDX));

  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded(insn.addr))
  {
    get_long_prod(RDX, tmp2);
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AddPAxZOp = {
    "AddPAxZOp", &DSPEmitterIR::iremit_AddPAxZOp,
    0x0000,      SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,      0x0000,
    false,       {},
    {},          {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_MulOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);  // PROD
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::Sign);
  m_gpr.ReadReg(in_reg1, RCX, RegisterExtension::Sign);
  multiply(RAX, RCX);
  _assert_msg_(DSPLLE, out_reg == DSP_REG_PROD_64, "out_reg must be PROD for MulOp");
  set_long_prod(RAX, tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MulOp = {
    "MulOp",     &DSPEmitterIR::iremit_MulOp, 0x0000, 0x0000, 0x0000, 0x0000, false, {}, {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_MulUUOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);  // PROD
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::None);
  m_gpr.ReadReg(in_reg1, RCX, RegisterExtension::None);
  multiply_uu(RAX, RCX);
  _assert_msg_(DSPLLE, out_reg == DSP_REG_PROD_64, "out_reg must be PROD for MulUUOp");
  set_long_prod(RAX, tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MulUUOp = {
    "MulUUOp",   &DSPEmitterIR::iremit_MulUUOp, 0x0000, 0x0000, 0x0000, 0x0000, false, {}, {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_MulSUOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);  // PROD
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::None);
  m_gpr.ReadReg(in_reg1, RCX, RegisterExtension::Sign);
  multiply_us(RAX, RCX);
  _assert_msg_(DSPLLE, out_reg == DSP_REG_PROD_64, "out_reg must be PROD for MulSUOp");
  set_long_prod(RAX, tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MulSUOp = {
    "MulSUOp",   &DSPEmitterIR::iremit_MulSUOp, 0x0000, 0x0000, 0x0000, 0x0000, false, {}, {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_MulUSOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);  // PROD
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  m_gpr.ReadReg(in_reg1, RAX, RegisterExtension::None);
  m_gpr.ReadReg(in_reg0, RCX, RegisterExtension::Sign);
  multiply_us(RAX, RCX);
  _assert_msg_(DSPLLE, out_reg == DSP_REG_PROD_64, "out_reg must be PROD for MulUSOp");
  set_long_prod(RAX, tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MulUSOp = {
    "MulUSOp",   &DSPEmitterIR::iremit_MulUSOp, 0x0000, 0x0000, 0x0000, 0x0000, false, {}, {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_MAddOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);  // PROD
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int in_reg2 = ir_to_regcache_reg(insn.inputs[2].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);  // PROD
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  m_gpr.ReadReg(in_reg1, RAX, RegisterExtension::Sign);
  m_gpr.ReadReg(in_reg2, RCX, RegisterExtension::Sign);
  multiply(RAX, RCX);
  _assert_msg_(DSPLLE, in_reg0 == DSP_REG_PROD_64, "in_reg0 must be PROD for MAddOp");
  get_long_prod(RCX, tmp1);
  ADD(64, R(RAX), R(RCX));
  _assert_msg_(DSPLLE, out_reg == DSP_REG_PROD_64, "out_reg must be PROD for MAddOp");
  set_long_prod(RAX, tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MAddOp = {
    "MAddOp",    &DSPEmitterIR::iremit_MAddOp, 0x0000, 0x0000, 0x0000, 0x0000, false, {}, {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_MSubOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);  // PROD
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int in_reg2 = ir_to_regcache_reg(insn.inputs[2].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);  // PROD
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  m_gpr.ReadReg(in_reg1, RAX, RegisterExtension::Sign);
  m_gpr.ReadReg(in_reg2, RCX, RegisterExtension::Sign);
  multiply(RAX, RCX);
  _assert_msg_(DSPLLE, in_reg0 == DSP_REG_PROD_64, "in_reg0 must be PROD for MSubOp");
  get_long_prod(RCX, tmp1);
  SUB(64, R(RAX), R(RCX));
  _assert_msg_(DSPLLE, out_reg == DSP_REG_PROD_64, "out_reg must be PROD for MSubOp");
  set_long_prod(RAX, tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MSubOp = {
    "MSubOp",    &DSPEmitterIR::iremit_MSubOp, 0x0000, 0x0000, 0x0000, 0x0000, false, {}, {},
    {{OpAnyReg}}};

}  // namespace DSP::JITIR::x64
