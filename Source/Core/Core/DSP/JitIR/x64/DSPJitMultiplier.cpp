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
// Returns s64 in RAX
// In: RCX = s16 a, RAX = s16 b
void DSPEmitterIR::multiply()
{
  //	prod = (s16)a * (s16)b; //signed
  IMUL(64, R(ECX));

  //	Conditionally multiply by 2.
  //	if ((g_dsp.r.sr & SR_MUL_MODIFY) == 0)
  const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
  TEST(16, sr_reg, Imm16(SR_MUL_MODIFY));
  FixupBranch noMult2 = J_CC(CC_NZ);
  //		prod <<= 1;
  ADD(64, R(RAX), R(RAX));
  SetJumpTarget(noMult2);
  m_gpr.PutReg(DSP_REG_SR, false);
  //	return prod;
}

// Returns s64 in RAX
// Clobbers RDX
void DSPEmitterIR::multiply_add()
{
  //	s64 prod = dsp_get_long_prod() + dsp_get_multiply_prod(a, b, sign);
  multiply();
  MOV(64, R(RDX), R(RAX));
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  get_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp1);
  ADD(64, R(RAX), R(RDX));
  //	return prod;
}

// Returns s64 in RAX
// Clobbers RDX
void DSPEmitterIR::multiply_sub()
{
  //	s64 prod = dsp_get_long_prod() - dsp_get_multiply_prod(a, b, sign);
  multiply();
  MOV(64, R(RDX), R(RAX));
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  get_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp1);
  SUB(64, R(RAX), R(RDX));
  //	return prod;
}

// Only MULX family instructions have unsigned/mixed support.
// Returns s64 in EAX
// In: RCX = s16 a, RAX = s16 b
// Returns s64 in RAX
void DSPEmitterIR::multiply_mulx(u8 axh0, u8 axh1)
{
  //	s64 result;

  //	if ((axh0==0) && (axh1==0))
  //		result = dsp_multiply(val1, val2, 1); // unsigned support ON if both ax?.l regs are used
  //	else if ((axh0==0) && (axh1==1))
  //		result = dsp_multiply(val1, val2, 2); // mixed support ON (u16)axl.0  * (s16)axh.1
  //	else if ((axh0==1) && (axh1==0))
  //		result = dsp_multiply(val2, val1, 2); // mixed support ON (u16)axl.1  * (s16)axh.0
  //	else
  //		result = dsp_multiply(val1, val2, 0); // unsigned support OFF if both ax?.h regs are used

  //	if ((sign == 1) && (g_dsp.r.sr & SR_MUL_UNSIGNED)) //unsigned
  const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
  TEST(16, sr_reg, Imm16(SR_MUL_UNSIGNED));
  FixupBranch unsignedMul = J_CC(CC_NZ);
  //		prod = (s16)a * (s16)b; //signed
  MOVSX(64, 16, RAX, R(RAX));
  IMUL(64, R(RCX));
  FixupBranch signedMul = J(true);

  SetJumpTarget(unsignedMul);
  DSPJitIRRegCache c(m_gpr);
  m_gpr.PutReg(DSP_REG_SR, false);
  if ((axh0 == 0) && (axh1 == 0))
  {
    // unsigned support ON if both ax?.l regs are used
    //		prod = (u32)(a * b);
    MOVZX(64, 16, RCX, R(RCX));
    MOVZX(64, 16, RAX, R(RAX));
    MUL(64, R(RCX));
  }
  else if ((axh0 == 0) && (axh1 == 1))
  {
    // mixed support ON (u16)axl.0  * (s16)axh.1
    //		prod = a * (s16)b;
    X64Reg tmp = m_gpr.GetFreeXReg();
    MOV(64, R(tmp), R(RAX));
    MOVZX(64, 16, RAX, R(RCX));
    IMUL(64, R(tmp));
    m_gpr.PutXReg(tmp);
  }
  else if ((axh0 == 1) && (axh1 == 0))
  {
    // mixed support ON (u16)axl.1  * (s16)axh.0
    //		prod = (s16)a * b;
    MOVZX(64, 16, RAX, R(RAX));
    IMUL(64, R(RCX));
  }
  else
  {
    // unsigned support OFF if both ax?.h regs are used
    //		prod = (s16)a * (s16)b; //signed
    MOVSX(64, 16, RAX, R(RAX));
    IMUL(64, R(RCX));
  }

  m_gpr.FlushRegs(c);
  SetJumpTarget(signedMul);

  //	Conditionally multiply by 2.
  //	if ((g_dsp.r.sr & SR_MUL_MODIFY) == 0)
  TEST(16, sr_reg, Imm16(SR_MUL_MODIFY));
  FixupBranch noMult2 = J_CC(CC_NZ);
  //		prod <<= 1;
  ADD(64, R(RAX), R(RAX));
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
void DSPEmitterIR::clrp(const UDSPInstruction opc)
{
  int offset = static_cast<int>(offsetof(SDSP, r.prod.val));
  // 64bit move to memory does not work. use 2 32bits
  MOV(32, MDisp(R15, offset + 0 * sizeof(u32)), Imm32(0xfff00000U));
  MOV(32, MDisp(R15, offset + 1 * sizeof(u32)), Imm32(0x001000ffU));
}

// TSTPROD
// 1000 0101 xxxx xxxx
// Test prod regs value.

// flags out: --xx xx0x
void DSPEmitterIR::tstprod(const UDSPInstruction opc)
{
  if (FlagsNeeded())
  {
    //		s64 prod = dsp_get_long_prod();
    X64Reg tmp1 = m_gpr.GetFreeXReg();
    get_long_prod(RAX, tmp1);
    m_gpr.PutXReg(tmp1);
    //		Update_SR_Register64(prod);
    Update_SR_Register64();
  }
}

//----

// MOVP $acD
// 0110 111d xxxx xxxx
// Moves multiply product from $prod register to accumulator $acD register.

// flags out: --xx xx0x
void DSPEmitterIR::movp(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;

  //	s64 acc = dsp_get_long_prod();
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  get_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp1);
  //	dsp_set_long_acc(dreg, acc);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg, false);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  //	Update_SR_Register64(acc);
  if (FlagsNeeded())
  {
    Update_SR_Register64();
  }
}

// MOVNP $acD
// 0111 111d xxxx xxxx
// Moves negative of multiply product from $prod register to accumulator
// $acD register.

// flags out: --xx xx0x
void DSPEmitterIR::movnp(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;

  //	s64 acc = -dsp_get_long_prod();
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  get_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp1);
  NEG(64, R(EAX));
  //	dsp_set_long_acc(dreg, acc);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg, false);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  //	Update_SR_Register64(acc);
  if (FlagsNeeded())
  {
    Update_SR_Register64();
  }
}

// MOVPZ $acD
// 1111 111d xxxx xxxx
// Moves multiply product from $prod register to accumulator $acD
// register and sets (rounds) $acD.l to 0

// flags out: --xx xx0x
void DSPEmitterIR::movpz(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x01;

  //	s64 acc = dsp_get_long_prod_round_prodl();
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  get_long_prod_round_prodl(RAX, tmp1);
  m_gpr.PutXReg(tmp1);
  //	dsp_set_long_acc(dreg, acc);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg, false);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  //	Update_SR_Register64(acc);
  if (FlagsNeeded())
  {
    Update_SR_Register64();
  }
}

// ADDPAXZ $acD, $axS
// 1111 10sd xxxx xxxx
// Adds secondary accumulator $axS to product register and stores result
// in accumulator register. Low 16-bits of $acD ($acD.l) are set (round) to 0.

// flags out: --xx xx0x
void DSPEmitterIR::addpaxz(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;

  //	s64 ax = dsp_get_long_acx(sreg);
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  m_gpr.ReadReg(sreg + DSP_REG_AX0_32, tmp1, RegisterExtension::Sign);
  MOV(64, R(RDX), R(tmp1));
  //	s64 res = prod + (ax & ~0xffff);
  AND(64, R(RDX), Imm32(~0xffff));
  //	s64 prod = dsp_get_long_prod_round_prodl();
  get_long_prod_round_prodl(RAX, tmp2);
  ADD(64, R(RAX), R(RDX));

  //	s64 oldprod = dsp_get_long_prod();
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry(oldprod, res), false);
  if (FlagsNeeded())
  {
    get_long_prod(RDX, tmp2);
    MOV(64, R(RCX), R(RAX));

    OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg, false);
    MOV(64, accreg, R(RCX));
    m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);

    Update_SR_Register64_Carry(EAX, tmp1);
  }
  else
  {
    OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg, false);
    MOV(64, accreg, R(RAX));
    m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  }
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

//----

// MULAXH
// 1000 0011 xxxx xxxx
// Multiply $ax0.h by $ax0.h
void DSPEmitterIR::mulaxh(const UDSPInstruction opc)
{
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  //	s64 prod = dsp_multiply(dsp_get_ax_h(0), dsp_get_ax_h(0));
  dsp_op_read_reg(DSP_REG_AXH0, RCX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  MOV(64, R(RAX), R(RCX));
  multiply();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

//----

// MUL $axS.l, $axS.h
// 1001 s000 xxxx xxxx
// Multiply low part $axS.l of secondary accumulator $axS by high part
// $axS.h of secondary accumulator $axS (treat them both as signed).
void DSPEmitterIR::mul(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 11) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  //	u16 axl = dsp_get_ax_l(sreg);
  dsp_op_read_reg(DSP_REG_AXL0 + sreg, RCX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	u16 axh = dsp_get_ax_h(sreg);
  dsp_op_read_reg(DSP_REG_AXH0 + sreg, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply(axh, axl);
  multiply();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MULAC $axS.l, $axS.h, $acR
// 1001 s10r xxxx xxxx
// Add product register to accumulator register $acR. Multiply low part
// $axS.l of secondary accumulator $axS by high part $axS.h of secondary
// accumulator $axS (treat them both as signed).

// flags out: --xx xx0x
void DSPEmitterIR::mulac(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 11) & 0x1;

  //	s64 acc = dsp_get_long_acc(rreg) + dsp_get_long_prod();
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg);
  MOV(64, R(RAX), accreg);
  MOV(64, R(RDX), R(RAX));
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  get_long_prod(RAX, tmp1);

  ADD(64, R(RAX), R(RDX));
  PUSH(64, R(RAX));
  //	u16 axl = dsp_get_ax_l(sreg);
  dsp_op_read_reg(DSP_REG_AXL0 + sreg, RCX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	u16 axh = dsp_get_ax_h(sreg);
  dsp_op_read_reg(DSP_REG_AXH0 + sreg, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply(axl, axh);
  multiply();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);
  //	dsp_set_long_acc(rreg, acc);
  POP(64, R(RAX));
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
  if (FlagsNeeded())
  {
    Update_SR_Register64();
  }
}

// MULMV $axS.l, $axS.h, $acR
// 1001 s11r xxxx xxxx
// Move product register to accumulator register $acR. Multiply low part
// $axS.l of secondary accumulator $axS by high part $axS.h of secondary
// accumulator $axS (treat them both as signed).

// flags out: --xx xx0x
void DSPEmitterIR::mulmv(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;

  //	s64 acc = dsp_get_long_prod();
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  get_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp1);
  PUSH(64, R(RAX));
  mul(opc);
  //	dsp_set_long_acc(rreg, acc);
  POP(64, R(RAX));
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg, false);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64();
  }
}

// MULMVZ $axS.l, $axS.h, $acR
// 1001 s01r xxxx xxxx
// Move product register to accumulator register $acR and clear (round) low part
// of accumulator register $acR.l. Multiply low part $axS.l of secondary
// accumulator $axS by high part $axS.h of secondary accumulator $axS (treat
// them both as signed).

// flags out: --xx xx0x
void DSPEmitterIR::mulmvz(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;

  //	s64 acc = dsp_get_long_prod_round_prodl();
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  get_long_prod_round_prodl(RDX, tmp1);
  m_gpr.PutXReg(tmp1);
  //	dsp_set_long_acc(rreg, acc);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg, false);
  MOV(64, accreg, R(RDX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  mul(opc);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RDX, RCX);
  }
}

//----

// MULX $ax0.S, $ax1.T
// 101s t000 xxxx xxxx
// Multiply one part $ax0 by one part $ax1.
// Part is selected by S and T bits. Zero selects low part, one selects high part.
void DSPEmitterIR::mulx(const UDSPInstruction opc)
{
  u8 treg = ((opc >> 11) & 0x1);
  u8 sreg = ((opc >> 12) & 0x1);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  //	u16 val1 = (sreg == 0) ? dsp_get_ax_l(0) : dsp_get_ax_h(0);
  dsp_op_read_reg(DSP_REG_AXL0 + sreg * 2, RCX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	u16 val2 = (treg == 0) ? dsp_get_ax_l(1) : dsp_get_ax_h(1);
  dsp_op_read_reg(DSP_REG_AXL1 + treg * 2, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply_mulx(sreg, treg, val1, val2);
  multiply_mulx(sreg, treg);
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MULXAC $ax0.S, $ax1.T, $acR
// 101s t01r xxxx xxxx
// Add product register to accumulator register $acR. Multiply one part
// $ax0 by one part $ax1. Part is selected by S and
// T bits. Zero selects low part, one selects high part.

// flags out: --xx xx0x
void DSPEmitterIR::mulxac(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;

  //	s64 acc = dsp_get_long_acc(rreg) + dsp_get_long_prod();
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();
  const OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg);
  MOV(64, R(tmp1), accreg);
  get_long_prod(RAX, tmp2);
  ADD(64, R(tmp1), R(RAX));
  //	u16 val1 = (sreg == 0) ? dsp_get_ax_l(0) : dsp_get_ax_h(0);
  dsp_op_read_reg(DSP_REG_AXL0 + sreg * 2, RCX, RegisterExtension::Sign, tmp2, tmp3, RAX);
  //	u16 val2 = (treg == 0) ? dsp_get_ax_l(1) : dsp_get_ax_h(1);
  dsp_op_read_reg(DSP_REG_AXL1 + treg * 2, RAX, RegisterExtension::Sign, tmp2, tmp3, RAX);
  //	s64 prod = dsp_multiply_mulx(sreg, treg, val1, val2);
  multiply_mulx(sreg, treg);

  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp2);
  //	dsp_set_long_acc(rreg, acc);
  MOV(64, accreg, R(tmp1));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(tmp1);
  }
  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MULXMV $ax0.S, $ax1.T, $acR
// 101s t11r xxxx xxxx
// Move product register to accumulator register $acR. Multiply one part
// $ax0 by one part $ax1. Part is selected by S and
// T bits. Zero selects low part, one selects high part.

// flags out: --xx xx0x
void DSPEmitterIR::mulxmv(const UDSPInstruction opc)
{
  u8 rreg = ((opc >> 8) & 0x1);
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;

  //	s64 acc = dsp_get_long_prod();
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();
  get_long_prod(tmp1, tmp2);
  //	u16 val1 = (sreg == 0) ? dsp_get_ax_l(0) : dsp_get_ax_h(0);
  dsp_op_read_reg(DSP_REG_AXL0 + sreg * 2, RCX, RegisterExtension::Sign, tmp2, tmp3, RAX);
  //	u16 val2 = (treg == 0) ? dsp_get_ax_l(1) : dsp_get_ax_h(1);
  dsp_op_read_reg(DSP_REG_AXL1 + treg * 2, RAX, RegisterExtension::Sign, tmp2, tmp3, RAX);
  //	s64 prod = dsp_multiply_mulx(sreg, treg, val1, val2);
  multiply_mulx(sreg, treg);

  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp2);
  //	dsp_set_long_acc(rreg, acc);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg, false);
  MOV(64, accreg, R(tmp1));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(tmp1);
  }
  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MULXMV $ax0.S, $ax1.T, $acR
// 101s t01r xxxx xxxx
// Move product register to accumulator register $acR and clear (round) low part
// of accumulator register $acR.l. Multiply one part $ax0 by one part $ax1
// Part is selected by S and T bits. Zero selects low part,
// one selects high part.

// flags out: --xx xx0x
void DSPEmitterIR::mulxmvz(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;

  //	s64 acc = dsp_get_long_prod_round_prodl();
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();
  get_long_prod_round_prodl(tmp1, tmp2);
  //	u16 val1 = (sreg == 0) ? dsp_get_ax_l(0) : dsp_get_ax_h(0);
  dsp_op_read_reg(DSP_REG_AXL0 + sreg * 2, RCX, RegisterExtension::Sign, tmp2, tmp3, RAX);
  //	u16 val2 = (treg == 0) ? dsp_get_ax_l(1) : dsp_get_ax_h(1);
  dsp_op_read_reg(DSP_REG_AXL1 + treg * 2, RAX, RegisterExtension::Sign, tmp2, tmp3, RAX);
  //	s64 prod = dsp_multiply_mulx(sreg, treg, val1, val2);
  multiply_mulx(sreg, treg);

  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp2);
  //	dsp_set_long_acc(rreg, acc);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg, false);
  MOV(64, accreg, R(tmp1));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(tmp1);
  }
  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

//----

// MULC $acS.m, $axT.h
// 110s t000 xxxx xxxx
// Multiply mid part of accumulator register $acS.m by high part $axS.h of
// secondary accumulator $axS (treat them both as signed).
void DSPEmitterIR::mulc(const UDSPInstruction opc)
{
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  //	u16 accm = dsp_get_acc_m(sreg);
  m_gpr.ReadReg(sreg + DSP_REG_ACM0, RCX, RegisterExtension::Sign);
  //	u16 axh = dsp_get_ax_h(treg);
  dsp_op_read_reg(DSP_REG_AXH0 + treg, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply(accm, axh);
  multiply();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MULCAC $acS.m, $axT.h, $acR
// 110s	t10r xxxx xxxx
// Multiply mid part of accumulator register $acS.m by high part $axS.h of
// secondary accumulator $axS  (treat them both as signed). Add product
// register before multiplication to accumulator $acR.

// flags out: --xx xx0x
void DSPEmitterIR::mulcac(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;

  //	s64 acc = dsp_get_long_acc(rreg) + dsp_get_long_prod();
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg);
  MOV(64, R(RAX), accreg);
  MOV(64, R(RDX), R(RAX));
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  get_long_prod(RAX, tmp1);
  ADD(64, R(RAX), R(RDX));
  PUSH(64, R(RAX));
  //	u16 accm = dsp_get_acc_m(sreg);
  m_gpr.ReadReg(sreg + DSP_REG_ACM0, RCX, RegisterExtension::Sign);
  //	u16 axh = dsp_get_ax_h(treg);
  dsp_op_read_reg(DSP_REG_AXH0 + treg, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply(accm, axh);
  multiply();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);
  //	dsp_set_long_acc(rreg, acc);
  POP(64, R(RAX));
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64();
  }
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MULCMV $acS.m, $axT.h, $acR
// 110s t11r xxxx xxxx
// Multiply mid part of accumulator register $acS.m by high part $axT.h of
// secondary accumulator $axT  (treat them both as signed). Move product
// register before multiplication to accumulator $acR.
// possible mistake in duddie's doc axT.h rather than axS.h

// flags out: --xx xx0x
void DSPEmitterIR::mulcmv(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;

  //	s64 acc = dsp_get_long_prod();
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  get_long_prod(RAX, tmp1);
  PUSH(64, R(RAX));
  //	u16 accm = dsp_get_acc_m(sreg);
  m_gpr.ReadReg(sreg + DSP_REG_ACM0, RCX, RegisterExtension::Sign);
  //	u16 axh = dsp_get_ax_h(treg);
  dsp_op_read_reg(DSP_REG_AXH0 + treg, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply(accm, axh);
  multiply();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);
  //	dsp_set_long_acc(rreg, acc);
  POP(64, R(RAX));
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg, false);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64();
  }
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MULCMVZ $acS.m, $axT.h, $acR
// 110s	t01r xxxx xxxx
// (fixed possible bug in duddie's description, s->t)
// Multiply mid part of accumulator register $acS.m by high part $axT.h of
// secondary accumulator $axT  (treat them both as signed). Move product
// register before multiplication to accumulator $acR, set (round) low part of
// accumulator $acR.l to zero.

// flags out: --xx xx0x
void DSPEmitterIR::mulcmvz(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x1;
  u8 treg = (opc >> 11) & 0x1;
  u8 sreg = (opc >> 12) & 0x1;

  //	s64 acc = dsp_get_long_prod_round_prodl();
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  get_long_prod_round_prodl(RAX, tmp1);
  PUSH(64, R(RAX));
  //	u16 accm = dsp_get_acc_m(sreg);
  m_gpr.ReadReg(sreg + DSP_REG_ACM0, RCX, RegisterExtension::Sign);
  //	u16 axh = dsp_get_ax_h(treg);
  dsp_op_read_reg(DSP_REG_AXH0 + treg, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply(accm, axh);
  multiply();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);
  //	dsp_set_long_acc(rreg, acc);
  POP(64, R(RAX));
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg, false);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64();
  }
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

//----

// MADDX ax0.S ax1.T
// 1110 00st xxxx xxxx
// Multiply one part of secondary accumulator $ax0 (selected by S) by
// one part of secondary accumulator $ax1 (selected by T) (treat them both as
// signed) and add result to product register.
void DSPEmitterIR::maddx(const UDSPInstruction opc)
{
  u8 treg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  //	u16 val1 = (sreg == 0) ? dsp_get_ax_l(0) : dsp_get_ax_h(0);
  dsp_op_read_reg(DSP_REG_AXL0 + sreg * 2, RCX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	u16 val2 = (treg == 0) ? dsp_get_ax_l(1) : dsp_get_ax_h(1);
  dsp_op_read_reg(DSP_REG_AXL1 + treg * 2, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply_add(val1, val2);
  multiply_add();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MSUBX $(0x18+S*2), $(0x19+T*2)
// 1110 01st xxxx xxxx
// Multiply one part of secondary accumulator $ax0 (selected by S) by
// one part of secondary accumulator $ax1 (selected by T) (treat them both as
// signed) and subtract result from product register.
void DSPEmitterIR::msubx(const UDSPInstruction opc)
{
  u8 treg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  //	u16 val1 = (sreg == 0) ? dsp_get_ax_l(0) : dsp_get_ax_h(0);
  dsp_op_read_reg(DSP_REG_AXL0 + sreg * 2, RCX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	u16 val2 = (treg == 0) ? dsp_get_ax_l(1) : dsp_get_ax_h(1);
  dsp_op_read_reg(DSP_REG_AXL1 + treg * 2, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply_sub(val1, val2);
  multiply_sub();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MADDC $acS.m, $axT.h
// 1110 10st xxxx xxxx
// Multiply middle part of accumulator $acS.m by high part of secondary
// accumulator $axT.h (treat them both as signed) and add result to product
// register.
void DSPEmitterIR::maddc(const UDSPInstruction opc)
{
  u8 treg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  //	u16 accm = dsp_get_acc_m(sreg);
  m_gpr.ReadReg(sreg + DSP_REG_ACM0, RCX, RegisterExtension::Sign);
  //	u16 axh = dsp_get_ax_h(treg);
  dsp_op_read_reg(DSP_REG_AXH0 + treg, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply_add(accm, axh);
  multiply_add();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MSUBC $acS.m, $axT.h
// 1110 11st xxxx xxxx
// Multiply middle part of accumulator $acS.m by high part of secondary
// accumulator $axT.h (treat them both as signed) and subtract result from
// product register.
void DSPEmitterIR::msubc(const UDSPInstruction opc)
{
  u8 treg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  //	u16 accm = dsp_get_acc_m(sreg);
  m_gpr.ReadReg(sreg + DSP_REG_ACM0, RCX, RegisterExtension::Sign);
  //	u16 axh = dsp_get_ax_h(treg);
  dsp_op_read_reg(DSP_REG_AXH0 + treg, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply_sub(accm, axh);
  multiply_sub();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MADD $axS.l, $axS.h
// 1111 001s xxxx xxxx
// Multiply low part $axS.l of secondary accumulator $axS by high part
// $axS.h of secondary accumulator $axS (treat them both as signed) and add
// result to product register.
void DSPEmitterIR::madd(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 8) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  //	u16 axl = dsp_get_ax_l(sreg);
  dsp_op_read_reg(DSP_REG_AXL0 + sreg, RCX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	u16 axh = dsp_get_ax_h(sreg);
  dsp_op_read_reg(DSP_REG_AXH0 + sreg, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply_add(axl, axh);
  multiply_add();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// MSUB $axS.l, $axS.h
// 1111 011s xxxx xxxx
// Multiply low part $axS.l of secondary accumulator $axS by high part
// $axS.h of secondary accumulator $axS (treat them both as signed) and
// subtract result from product register.
void DSPEmitterIR::msub(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 8) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  //	u16 axl = dsp_get_ax_l(sreg);
  dsp_op_read_reg(DSP_REG_AXL0 + sreg, RCX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	u16 axh = dsp_get_ax_h(sreg);
  dsp_op_read_reg(DSP_REG_AXH0 + sreg, RAX, RegisterExtension::Sign, tmp1, tmp2, RAX);
  //	s64 prod = dsp_multiply_sub(axl, axh);
  multiply_sub();
  //	dsp_set_long_prod(prod);
  set_long_prod(RAX, tmp1);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

}  // namespace DSP::JITIR::x64
