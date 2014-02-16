// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

// Additional copyrights go to Duddie and Tratax (c) 2004

#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP::JITIR::x64
{
// In: (val): s64 _Value
// clobbers (val), (tmp1)
// modifies SR bits 2, 3, 4, 5
void DSPEmitterIR::Update_SR_Register(Gen::X64Reg val, OpArg const& sr_reg, Gen::X64Reg tmp1)
{
  ASSERT(val != tmp1);

  //	// 0x04
  //	if (_Value == 0) g_dsp.r[DSP_REG_SR] |= SR_ARITH_ZERO;
  TEST(64, R(val), R(val));
  FixupBranch notZero = J_CC(CC_NZ);
  OR(16, sr_reg, Imm16(SR_ARITH_ZERO | SR_TOP2BITS));
  FixupBranch end = J();
  SetJumpTarget(notZero);

  //	// 0x08
  //	if (_Value < 0) g_dsp.r[DSP_REG_SR] |= SR_SIGN;
  FixupBranch greaterThanEqual = J_CC(CC_GE);
  OR(16, sr_reg, Imm16(SR_SIGN));
  SetJumpTarget(greaterThanEqual);

  //	// 0x10
  //	if (_Value != (s32)_Value) g_dsp.r[DSP_REG_SR] |= SR_OVER_S32;
  MOVSX(64, 32, tmp1, R(val));
  CMP(64, R(tmp1), R(val));
  FixupBranch noOverS32 = J_CC(CC_E);
  OR(16, sr_reg, Imm16(SR_OVER_S32));
  SetJumpTarget(noOverS32);

  //	// 0x20 - Checks if top bits of m are equal
  //	if (((_Value & 0xc0000000) == 0) || ((_Value & 0xc0000000) == 0xc0000000))
  MOV(32, R(tmp1), Imm32(0xc0000000));
  AND(32, R(val), R(tmp1));
  FixupBranch zeroC = J_CC(CC_Z);
  CMP(32, R(val), R(tmp1));
  FixupBranch cC = J_CC(CC_NE);
  SetJumpTarget(zeroC);
  //		g_dsp.r[DSP_REG_SR] |= SR_TOP2BITS;
  OR(16, sr_reg, Imm16(SR_TOP2BITS));
  SetJumpTarget(cC);
  SetJumpTarget(end);
}

// In: (val): s64 _Value
// clobbers (val), (tmp1)
// modifies SR bits 0, 1, 2, 3, 4, 5, fixed bits: 0: 0, 1: 0
void DSPEmitterIR::Update_SR_Register64(X64Reg val, OpArg const& sr_reg, X64Reg tmp1)
{
  //	g_dsp.r[DSP_REG_SR] &= ~SR_CMP_MASK;
  AND(16, sr_reg, Imm16(~SR_CMP_MASK));
  Update_SR_Register(val, sr_reg, tmp1);
}

// In: (new_val): value after the add/subtract
// In: (old_val): value before the add/subtract
// In: (add_nsub_val): value that was added/negative of value that was subtracted
// clobbers (new_val), (old_val), add_nsub_val
// modifies SR bits 0, 1, 2, 3, 4, 5, 7, fixed bits: none
void DSPEmitterIR::Update_SR_Register64_Carry(X64Reg new_val, X64Reg old_val, X64Reg add_nsub_val,
                                              OpArg const& sr_reg, bool subtraction)
{
  //	g_dsp.r[DSP_REG_SR] &= ~SR_CMP_MASK;
  AND(16, sr_reg, Imm16(~SR_CMP_MASK));

  CMP(64, R(old_val), R(new_val));

  // 0x01
  //	g_dsp.r[DSP_REG_SR] |= SR_CARRY;
  // Carry = (acc>res)
  // Carry2 = (acc>=res)
  FixupBranch noCarry = J_CC(subtraction ? CC_B : CC_BE);
  OR(16, sr_reg, Imm16(SR_CARRY));
  SetJumpTarget(noCarry);

  // 0x02 and 0x80
  //	g_dsp.r[DSP_REG_SR] |= SR_OVERFLOW;
  //	g_dsp.r[DSP_REG_SR] |= SR_OVERFLOW_STICKY;
  // Overflow = ((acc ^ res) & (ax ^ res)) < 0
  XOR(64, R(old_val), R(new_val));
  XOR(64, R(add_nsub_val), R(new_val));
  TEST(64, R(old_val), R(add_nsub_val));
  FixupBranch noOverflow = J_CC(CC_GE);
  OR(16, sr_reg, Imm16(SR_OVERFLOW | SR_OVERFLOW_STICKY));
  SetJumpTarget(noOverflow);

  Update_SR_Register(new_val, sr_reg, add_nsub_val);
}

// In: (val): s64 _Value
// modifies SR bits 0, 1, 2, 3, 4, 5, fixed bits: 0: 0, 1: 0, 4: 0
void DSPEmitterIR::Update_SR_Register16(X64Reg val, OpArg const& sr_reg)
{
  AND(16, sr_reg, Imm16(~SR_CMP_MASK));

  //	// 0x04
  //	if (_Value == 0) g_dsp.r[DSP_REG_SR] |= SR_ARITH_ZERO;
  TEST(64, R(val), R(val));
  FixupBranch notZero = J_CC(CC_NZ);
  OR(16, sr_reg, Imm16(SR_ARITH_ZERO | SR_TOP2BITS));
  FixupBranch end = J();
  SetJumpTarget(notZero);

  //	// 0x08
  //	if (_Value < 0) g_dsp.r[DSP_REG_SR] |= SR_SIGN;
  FixupBranch greaterThanEqual = J_CC(CC_GE);
  OR(16, sr_reg, Imm16(SR_SIGN));
  SetJumpTarget(greaterThanEqual);

  //	// 0x20 - Checks if top bits of m are equal
  //	if ((((u16)_Value >> 14) == 0) || (((u16)_Value >> 14) == 3))
  SHR(16, R(val), Imm8(14));
  TEST(16, R(val), R(val));
  FixupBranch isZero = J_CC(CC_Z);
  CMP(16, R(val), Imm16(3));
  FixupBranch notThree = J_CC(CC_NE);
  SetJumpTarget(isZero);
  //		g_dsp.r[DSP_REG_SR] |= SR_TOP2BITS;
  OR(16, sr_reg, Imm16(SR_TOP2BITS));
  SetJumpTarget(notThree);
  SetJumpTarget(end);
}

// In: (val): s64 _Value
// In: (acc): s64 new accumulator value
// modifies SR bits 0, 1, 2, 3, 4, 5, fixed bits: 0: 0, 1: 0
void DSPEmitterIR::Update_SR_Register16_OverS32(Gen::X64Reg val, Gen::X64Reg acc,
                                                OpArg const& sr_reg)
{
  Update_SR_Register16(val, sr_reg);

  // using val as temporary

  // the OVER_S32 bit has been cleared in Update_SR_Register16

  //	// 0x10
  //	if (_Value != (s32)_Value) g_dsp.r[DSP_REG_SR] |= SR_OVER_S32;
  MOVSX(64, 32, val, R(acc));
  CMP(64, R(val), R(acc));
  FixupBranch noOverS32 = J_CC(CC_E);
  OR(16, sr_reg, Imm16(SR_OVER_S32));
  SetJumpTarget(noOverS32);
}

}  // namespace DSP::JITIR::x64
