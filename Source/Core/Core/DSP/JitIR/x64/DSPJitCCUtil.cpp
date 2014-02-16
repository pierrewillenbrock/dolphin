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
void DSPEmitterIR::Update_SR_Register(DSPEmitterIR::IRInsn const& insn, Gen::OpArg const& val,
                                      OpArg const& sr_reg, Gen::X64Reg tmp1)
{
  if (!FlagsNeeded(insn))
    return;
  FixupBranch end = m_unused_jump;
  if (insn.later_needs_SR & (SR_ARITH_ZERO | SR_TOP2BITS | SR_SIGN))
  {
    TEST(64, val, val);
  }
  if (insn.later_needs_SR & (SR_ARITH_ZERO | SR_TOP2BITS))
  {
    //	// 0x04
    //	if (_Value == 0) g_dsp.r[DSP_REG_SR] |= SR_ARITH_ZERO;
    FixupBranch notZero = J_CC(CC_NZ);
    OR(16, sr_reg, Imm16(SR_ARITH_ZERO | SR_TOP2BITS));
    end = J();
    SetJumpTarget(notZero);
  }

  if (insn.later_needs_SR & SR_SIGN)
  {
    //	// 0x08
    //	if (_Value < 0) g_dsp.r[DSP_REG_SR] |= SR_SIGN;
    FixupBranch greaterThanEqual = J_CC(CC_GE);
    OR(16, sr_reg, Imm16(SR_SIGN));
    SetJumpTarget(greaterThanEqual);
  }

  if (insn.later_needs_SR & SR_OVER_S32)
  {
    //// 0x10
    // if (_Value != (s32)_Value) g_dsp.r[DSP_REG_SR] |= SR_OVER_S32;
    MOVSX(64, 32, tmp1, val);
    CMP(64, R(tmp1), val);
    FixupBranch noOverS32 = J_CC(CC_E);
    OR(16, sr_reg, Imm16(SR_OVER_S32));
    SetJumpTarget(noOverS32);
  }

  if (insn.later_needs_SR & SR_TOP2BITS)
  {
    //	// 0x20 - Checks if top bits of m are equal
    //	if (((_Value & 0xc0000000) == 0) || ((_Value & 0xc0000000) == 0xc0000000))
    AND(32, val, Imm32(0xc0000000));
    FixupBranch zeroC = J_CC(CC_Z);
    CMP(32, val, Imm32(0xc0000000));
    FixupBranch cC = J_CC(CC_NE);
    SetJumpTarget(zeroC);
    //		g_dsp.r[DSP_REG_SR] |= SR_TOP2BITS;
    OR(16, sr_reg, Imm16(SR_TOP2BITS));
    SetJumpTarget(cC);
  }
  SetJumpTarget(end);
}

// In: (val): s64 _Value
// clobbers (val), (tmp1)
// modifies SR bits 0, 1, 2, 3, 4, 5, fixed bits: 0: 0, 1: 0
void DSPEmitterIR::Update_SR_Register64(IRInsn const& insn, OpArg const& val, OpArg const& sr_reg,
                                        X64Reg tmp1)
{
  if (!FlagsNeeded(insn))
    return;
  //	g_dsp.r[DSP_REG_SR] &= ~SR_CMP_MASK;
  AND(16, sr_reg, Imm16(~SR_CMP_MASK));
  Update_SR_Register(insn, val, sr_reg, tmp1);
}

// In: (new_val): value after the add/subtract
// In: (old_val): value before the add/subtract
// In: (add_nsub_val): value that was added/negative of value that was subtracted
// clobbers (new_val), (old_val), add_nsub_val
// modifies SR bits 0, 1, 2, 3, 4, 5, 7, fixed bits: none
void DSPEmitterIR::Update_SR_Register64_Carry(DSPEmitterIR::IRInsn const& insn,
                                              Gen::OpArg const& new_val, Gen::OpArg const& old_val,
                                              Gen::OpArg const& add_nsub_val,
                                              Gen::OpArg const& sr_reg, bool subtraction)
{
  if (!FlagsNeeded(insn))
    return;
  //	g_dsp.r[DSP_REG_SR] &= ~SR_CMP_MASK;
  AND(16, sr_reg, Imm16(~SR_CMP_MASK));

  if (insn.later_needs_SR & SR_CARRY)
  {
    // 0x01
    //	g_dsp.r[DSP_REG_SR] |= SR_CARRY;
    // Carry = (acc>res)
    // Carry2 = (acc>=res)
    CMP(64, old_val, new_val);
    FixupBranch noCarry = J_CC(subtraction ? CC_B : CC_BE);
    OR(16, sr_reg, Imm16(SR_CARRY));
    SetJumpTarget(noCarry);
  }

  if (insn.later_needs_SR & (SR_OVERFLOW | SR_OVERFLOW_STICKY))
  {
    // 0x02 and 0x80
    //	g_dsp.r[DSP_REG_SR] |= SR_OVERFLOW;
    //	g_dsp.r[DSP_REG_SR] |= SR_OVERFLOW_STICKY;
    // Overflow = ((acc ^ res) & (ax ^ res)) < 0
    XOR(64, old_val, new_val);
    XOR(64, add_nsub_val, new_val);
    TEST(64, old_val, add_nsub_val);
    FixupBranch noOverflow = J_CC(CC_GE);
    OR(16, sr_reg, Imm16(SR_OVERFLOW | SR_OVERFLOW_STICKY));
    SetJumpTarget(noOverflow);
  }

  Update_SR_Register(insn, new_val, sr_reg, add_nsub_val.GetSimpleReg());
}

// In: (val): s64 _Value
// modifies SR bits 0, 1, 2, 3, 4, 5, fixed bits: 0: 0, 1: 0, 4: 0
void DSPEmitterIR::Update_SR_Register16(IRInsn const& insn, OpArg const& val, OpArg const& sr_reg)
{
  if (!FlagsNeeded(insn))
    return;
  AND(16, sr_reg, Imm16(~SR_CMP_MASK));

  FixupBranch end = m_unused_jump;
  if (insn.later_needs_SR & (SR_ARITH_ZERO | SR_TOP2BITS | SR_SIGN))
  {
    TEST(64, val, val);
  }
  if (insn.later_needs_SR & (SR_ARITH_ZERO | SR_TOP2BITS))
  {
    //// 0x04
    // if (_Value == 0) g_dsp.r[DSP_REG_SR] |= SR_ARITH_ZERO;
    FixupBranch notZero = J_CC(CC_NZ);
    OR(16, sr_reg, Imm16(SR_ARITH_ZERO | SR_TOP2BITS));
    end = J();
    SetJumpTarget(notZero);
  }

  if (insn.later_needs_SR & SR_SIGN)
  {
    //// 0x08
    // if (_Value < 0) g_dsp.r[DSP_REG_SR] |= SR_SIGN;
    FixupBranch greaterThanEqual = J_CC(CC_GE);
    OR(16, sr_reg, Imm16(SR_SIGN));
    SetJumpTarget(greaterThanEqual);
  }

  if (insn.later_needs_SR & SR_TOP2BITS)
  {
    //// 0x20 - Checks if top bits of m are equal
    // if ((((u16)_Value >> 14) == 0) || (((u16)_Value >> 14) == 3))
    SHR(16, val, Imm8(14));
    TEST(16, val, val);
    FixupBranch isZero = J_CC(CC_Z);
    CMP(16, val, Imm16(3));
    FixupBranch notThree = J_CC(CC_NE);
    SetJumpTarget(isZero);
    //		g_dsp.r[DSP_REG_SR] |= SR_TOP2BITS;
    OR(16, sr_reg, Imm16(SR_TOP2BITS));
    SetJumpTarget(notThree);
  }
  SetJumpTarget(end);
}

// In: (val): s64 _Value
// In: (acc): s64 new accumulator value
// modifies SR bits 0, 1, 2, 3, 4, 5, fixed bits: 0: 0, 1: 0
void DSPEmitterIR::Update_SR_Register16_OverS32(IRInsn const& insn, Gen::OpArg const& val,
                                                Gen::OpArg const& acc, Gen::OpArg const& sr_reg)
{
  if (!FlagsNeeded(insn))
    return;
  Update_SR_Register16(insn, val, sr_reg);

  if (insn.later_needs_SR & SR_OVER_S32)
  {
    // using val as temporary

    // the OVER_S32 bit has been cleared in Update_SR_Register16

    //// 0x10
    // if (_Value != (s32)_Value) g_dsp.r[DSP_REG_SR] |= SR_OVER_S32;
    MOVSX(64, 32, val.GetSimpleReg(), acc);
    CMP(64, val, acc);
    FixupBranch noOverS32 = J_CC(CC_E);
    OR(16, sr_reg, Imm16(SR_OVER_S32));
    SetJumpTarget(noOverS32);
  }
}

}  // namespace DSP::JITIR::x64
