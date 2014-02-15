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
// CLR $acR
// 1000 r001 xxxx xxxx
// Clears accumulator $acR
//
// flags out: --10 0100
void DSPEmitterIR::clr(const UDSPInstruction opc)
{
  u8 reg = (opc >> 11) & 0x1;
  //	dsp_set_long_acc(reg, 0);
  XOR(32, R(EAX), R(EAX));

  m_gpr.WriteReg(DSP_REG_ACC0_64 + reg, R(RAX));

  //	Update_SR_Register64(0);
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

// CLRL $acR.l
// 1111 110r xxxx xxxx
// Clears (and rounds!) $acR.l - low 16 bits of accumulator $acR.
//
// flags out: --xx xx00
void DSPEmitterIR::clrl(const UDSPInstruction opc)
{
  u8 reg = (opc >> 8) & 0x1;
  //	s64 acc = dsp_round_long_acc(dsp_get_long_acc(reg));
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + reg);
  MOV(64, R(RAX), accreg);

  round_long(RAX);
  //	dsp_set_long_acc(reg, acc);

  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + reg);

  //	Update_SR_Register64(acc);
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

//----

// ANDCF $acD.m, #I
// 0000 001r 1100 0000
// iiii iiii iiii iiii
// Set logic zero (LZ) flag in status register $sr if result of logic AND of
// accumulator mid part $acD.m with immediate value I is equal I.
//
// flags out: -x-- ----
void DSPEmitterIR::andcf(const UDSPInstruction opc)
{
  if (FlagsNeeded())
  {
    const u8 reg = (opc >> 8) & 0x1;
    //		u16 imm = dsp_fetch_code();
    const u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
    //		u16 val = dsp_get_acc_m(reg);
    m_gpr.ReadReg(reg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
    //		Update_SR_LZ(((val & imm) == imm) ? true : false);
    //		if ((val & imm) == imm)
    //			g_dsp.r.sr |= SR_LOGIC_ZERO;
    //		else
    //			g_dsp.r.sr &= ~SR_LOGIC_ZERO;
    const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
    AND(16, R(RAX), Imm16(imm));
    CMP(16, R(RAX), Imm16(imm));
    FixupBranch notLogicZero = J_CC(CC_NE);
    OR(16, sr_reg, Imm16(SR_LOGIC_ZERO));
    FixupBranch exit = J();
    SetJumpTarget(notLogicZero);
    AND(16, sr_reg, Imm16(~SR_LOGIC_ZERO));
    SetJumpTarget(exit);
    m_gpr.PutReg(DSP_REG_SR);
  }
}

// ANDF $acD.m, #I
// 0000 001r 1010 0000
// iiii iiii iiii iiii
// Set logic zero (LZ) flag in status register $sr if result of logical AND
// operation of accumulator mid part $acD.m with immediate value I is equal
// immediate value 0.
//
// flags out: -x-- ----
void DSPEmitterIR::andf(const UDSPInstruction opc)
{
  if (FlagsNeeded())
  {
    const u8 reg = (opc >> 8) & 0x1;
    //		u16 imm = dsp_fetch_code();
    const u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
    //		u16 val = dsp_get_acc_m(reg);
    m_gpr.ReadReg(reg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
    //		Update_SR_LZ(((val & imm) == 0) ? true : false);
    //		if ((val & imm) == 0)
    //			g_dsp.r.sr |= SR_LOGIC_ZERO;
    //		else
    //			g_dsp.r.sr &= ~SR_LOGIC_ZERO;
    const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
    TEST(16, R(RAX), Imm16(imm));
    FixupBranch notLogicZero = J_CC(CC_NE);
    OR(16, sr_reg, Imm16(SR_LOGIC_ZERO));
    FixupBranch exit = J();
    SetJumpTarget(notLogicZero);
    AND(16, sr_reg, Imm16(~SR_LOGIC_ZERO));
    SetJumpTarget(exit);
    m_gpr.PutReg(DSP_REG_SR);
  }
}

//----

// TST
// 1011 r001 xxxx xxxx
// Test accumulator %acR.
//
// flags out: --xx xx00
void DSPEmitterIR::tst(const UDSPInstruction opc)
{
  if (FlagsNeeded())
  {
    u8 reg = (opc >> 11) & 0x1;
    //		s64 acc = dsp_get_long_acc(reg);

    m_gpr.ReadReg(DSP_REG_ACC0_64 + reg, RAX, RegisterExtension::None);

    //		Update_SR_Register64(acc);
    Update_SR_Register64(RAX, RDX);
  }
}

// TSTAXH $axR.h
// 1000 011r xxxx xxxx
// Test high part of secondary accumulator $axR.h.
//
// flags out: --x0 xx00
void DSPEmitterIR::tstaxh(const UDSPInstruction opc)
{
  if (FlagsNeeded())
  {
    u8 reg = (opc >> 8) & 0x1;
    //		s16 val = dsp_get_ax_h(reg);
    m_gpr.ReadReg(reg + DSP_REG_AXH0, RAX, RegisterExtension::Sign);
    //		Update_SR_Register16(val);
    Update_SR_Register16(RAX);
  }
}

//----

// CMP
// 1000 0010 xxxx xxxx
// Compares accumulator $ac0 with accumulator $ac1.
//
// flags out: x-xx xxxx
void DSPEmitterIR::cmp(const UDSPInstruction opc)
{
  if (FlagsNeeded())
  {
    X64Reg tmp1 = m_gpr.GetFreeXReg();
    //		s64 acc0 = dsp_get_long_acc(0);

    m_gpr.ReadReg(DSP_REG_ACC0_64, tmp1, RegisterExtension::None);

    MOV(64, R(RAX), R(tmp1));
    //		s64 acc1 = dsp_get_long_acc(1);

    m_gpr.ReadReg(DSP_REG_ACC1_64, RDX, RegisterExtension::None);

    //		s64 res = dsp_convert_long_acc(acc0 - acc1);
    SUB(64, R(RAX), R(RDX));
    //		Update_SR_Register64(res, isCarry2(acc0, res), isOverflow(acc0, -acc1, res)); // CF ->
    // influence on ABS/0xa100
    NEG(64, R(RDX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
    m_gpr.PutXReg(tmp1);
  }
}

// CMPAR $acS axR.h
// 110r s001 xxxx xxxx
// Compares accumulator $acS with accumulator axR.h.
// Not described by Duddie's doc - at least not as a separate instruction.
//
// flags out: x-xx xxxx
void DSPEmitterIR::cmpar(const UDSPInstruction opc)
{
  if (FlagsNeeded())
  {
    u8 rreg = ((opc >> 12) & 0x1);
    u8 sreg = (opc >> 11) & 0x1;

    X64Reg tmp1 = m_gpr.GetFreeXReg();
    //		s64 sr = dsp_get_long_acc(sreg);

    m_gpr.ReadReg(DSP_REG_ACC0_64 + sreg, tmp1, RegisterExtension::None);

    MOV(64, R(RAX), R(tmp1));
    //		s64 rr = (s16)g_dsp.r.axh[rreg];
    m_gpr.ReadReg(rreg + DSP_REG_AXH0, RDX, RegisterExtension::Sign);
    //		rr <<= 16;
    SHL(64, R(RDX), Imm8(16));
    //		s64 res = dsp_convert_long_acc(sr - rr);
    SUB(64, R(RAX), R(RDX));
    //		Update_SR_Register64(res, isCarry2(sr, res), isOverflow(sr, -rr, res));
    NEG(64, R(RDX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
    m_gpr.PutXReg(tmp1);
  }
}

// CMPI $amD, #I
// 0000 001r 1000 0000
// iiii iiii iiii iiii
// Compares mid accumulator $acD.hm ($amD) with sign extended immediate value I.
// Although flags are being set regarding whole accumulator register.
//
// flags out: x-xx xxxx
void DSPEmitterIR::cmpi(const UDSPInstruction opc)
{
  if (FlagsNeeded())
  {
    const u8 reg = (opc >> 8) & 0x1;
    const X64Reg tmp1 = m_gpr.GetFreeXReg();
    //		s64 val = dsp_get_long_acc(reg);

    m_gpr.ReadReg(DSP_REG_ACC0_64 + reg, tmp1, RegisterExtension::None);

    MOV(64, R(RAX), R(tmp1));
    //		s64 imm = (s64)(s16)dsp_fetch_code() << 16; // Immediate is considered to be at M level in
    // the 40-bit accumulator.
    const u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
    MOV(64, R(RDX), Imm64((s64)(s16)imm << 16));
    //		s64 res = dsp_convert_long_acc(val - imm);
    SUB(64, R(RAX), R(RDX));
    //		Update_SR_Register64(res, isCarry2(val, res), isOverflow(val, -imm, res));
    NEG(64, R(RDX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
    m_gpr.PutXReg(tmp1);
  }
}

// CMPIS $acD, #I
// 0000 011d iiii iiii
// Compares accumulator with short immediate. Comparison is executed
// by subtracting short immediate (8bit sign extended) from mid accumulator
// $acD.hm and computing flags based on whole accumulator $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::cmpis(const UDSPInstruction opc)
{
  if (FlagsNeeded())
  {
    u8 areg = (opc >> 8) & 0x1;
    //		s64 acc = dsp_get_long_acc(areg);
    X64Reg tmp1 = m_gpr.GetFreeXReg();
    m_gpr.ReadReg(DSP_REG_ACC0_64 + areg, tmp1, RegisterExtension::None);
    MOV(64, R(RAX), R(tmp1));
    //		s64 val = (s8)opc;
    //		val <<= 16;
    MOV(64, R(RDX), Imm64((s64)(s8)opc << 16));
    //		s64 res = dsp_convert_long_acc(acc - val);
    SUB(64, R(RAX), R(RDX));
    //		Update_SR_Register64(res, isCarry2(acc, res), isOverflow(acc, -val, res));
    NEG(64, R(RDX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
    m_gpr.PutXReg(tmp1);
  }
}

//----

// XORR $acD.m, $axS.h
// 0011 00sd 0xxx xxxx
// Logic XOR (exclusive or) middle part of accumulator $acD.m with
// high part of secondary accumulator $axS.h.
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::xorr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  //	u16 accm = g_dsp.r.acm[dreg] ^ g_dsp.r.axh[sreg];
  m_gpr.ReadReg(dreg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  m_gpr.ReadReg(sreg + DSP_REG_AXH0, RDX, RegisterExtension::Sign);
  XOR(64, R(RAX), R(RDX));
  //	g_dsp.r.acm[dreg] = accm;
  m_gpr.WriteReg(dreg + DSP_REG_ACM0, R(RAX));
  //	Update_SR_Register16((s16)accm, false, false, isOverS32(dsp_get_long_acc(dreg)));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(DSP_REG_ACC0_64 + dreg, RCX, RegisterExtension::Sign);

    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

// ANDR $acD.m, $axS.h
// 0011 01sd 0xxx xxxx
// Logic AND middle part of accumulator $acD.m with high part of
// secondary accumulator $axS.h.
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::andr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  //	u16 accm = g_dsp.r.acm[dreg] & g_dsp.r.axh[sreg];
  m_gpr.ReadReg(dreg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  m_gpr.ReadReg(sreg + DSP_REG_AXH0, RDX, RegisterExtension::Sign);
  AND(64, R(RAX), R(RDX));
  //	g_dsp.r.acm[dreg] = accm;
  m_gpr.WriteReg(dreg + DSP_REG_ACM0, R(RAX));
  //	Update_SR_Register16((s16)accm, false, false, isOverS32(dsp_get_long_acc(dreg)));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(DSP_REG_ACC0_64 + dreg, RCX, RegisterExtension::Sign);

    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

// ORR $acD.m, $axS.h
// 0011 10sd 0xxx xxxx
// Logic OR middle part of accumulator $acD.m with high part of
// secondary accumulator $axS.h.
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::orr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  //	u16 accm = g_dsp.r.acm[dreg] | g_dsp.r.axh[sreg];
  m_gpr.ReadReg(dreg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  m_gpr.ReadReg(sreg + DSP_REG_AXH0, RDX, RegisterExtension::Sign);
  OR(64, R(RAX), R(RDX));
  //	g_dsp.r.acm[dreg] = accm;
  m_gpr.WriteReg(dreg + DSP_REG_ACM0, R(RAX));
  //	Update_SR_Register16((s16)accm, false, false, isOverS32(dsp_get_long_acc(dreg)));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(DSP_REG_ACC0_64 + dreg, RCX, RegisterExtension::Sign);

    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

// ANDC $acD.m, $ac(1-D).m
// 0011 110d 0xxx xxxx
// Logic AND middle part of accumulator $acD.m with middle part of
// accumulator $ac(1-D).m
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::andc(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  //	u16 accm = g_dsp.r.acm[dreg] & g_dsp.r.acm[1 - dreg];
  m_gpr.ReadReg(dreg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  m_gpr.ReadReg(1 - dreg + DSP_REG_ACM0, RDX, RegisterExtension::Sign);
  AND(64, R(RAX), R(RDX));
  //	g_dsp.r.acm[dreg] = accm;
  m_gpr.WriteReg(dreg + DSP_REG_ACM0, R(RAX));
  //	Update_SR_Register16((s16)accm, false, false, isOverS32(dsp_get_long_acc(dreg)));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(DSP_REG_ACC0_64 + dreg, RCX, RegisterExtension::Sign);

    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

// ORC $acD.m, $ac(1-D).m
// 0011 111d 0xxx xxxx
// Logic OR middle part of accumulator $acD.m with middle part of
// accumulator $ac(1-D).m.
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::orc(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  //	u16 accm = g_dsp.r.acm[dreg] | g_dsp.r.acm[1 - dreg];
  m_gpr.ReadReg(dreg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  m_gpr.ReadReg(1 - dreg + DSP_REG_ACM0, RDX, RegisterExtension::Sign);
  OR(64, R(RAX), R(RDX));
  //	g_dsp.r.acm[dreg] = accm;
  m_gpr.WriteReg(dreg + DSP_REG_ACM0, R(RAX));
  //	Update_SR_Register16((s16)accm, false, false, isOverS32(dsp_get_long_acc(dreg)));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(DSP_REG_ACC0_64 + dreg, RCX, RegisterExtension::Sign);

    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

// XORC $acD.m
// 0011 000d 1xxx xxxx
// Logic XOR (exclusive or) middle part of accumulator $acD.m with $ac(1-D).m
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::xorc(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  //	u16 accm = g_dsp.r.acm[dreg] ^ g_dsp.r.acm[1 - dreg];
  m_gpr.ReadReg(dreg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  m_gpr.ReadReg(1 - dreg + DSP_REG_ACM0, RDX, RegisterExtension::Sign);
  XOR(64, R(RAX), R(RDX));
  //	g_dsp.r.acm[dreg] = accm;
  m_gpr.WriteReg(dreg + DSP_REG_ACM0, R(RAX));
  //	Update_SR_Register16((s16)accm, false, false, isOverS32(dsp_get_long_acc(dreg)));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(DSP_REG_ACC0_64 + dreg, RCX, RegisterExtension::Sign);

    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

// NOT $acD.m
// 0011 001d 1xxx xxxx
// Invert all bits in dest reg, aka xor with 0xffff
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::notc(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  //	u16 accm = g_dsp.r.acm[dreg] ^ 0xffff;
  m_gpr.ReadReg(dreg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  NOT(16, R(AX));
  //	g_dsp.r.acm[dreg] = accm;
  m_gpr.WriteReg(dreg + DSP_REG_ACM0, R(RAX));
  //	Update_SR_Register16((s16)accm, false, false, isOverS32(dsp_get_long_acc(dreg)));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(DSP_REG_ACC0_64 + dreg, RCX, RegisterExtension::Sign);

    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

// XORI $acD.m, #I
// 0000 001r 0010 0000
// iiii iiii iiii iiii
// Logic exclusive or (XOR) of accumulator mid part $acD.m with
// immediate value I.
//
// flags out: --xx xx00
void DSPEmitterIR::xori(const UDSPInstruction opc)
{
  const u8 reg = (opc >> 8) & 0x1;
  //	u16 imm = dsp_fetch_code();
  const u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  //	g_dsp.r.acm[reg] ^= imm;
  m_gpr.ReadReg(reg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  XOR(16, R(RAX), Imm16(imm));
  m_gpr.WriteReg(reg + DSP_REG_ACM0, R(RAX));
  //	Update_SR_Register16((s16)g_dsp.r.acm[reg], false, false, isOverS32(dsp_get_long_acc(reg)));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(DSP_REG_ACC0_64 + reg, RCX, RegisterExtension::Sign);

    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

// ANDI $acD.m, #I
// 0000 001r 0100 0000
// iiii iiii iiii iiii
// Logic AND of accumulator mid part $acD.m with immediate value I.
//
// flags out: --xx xx00
void DSPEmitterIR::andi(const UDSPInstruction opc)
{
  const u8 reg = (opc >> 8) & 0x1;
  //	u16 imm = dsp_fetch_code();
  const u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  //	g_dsp.r.acm[reg] &= imm;
  m_gpr.ReadReg(reg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  AND(16, R(RAX), Imm16(imm));
  m_gpr.WriteReg(reg + DSP_REG_ACM0, R(RAX));
  //	Update_SR_Register16((s16)g_dsp.r.acm[reg], false, false, isOverS32(dsp_get_long_acc(reg)));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(DSP_REG_ACC0_64 + reg, RCX, RegisterExtension::Sign);

    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

// ORI $acD.m, #I
// 0000 001r 0110 0000
// iiii iiii iiii iiii
// Logic OR of accumulator mid part $acD.m with immediate value I.
//
// flags out: --xx xx00
void DSPEmitterIR::ori(const UDSPInstruction opc)
{
  const u8 reg = (opc >> 8) & 0x1;
  //	u16 imm = dsp_fetch_code();
  const u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  //	g_dsp.r.acm[reg] |= imm;
  m_gpr.ReadReg(reg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  OR(16, R(RAX), Imm16(imm));
  m_gpr.WriteReg(reg + DSP_REG_ACM0, R(RAX));
  //	Update_SR_Register16((s16)g_dsp.r.acm[reg], false, false, isOverS32(dsp_get_long_acc(reg)));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(DSP_REG_ACC0_64 + reg, RCX, RegisterExtension::Sign);

    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

//----

// ADDR $acD.M, $axS.L
// 0100 0ssd xxxx xxxx
// Adds register $axS.L to accumulator $acD.M register.
//
// flags out: x-xx xxxx
void DSPEmitterIR::addr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = ((opc >> 9) & 0x3) + DSP_REG_AXL0;

  //	s64 acc = dsp_get_long_acc(dreg);
  X64Reg tmp1 = m_gpr.GetFreeXReg();

  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  MOV(64, R(RAX), R(tmp1));
  //	s64 ax = (s16)g_dsp.r[sreg];
  m_gpr.ReadReg(sreg, RDX, RegisterExtension::Sign);
  //	ax <<= 16;
  SHL(64, R(RDX), Imm8(16));
  //	s64 res = acc + ax;
  ADD(64, R(RAX), R(RDX));
  //	dsp_set_long_acc(dreg, res);
  //	Update_SR_Register64(res, isCarry(acc, res), isOverflow(acc, ax, res));
  if (FlagsNeeded())
  {
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

// ADDAX $acD, $axS
// 0100 10sd xxxx xxxx
// Adds secondary accumulator $axS to accumulator register $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::addax(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  MOV(64, R(RAX), R(tmp1));
  //	s64 ax  = dsp_get_long_acx(sreg);
  m_gpr.ReadReg(sreg + DSP_REG_AX0_32, RDX, RegisterExtension::Sign);
  //	s64 res = acc + ax;
  ADD(64, R(RAX), R(RDX));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry(acc, res), isOverflow(acc, ax, res));
  if (FlagsNeeded())
  {
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

// ADD $acD, $ac(1-D)
// 0100 110d xxxx xxxx
// Adds accumulator $ac(1-D) to accumulator register $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::add(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	s64 acc0 = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  MOV(64, R(RAX), R(tmp1));
  //	s64 acc1 = dsp_get_long_acc(1 - dreg);

  m_gpr.ReadReg(DSP_REG_ACC0_64 + 1 - dreg, RDX, RegisterExtension::None);

  //	s64 res = acc0 + acc1;
  ADD(64, R(RAX), R(RDX));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry(acc0, res), isOverflow(acc0, acc1, res));
  if (FlagsNeeded())
  {
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

// ADDP $acD
// 0100 111d xxxx xxxx
// Adds product register to accumulator register.
//
// flags out: x-xx xxxx
void DSPEmitterIR::addp(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  MOV(64, R(RAX), R(tmp1));
  //	s64 prod = dsp_get_long_prod();
  get_long_prod(RDX, tmp2);
  //	s64 res = acc + prod;
  ADD(64, R(RAX), R(RDX));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry(acc, res), isOverflow(acc, prod, res));
  if (FlagsNeeded())
  {
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// ADDAXL $acD, $axS.l
// 0111 00sd xxxx xxxx
// Adds secondary accumulator $axS.l to accumulator register $acD.
// should be unsigned values!!
//
// flags out: x-xx xxxx
void DSPEmitterIR::addaxl(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 9) & 0x1;
  u8 dreg = (opc >> 8) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	u64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  MOV(64, R(RAX), R(tmp1));
  //	u16 acx = (u16)dsp_get_ax_l(sreg);
  m_gpr.ReadReg(sreg + DSP_REG_AXL0, RDX, RegisterExtension::Sign);
  MOVZX(64, 16, RDX, R(RDX));
  //	u64 res = acc + acx;
  ADD(64, R(RAX), R(RDX));
  //	dsp_set_long_acc(dreg, (s64)res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64((s64)res, isCarry(acc, res), isOverflow((s64)acc, (s64)acx, (s64)res));
  if (FlagsNeeded())
  {
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

// ADDI $amR, #I
// 0000 001r 0000 0000
// iiii iiii iiii iiii
// Adds immediate (16-bit sign extended) to mid accumulator $acD.hm.
//
// flags out: x-xx xxxx
void DSPEmitterIR::addi(const UDSPInstruction opc)
{
  u8 areg = (opc >> 8) & 0x1;
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	s64 acc = dsp_get_long_acc(areg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + areg);
  MOV(64, R(tmp1), accreg);

  MOV(64, R(RAX), R(tmp1));
  //	s64 imm = (s16)dsp_fetch_code();
  const s16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  // imm <<= 16;
  MOV(64, R(RDX), Imm32(imm << 16));
  //	s64 res = acc + imm;
  ADD(64, R(RAX), R(RDX));
  //	dsp_set_long_acc(areg, res);
  //	res = dsp_get_long_acc(areg);
  //	Update_SR_Register64(res, isCarry(acc, res), isOverflow(acc, imm, res));
  if (FlagsNeeded())
  {
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + areg);
  m_gpr.PutXReg(tmp1);
}

// ADDIS $acD, #I
// 0000 010d iiii iiii
// Adds short immediate (8-bit sign extended) to mid accumulator $acD.hm.
//
// flags out: x-xx xxxx
void DSPEmitterIR::addis(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  MOV(64, R(RAX), R(tmp1));
  //	s64 imm = (s8)(u8)opc;
  //	imm <<= 16;
  s32 imm = static_cast<u8>(opc) << 24 >> 8;
  MOV(64, R(RDX), Imm32(imm));
  //	s64 res = acc + imm;
  ADD(64, R(RAX), R(RDX));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry(acc, res), isOverflow(acc, imm, res));
  if (FlagsNeeded())
  {
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

// INCM $acsD
// 0111 010d xxxx xxxx
// Increment 24-bit mid-accumulator $acsD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::incm(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  s64 subtract = 0x10000;
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  //	s64 res = acc + sub;
  LEA(64, RAX, MDisp(tmp1, subtract));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry(acc, res), isOverflow(acc, subtract, res));
  if (FlagsNeeded())
  {
    MOV(64, R(RDX), Imm32((u32)subtract));
    MOV(64, R(RCX), R(RAX));

    MOV(64, accreg, R(RCX));

    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

// INC $acD
// 0111 011d xxxx xxxx
// Increment accumulator $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::inc(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  //	s64 res = acc + 1;
  LEA(64, RAX, MDisp(tmp1, 1));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry(acc, res), isOverflow(acc, 1, res));
  if (FlagsNeeded())
  {
    MOV(64, R(RDX), Imm64(1));
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

//----

// SUBR $acD.M, $axS.L
// 0101 0ssd xxxx xxxx
// Subtracts register $axS.L from accumulator $acD.M register.
//
// flags out: x-xx xxxx
void DSPEmitterIR::subr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = ((opc >> 9) & 0x3) + DSP_REG_AXL0;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  MOV(64, R(RAX), R(tmp1));
  //	s64 ax = (s16)g_dsp.r[sreg];
  m_gpr.ReadReg(sreg, RDX, RegisterExtension::Sign);
  //	ax <<= 16;
  SHL(64, R(RDX), Imm8(16));
  //	s64 res = acc - ax;
  SUB(64, R(RAX), R(RDX));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry2(acc, res), isOverflow(acc, -ax, res));
  if (FlagsNeeded())
  {
    NEG(64, R(RDX));
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

// SUBAX $acD, $axS
// 0101 10sd xxxx xxxx
// Subtracts secondary accumulator $axS from accumulator register $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::subax(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  MOV(64, R(RAX), R(tmp1));
  //	s64 acx = dsp_get_long_acx(sreg);
  m_gpr.ReadReg(sreg + DSP_REG_AX0_32, RDX, RegisterExtension::Sign);
  //	s64 res = acc - acx;
  SUB(64, R(RAX), R(RDX));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry2(acc, res), isOverflow(acc, -acx, res));
  if (FlagsNeeded())
  {
    NEG(64, R(RDX));
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

// SUB $acD, $ac(1-D)
// 0101 110d xxxx xxxx
// Subtracts accumulator $ac(1-D) from accumulator register $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::sub(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	s64 acc1 = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  MOV(64, R(RAX), R(tmp1));
  //	s64 acc2 = dsp_get_long_acc(1 - dreg);
  m_gpr.ReadReg(DSP_REG_ACC0_64 + 1 - dreg, RDX, RegisterExtension::None);
  //	s64 res = acc1 - acc2;
  SUB(64, R(RAX), R(RDX));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry2(acc1, res), isOverflow(acc1, -acc2, res));
  if (FlagsNeeded())
  {
    NEG(64, R(RDX));
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

// SUBP $acD
// 0101 111d xxxx xxxx
// Subtracts product register from accumulator register.
//
// flags out: x-xx xxxx
void DSPEmitterIR::subp(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  MOV(64, R(RAX), R(tmp1));
  //	s64 prod = dsp_get_long_prod();
  get_long_prod(RDX, tmp2);
  //	s64 res = acc - prod;
  SUB(64, R(RAX), R(RDX));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry2(acc, res), isOverflow(acc, -prod, res));
  if (FlagsNeeded())
  {
    NEG(64, R(RDX));
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// DECM $acsD
// 0111 100d xxxx xxxx
// Decrement 24-bit mid-accumulator $acsD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::decm(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x01;
  s64 subtract = 0x10000;
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  //	s64 res = acc - sub;
  LEA(64, RAX, MDisp(tmp1, -subtract));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry2(acc, res), isOverflow(acc, -subtract, res));
  if (FlagsNeeded())
  {
    MOV(64, R(RDX), Imm64(-subtract));
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

// DEC $acD
// 0111 101d xxxx xxxx
// Decrement accumulator $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::dec(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x01;
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(tmp1), accreg);

  //	s64 res = acc - 1;
  LEA(64, RAX, MDisp(tmp1, -1));
  //	dsp_set_long_acc(dreg, res);
  //	res = dsp_get_long_acc(dreg);
  //	Update_SR_Register64(res, isCarry2(acc, res), isOverflow(acc, -1, res));
  if (FlagsNeeded())
  {
    MOV(64, R(RDX), Imm64(-1));
    MOV(64, R(RCX), R(RAX));
    MOV(64, accreg, R(RCX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
  }
  else
  {
    MOV(64, accreg, R(RAX));
  }
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  m_gpr.PutXReg(tmp1);
}

//----

// NEG $acD
// 0111 110d xxxx xxxx
// Negate accumulator $acD.
//
// flags out: --xx xx00
void DSPEmitterIR::neg(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(RAX), accreg);
  //	acc = 0 - acc;
  NEG(64, R(RAX));
  //	dsp_set_long_acc(dreg, acc);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  //	Update_SR_Register64(dsp_get_long_acc(dreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

// ABS  $acD
// 1010 d001 xxxx xxxx
// absolute value of $acD
//
// flags out: --xx xx00
void DSPEmitterIR::abs(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 11) & 0x1;

  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(RAX), accreg);
  //	if (acc < 0) acc = 0 - acc;
  TEST(64, R(RAX), R(RAX));
  FixupBranch GreaterThanOrEqual = J_CC(CC_GE);
  NEG(64, R(RAX));
  MOV(64, accreg, R(RAX));
  SetJumpTarget(GreaterThanOrEqual);
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  //	Update_SR_Register64(dsp_get_long_acc(dreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}
//----

// MOVR $acD, $axS.R
// 0110 0srd xxxx xxxx
// Moves register $axS.R (sign extended) to middle accumulator $acD.hm.
// Sets $acD.l to 0.
// TODO: Check what happens to acD.h.
//
// flags out: --xx xx00
void DSPEmitterIR::movr(const UDSPInstruction opc)
{
  u8 areg = (opc >> 8) & 0x1;
  u8 sreg = ((opc >> 9) & 0x3) + DSP_REG_AXL0;

  //	s64 acc = (s16)g_dsp.r[sreg];
  m_gpr.ReadReg(sreg, RAX, RegisterExtension::Sign);
  //	acc <<= 16;
  SHL(64, R(RAX), Imm8(16));
  //	acc &= ~0xffff;
  //	dsp_set_long_acc(areg, acc);
  m_gpr.WriteReg(DSP_REG_ACC0_64 + areg, R(RAX));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

// MOVAX $acD, $axS
// 0110 10sd xxxx xxxx
// Moves secondary accumulator $axS to accumulator $axD.
//
// flags out: --xx xx00
void DSPEmitterIR::movax(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;

  //	s64 acx = dsp_get_long_acx(sreg);
  m_gpr.ReadReg(sreg + DSP_REG_AX0_32, RAX, RegisterExtension::Sign);
  //	dsp_set_long_acc(dreg, acx);
  m_gpr.WriteReg(DSP_REG_ACC0_64 + dreg, R(RAX));
  //	Update_SR_Register64(acx);
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

// MOV $acD, $ac(1-D)
// 0110 110d xxxx xxxx
// Moves accumulator $ax(1-D) to accumulator $axD.
//
// flags out: --x0 xx00
void DSPEmitterIR::mov(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  //	u64 acc = dsp_get_long_acc(1 - dreg);
  m_gpr.ReadReg(DSP_REG_ACC0_64 + 1 - dreg, RAX, RegisterExtension::None);
  //	dsp_set_long_acc(dreg, acc);
  m_gpr.WriteReg(DSP_REG_ACC0_64 + dreg, R(RAX));
  //	Update_SR_Register64(acc);
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

//----

// LSL16 $acR
// 1111 000r xxxx xxxx
// Logically shifts left accumulator $acR by 16.
//
// flags out: --xx xx00
void DSPEmitterIR::lsl16(const UDSPInstruction opc)
{
  u8 areg = (opc >> 8) & 0x1;
  //	s64 acc = dsp_get_long_acc(areg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + areg);
  MOV(64, R(RAX), accreg);
  //	acc <<= 16;
  SHL(64, R(RAX), Imm8(16));
  //	dsp_set_long_acc(areg, acc);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + areg);
  //	Update_SR_Register64(dsp_get_long_acc(areg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

// LSR16 $acR
// 1111 010r xxxx xxxx
// Logically shifts right accumulator $acR by 16.
//
// flags out: --xx xx00
void DSPEmitterIR::lsr16(const UDSPInstruction opc)
{
  u8 areg = (opc >> 8) & 0x1;

  //	u64 acc = dsp_get_long_acc(areg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + areg);
  MOV(64, R(RAX), accreg);
  //	acc &= 0x000000FFFFFFFFFFULL; 	// Lop off the extraneous sign extension our 64-bit fake accum
  // causes
  //	acc >>= 16;
  SHR(64, R(RAX), Imm8(16));
  AND(32, R(EAX), Imm32(0xffffff));
  //	dsp_set_long_acc(areg, (s64)acc);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + areg);
  //	Update_SR_Register64(dsp_get_long_acc(areg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

// ASR16 $acR
// 1001 r001 xxxx xxxx
// Arithmetically shifts right accumulator $acR by 16.
//
// flags out: --xx xx00
void DSPEmitterIR::asr16(const UDSPInstruction opc)
{
  u8 areg = (opc >> 11) & 0x1;

  //	s64 acc = dsp_get_long_acc(areg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + areg);
  MOV(64, R(RAX), accreg);
  //	acc >>= 16;
  SAR(64, R(RAX), Imm8(16));
  //	dsp_set_long_acc(areg, acc);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + areg);
  //	Update_SR_Register64(dsp_get_long_acc(areg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

// LSL $acR, #I
// 0001 010r 00ii iiii
// Logically shifts left accumulator $acR by number specified by value I.
//
// flags out: --xx xx00
void DSPEmitterIR::lsl(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x01;
  u16 shift = opc & 0x3f;
  //	u64 acc = dsp_get_long_acc(rreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg);
  MOV(64, R(RAX), accreg);

  //	acc <<= shift;
  SHL(64, R(RAX), Imm8((u8)shift));

  //	dsp_set_long_acc(rreg, acc);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

// LSR $acR, #I
// 0001 010r 01ii iiii
// Logically shifts right accumulator $acR by number specified by value
// calculated by negating sign extended bits 0-6.
//
// flags out: --xx xx00
void DSPEmitterIR::lsr(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x01;
  u16 shift;
  //	u64 acc = dsp_get_long_acc(rreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg);
  MOV(64, R(RAX), accreg);

  if ((opc & 0x3f) == 0)
    shift = 0;
  else
    shift = 0x40 - (opc & 0x3f);

  if (shift)
  {
    //	acc &= 0x000000FFFFFFFFFFULL; 	// Lop off the extraneous sign extension our 64-bit fake
    // accum causes
    SHL(64, R(RAX), Imm8(24));
    //	acc >>= shift;
    SHR(64, R(RAX), Imm8(shift + 24));
  }

  //	dsp_set_long_acc(rreg, (s64)acc);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

// ASL $acR, #I
// 0001 010r 10ii iiii
// Logically shifts left accumulator $acR by number specified by value I.
//
// flags out: --xx xx00
void DSPEmitterIR::asl(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x01;
  u16 shift = opc & 0x3f;
  //	u64 acc = dsp_get_long_acc(rreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + rreg);
  MOV(64, R(RAX), accreg);
  //	acc <<= shift;
  SHL(64, R(RAX), Imm8((u8)shift));
  //	dsp_set_long_acc(rreg, acc);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + rreg);
  //	Update_SR_Register64(dsp_get_long_acc(rreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

// ASR $acR, #I
// 0001 010r 11ii iiii
// Arithmetically shifts right accumulator $acR by number specified by
// value calculated by negating sign extended bits 0-6.
//
// flags out: --xx xx00
void DSPEmitterIR::asr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x01;
  u16 shift;

  if ((opc & 0x3f) == 0)
    shift = 0;
  else
    shift = 0x40 - (opc & 0x3f);

  // arithmetic shift
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(RAX), accreg);
  //	acc >>= shift;
  SAR(64, R(RAX), Imm8((u8)shift));

  //	dsp_set_long_acc(dreg, acc);
  MOV(64, accreg, R(RAX));
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  //	Update_SR_Register64(dsp_get_long_acc(dreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

// LSRN  (fixed parameters)
// 0000 0010 1100 1010
// Logically shifts right accumulator $ACC0 by lower 7-bit (signed) value in $AC1.M
// (if value negative, becomes left shift).
//
// flags out: --xx xx00
void DSPEmitterIR::lsrn(const UDSPInstruction opc)
{
  //	s16 shift;
  //	u16 accm = (u16)dsp_get_acc_m(1);
  m_gpr.ReadReg(DSP_REG_ACM1, RAX, RegisterExtension::Sign);
  //	u64 acc = dsp_get_long_acc(0);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64);
  MOV(64, R(RDX), accreg);
  //	acc &= 0x000000FFFFFFFFFFULL;
  SHL(64, R(RDX), Imm8(24));
  SHR(64, R(RDX), Imm8(24));

  //	if ((accm & 0x3f) == 0)
  //		shift = 0;
  //	else if (accm & 0x40)
  //		shift = -0x40 + (accm & 0x3f);
  //	else
  //		shift = accm & 0x3f;

  //	if (shift > 0)
  //	{
  //		acc >>= shift;
  //	}
  //	else if (shift < 0)
  //	{
  //		acc <<= -shift;
  //	}

  TEST(64, R(RDX), R(RDX));  // is this actually worth the branch cost?
  FixupBranch zero = J_CC(CC_E);
  TEST(16, R(RAX), Imm16(0x3f));  // is this actually worth the branch cost?
  FixupBranch noShift = J_CC(CC_Z);
  // CL gets automatically masked with 0x3f on IA32/AMD64
  // MOVZX(64, 16, RCX, R(RAX));
  MOV(64, R(RCX), R(RAX));
  // AND(16, R(RCX), Imm16(0x3f));
  TEST(16, R(RAX), Imm16(0x40));
  FixupBranch shiftLeft = J_CC(CC_Z);
  NEG(16, R(RCX));
  // ADD(16, R(RCX), Imm16(0x40));
  SHL(64, R(RDX), R(RCX));
  FixupBranch exit = J();
  SetJumpTarget(shiftLeft);
  SHR(64, R(RDX), R(RCX));
  SetJumpTarget(noShift);
  SetJumpTarget(exit);

  //	dsp_set_long_acc(0, (s64)acc);

  MOV(64, accreg, R(RDX));

  SetJumpTarget(zero);
  m_gpr.PutReg(DSP_REG_ACC0_64);
  //	Update_SR_Register64(dsp_get_long_acc(0));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RDX, RCX);
  }
}

// ASRN  (fixed parameters)
// 0000 0010 1100 1011
// Arithmetically shifts right accumulator $ACC0 by lower 7-bit (signed) value in $AC1.M
// (if value negative, becomes left shift).
//
// flags out: --xx xx00
void DSPEmitterIR::asrn(const UDSPInstruction opc)
{
  //	s16 shift;
  //	u16 accm = (u16)dsp_get_acc_m(1);
  m_gpr.ReadReg(DSP_REG_ACM1, RAX, RegisterExtension::Sign);
  //	s64 acc = dsp_get_long_acc(0);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64);
  MOV(64, R(RDX), accreg);

  //	if ((accm & 0x3f) == 0)
  //		shift = 0;
  //	else if (accm & 0x40)
  //		shift = -0x40 + (accm & 0x3f);
  //	else
  //		shift = accm & 0x3f;

  //	if (shift > 0)
  //	{
  //		acc >>= shift;
  //	}
  //	else if (shift < 0)
  //	{
  //		acc <<= -shift;
  //	}

  TEST(64, R(RDX), R(RDX));
  FixupBranch zero = J_CC(CC_E);
  TEST(16, R(RAX), Imm16(0x3f));
  FixupBranch noShift = J_CC(CC_Z);
  MOVZX(64, 16, RCX, R(RAX));
  AND(16, R(RCX), Imm16(0x3f));
  TEST(16, R(RAX), Imm16(0x40));
  FixupBranch shiftLeft = J_CC(CC_Z);
  NEG(16, R(RCX));
  ADD(16, R(RCX), Imm16(0x40));
  SHL(64, R(RDX), R(RCX));
  FixupBranch exit = J();
  SetJumpTarget(shiftLeft);
  SAR(64, R(RDX), R(RCX));
  SetJumpTarget(noShift);
  SetJumpTarget(exit);

  //	dsp_set_long_acc(0, acc);
  //	Update_SR_Register64(dsp_get_long_acc(0));

  MOV(64, accreg, R(RDX));

  SetJumpTarget(zero);
  m_gpr.PutReg(DSP_REG_ACC0_64);
  if (FlagsNeeded())
  {
    Update_SR_Register64(RDX, RCX);
  }
}

// LSRNRX $acD, $axS.h
// 0011 01sd 1xxx xxxx
// Logically shifts left/right accumulator $ACC[D] by lower 7-bit (signed) value in $AX[S].H
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::lsrnrx(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;

  //	s16 shift;
  //	u16 axh = g_dsp.r.axh[sreg];
  m_gpr.ReadReg(sreg + DSP_REG_AXH0, RAX, RegisterExtension::Sign);
  //	u64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(RDX), accreg);
  //	acc &= 0x000000FFFFFFFFFFULL;
  SHL(64, R(RDX), Imm8(24));
  SHR(64, R(RDX), Imm8(24));

  //	if ((axh & 0x3f) == 0)
  //		shift = 0;
  //	else if (axh & 0x40)
  //		shift = -0x40 + (axh & 0x3f);
  //	else
  //		shift = axh & 0x3f;

  //	if (shift > 0)
  //	{
  //		acc <<= shift;
  //	}
  //	else if (shift < 0)
  //	{
  //		acc >>= -shift;
  //	}

  TEST(64, R(RDX), R(RDX));
  FixupBranch zero = J_CC(CC_E);
  TEST(16, R(RAX), Imm16(0x3f));
  FixupBranch noShift = J_CC(CC_Z);
  MOVZX(64, 16, RCX, R(RAX));
  AND(16, R(RCX), Imm16(0x3f));
  TEST(16, R(RAX), Imm16(0x40));
  FixupBranch shiftLeft = J_CC(CC_Z);
  NEG(16, R(RCX));
  ADD(16, R(RCX), Imm16(0x40));
  SHR(64, R(RDX), R(RCX));
  FixupBranch exit = J();
  SetJumpTarget(shiftLeft);
  SHL(64, R(RDX), R(RCX));
  SetJumpTarget(noShift);
  SetJumpTarget(exit);

  //	dsp_set_long_acc(dreg, (s64)acc);
  //	Update_SR_Register64(dsp_get_long_acc(dreg));

  MOV(64, accreg, R(RDX));

  SetJumpTarget(zero);
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  if (FlagsNeeded())
  {
    Update_SR_Register64(RDX, RCX);
  }
}

// ASRNRX $acD, $axS.h
// 0011 10sd 1xxx xxxx
// Arithmetically shifts left/right accumulator $ACC[D] by lower 7-bit (signed) value in $AX[S].H
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::asrnrx(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;

  //	s16 shift;
  //	u16 axh = g_dsp.r.axh[sreg];
  m_gpr.ReadReg(sreg + DSP_REG_AXH0, RAX, RegisterExtension::Sign);
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(RDX), accreg);

  //	if ((axh & 0x3f) == 0)
  //		shift = 0;
  //	else if (axh & 0x40)
  //		shift = -0x40 + (axh & 0x3f);
  //	else
  //		shift = axh & 0x3f;

  //	if (shift > 0) {
  //		acc <<= shift;
  //	} else if (shift < 0) {
  //		acc >>= -shift;
  //	}

  TEST(64, R(RDX), R(RDX));
  FixupBranch zero = J_CC(CC_E);
  TEST(16, R(RAX), Imm16(0x3f));
  FixupBranch noShift = J_CC(CC_Z);
  MOVZX(64, 16, RCX, R(RAX));
  AND(16, R(RCX), Imm16(0x3f));
  TEST(16, R(RAX), Imm16(0x40));
  FixupBranch shiftLeft = J_CC(CC_Z);
  NEG(16, R(RCX));
  ADD(16, R(RCX), Imm16(0x40));
  SAR(64, R(RDX), R(RCX));
  FixupBranch exit = J();
  SetJumpTarget(shiftLeft);
  SHL(64, R(RDX), R(RCX));
  SetJumpTarget(noShift);
  SetJumpTarget(exit);

  //	dsp_set_long_acc(dreg, acc);

  MOV(64, accreg, R(RDX));

  SetJumpTarget(zero);
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  //	Update_SR_Register64(dsp_get_long_acc(dreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RDX, RCX);
  }
}

// LSRNR  $acD
// 0011 110d 1xxx xxxx
// Logically shifts left/right accumulator $ACC[D] by lower 7-bit (signed) value in $AC[1-D].M
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::lsrnr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;

  //	s16 shift;
  //	u16 accm = (u16)dsp_get_acc_m(1 - dreg);
  m_gpr.ReadReg(1 - dreg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  //	u64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(RDX), accreg);
  //	acc &= 0x000000FFFFFFFFFFULL;
  SHL(64, R(RDX), Imm8(24));
  SHR(64, R(RDX), Imm8(24));

  //	if ((accm & 0x3f) == 0)
  //		shift = 0;
  //	else if (accm & 0x40)
  //		shift = -0x40 + (accm & 0x3f);
  //	else
  //		shift = accm & 0x3f;

  //	if (shift > 0)
  //		acc <<= shift;
  //	else if (shift < 0)
  //		acc >>= -shift;

  TEST(64, R(RDX), R(RDX));
  FixupBranch zero = J_CC(CC_E);
  TEST(16, R(RAX), Imm16(0x3f));
  FixupBranch noShift = J_CC(CC_Z);
  MOVZX(64, 16, RCX, R(RAX));
  AND(16, R(RCX), Imm16(0x3f));
  TEST(16, R(RAX), Imm16(0x40));
  FixupBranch shiftLeft = J_CC(CC_Z);
  NEG(16, R(RCX));
  ADD(16, R(RCX), Imm16(0x40));
  SHR(64, R(RDX), R(RCX));
  FixupBranch exit = J();
  SetJumpTarget(shiftLeft);
  SHL(64, R(RDX), R(RCX));
  SetJumpTarget(noShift);
  SetJumpTarget(exit);

  //	dsp_set_long_acc(dreg, (s64)acc);

  MOV(64, accreg, R(RDX));

  SetJumpTarget(zero);
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  //	Update_SR_Register64(dsp_get_long_acc(dreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RDX, RCX);
  }
}

// ASRNR  $acD
// 0011 111d 1xxx xxxx
// Arithmetically shift left/right accumulator $ACC[D] by lower 7-bit (signed) value in $AC[1-D].M
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::asrnr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;

  //	s16 shift;
  //	u16 accm = (u16)dsp_get_acc_m(1 - dreg);
  m_gpr.ReadReg(1 - dreg + DSP_REG_ACM0, RAX, RegisterExtension::Sign);
  //	s64 acc = dsp_get_long_acc(dreg);
  OpArg accreg = m_gpr.GetReg(DSP_REG_ACC0_64 + dreg);
  MOV(64, R(RDX), accreg);

  //	if ((accm & 0x3f) == 0)
  //		shift = 0;
  //	else if (accm & 0x40)
  //		shift = -0x40 + (accm & 0x3f);
  //	else
  //		shift = accm & 0x3f;

  //	if (shift > 0)
  //		acc <<= shift;
  //	else if (shift < 0)
  //		acc >>= -shift;

  TEST(64, R(RDX), R(RDX));
  FixupBranch zero = J_CC(CC_E);
  TEST(16, R(RAX), Imm16(0x3f));
  FixupBranch noShift = J_CC(CC_Z);
  MOVZX(64, 16, RCX, R(RAX));
  AND(16, R(RCX), Imm16(0x3f));
  TEST(16, R(RAX), Imm16(0x40));
  FixupBranch shiftLeft = J_CC(CC_Z);
  NEG(16, R(RCX));
  ADD(16, R(RCX), Imm16(0x40));
  SAR(64, R(RDX), R(RCX));
  FixupBranch exit = J();
  SetJumpTarget(shiftLeft);
  SHL(64, R(RDX), R(RCX));
  SetJumpTarget(noShift);
  SetJumpTarget(exit);

  //	dsp_set_long_acc(dreg, acc);

  MOV(64, accreg, R(RDX));

  SetJumpTarget(zero);
  m_gpr.PutReg(DSP_REG_ACC0_64 + dreg);
  //	Update_SR_Register64(dsp_get_long_acc(dreg));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RDX, RCX);
  }
}

// CLR $acR
// 1000 r001 xxxx xxxx
// Clears accumulator $acR
//
// flags out: --10 0100
void DSPEmitterIR::ir_clr(const UDSPInstruction opc)
{
  u8 reg = (opc >> 11) & 0x1;
  IRInsn p = {&Mov40Op, {IROp::Imm(0)}, IROp::R(reg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// CLRL $acR.l
// 1111 110r xxxx xxxx
// Clears (and rounds!) $acR.l - low 16 bits of accumulator $acR.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_clrl(const UDSPInstruction opc)
{
  u8 reg = (opc >> 8) & 0x1;
  IRInsn p = {&RoundOp, {IROp::R(reg + DSP_REG_ACC0)}, IROp::R(reg + DSP_REG_ACC0)};
  ir_add_op(p);
}

//----

// ANDCF $acD.m, #I
// 0000 001r 1100 0000
// iiii iiii iiii iiii
// Set logic zero (LZ) flag in status register $sr if result of logic AND of
// accumulator mid part $acD.m with immediate value I is equal I.
//
// flags out: -x-- ----
void DSPEmitterIR::ir_andcf(const UDSPInstruction opc)
{
  u8 reg = (opc >> 8) & 0x1;
  u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&AndCFOp, {IROp::R(reg + DSP_REG_ACM0), IROp::Imm(imm)}, IROp::None()};
  ir_add_op(p);
}

// ANDF $acD.m, #I
// 0000 001r 1010 0000
// iiii iiii iiii iiii
// Set logic zero (LZ) flag in status register $sr if result of logical AND
// operation of accumulator mid part $acD.m with immediate value I is equal
// immediate value 0.
//
// flags out: -x-- ----
void DSPEmitterIR::ir_andf(const UDSPInstruction opc)
{
  u8 reg = (opc >> 8) & 0x1;
  u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&AndFOp, {IROp::R(reg + DSP_REG_ACM0), IROp::Imm(imm)}, IROp::None()};
  ir_add_op(p);
}

//----

// TST
// 1011 r001 xxxx xxxx
// Test accumulator %acR.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_tst(const UDSPInstruction opc)
{
  u8 reg = (opc >> 11) & 0x1;
  IRInsn p = {&Tst40Op, {IROp::R(reg + IROp::DSP_REG_ACC0_ALL)}, IROp::None()};
  ir_add_op(p);
}

// TSTAXH $axR.h
// 1000 011r xxxx xxxx
// Test high part of secondary accumulator $axR.h.
//
// flags out: --x0 xx00
void DSPEmitterIR::ir_tstaxh(const UDSPInstruction opc)
{
  u8 reg = (opc >> 8) & 0x1;
  IRInsn p = {&Tst16Op, {IROp::R(reg + DSP_REG_AXH0)}, {}};
  ir_add_op(p);
}

//----

// CMP
// 1000 0010 xxxx xxxx
// Compares accumulator $ac0 with accumulator $ac1.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_cmp(const UDSPInstruction opc)
{
  IRInsn p = {&Cmp40Op, {IROp::R(IROp::DSP_REG_ACC0_ALL), IROp::R(IROp::DSP_REG_ACC1_ALL)}, {}};
  ir_add_op(p);
}

// CMPAR $acS axR.h
// 110r s001 xxxx xxxx
// Compares accumulator $acS with accumulator axR.h.
// Not described by Duddie's doc - at least not as a separate instruction.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_cmpar(const UDSPInstruction opc)
{
  u8 rreg = ((opc >> 12) & 0x1);
  u8 sreg = (opc >> 11) & 0x1;
  IRInsn p = {&Cmp16Op, {IROp::R(sreg + IROp::DSP_REG_ACC0_ALL), IROp::R(rreg + DSP_REG_AXH0)}, {}};
  ir_add_op(p);
}

// CMPI $amD, #I
// 0000 001r 1000 0000
// iiii iiii iiii iiii
// Compares mid accumulator $acD.hm ($amD) with sign extended immediate value I.
// Although flags are being set regarding whole accumulator register.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_cmpi(const UDSPInstruction opc)
{
  u8 reg = (opc >> 8) & 0x1;
  s16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&Cmp16Op, {IROp::R(reg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(imm)}, {}};
  ir_add_op(p);
}

// CMPIS $acD, #I
// 0000 011d iiii iiii
// Compares accumulator with short immediate. Comaprison is executed
// by subtracting short immediate (8bit sign extended) from mid accumulator
// $acD.hm and computing flags based on whole accumulator $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_cmpis(const UDSPInstruction opc)
{
  u8 areg = (opc >> 8) & 0x1;
  s8 imm = opc & 0xff;
  IRInsn p = {&Cmp16Op, {IROp::R(areg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(imm)}, {}};
  ir_add_op(p);
}

//----

// XORR $acD.m, $axS.h
// 0011 00sd 0xxx xxxx
// Logic XOR (exclusive or) middle part of accumulator $acD.m with
// high part of secondary accumulator $axS.h.
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::ir_xorr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {&XorOp,
              {IROp::R(dreg + DSP_REG_ACM0), IROp::R(sreg + DSP_REG_AXH0)},
              IROp::R(dreg + DSP_REG_ACM0)};
  ir_add_op(p);
}

// ANDR $acD.m, $axS.h
// 0011 01sd 0xxx xxxx
// Logic AND middle part of accumulator $acD.m with high part of
// secondary accumulator $axS.h.
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::ir_andr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {&AndOp,
              {IROp::R(dreg + DSP_REG_ACM0), IROp::R(sreg + DSP_REG_AXH0)},
              IROp::R(dreg + DSP_REG_ACM0)};
  ir_add_op(p);
}

// ORR $acD.m, $axS.h
// 0011 10sd 0xxx xxxx
// Logic OR middle part of accumulator $acD.m with high part of
// secondary accumulator $axS.h.
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::ir_orr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {&OrOp,
              {IROp::R(dreg + DSP_REG_ACM0), IROp::R(sreg + DSP_REG_AXH0)},
              IROp::R(dreg + DSP_REG_ACM0)};
  ir_add_op(p);
}

// ANDC $acD.m, $ac(1-D).m
// 0011 110d 0xxx xxxx
// Logic AND middle part of accumulator $acD.m with middle part of
// accumulator $ac(1-D).m
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::ir_andc(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&AndOp,
              {IROp::R(dreg + DSP_REG_ACM0), IROp::R((1 - dreg) + DSP_REG_ACM0)},
              IROp::R(dreg + DSP_REG_ACM0)};
  ir_add_op(p);
}

// ORC $acD.m, $ac(1-D).m
// 0011 111d 0xxx xxxx
// Logic OR middle part of accumulator $acD.m with middle part of
// accumulator $ac(1-D).m.
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::ir_orc(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&OrOp,
              {IROp::R(dreg + DSP_REG_ACM0), IROp::R((1 - dreg) + DSP_REG_ACM0)},
              IROp::R(dreg + DSP_REG_ACM0)};
  ir_add_op(p);
}

// XORC $acD.m
// 0011 000d 1xxx xxxx
// Logic XOR (exclusive or) middle part of accumulator $acD.m with $ac(1-D).m
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::ir_xorc(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&XorOp,
              {IROp::R(dreg + DSP_REG_ACM0), IROp::R((1 - dreg) + DSP_REG_ACM0)},
              IROp::R(dreg + DSP_REG_ACM0)};
  ir_add_op(p);
}

// NOT $acD.m
// 0011 001d 1xxx xxxx
// Invert all bits in dest reg, aka xor with 0xffff
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::ir_notc(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&NotOp, {IROp::R(dreg + DSP_REG_ACM0)}, IROp::R(dreg + DSP_REG_ACM0)};
  ir_add_op(p);
}

// XORI $acD.m, #I
// 0000 001r 0010 0000
// iiii iiii iiii iiii
// Logic exclusive or (XOR) of accumulator mid part $acD.m with
// immediate value I.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_xori(const UDSPInstruction opc)
{
  u8 reg = (opc >> 8) & 0x1;
  u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&XorOp, {IROp::R(reg + DSP_REG_ACM0), IROp::Imm(imm)}, IROp::R(reg + DSP_REG_ACM0)};
  ir_add_op(p);
}

// ANDI $acD.m, #I
// 0000 001r 0100 0000
// iiii iiii iiii iiii
// Logic AND of accumulator mid part $acD.m with immediate value I.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_andi(const UDSPInstruction opc)
{
  u8 reg = (opc >> 8) & 0x1;
  u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&AndOp, {IROp::R(reg + DSP_REG_ACM0), IROp::Imm(imm)}, IROp::R(reg + DSP_REG_ACM0)};
  ir_add_op(p);
}

// ORI $acD.m, #I
// 0000 001r 0110 0000
// iiii iiii iiii iiii
// Logic OR of accumulator mid part $acD.m with immediate value I.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_ori(const UDSPInstruction opc)
{
  u8 reg = (opc >> 8) & 0x1;
  u16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&OrOp, {IROp::R(reg + DSP_REG_ACM0), IROp::Imm(imm)}, IROp::R(reg + DSP_REG_ACM0)};
  ir_add_op(p);
}

//----

// ADDR $acD.M, $axS.L
// 0100 0ssd xxxx xxxx
// Adds register $axS.L to accumulator $acD.M register.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_addr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = ((opc >> 9) & 0x3) + DSP_REG_AXL0;
  IRInsn p = {&Add16Op,  // 16bit fix point
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(sreg)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ADDAX $acD, $axS
// 0100 10sd xxxx xxxx
// Adds secondary accumulator $axS to accumulator register $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_addax(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {&Add32Op,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(sreg + IROp::DSP_REG_AX0_ALL)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ADD $acD, $ac(1-D)
// 0100 110d xxxx xxxx
// Adds accumulator $ac(1-D) to accumulator register $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_add(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {
      &Add40Op,
      {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R((1 - dreg) + IROp::DSP_REG_ACC0_ALL)},
      IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ADDP $acD
// 0100 111d xxxx xxxx
// Adds product register to accumulator register.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_addp(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&AddPOp,  // actually a variant of Add40Op, but prod
              // is not available in a precalculated format.
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(IROp::DSP_REG_PROD_ALL)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ADDAXL $acD, $axS.l
// 0111 00sd xxxx xxxx
// Adds secondary accumulator $axS.l to accumulator register $acD.
// should be unsigned values!!
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_addaxl(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 9) & 0x1;
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&AddUOp,  // 16=>40 bit add(?)
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(sreg + DSP_REG_AXL0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ADDI $amR, #I
// 0000 001r 0000 0000
// iiii iiii iiii iiii
// Adds immediate (16-bit sign extended) to mid accumulator $acD.hm.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_addi(const UDSPInstruction opc)
{
  u8 areg = (opc >> 8) & 0x1;
  s16 imm = m_dsp_core.DSPState().ReadIMEM(m_compile_pc + 1);
  IRInsn p = {&Add16Op,
              {IROp::R(areg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(imm)},
              IROp::R(areg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ADDIS $acD, #I
// 0000 010d iiii iiii
// Adds short immediate (8-bit sign extended) to mid accumulator $acD.hm.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_addis(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  s8 imm = opc & 0xff;
  IRInsn p = {&Add16Op,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(imm)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// INCM $acsD
// 0111 010d xxxx xxxx
// Increment 24-bit mid-accumulator $acsD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_incm(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&Add16Op,  // Add
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(0x1)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// INC $acD
// 0111 011d xxxx xxxx
// Increment accumulator $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_inc(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&Add32Op,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(0x1)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

//----

// SUBR $acD.M, $axS.L
// 0101 0ssd xxxx xxxx
// Subtracts register $axS.L from accumulator $acD.M register.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_subr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x3;
  IRInsn p = {&Sub16Op,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(sreg + DSP_REG_AXL0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// SUBAX $acD, $axS
// 0101 10sd xxxx xxxx
// Subtracts secondary accumulator $axS from accumulator register $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_subax(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {&Sub32Op,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(sreg + IROp::DSP_REG_AX0_ALL)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// SUB $acD, $ac(1-D)
// 0101 110d xxxx xxxx
// Subtracts accumulator $ac(1-D) from accumulator register $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_sub(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {
      &Sub40Op,
      {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R((1 - dreg) + IROp::DSP_REG_ACC0_ALL)},
      IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// SUBP $acD
// 0101 111d xxxx xxxx
// Subtracts product register from accumulator register.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_subp(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&SubPOp,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(IROp::DSP_REG_PROD_ALL)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// DECM $acsD
// 0111 100d xxxx xxxx
// Decrement 24-bit mid-accumulator $acsD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_decm(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x01;
  IRInsn p = {&Sub16Op,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(0x1)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// DEC $acD
// 0111 101d xxxx xxxx
// Decrement accumulator $acD.
//
// flags out: x-xx xxxx
void DSPEmitterIR::ir_dec(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x01;
  IRInsn p = {&Sub32Op,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(0x1)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

//----

// NEG $acD
// 0111 110d xxxx xxxx
// Negate accumulator $acD.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_neg(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {
      &NegOp, {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)}, IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ABS  $acD
// 1010 d001 xxxx xxxx
// absolute value of $acD
//
// flags out: --xx xx00
void DSPEmitterIR::ir_abs(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 11) & 0x1;
  IRInsn p = {
      &AbsOp, {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)}, IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}
//----

// MOVR $acD, $axS.R
// 0110 0srd xxxx xxxx
// Moves register $axS.R (sign extended) to middle accumulator $acD.hm.
// Sets $acD.l to 0.
// TODO: Check what happens to acD.h.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_movr(const UDSPInstruction opc)
{
  u8 areg = (opc >> 8) & 0x1;
  u8 sreg = ((opc >> 9) & 0x3) + DSP_REG_AXL0;
  IRInsn p = {&MovROp,
              {IROp::R(sreg)},  // shifted left by 16, zeroing low 16bit
              IROp::R(areg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// MOVAX $acD, $axS
// 0110 10sd xxxx xxxx
// Moves secondary accumulator $axS to accumulator $axD.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_movax(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {
      &MovToAccOp, {IROp::R(sreg + IROp::DSP_REG_AX0_ALL)}, IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// MOV $acD, $ac(1-D)
// 0110 110d xxxx xxxx
// Moves accumulator $ax(1-D) to accumulator $axD.
//
// flags out: --x0 xx00
void DSPEmitterIR::ir_mov(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&Mov40Op,
              {IROp::R((1 - dreg) + IROp::DSP_REG_ACC0_ALL)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

//----

// LSL16 $acR
// 1111 000r xxxx xxxx
// Logically shifts left accumulator $acR by 16.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_lsl16(const UDSPInstruction opc)
{
  u8 areg = (opc >> 8) & 0x1;
  IRInsn p = {&LslOp,
              {IROp::R(areg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(16)},
              IROp::R(areg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// LSR16 $acR
// 1111 010r xxxx xxxx
// Logically shifts right accumulator $acR by 16.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_lsr16(const UDSPInstruction opc)
{
  u8 areg = (opc >> 8) & 0x1;
  IRInsn p = {&LslOp,
              {IROp::R(areg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(-16)},
              IROp::R(areg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ASR16 $acR
// 1001 r001 xxxx xxxx
// Arithmetically shifts right accumulator $acR by 16.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_asr16(const UDSPInstruction opc)
{
  u8 areg = (opc >> 11) & 0x1;
  IRInsn p = {&AslOp,
              {IROp::R(areg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(-16)},
              IROp::R(areg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// LSL $acR, #I
// 0001 010r 00ii iiii
// Logically shifts left accumulator $acR by number specified by value I.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_lsl(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x01;
  u16 shift = opc & 0x3f;
  IRInsn p = {&LslOp,
              {IROp::R(rreg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(shift)},
              IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// LSR $acR, #I
// 0001 010r 01ii iiii
// Logically shifts right accumulator $acR by number specified by value
// calculated by negating sign extended bits 0-6.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_lsr(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x01;
  u16 shift;
  if ((opc & 0x3f) == 0)
    shift = 0;
  else
    shift = 0x40 - (opc & 0x3f);
  IRInsn p = {&LslOp,
              {IROp::R(rreg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(-shift)},
              IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ASL $acR, #I
// 0001 010r 10ii iiii
// Logically shifts left accumulator $acR by number specified by value I.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_asl(const UDSPInstruction opc)
{
  u8 rreg = (opc >> 8) & 0x01;
  u16 shift = opc & 0x3f;
  IRInsn p = {&AslOp,
              {IROp::R(rreg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(shift)},
              IROp::R(rreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ASR $acR, #I
// 0001 010r 11ii iiii
// Arithmetically shifts right accumulator $acR by number specified by
// value calculated by negating sign extended bits 0-6.
//
// flags out: --xx xx00
void DSPEmitterIR::ir_asr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x01;
  u16 shift;

  if ((opc & 0x3f) == 0)
    shift = 0;
  else
    shift = 0x40 - (opc & 0x3f);

  IRInsn p = {&AslOp,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(-shift)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// LSRN  (fixed parameters)
// 0000 0010 1100 1010
// Logically shifts right accumulator $ACC0 by lower 7-bit (signed) value in $AC1.M
// (if value negative, becomes left shift).
//
// flags out: --xx xx00
void DSPEmitterIR::ir_lsrn(const UDSPInstruction opc)
{
  IRInsn p = {&LslOp,
              {IROp::R(IROp::DSP_REG_ACC0_ALL), IROp::R(DSP_REG_ACM1)},
              IROp::R(IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ASRN  (fixed parameters)
// 0000 0010 1100 1011
// Arithmetically shifts right accumulator $ACC0 by lower 7-bit (signed) value in $AC1.M
// (if value negative, becomes left shift).
//
// flags out: --xx xx00
void DSPEmitterIR::ir_asrn(const UDSPInstruction opc)
{
  IRInsn p = {&AslOp,
              {IROp::R(IROp::DSP_REG_ACC0_ALL), IROp::R(DSP_REG_ACM1)},
              IROp::R(IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// LSRNRX $acD, $axS.h
// 0011 01sd 1xxx xxxx
// Logically shifts left/right accumulator $ACC[D] by lower 7-bit (signed) value in $AX[S].H
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::ir_lsrnrx(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {&LslOp,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(sreg + DSP_REG_AXH0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ASRNRX $acD, $axS.h
// 0011 10sd 1xxx xxxx
// Arithmetically shifts left/right accumulator $ACC[D] by lower 7-bit (signed) value in $AX[S].H
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::ir_asrnrx(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  u8 sreg = (opc >> 9) & 0x1;
  IRInsn p = {&AslOp,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(sreg + DSP_REG_AXH0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// LSRNR  $acD
// 0011 110d 1xxx xxxx
// Logically shifts left/right accumulator $ACC[D] by lower 7-bit (signed) value in $AC[1-D].M
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::ir_lsrnr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&LslOp,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R((1 - dreg) + DSP_REG_ACM0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

// ASRNR  $acD
// 0011 111d 1xxx xxxx
// Arithmetically shift left/right accumulator $ACC[D] by lower 7-bit (signed) value in $AC[1-D].M
// x = extension (7 bits!!)
//
// flags out: --xx xx00
void DSPEmitterIR::ir_asrnr(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 8) & 0x1;
  IRInsn p = {&AslOp,
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R((1 - dreg) + DSP_REG_ACM0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
  ir_add_op(p);
}

void DSPEmitterIR::iremit_MovToAccOp(IRInsn const& insn)
{
  int in_reg = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  m_gpr.ReadReg(in_reg, RAX, RegisterExtension::Sign);
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
    Update_SR_Register64(RAX, RDX);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MovToAccOp = {
    "MovToAccOp", &DSPEmitterIR::iremit_MovToAccOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_MovROp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::Sign);
  SHL(64, R(RAX), Imm8(16));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MovROp = {
    "MovROp", &DSPEmitterIR::iremit_MovROp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_Mov40Op(IRInsn const& insn)
{
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  if (insn.inputs[0].type == IROp::REG)
  {
    int in_reg = ir_to_regcache_reg(insn.inputs[0].guest_reg);
    m_gpr.ReadReg(in_reg, RAX, RegisterExtension::None);
    m_gpr.WriteReg(out_reg, R(RAX));
  }
  else if (insn.inputs[0].type == IROp::IMM)
  {
    m_gpr.WriteReg(out_reg, Imm64(insn.inputs[0].imm));
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled Mov40Op variant");
  }
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(out_reg, RAX, RegisterExtension::None);
    Update_SR_Register64(RAX, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Mov40Op = {
    "Mov40Op", &DSPEmitterIR::iremit_Mov40Op, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_RoundOp(IRInsn const& insn)
{
  int in_reg = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  m_gpr.ReadReg(in_reg, RAX, RegisterExtension::Sign);
  round_long(RAX);
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
    Update_SR_Register64(RAX, RDX);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::RoundOp = {
    "RoundOp", &DSPEmitterIR::iremit_RoundOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_AndCFOp(IRInsn const& insn)
{
  if (FlagsNeeded())
  {
    int in_reg = ir_to_regcache_reg(insn.inputs[0].guest_reg);
    m_gpr.ReadReg(in_reg, RAX, RegisterExtension::None);
    u16 imm = insn.inputs[1].imm;
    const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
    AND(16, R(RAX), Imm16(imm));
    CMP(16, R(RAX), Imm16(imm));
    FixupBranch notLogicZero = J_CC(CC_NE);
    OR(16, sr_reg, Imm16(SR_LOGIC_ZERO));
    FixupBranch exit = J();
    SetJumpTarget(notLogicZero);
    AND(16, sr_reg, Imm16(~SR_LOGIC_ZERO));
    SetJumpTarget(exit);
    m_gpr.PutReg(DSP_REG_SR);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AndCFOp = {
    "AndCFOp", &DSPEmitterIR::iremit_AndCFOp, 0x0000, SR_LOGIC_ZERO, 0x0000, 0x0000};

void DSPEmitterIR::iremit_AndFOp(IRInsn const& insn)
{
  if (FlagsNeeded())
  {
    int in_reg = ir_to_regcache_reg(insn.inputs[0].guest_reg);
    m_gpr.ReadReg(in_reg, RAX, RegisterExtension::None);
    u16 imm = insn.inputs[1].imm;
    const OpArg sr_reg = m_gpr.GetReg(DSP_REG_SR);
    TEST(16, R(RAX), Imm16(imm));
    FixupBranch notLogicZero = J_CC(CC_NE);
    OR(16, sr_reg, Imm16(SR_LOGIC_ZERO));
    FixupBranch exit = J();
    SetJumpTarget(notLogicZero);
    AND(16, sr_reg, Imm16(~SR_LOGIC_ZERO));
    SetJumpTarget(exit);
    m_gpr.PutReg(DSP_REG_SR);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AndFOp = {
    "AndFOp", &DSPEmitterIR::iremit_AndFOp, 0x0000, SR_LOGIC_ZERO, 0x0000, 0x0000};

void DSPEmitterIR::iremit_Tst40Op(IRInsn const& insn)
{
  if (FlagsNeeded())
  {
    int in_reg = ir_to_regcache_reg(insn.inputs[0].guest_reg);
    m_gpr.ReadReg(in_reg, RAX, RegisterExtension::Sign);
    Update_SR_Register64(RAX, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Tst40Op = {
    "Tst40Op", &DSPEmitterIR::iremit_Tst40Op, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_Tst16Op(IRInsn const& insn)
{
  if (FlagsNeeded())
  {
    int in_reg = ir_to_regcache_reg(insn.inputs[0].guest_reg);
    m_gpr.ReadReg(in_reg, RAX, RegisterExtension::Sign);
    Update_SR_Register16(RAX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Tst16Op = {
    "Tst16Op", &DSPEmitterIR::iremit_Tst16Op, 0x0000, SR_CMP_MASK, 0x0013, 0x0000};

void DSPEmitterIR::iremit_Cmp40Op(IRInsn const& insn)
{
  if (FlagsNeeded())
  {
    int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
    int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
    X64Reg tmp1 = m_gpr.GetFreeXReg();
    m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
    m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::None);
    MOV(64, R(RAX), R(tmp1));
    SUB(64, R(RAX), R(RDX));
    NEG(64, R(RDX));
    Update_SR_Register64_Carry(RAX, tmp1, RDX, true);
    m_gpr.PutXReg(tmp1);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Cmp40Op = {
    "Cmp40Op", &DSPEmitterIR::iremit_Cmp40Op, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000,
};

void DSPEmitterIR::iremit_Cmp16Op(IRInsn const& insn)
{
  if (FlagsNeeded())
  {
    X64Reg tmp1 = m_gpr.GetFreeXReg();
    int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
    m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
    if (insn.inputs[1].type == IROp::REG)
    {
      int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
      m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Sign);
    }
    else if (insn.inputs[1].type == IROp::IMM)
    {
      MOV(64, R(RDX), Imm64(insn.inputs[1].imm));
    }
    else
    {
      ASSERT_MSG(DSPLLE, 0, "unhandled Cmp16Op variant");
    }
    MOV(64, R(RAX), R(tmp1));
    SHL(64, R(RDX), Imm8(16));
    SUB(64, R(RAX), R(RDX));
    NEG(64, R(RDX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
    m_gpr.PutXReg(tmp1);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Cmp16Op = {
    "Cmp16Op", &DSPEmitterIR::iremit_Cmp16Op, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000,
};

void DSPEmitterIR::iremit_XorOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::Sign);
  if (insn.inputs[1].type == IROp::REG)
  {
    int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
    m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Sign);
  }
  else if (insn.inputs[1].type == IROp::IMM)
  {
    MOV(16, R(RDX), Imm16(insn.inputs[1].imm));
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled XorOp variant");
  }
  XOR(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(in_reg0 - DSP_REG_ACM0 + DSP_REG_ACC0_64, RCX, RegisterExtension::Sign);
    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::XorOp = {
    "XorOp", &DSPEmitterIR::iremit_XorOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_AndOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::Sign);
  if (insn.inputs[1].type == IROp::REG)
  {
    int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
    m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Sign);
  }
  else if (insn.inputs[1].type == IROp::IMM)
  {
    MOV(16, R(RDX), Imm16(insn.inputs[1].imm));
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled XorOp variant");
  }
  AND(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(in_reg0 - DSP_REG_ACM0 + DSP_REG_ACC0_64, RCX, RegisterExtension::Sign);
    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AndOp = {
    "AndOp", &DSPEmitterIR::iremit_AndOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_OrOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::Sign);
  if (insn.inputs[1].type == IROp::REG)
  {
    int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
    m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Sign);
  }
  else if (insn.inputs[1].type == IROp::IMM)
  {
    MOV(16, R(RDX), Imm16(insn.inputs[1].imm));
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled XorOp variant");
  }
  OR(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(in_reg0 - DSP_REG_ACM0 + DSP_REG_ACC0_64, RCX, RegisterExtension::Sign);
    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::OrOp = {
    "OrOp", &DSPEmitterIR::iremit_OrOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_NotOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);
  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::Sign);
  NOT(16, R(AX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    m_gpr.ReadReg(in_reg0 - DSP_REG_ACM0 + DSP_REG_ACC0_64, RCX, RegisterExtension::Sign);
    Update_SR_Register16_OverS32(RAX, RCX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::NotOp = {
    "NotOp", &DSPEmitterIR::iremit_NotOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_Add16Op(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
  MOV(64, R(RAX), R(tmp1));
  if (insn.inputs[1].type == IROp::REG)
  {
    int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
    m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Sign);
    SHL(64, R(RDX), Imm8(16));
  }
  else if (insn.inputs[1].type == IROp::IMM)
  {
    MOV(64, R(RDX), Imm32(insn.inputs[1].imm << 16));
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled Add16Op variant");
  }
  ADD(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Add16Op = {
    "Add16Op", &DSPEmitterIR::iremit_Add16Op, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000,
};

void DSPEmitterIR::iremit_Add32Op(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
  MOV(64, R(RAX), R(tmp1));
  if (insn.inputs[1].type == IROp::REG)
  {
    m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Sign);
  }
  else if (insn.inputs[1].type == IROp::IMM)
  {
    MOV(64, R(RDX), Imm32(insn.inputs[1].imm));
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled Add32Op variant");
  }
  ADD(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Add32Op = {
    "Add32Op", &DSPEmitterIR::iremit_Add32Op, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000,
};

void DSPEmitterIR::iremit_Add40Op(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
  MOV(64, R(RAX), R(tmp1));
  if (insn.inputs[1].type == IROp::REG)
  {
    m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Sign);
  }
  else if (insn.inputs[1].type == IROp::IMM)
  {
    MOV(64, R(RDX), Imm32(insn.inputs[1].imm));
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled Add32Op variant");
  }
  ADD(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Add40Op = {
    "Add40Op", &DSPEmitterIR::iremit_Add40Op, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000};

void DSPEmitterIR::iremit_AddPOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);  // prod reg
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
  MOV(64, R(RAX), R(tmp1));
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  ASSERT_MSG(DSPLLE, in_reg1 == DSP_REG_PROD_64, "in_reg1 must be PROD for AddPOp");
  get_long_prod(RDX, tmp2);
  m_gpr.PutXReg(tmp2);
  ADD(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AddPOp = {
    "AddPOp", &DSPEmitterIR::iremit_AddPOp, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000,
};

void DSPEmitterIR::iremit_AddUOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
  MOV(64, R(RAX), R(tmp1));
  m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Zero);
  ADD(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    Update_SR_Register64_Carry(EAX, tmp1, RDX);
  }
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AddUOp = {
    "AddUOp", &DSPEmitterIR::iremit_AddUOp, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000,
};

void DSPEmitterIR::iremit_Sub16Op(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
  MOV(64, R(RAX), R(tmp1));
  if (insn.inputs[1].type == IROp::REG)
  {
    int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
    m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Sign);
    SHL(64, R(RDX), Imm8(16));
  }
  else if (insn.inputs[1].type == IROp::IMM)
  {
    MOV(64, R(RDX), Imm32(insn.inputs[1].imm << 16));
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled Sub16Op variant");
  }
  SUB(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    NEG(64, R(RDX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
  }
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Sub16Op = {
    "Sub16Op", &DSPEmitterIR::iremit_Sub16Op, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000,
};

void DSPEmitterIR::iremit_Sub32Op(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
  MOV(64, R(RAX), R(tmp1));
  if (insn.inputs[1].type == IROp::REG)
  {
    m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Sign);
  }
  else if (insn.inputs[1].type == IROp::IMM)
  {
    MOV(64, R(RDX), Imm32(insn.inputs[1].imm));
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled Sub32Op variant");
  }
  SUB(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    NEG(64, R(RDX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
  }
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Sub32Op = {
    "Sub32Op", &DSPEmitterIR::iremit_Sub32Op, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000,
};

void DSPEmitterIR::iremit_Sub40Op(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
  MOV(64, R(RAX), R(tmp1));
  if (insn.inputs[1].type == IROp::REG)
  {
    m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Sign);
  }
  else if (insn.inputs[1].type == IROp::IMM)
  {
    MOV(64, R(RDX), Imm32(insn.inputs[1].imm));
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled Sub32Op variant");
  }
  SUB(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    NEG(64, R(RDX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
  }
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Sub40Op = {
    "Sub40Op", &DSPEmitterIR::iremit_Sub40Op, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000};

void DSPEmitterIR::iremit_SubPOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);  // prod reg
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
  MOV(64, R(RAX), R(tmp1));
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  ASSERT_MSG(DSPLLE, in_reg1 == DSP_REG_PROD_64, "in_reg1 must be PROD for Sub40Op");
  get_long_prod(RDX, tmp2);
  m_gpr.PutXReg(tmp2);
  SUB(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    NEG(64, R(RDX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
  }
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::SubPOp = {
    "SubPOp", &DSPEmitterIR::iremit_SubPOp, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000,
};

void DSPEmitterIR::iremit_SubUOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  m_gpr.ReadReg(in_reg0, tmp1, RegisterExtension::None);
  MOV(64, R(RAX), R(tmp1));
  m_gpr.ReadReg(in_reg1, RDX, RegisterExtension::Zero);
  SUB(64, R(RAX), R(RDX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    NEG(64, R(RDX));
    Update_SR_Register64_Carry(EAX, tmp1, RDX, true);
  }
  m_gpr.PutXReg(tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::SubUOp = {
    "SubUOp", &DSPEmitterIR::iremit_SubUOp, 0x0000, SR_CMP_MASK | SR_OVERFLOW_STICKY, 0x0000,
    0x0000,
};

void DSPEmitterIR::iremit_NegOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::None);
  NEG(64, R(RAX));
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::NegOp = {
    "NegOp", &DSPEmitterIR::iremit_NegOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_AbsOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::None);
  TEST(64, R(RAX), R(RAX));
  FixupBranch GreaterThanOrEqual = J_CC(CC_GE);
  NEG(64, R(RAX));
  m_gpr.WriteReg(out_reg, R(RAX));
  SetJumpTarget(GreaterThanOrEqual);
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AbsOp = {
    "AbsOp", &DSPEmitterIR::iremit_AbsOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_LslOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::None);

  if (insn.inputs[1].type == IROp::REG)
  {
    int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);

    m_gpr.ReadReg(in_reg1, RCX, RegisterExtension::None);
    TEST(8, R(CL), Imm8(0x40));
    FixupBranch shiftLeft = J_CC(CC_Z);
    NEG(8, R(CL));
    // need to fix registers since we still
    // don't guarantee sanity here...
    SHL(64, R(RAX), Imm8(24));
    ADD(8, R(CL), Imm8(24));
    SHR(64, R(RAX), R(CL));
    FixupBranch exit = J();
    SetJumpTarget(shiftLeft);
    SHL(64, R(RAX), R(CL));
    SetJumpTarget(exit);
  }
  else if (insn.inputs[1].type == IROp::IMM)
  {
    if (insn.inputs[1].imm < 0)
    {
      // need to fix registers since we still
      // don't guarantee sanity here...
      SHL(64, R(RAX), Imm8(24));
      SHR(64, R(RAX), Imm8(24 - insn.inputs[1].imm));
    }
    else
    {
      SHL(64, R(RAX), Imm8(insn.inputs[1].imm));
    }
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled LslOp variant");
  }
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LslOp = {
    "LslOp", &DSPEmitterIR::iremit_LslOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

void DSPEmitterIR::iremit_AslOp(IRInsn const& insn)
{
  int in_reg0 = ir_to_regcache_reg(insn.inputs[0].guest_reg);
  int out_reg = ir_to_regcache_reg(insn.output.guest_reg);

  m_gpr.ReadReg(in_reg0, RAX, RegisterExtension::Sign);

  if (insn.inputs[1].type == IROp::REG)
  {
    int in_reg1 = ir_to_regcache_reg(insn.inputs[1].guest_reg);

    m_gpr.ReadReg(in_reg1, RCX, RegisterExtension::None);
    TEST(8, R(CL), Imm8(0x40));
    FixupBranch shiftLeft = J_CC(CC_Z);
    NEG(8, R(CL));
    // need to fix registers since we still
    // don't guarantee sanity here...
    SHL(64, R(RAX), Imm8(24));
    ADD(8, R(CL), Imm8(24));
    SAR(64, R(RAX), R(CL));
    FixupBranch exit = J();
    SetJumpTarget(shiftLeft);
    SHL(64, R(RAX), R(CL));
    SetJumpTarget(exit);
  }
  else if (insn.inputs[1].type == IROp::IMM)
  {
    if (insn.inputs[1].imm < 0)
    {
      // need to fix registers since we still
      // don't guarantee sanity here...
      SHL(64, R(RAX), Imm8(24));
      SAR(64, R(RAX), Imm8(24 - insn.inputs[1].imm));
    }
    else
    {
      SHL(64, R(RAX), Imm8(insn.inputs[1].imm));
    }
  }
  else
  {
    ASSERT_MSG(DSPLLE, 0, "unhandled AslOp variant");
  }
  m_gpr.WriteReg(out_reg, R(RAX));
  if (FlagsNeeded())
  {
    Update_SR_Register64(RAX, RDX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AslOp = {
    "AslOp", &DSPEmitterIR::iremit_AslOp, 0x0000, SR_CMP_MASK, 0x0003, 0x0000};

}  // namespace DSP::JITIR::x64
