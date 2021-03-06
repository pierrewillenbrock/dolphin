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
  IRInsn p = {&Tst16Op, {IROp::R(reg + DSP_REG_AXH0)}, IROp::None()};
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
  IRInsn p = {
      &Cmp40Op, {IROp::R(IROp::DSP_REG_ACC0_ALL), IROp::R(IROp::DSP_REG_ACC1_ALL)}, IROp::None()};
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
  IRInsn p = {&Cmp16Op,
              {IROp::R(sreg + IROp::DSP_REG_ACC0_ALL), IROp::R(rreg + DSP_REG_AXH0)},
              IROp::None()};
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
  IRInsn p = {&Cmp16Op, {IROp::R(reg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(imm)}, IROp::None()};
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
  IRInsn p = {&Cmp16Op, {IROp::R(areg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(imm)}, IROp::None()};
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
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(sreg + DSP_REG_AXH0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
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
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(sreg + DSP_REG_AXH0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
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
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R(sreg + DSP_REG_AXH0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
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
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R((1 - dreg) + DSP_REG_ACM0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
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
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R((1 - dreg) + DSP_REG_ACM0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
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
              {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL), IROp::R((1 - dreg) + DSP_REG_ACM0)},
              IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
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
  IRInsn p = {
      &NotOp, {IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)}, IROp::R(dreg + IROp::DSP_REG_ACC0_ALL)};
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
  IRInsn p = {&XorOp,
              {IROp::R(reg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(imm)},
              IROp::R(reg + IROp::DSP_REG_ACC0_ALL)};
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
  IRInsn p = {&AndOp,
              {IROp::R(reg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(imm)},
              IROp::R(reg + IROp::DSP_REG_ACC0_ALL)};
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
  IRInsn p = {&OrOp,
              {IROp::R(reg + IROp::DSP_REG_ACC0_ALL), IROp::Imm(imm)},
              IROp::R(reg + IROp::DSP_REG_ACC0_ALL)};
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
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  if (FlagsNeeded(insn))
  {
    MOV(64, R(tmp1), insn.output.oparg);
    Update_SR_Register64(insn, R(tmp1), insn.SR, tmp2);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MovToAccOp = {
    "MovToAccOp", &DSPEmitterIR::iremit_MovToAccOp,
    0x0000,       SR_CMP_MASK,
    0x0003,       0x0000,
    false,        {{OpAnyReg | SameAsOutput | ExtendSign32}},
    {OpAnyReg},   {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_MovROp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  SHL(64, insn.inputs[0].oparg, Imm8(16));
  if (FlagsNeeded(insn))
  {
    MOV(64, R(tmp1), insn.output.oparg);
    Update_SR_Register64(insn, R(tmp1), insn.SR, tmp2);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::MovROp = {
    "MovROp",   &DSPEmitterIR::iremit_MovROp,
    0x0000,     SR_CMP_MASK,
    0x0003,     0x0000,
    false,      {{OpAnyReg | SameAsOutput | ExtendSign16}},
    {OpAnyReg}, {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_Mov40Op(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  if (FlagsNeeded(insn))
  {
    if (insn.output.oparg.IsImm())
    {
      if (insn.output.oparg.GetImmBits() == 64)
        MOV(64, R(tmp1), insn.output.oparg);
      else
        MOV(64, R(tmp1), insn.output.oparg.AsImm32());
    }
    else
      MOV(64, R(tmp1), insn.output.oparg);
    Update_SR_Register64(insn, R(tmp1), insn.SR, tmp2);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Mov40Op = {
    "Mov40Op", &DSPEmitterIR::iremit_Mov40Op,
    0x0000,    SR_CMP_MASK,
    0x0003,    0x0000,
    false,     {{OpAny | SameAsOutput}},
    {OpAny},   {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_RoundOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  round_long(insn.inputs[0].oparg.GetSimpleReg());
  if (FlagsNeeded(insn))
  {
    MOV(64, R(tmp1), insn.output.oparg);
    Update_SR_Register64(insn, R(tmp1), insn.SR, tmp2);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::RoundOp = {
    "RoundOp",  &DSPEmitterIR::iremit_RoundOp,
    0x0000,     SR_CMP_MASK,
    0x0003,     0x0000,
    false,      {{OpAnyReg | SameAsOutput | ExtendSign64}},
    {OpAnyReg}, {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_AndCFOp(IRInsn const& insn)
{
  if (FlagsNeeded(insn))
  {
    OpArg const& sr_reg = insn.SR;
    AND(16, insn.inputs[0].oparg, insn.inputs[1].oparg.AsImm16());
    CMP(16, insn.inputs[0].oparg, insn.inputs[1].oparg.AsImm16());
    FixupBranch notLogicZero = J_CC(CC_NE);
    OR(16, sr_reg, Imm16(SR_LOGIC_ZERO));
    FixupBranch exit = J();
    SetJumpTarget(notLogicZero);
    AND(16, sr_reg, Imm16(~SR_LOGIC_ZERO));
    SetJumpTarget(exit);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AndCFOp = {
    "AndCFOp", &DSPEmitterIR::iremit_AndCFOp,
    0x0000,    SR_LOGIC_ZERO,
    0x0000,    0x0000,
    false,     {{OpAnyReg | Clobbered | ExtendNone}, {OpImm}},
    {},        {}};

void DSPEmitterIR::iremit_AndFOp(IRInsn const& insn)
{
  if (FlagsNeeded(insn))
  {
    OpArg const& sr_reg = insn.SR;
    TEST(16, insn.inputs[0].oparg, insn.inputs[1].oparg.AsImm16());
    FixupBranch notLogicZero = J_CC(CC_NE);
    OR(16, sr_reg, Imm16(SR_LOGIC_ZERO));
    FixupBranch exit = J();
    SetJumpTarget(notLogicZero);
    AND(16, sr_reg, Imm16(~SR_LOGIC_ZERO));
    SetJumpTarget(exit);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AndFOp = {
    "AndFOp", &DSPEmitterIR::iremit_AndFOp,
    0x0000,   SR_LOGIC_ZERO,
    0x0000,   0x0000,
    false,    {{OpAnyReg | Clobbered | ExtendNone}, {OpImm}},
    {},       {}};

void DSPEmitterIR::iremit_Tst40Op(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  Update_SR_Register64(insn, insn.inputs[0].oparg, insn.SR, tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Tst40Op = {
    "Tst40Op", &DSPEmitterIR::iremit_Tst40Op,           0x0000, SR_CMP_MASK, 0x0003, 0x0000,
    false,     {{OpAnyReg | Clobbered | ExtendSign64}}, {},     {{OpAnyReg}}};

void DSPEmitterIR::iremit_Tst16Op(IRInsn const& insn)
{
  Update_SR_Register16(insn, insn.inputs[0].oparg, insn.SR);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Tst16Op = {
    "Tst16Op", &DSPEmitterIR::iremit_Tst16Op,           0x0000, SR_CMP_MASK, 0x0013, 0x0000,
    false,     {{OpAnyReg | Clobbered | ExtendSign16}}, {},     {}};

void DSPEmitterIR::iremit_Cmp40Op(IRInsn const& insn)
{
  if (FlagsNeeded(insn))
  {
    X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

    MOV(64, R(tmp1), insn.inputs[0].oparg);
    SUB(64, R(tmp1), insn.inputs[1].oparg);
    NEG(64, insn.inputs[1].oparg);
    Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, insn.inputs[1].oparg, insn.SR,
                               true);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Cmp40Op = {
    "Cmp40Op", &DSPEmitterIR::iremit_Cmp40Op,
    0x0000,    SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,    0x0000,
    false,     {{OpAnyReg | Clobbered}, {OpAnyReg | Clobbered}},
    {},        {{OpAnyReg}}};

void DSPEmitterIR::iremit_Cmp16Op(IRInsn const& insn)
{
  if (FlagsNeeded(insn))
  {
    X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
    X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

    if (insn.inputs[1].oparg.IsImm())
    {
      MOV(64, R(tmp2), Imm64(insn.inputs[1].oparg.AsImm16().SImm16() << 16));
    }
    else
    {
      MOV(64, R(tmp2), insn.inputs[1].oparg);
      SHL(64, R(tmp2), Imm8(16));
    }
    MOV(64, R(tmp1), insn.inputs[0].oparg);
    SUB(64, R(tmp1), R(tmp2));
    NEG(64, R(tmp2));
    Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, R(tmp2), insn.SR, true);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Cmp16Op = {
    "Cmp16Op", &DSPEmitterIR::iremit_Cmp16Op,
    0x0000,    SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,    0x0000,
    false,     {{OpAnyReg | Clobbered}, {OpAny | ExtendSign16}},
    {},        {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_XorOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  if (insn.inputs[1].oparg.IsImm())
  {
    MOV(64, R(tmp1), Imm64(insn.inputs[1].oparg.AsImm16().Imm16() << 16));
  }
  else
  {
    MOV(64, R(tmp1), insn.inputs[1].oparg);
    SHL(64, R(tmp1), Imm8(16));
  }
  XOR(64, insn.inputs[0].oparg, R(tmp1));
  if (FlagsNeeded(insn))
  {
    MOV(64, R(tmp2), insn.output.oparg);
    MOV(64, R(tmp1), R(tmp2));
    SHR(64, R(tmp1), Imm8(16));
    MOVSX(64, 16, tmp1, R(tmp1));
    Update_SR_Register16_OverS32(insn, R(tmp1), R(tmp2), insn.SR);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::XorOp = {
    "XorOp",
    &DSPEmitterIR::iremit_XorOp,
    0x0000,
    SR_CMP_MASK,
    0x0003,
    0x0000,
    false,
    {{OpAnyReg | SameAsOutput}, {OpAny | ExtendZero16 | NoSaturate}},
    {OpAnyReg | NoACMExtend},
    {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_AndOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  if (insn.inputs[1].oparg.IsImm())
  {
    MOV(64, R(tmp1), Imm64((insn.inputs[1].oparg.AsImm16().Imm16() << 16) | ~0xffff0000ULL));
  }
  else
  {
    MOV(64, R(tmp1), insn.inputs[1].oparg);
    OR(64, R(tmp1), Imm32(0xffff0000));
    SHL(64, R(tmp1), Imm8(16));
    OR(64, R(tmp1), Imm32(0x0000ffff));
  }
  AND(64, insn.inputs[0].oparg, R(tmp1));
  if (FlagsNeeded(insn))
  {
    MOV(64, R(tmp2), insn.output.oparg);
    MOV(64, R(tmp1), R(tmp2));
    SHR(64, R(tmp1), Imm8(16));
    MOVSX(64, 16, tmp1, R(tmp1));
    Update_SR_Register16_OverS32(insn, R(tmp1), R(tmp2), insn.SR);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AndOp = {
    "AndOp",    &DSPEmitterIR::iremit_AndOp,
    0x0000,     SR_CMP_MASK,
    0x0003,     0x0000,
    false,      {{OpAnyReg | SameAsOutput}, {OpAny | NoSaturate}},
    {OpAnyReg}, {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_OrOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  if (insn.inputs[1].oparg.IsImm())
  {
    MOV(64, R(tmp1), Imm64(insn.inputs[1].oparg.AsImm16().Imm16() << 16));
  }
  else
  {
    MOV(64, R(tmp1), insn.inputs[1].oparg);
    SHL(64, R(tmp1), Imm8(16));
  }
  OR(64, insn.inputs[0].oparg, R(tmp1));
  if (FlagsNeeded(insn))
  {
    MOV(64, R(tmp2), insn.output.oparg);
    MOV(64, R(tmp1), R(tmp2));
    SHR(64, R(tmp1), Imm8(16));
    MOVSX(64, 16, tmp1, R(tmp1));
    Update_SR_Register16_OverS32(insn, R(tmp1), R(tmp2), insn.SR);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::OrOp = {
    "OrOp",     &DSPEmitterIR::iremit_OrOp,
    0x0000,     SR_CMP_MASK,
    0x0003,     0x0000,
    false,      {{OpAnyReg | SameAsOutput}, {OpAny | ExtendZero16 | NoSaturate}},
    {OpAnyReg}, {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_NotOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  NOT(16, insn.inputs[0].oparg);
  if (FlagsNeeded(insn))
  {
    MOV(64, R(tmp2), insn.output.oparg);
    MOV(64, R(tmp1), R(tmp2));
    SHR(64, R(tmp1), Imm8(16));
    MOVSX(64, 16, tmp1, R(tmp1));
    Update_SR_Register16_OverS32(insn, R(tmp1), R(tmp2), insn.SR);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::NotOp = {
    "NotOp",    &DSPEmitterIR::iremit_NotOp,
    0x0000,     SR_CMP_MASK,
    0x0003,     0x0000,
    false,      {{OpAnyReg | SameAsOutput}},
    {OpAnyReg}, {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_Add16Op(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  if (insn.inputs[1].oparg.IsImm())
  {
    MOV(64, R(tmp2), Imm32(insn.inputs[1].oparg.AsImm16().SImm16() << 16));
  }
  else
  {
    MOV(64, R(tmp2), insn.inputs[1].oparg);
    SHL(64, R(tmp2), Imm8(16));
  }
  MOV(64, R(tmp1), insn.inputs[0].oparg);
  ADD(64, R(tmp1), R(tmp2));
  MOV(64, insn.output.oparg, R(tmp1));
  Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, R(tmp2), insn.SR);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Add16Op = {
    "Add16Op",  &DSPEmitterIR::iremit_Add16Op,
    0x0000,     SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,     0x0000,
    false,      {{OpAnyReg | Clobbered}, {OpAny | ExtendSign16}},
    {OpAnyReg}, {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_Add32Op(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  MOV(64, R(tmp1), insn.inputs[0].oparg);
  ADD(64, R(tmp1), insn.inputs[1].oparg);
  MOV(64, insn.output.oparg, R(tmp1));
  Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, insn.inputs[1].oparg, insn.SR);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Add32Op = {
    "Add32Op",  &DSPEmitterIR::iremit_Add32Op,
    0x0000,     SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,     0x0000,
    false,      {{OpAnyReg | Clobbered}, {OpAnyReg | Clobbered | ExtendSign32}},
    {OpAnyReg}, {{OpAnyReg}}};

void DSPEmitterIR::iremit_Add40Op(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  MOV(64, R(tmp1), insn.inputs[0].oparg);
  ADD(64, R(tmp1), insn.inputs[1].oparg);
  MOV(64, insn.output.oparg, R(tmp1));
  Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, insn.inputs[1].oparg, insn.SR);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Add40Op = {
    "Add40Op",  &DSPEmitterIR::iremit_Add40Op,
    0x0000,     SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,     0x0000,
    false,      {{OpAnyReg | Clobbered}, {OpAnyReg | Clobbered | ExtendSign64}},
    {OpAnyReg}, {{OpAnyReg}}};

void DSPEmitterIR::iremit_AddPOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  MOV(64, R(tmp1), insn.inputs[0].oparg);
  ADD(64, R(tmp1), insn.inputs[1].oparg);
  MOV(64, insn.output.oparg, R(tmp1));
  Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, insn.inputs[1].oparg, insn.SR);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AddPOp = {
    "AddPOp",   &DSPEmitterIR::iremit_AddPOp,
    0x0000,     SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,     0x0000,
    false,      {{OpAnyReg | Clobbered}, {OpAnyReg | Clobbered}},
    {OpAnyReg}, {{OpAnyReg}}};

void DSPEmitterIR::iremit_AddUOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  MOV(64, R(tmp1), insn.inputs[0].oparg);
  ADD(64, R(tmp1), insn.inputs[1].oparg);
  MOV(64, insn.output.oparg, R(tmp1));
  Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, insn.inputs[1].oparg, insn.SR);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AddUOp = {
    "AddUOp",   &DSPEmitterIR::iremit_AddUOp,
    0x0000,     SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,     0x0000,
    false,      {{OpAnyReg | Clobbered}, {OpAnyReg | Clobbered | ExtendZero16}},
    {OpAnyReg}, {{OpAnyReg}}};

void DSPEmitterIR::iremit_Sub16Op(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();

  if (insn.inputs[1].oparg.IsImm())
  {
    MOV(64, R(tmp2), Imm32(insn.inputs[1].oparg.AsImm16().SImm16() << 16));
  }
  else
  {
    MOV(64, R(tmp2), insn.inputs[1].oparg);
    SHL(64, R(tmp2), Imm8(16));
  }
  MOV(64, R(tmp1), insn.inputs[0].oparg);
  SUB(64, R(tmp1), R(tmp2));
  MOV(64, insn.output.oparg, R(tmp1));
  if (FlagsNeeded(insn))
  {
    NEG(64, R(tmp2));
  }
  // Update_SR_Register64_Carry checks FlagsNeeded as well
  Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, R(tmp2), insn.SR, true);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Sub16Op = {
    "Sub16Op",  &DSPEmitterIR::iremit_Sub16Op,
    0x0000,     SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,     0x0000,
    false,      {{OpAnyReg | Clobbered}, {OpAny | ExtendSign16}},
    {OpAnyReg}, {{OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_Sub32Op(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  MOV(64, R(tmp1), insn.inputs[0].oparg);
  SUB(64, R(tmp1), insn.inputs[1].oparg);
  MOV(64, insn.output.oparg, R(tmp1));
  if (FlagsNeeded(insn))
  {
    NEG(64, insn.inputs[1].oparg);
  }
  // Update_SR_Register64_Carry checks FlagsNeeded as well
  Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, insn.inputs[1].oparg, insn.SR,
                             true);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Sub32Op = {
    "Sub32Op",  &DSPEmitterIR::iremit_Sub32Op,
    0x0000,     SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,     0x0000,
    false,      {{OpAnyReg | Clobbered}, {OpAnyReg | Clobbered | ExtendSign32}},
    {OpAnyReg}, {{OpAnyReg}}};

void DSPEmitterIR::iremit_Sub40Op(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  MOV(64, R(tmp1), insn.inputs[0].oparg);
  SUB(64, R(tmp1), insn.inputs[1].oparg);
  MOV(64, insn.output.oparg, R(tmp1));
  if (FlagsNeeded(insn))
  {
    NEG(64, insn.inputs[1].oparg);
  }
  // Update_SR_Register64_Carry checks FlagsNeeded as well
  Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, insn.inputs[1].oparg, insn.SR,
                             true);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::Sub40Op = {
    "Sub40Op",  &DSPEmitterIR::iremit_Sub40Op,
    0x0000,     SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,     0x0000,
    false,      {{OpAnyReg | Clobbered}, {OpAnyReg | Clobbered | ExtendSign64}},
    {OpAnyReg}, {{OpAnyReg}}};

void DSPEmitterIR::iremit_SubPOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  MOV(64, R(tmp1), insn.inputs[0].oparg);
  SUB(64, R(tmp1), insn.inputs[1].oparg);
  MOV(64, insn.output.oparg, R(tmp1));
  if (FlagsNeeded(insn))
  {
    NEG(64, insn.inputs[1].oparg);
  }
  // Update_SR_Register64_Carry checks FlagsNeeded as well
  Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, insn.inputs[1].oparg, insn.SR,
                             true);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::SubPOp = {
    "SubPOp",   &DSPEmitterIR::iremit_SubPOp,
    0x0000,     SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,     0x0000,
    false,      {{OpAnyReg | Clobbered}, {OpAnyReg | Clobbered}},
    {OpAnyReg}, {{OpAnyReg}}};

void DSPEmitterIR::iremit_SubUOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  MOV(64, R(tmp1), insn.inputs[0].oparg);
  SUB(64, R(tmp1), insn.inputs[1].oparg);
  MOV(64, insn.output.oparg, R(tmp1));
  if (FlagsNeeded(insn))
  {
    NEG(64, insn.inputs[1].oparg);
  }
  // Update_SR_Register64_Carry checks FlagsNeeded as well
  Update_SR_Register64_Carry(insn, R(tmp1), insn.inputs[0].oparg, insn.inputs[1].oparg, insn.SR,
                             true);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::SubUOp = {
    "SubUOp",   &DSPEmitterIR::iremit_SubUOp,
    0x0000,     SR_CMP_MASK | SR_OVERFLOW_STICKY,
    0x0000,     0x0000,
    false,      {{OpAnyReg | Clobbered}, {OpAnyReg | Clobbered | ExtendZero16}},
    {OpAnyReg}, {{OpAnyReg}}};

void DSPEmitterIR::iremit_NegOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  NEG(64, insn.inputs[0].oparg);
  Update_SR_Register64(insn, insn.output.oparg, insn.SR, tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::NegOp = {
    "NegOp", &DSPEmitterIR::iremit_NegOp, 0x0000,     SR_CMP_MASK, 0x0003, 0x0000,
    false,   {{OpAnyReg | SameAsOutput}}, {OpAnyReg}, {{OpAnyReg}}};

void DSPEmitterIR::iremit_AbsOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  CMP(64, insn.inputs[0].oparg, Imm8(0));
  FixupBranch GreaterThanOrEqual = J_CC(CC_GE);
  NEG(64, insn.inputs[0].oparg);
  SetJumpTarget(GreaterThanOrEqual);
  Update_SR_Register64(insn, insn.output.oparg, insn.SR, tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AbsOp = {
    "AbsOp", &DSPEmitterIR::iremit_AbsOp, 0x0000,     SR_CMP_MASK, 0x0003, 0x0000,
    false,   {{OpAnyReg | SameAsOutput}}, {OpAnyReg}, {{OpAnyReg}}};

void DSPEmitterIR::iremit_LslOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  if (insn.inputs[1].oparg.IsImm())
  {
    s16 imm = insn.inputs[1].oparg.AsImm16().SImm16();
    if (imm < 0)
    {
      // need to fix registers since we still
      // don't guarantee sanity here...
      SHL(64, insn.inputs[0].oparg, Imm8(24));
      SHR(64, insn.inputs[0].oparg, Imm8(24 - imm));
    }
    else
    {
      SHL(64, insn.inputs[0].oparg, Imm8(imm));
    }
  }
  else
  {
    MOV(64, R(RCX), insn.inputs[1].oparg);
    TEST(8, R(CL), Imm8(0x40));
    FixupBranch shiftLeft = J_CC(CC_Z);
    NEG(8, R(CL));
    // need to fix registers since we still
    // don't guarantee sanity here...
    SHL(64, insn.inputs[0].oparg, Imm8(24));
    ADD(8, R(CL), Imm8(24));
    SHR(64, insn.inputs[0].oparg, R(CL));
    FixupBranch exit = J();
    SetJumpTarget(shiftLeft);
    SHL(64, insn.inputs[0].oparg, R(CL));
    SetJumpTarget(exit);
  }
  if (FlagsNeeded(insn))
  {
    MOV(64, R(tmp1), insn.output.oparg);
    Update_SR_Register64(insn, R(tmp1), insn.SR, RCX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LslOp = {
    "LslOp",    &DSPEmitterIR::iremit_LslOp,
    0x0000,     SR_CMP_MASK,
    0x0003,     0x0000,
    false,      {{OpAnyReg | SameAsOutput}, {OpAny | Clobbered | NoSaturate}},
    {OpAnyReg}, {{OpAnyReg}, {OpRCX}}};

void DSPEmitterIR::iremit_AslOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  if (insn.inputs[1].oparg.IsImm())
  {
    s16 imm = insn.inputs[1].oparg.AsImm16().SImm16();
    if (imm < 0)
    {
      // need to fix registers since we still
      // don't guarantee sanity here...
      SHL(64, insn.inputs[0].oparg, Imm8(24));
      SAR(64, insn.inputs[0].oparg, Imm8(24 - imm));
    }
    else
    {
      SHL(64, insn.inputs[0].oparg, Imm8(imm));
    }
  }
  else
  {
    MOV(64, R(RCX), insn.inputs[1].oparg);
    TEST(8, R(CL), Imm8(0x40));
    FixupBranch shiftLeft = J_CC(CC_Z);
    NEG(8, R(CL));
    // need to fix registers since we still
    // don't guarantee sanity here...
    SHL(64, insn.inputs[0].oparg, Imm8(24));
    ADD(8, R(CL), Imm8(24));
    SAR(64, insn.inputs[0].oparg, R(CL));
    FixupBranch exit = J();
    SetJumpTarget(shiftLeft);
    SHL(64, insn.inputs[0].oparg, R(CL));
    SetJumpTarget(exit);
  }
  if (FlagsNeeded(insn))
  {
    MOV(64, R(tmp1), insn.output.oparg);
    Update_SR_Register64(insn, R(tmp1), insn.SR, RCX);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::AslOp = {
    "AslOp",    &DSPEmitterIR::iremit_AslOp,
    0x0000,     SR_CMP_MASK,
    0x0003,     0x0000,
    false,      {{OpAnyReg | SameAsOutput}, {OpAny | Clobbered | NoSaturate}},
    {OpAnyReg}, {{OpAnyReg}, {OpRCX}}};

}  // namespace DSP::JITIR::x64
