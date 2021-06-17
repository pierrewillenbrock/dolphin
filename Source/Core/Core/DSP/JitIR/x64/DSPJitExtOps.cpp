// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/CommonTypes.h"

#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP::JITIR::x64
{
// DR $arR
// xxxx xxxx 0000 01rr
// Decrement addressing register $arR.
void DSPEmitterIR::ir_dr(const UDSPInstruction opc)
{
  u8 reg = opc & 0x3;
  IRInsn p = {&SubAOp, {IROp::R(reg), IROp::R(reg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(reg)};
  ir_add_op(p);
}

// IR $arR
// xxxx xxxx 0000 10rr
// Increment addressing register $arR.
void DSPEmitterIR::ir_ir(const UDSPInstruction opc)
{
  u8 reg = opc & 0x3;
  IRInsn p = {&AddAOp, {IROp::R(reg), IROp::R(reg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(reg)};
  ir_add_op(p);
}

// NR $arR
// xxxx xxxx 0000 11rr
// Add corresponding indexing register $ixR to addressing register $arR.
void DSPEmitterIR::ir_nr(const UDSPInstruction opc)
{
  u8 reg = opc & 0x3;
  IRInsn p = {&AddAOp,
              {IROp::R(reg), IROp::R(reg + DSP_REG_WR0), IROp::R(reg + DSP_REG_IX0)},
              IROp::R(reg)};
  ir_add_op(p);
}

// MV $axD.D, $acS.S
// xxxx xxxx 0001 ddss
// Move value of $acS.S to the $axD.D.
void DSPEmitterIR::ir_mv(const UDSPInstruction opc)
{
  u8 sreg = (opc & 0x3) + DSP_REG_ACL0;
  u8 dreg = ((opc >> 2) & 0x3) + DSP_REG_AXL0;
  IRInsn p = {&Mov16Op, {IROp::R(sreg)}, IROp::R(dreg)};
  ir_add_op(p);
}

// S @$arD, $acS.S
// xxxx xxxx 001s s0dd
// Store value of $acS.S in the memory pointed by register $arD.
// Post increment register $arD.
void DSPEmitterIR::ir_s(const UDSPInstruction opc)
{
  u8 dreg = opc & 0x3;
  u8 sreg = ((opc >> 3) & 0x3) + DSP_REG_ACL0;
  IRInsn p1 = {&Store16Op, {IROp::R(dreg), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p1);
  IRInsn p2 = {&AddAOp, {IROp::R(dreg), IROp::R(dreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(dreg)};
  ir_add_op(p2);
}

// SN @$arD, $acS.S
// xxxx xxxx 001s s1dd
// Store value of register $acS.S in the memory pointed by register $arD.
// Add indexing register $ixD to register $arD.
void DSPEmitterIR::ir_sn(const UDSPInstruction opc)
{
  u8 dreg = opc & 0x3;
  u8 sreg = ((opc >> 3) & 0x3) + DSP_REG_ACL0;
  IRInsn p1 = {&Store16Op, {IROp::R(dreg), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p1);
  IRInsn p2 = {&AddAOp,
               {IROp::R(dreg), IROp::R(dreg + DSP_REG_WR0), IROp::R(dreg + DSP_REG_IX0)},
               IROp::R(dreg)};
  ir_add_op(p2);
}

// L $axD.D, @$arS
// xxxx xxxx 01dd d0ss
// Load $axD.D/$acD.D with value from memory pointed by register $arS.
// Post increment register $arS.
void DSPEmitterIR::ir_l(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x3;
  u8 dreg = ((opc >> 3) & 0x7) + DSP_REG_AXL0;  // AX?.?, AC?.[LM]
  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R(dreg)};
  ir_add_op(p1);
  IRInsn p2 = {&AddAOp, {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(sreg)};
  ir_add_op(p2);
}

// LN $axD.D, @$arS
// xxxx xxxx 01dd d0ss
// Load $axD.D/$acD.D with value from memory pointed by register $arS.
// Add indexing register register $ixS to register $arS.
void DSPEmitterIR::ir_ln(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x3;
  u8 dreg = ((opc >> 3) & 0x7) + DSP_REG_AXL0;
  IRInsn p1 = {
      &Load16Op, {IROp::R(sreg)}, IROp::R(dreg), {},
  };
  ir_add_op(p1);
  IRInsn p2 = {&AddAOp,
               {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::R(sreg + DSP_REG_IX0)},
               IROp::R(sreg)};
  ir_add_op(p2);
}

// LS $axD.D, $acS.m
// xxxx xxxx 10dd 000s
// Load register $axD.D with value from memory pointed by register
// $ar0. Store value from register $acS.m to memory location pointed by
// register $ar3. Increment both $ar0 and $ar3.
void DSPEmitterIR::ir_ls(const UDSPInstruction opc)
{
  u8 sreg = (opc & 0x1) + DSP_REG_ACM0;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;
  IRInsn p1 = {&Load16Op, {IROp::R(DSP_REG_AR0)}, IROp::R(dreg)};
  ir_add_op(p1);
  IRInsn p2 = {&Store16Op, {IROp::R(DSP_REG_AR3), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p2);
  IRInsn p3 = {
      &AddAOp, {IROp::R(DSP_REG_AR0), IROp::R(DSP_REG_WR0), IROp::Imm(1)}, IROp::R(DSP_REG_AR0)};
  ir_add_op(p3);
  IRInsn p4 = {
      &AddAOp, {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::Imm(1)}, IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// LSN $axD.D, $acS.m
// xxxx xxxx 10dd 010s
// Load register $axD.D with value from memory pointed by register
// $ar0. Store value from register $acS.m to memory location pointed by
// register $ar3. Add corresponding indexing register $ix0 to addressing
// register $ar0 and increment $ar3.
void DSPEmitterIR::ir_lsn(const UDSPInstruction opc)
{
  u8 sreg = (opc & 0x1) + DSP_REG_ACM0;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;
  IRInsn p1 = {&Load16Op, {IROp::R(DSP_REG_AR0)}, IROp::R(dreg)};
  ir_add_op(p1);
  IRInsn p2 = {&Store16Op, {IROp::R(DSP_REG_AR3), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p2);
  IRInsn p3 = {&AddAOp,
               {IROp::R(DSP_REG_AR0), IROp::R(DSP_REG_WR0), IROp::R(DSP_REG_IX0)},
               IROp::R(DSP_REG_AR0)};
  ir_add_op(p3);
  IRInsn p4 = {
      &AddAOp, {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::Imm(1)}, IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// LSM $axD.D, $acS.m
// xxxx xxxx 10dd 100s
// Load register $axD.D with value from memory pointed by register
// $ar0. Store value from register $acS.m to memory location pointed by
// register $ar3. Add corresponding indexing register $ix3 to addressing
// register $ar3 and increment $ar0.
void DSPEmitterIR::ir_lsm(const UDSPInstruction opc)
{
  u8 sreg = (opc & 0x1) + DSP_REG_ACM0;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;
  IRInsn p1 = {&Load16Op, {IROp::R(DSP_REG_AR0)}, IROp::R(dreg)};
  ir_add_op(p1);
  IRInsn p2 = {&Store16Op, {IROp::R(DSP_REG_AR3), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p2);
  IRInsn p3 = {
      &AddAOp, {IROp::R(DSP_REG_AR0), IROp::R(DSP_REG_WR0), IROp::Imm(1)}, IROp::R(DSP_REG_AR0)};
  ir_add_op(p3);
  IRInsn p4 = {&AddAOp,
               {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::R(DSP_REG_IX3)},
               IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// LSMN $axD.D, $acS.m
// xxxx xxxx 10dd 110s
// Load register $axD.D with value from memory pointed by register
// $ar0. Store value from register $acS.m to memory location pointed by
// register $ar3. Add corresponding indexing register $ix0 to addressing
// register $ar0 and add corresponding indexing register $ix3 to addressing
// register $ar3.
void DSPEmitterIR::ir_lsnm(const UDSPInstruction opc)
{
  u8 sreg = (opc & 0x1) + DSP_REG_ACM0;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;
  IRInsn p1 = {&Load16Op, {IROp::R(DSP_REG_AR0)}, IROp::R(dreg)};
  ir_add_op(p1);
  IRInsn p2 = {&Store16Op, {IROp::R(DSP_REG_AR3), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p2);
  IRInsn p3 = {&AddAOp,
               {IROp::R(DSP_REG_AR0), IROp::R(DSP_REG_WR0), IROp::R(DSP_REG_IX0)},
               IROp::R(DSP_REG_AR0)};
  ir_add_op(p3);
  IRInsn p4 = {&AddAOp,
               {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::R(DSP_REG_IX3)},
               IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// SL $acS.m, $axD.D
// xxxx xxxx 10dd 001s
// Store value from register $acS.m to memory location pointed by register
// $ar0. Load register $axD.D with value from memory pointed by register
// $ar3. Increment both $ar0 and $ar3.
void DSPEmitterIR::ir_sl(const UDSPInstruction opc)
{
  u8 sreg = (opc & 0x1) + DSP_REG_ACM0;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;
  IRInsn p1 = {&Store16Op, {IROp::R(DSP_REG_AR0), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R(dreg)};
  ir_add_op(p2);
  IRInsn p3 = {
      &AddAOp, {IROp::R(DSP_REG_AR0), IROp::R(DSP_REG_WR0), IROp::Imm(1)}, IROp::R(DSP_REG_AR0)};
  ir_add_op(p3);
  IRInsn p4 = {
      &AddAOp, {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::Imm(1)}, IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// SLN $acS.m, $axD.D
// xxxx xxxx 10dd 011s
// Store value from register $acS.m to memory location pointed by register
// $ar0. Load register $axD.D with value from memory pointed by register
// $ar3. Add corresponding indexing register $ix0 to addressing register $ar0
// and increment $ar3.
void DSPEmitterIR::ir_sln(const UDSPInstruction opc)
{
  u8 sreg = (opc & 0x1) + DSP_REG_ACM0;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;
  IRInsn p1 = {&Store16Op, {IROp::R(DSP_REG_AR0), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R(dreg)};
  ir_add_op(p2);
  IRInsn p3 = {&AddAOp,
               {IROp::R(DSP_REG_AR0), IROp::R(DSP_REG_WR0), IROp::R(DSP_REG_IX0)},
               IROp::R(DSP_REG_AR0)};
  ir_add_op(p3);
  IRInsn p4 = {
      &AddAOp, {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::Imm(1)}, IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// SLM $acS.m, $axD.D
// xxxx xxxx 10dd 101s
// Store value from register $acS.m to memory location pointed by register
// $ar0. Load register $axD.D with value from memory pointed by register
// $ar3. Add corresponding indexing register $ix3 to addressing register $ar3
// and increment $ar0.
void DSPEmitterIR::ir_slm(const UDSPInstruction opc)
{
  u8 sreg = (opc & 0x1) + DSP_REG_ACM0;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;
  IRInsn p1 = {&Store16Op, {IROp::R(DSP_REG_AR0), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R(dreg)};
  ir_add_op(p2);
  IRInsn p3 = {
      &AddAOp, {IROp::R(DSP_REG_AR0), IROp::R(DSP_REG_WR0), IROp::Imm(1)}, IROp::R(DSP_REG_AR0)};
  ir_add_op(p3);
  IRInsn p4 = {&AddAOp,
               {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::R(DSP_REG_IX3)},
               IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// SLMN $acS.m, $axD.D
// xxxx xxxx 10dd 111s
// Store value from register $acS.m to memory location pointed by register
// $ar0. Load register $axD.D with value from memory pointed by register
// $ar3. Add corresponding indexing register $ix0 to addressing register $ar0
// and add corresponding indexing register $ix3 to addressing register $ar3.
void DSPEmitterIR::ir_slnm(const UDSPInstruction opc)
{
  u8 sreg = (opc & 0x1) + DSP_REG_ACM0;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;
  IRInsn p1 = {&Store16Op, {IROp::R(DSP_REG_AR0), IROp::R(sreg)}, IROp::None()};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R(dreg)};
  ir_add_op(p2);
  IRInsn p3 = {&AddAOp,
               {IROp::R(DSP_REG_AR0), IROp::R(DSP_REG_WR0), IROp::R(DSP_REG_IX0)},
               IROp::R(DSP_REG_AR0)};
  ir_add_op(p3);
  IRInsn p4 = {&AddAOp,
               {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::R(DSP_REG_IX3)},
               IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// LD $ax0.d, $ax1.r, @$arS
// xxxx xxxx 11dr 00ss, ss != 11
// example for "nx'ld $AX0.L, $AX1.L, @$AR3"
// Loads the word pointed by AR0 to AX0.H, then loads the word pointed by AR3
// to AX0.L.  Increments AR0 and AR3.  If AR0 and AR3 point into the same
// memory page (upper 6 bits of addr are the same -> games are not doing that!)
// then the value pointed by AR0 is loaded to BOTH AX0.H and AX0.L.  If AR0
// points into an invalid memory page (ie 0x2000), then AX0.H keeps its old
// value. (not implemented yet) If AR3 points into an invalid memory page, then
// AX0.L gets the same value as AX0.H. (not implemented yet)
void DSPEmitterIR::ir_ld(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;
  u8 sreg = opc & 0x3;
  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R((dreg << 1) + DSP_REG_AXL0)};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R((rreg << 1) + DSP_REG_AXL1)};
  ir_add_op(p2);
  IRInsn p3 = {&AddAOp, {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(sreg)};
  ir_add_op(p3);
  IRInsn p4 = {
      &AddAOp, {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::Imm(1)}, IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// LDAX $axR, @$arS
// xxxx xxxx 11sr 0011
void DSPEmitterIR::ir_ldax(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;

  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R(rreg + DSP_REG_AXH0)};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R(rreg + DSP_REG_AXL0)};
  ir_add_op(p2);

  IRInsn p3 = {&AddAOp, {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(sreg)};
  ir_add_op(p3);
  IRInsn p4 = {
      &AddAOp, {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::Imm(1)}, IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// LDN $ax0.d, $ax1.r, @$arS
// xxxx xxxx 11dr 01ss, ss != 11
void DSPEmitterIR::ir_ldn(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;
  u8 sreg = opc & 0x3;
  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R((dreg << 1) + DSP_REG_AXL0)};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R((rreg << 1) + DSP_REG_AXL1)};
  ir_add_op(p2);
  IRInsn p3 = {&AddAOp,
               {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::R(sreg + DSP_REG_IX0)},
               IROp::R(sreg)};
  ir_add_op(p3);
  IRInsn p4 = {
      &AddAOp, {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::Imm(1)}, IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// LDAXN $axR, @$arS
// xxxx xxxx 11sr 0111
void DSPEmitterIR::ir_ldaxn(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;
  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R(rreg + DSP_REG_AXH0)};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R(rreg + DSP_REG_AXL0)};
  ir_add_op(p2);

  IRInsn p3 = {&AddAOp,
               {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::R(sreg + DSP_REG_IX0)},
               IROp::R(sreg)};
  ir_add_op(p3);
  IRInsn p4 = {
      &AddAOp, {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::Imm(1)}, IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// LDM $ax0.d, $ax1.r, @$arS
// xxxx xxxx 11dr 10ss, ss != 11
void DSPEmitterIR::ir_ldm(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;
  u8 sreg = opc & 0x3;
  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R((dreg << 1) + DSP_REG_AXL0)};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R((rreg << 1) + DSP_REG_AXL1)};
  ir_add_op(p2);
  IRInsn p3 = {&AddAOp, {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(sreg)};
  ir_add_op(p3);
  IRInsn p4 = {&AddAOp,
               {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::R(DSP_REG_IX3)},
               IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// LDAXM $axR, @$arS
// xxxx xxxx 11sr 1011
void DSPEmitterIR::ir_ldaxm(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;
  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R(rreg + DSP_REG_AXH0)};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R(rreg + DSP_REG_AXL0)};
  ir_add_op(p2);

  IRInsn p3 = {&AddAOp, {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::Imm(1)}, IROp::R(sreg)};
  ir_add_op(p3);
  IRInsn p4 = {&AddAOp,
               {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::R(DSP_REG_IX3)},
               IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// LDNM $ax0.d, $ax1.r, @$arS
// xxxx xxxx 11dr 11ss, ss != 11
void DSPEmitterIR::ir_ldnm(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;
  u8 sreg = opc & 0x3;
  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R((dreg << 1) + DSP_REG_AXL0)};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R((rreg << 1) + DSP_REG_AXL1)};
  ir_add_op(p2);
  IRInsn p3 = {&AddAOp,
               {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::R(sreg + DSP_REG_IX0)},
               IROp::R(sreg)};
  ir_add_op(p3);
  IRInsn p4 = {&AddAOp,
               {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::R(DSP_REG_IX3)},
               IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

// LDAXNM $axR, @$arS
// xxxx xxxx 11sr 1111
void DSPEmitterIR::ir_ldaxnm(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;
  IRInsn p1 = {&Load16Op, {IROp::R(sreg)}, IROp::R(rreg + DSP_REG_AXH0)};
  ir_add_op(p1);
  IRInsn p2 = {&Load16Op, {IROp::R(DSP_REG_AR3)}, IROp::R(rreg + DSP_REG_AXL0)};
  ir_add_op(p2);

  IRInsn p3 = {&AddAOp,
               {IROp::R(sreg), IROp::R(sreg + DSP_REG_WR0), IROp::R(sreg + DSP_REG_IX0)},
               IROp::R(sreg)};
  ir_add_op(p3);
  IRInsn p4 = {&AddAOp,
               {IROp::R(DSP_REG_AR3), IROp::R(DSP_REG_WR3), IROp::R(DSP_REG_IX3)},
               IROp::R(DSP_REG_AR3)};
  ir_add_op(p4);
}

}  // namespace DSP::JITIR::x64
