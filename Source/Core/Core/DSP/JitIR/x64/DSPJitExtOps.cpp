// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/CommonTypes.h"

#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

/* It is safe to directly write to the address registers as they are
   neither read not written by any extendable opcode. The same is true
   for memory accesses.
   It probably even is safe to write to all registers except for
   SR, ACx.x, AXx.x and PROD, which may be modified by the main op.

   This code uses EBX to keep the values of the registers written by
   the extended op so the main op can still access the old values.
   storeIndex and storeIndex2 control where the lower and upper 16bits
   of EBX are written to. Additionally, the upper 16bits can contain the
   original SR so we can do sign extension in 40bit mode. There is only
   the 'ld family of opcodes writing to two registers at the same time,
   and those always are AXx.x, thus no need to leave space for SR for
   sign extension.
 */

namespace DSP::JITIR::x64
{
// DR $arR
// xxxx xxxx 0000 01rr
// Decrement addressing register $arR.
void DSPEmitterIR::dr(const UDSPInstruction opc)
{
  u8 reg = opc & 0x3;

  X64Reg tmp1 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_WR0 + reg, RDX, RegisterExtension::Zero);
  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
  MOVZX(32, 16, RAX, ar_reg);

  decrement_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0 + reg);

  m_gpr.PutXReg(tmp1);
}

// IR $arR
// xxxx xxxx 0000 10rr
// Increment addressing register $arR.
void DSPEmitterIR::ir(const UDSPInstruction opc)
{
  u8 reg = opc & 0x3;

  X64Reg tmp1 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_WR0 + reg, RDX, RegisterExtension::Zero);
  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
  MOVZX(32, 16, RAX, ar_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0 + reg);

  m_gpr.PutXReg(tmp1);
}

// NR $arR
// xxxx xxxx 0000 11rr
// Add corresponding indexing register $ixR to addressing register $arR.
void DSPEmitterIR::nr(const UDSPInstruction opc)
{
  u8 reg = opc & 0x3;

  X64Reg tmp1 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_WR0 + reg, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0 + reg, RCX, RegisterExtension::Sign);
  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + reg);
  MOVZX(32, 16, RAX, ar_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0 + reg);

  m_gpr.PutXReg(tmp1);
}

// MV $axD.D, $acS.S
// xxxx xxxx 0001 ddss
// Move value of $acS.S to the $axD.D.
void DSPEmitterIR::mv(const UDSPInstruction opc)
{
  u8 sreg = (opc & 0x3) + DSP_REG_ACL0;
  u8 dreg = ((opc >> 2) & 0x3);
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  dsp_op_read_reg(sreg, RBX, RegisterExtension::Zero, tmp1, tmp2, RAX);

  m_store_index = dreg + DSP_REG_AXL0;

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// S @$arD, $acS.S
// xxxx xxxx 001s s0dd
// Store value of $acS.S in the memory pointed by register $arD.
// Post increment register $arD.
void DSPEmitterIR::s(const UDSPInstruction opc)
{
  u8 dreg = opc & 0x3;
  u8 sreg = ((opc >> 3) & 0x3) + DSP_REG_ACL0;
  //	u16 addr = g_dsp.r[dest];
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(dreg, RAX, RegisterExtension::Zero);

  dsp_op_read_reg(sreg, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  //	u16 val = g_dsp.r[src];
  dmem_write(tmp1, RAX, RCX);

  m_gpr.ReadReg(DSP_REG_WR0 + dreg, RDX, RegisterExtension::Zero);
  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + dreg);
  MOVZX(32, 16, RAX, ar_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0 + dreg);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SN @$arD, $acS.S
// xxxx xxxx 001s s1dd
// Store value of register $acS.S in the memory pointed by register $arD.
// Add indexing register $ixD to register $arD.
void DSPEmitterIR::sn(const UDSPInstruction opc)
{
  u8 dreg = opc & 0x3;
  u8 sreg = ((opc >> 3) & 0x3) + DSP_REG_ACL0;
  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(dreg, RAX, RegisterExtension::Zero);

  dsp_op_read_reg(sreg, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  dmem_write(tmp1, RAX, RCX);

  m_gpr.ReadReg(DSP_REG_WR0 + dreg, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0 + dreg, RCX, RegisterExtension::Sign);
  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + dreg);
  MOVZX(32, 16, RAX, ar_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0 + dreg);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// L $axD.D, @$arS
// xxxx xxxx 01dd d0ss
// Load $axD.D/$acD.D with value from memory pointed by register $arS.
// Post increment register $arS.
void DSPEmitterIR::l(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x3;
  u8 dreg = ((opc >> 3) & 0x7) + DSP_REG_AXL0;  // AX?.?, AC?.[LM]

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  pushExtValueFromMem(dreg, sreg);

  if (dreg >= DSP_REG_ACM0)
  {
    // save SR too, so we can decide later.
    // even if only for one bit, can only
    // store (up to) two registers in EBX,
    // so store all of SR
    m_gpr.ReadReg(DSP_REG_SR, RAX, RegisterExtension::None);
    SHL(32, R(EAX), Imm8(16));
    OR(32, R(EBX), R(EAX));
  }

  m_gpr.ReadReg(DSP_REG_WR0 + sreg, RDX, RegisterExtension::Zero);
  const OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + sreg);
  MOVZX(32, 16, RAX, ar_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0 + sreg);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LN $axD.D, @$arS
// xxxx xxxx 01dd d0ss
// Load $axD.D/$acD.D with value from memory pointed by register $arS.
// Add indexing register $ixS to register $arS.
void DSPEmitterIR::ln(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x3;
  u8 dreg = ((opc >> 3) & 0x7) + DSP_REG_AXL0;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();

  pushExtValueFromMem(dreg, sreg);

  if (dreg >= DSP_REG_ACM0)
  {
    // save SR too, so we can decide later.
    // even if only for one bit, can only
    // store (up to) two registers in EBX,
    // so store all of SR
    m_gpr.ReadReg(DSP_REG_SR, RAX, RegisterExtension::None);
    SHL(32, R(EAX), Imm8(16));
    OR(32, R(EBX), R(EAX));
  }

  m_gpr.ReadReg(DSP_REG_WR0 + sreg, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0 + sreg, RCX, RegisterExtension::Sign);
  OpArg ar_reg = m_gpr.GetReg(DSP_REG_AR0 + sreg);
  MOVZX(32, 16, RAX, ar_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0 + sreg);

  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LS $axD.D, $acS.m
// xxxx xxxx 10dd 000s
// Load register $axD.D with value from memory pointed by register
// $ar0. Store value from register $acS.m to memory location pointed by
// register $ar3. Increment both $ar0 and $ar3.
void DSPEmitterIR::ls(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x1;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_AR3, RAX, RegisterExtension::Zero);
  dsp_op_read_reg(sreg + DSP_REG_ACM0, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  dmem_write(tmp1, RAX, RCX);

  pushExtValueFromMem(dreg, DSP_REG_AR0);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar3_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR0, RDX, RegisterExtension::Zero);
  const OpArg ar0_reg = m_gpr.GetReg(DSP_REG_AR0);
  MOVZX(32, 16, RAX, ar0_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar0_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LSN $axD.D, $acS.m
// xxxx xxxx 10dd 010s
// Load register $axD.D with value from memory pointed by register
// $ar0. Store value from register $acS.m to memory location pointed by
// register $ar3. Add corresponding indexing register $ix0 to addressing
// register $ar0 and increment $ar3.
void DSPEmitterIR::lsn(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x1;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_AR3, RAX, RegisterExtension::Zero);

  dsp_op_read_reg(sreg + DSP_REG_ACM0, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  dmem_write(tmp1, RAX, RCX);

  pushExtValueFromMem(dreg, DSP_REG_AR0);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar3_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR0, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0, RCX, RegisterExtension::Sign);
  const OpArg ar0_reg = m_gpr.GetReg(DSP_REG_AR0);
  MOVZX(32, 16, RAX, ar0_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar0_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LSM $axD.D, $acS.m
// xxxx xxxx 10dd 100s
// Load register $axD.D with value from memory pointed by register
// $ar0. Store value from register $acS.m to memory location pointed by
// register $ar3. Add corresponding indexing register $ix3 to addressing
// register $ar3 and increment $ar0.
void DSPEmitterIR::lsm(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x1;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_AR3, RAX, RegisterExtension::Zero);
  dsp_op_read_reg(sreg + DSP_REG_ACM0, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  dmem_write(tmp1, RAX, RCX);

  pushExtValueFromMem(dreg, DSP_REG_AR0);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX3, RCX, RegisterExtension::Sign);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar3_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR0, RDX, RegisterExtension::Zero);
  const OpArg ar0_reg = m_gpr.GetReg(DSP_REG_AR0);
  MOVZX(32, 16, RAX, ar0_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar0_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LSMN $axD.D, $acS.m
// xxxx xxxx 10dd 110s
// Load register $axD.D with value from memory pointed by register
// $ar0. Store value from register $acS.m to memory location pointed by
// register $ar3. Add corresponding indexing register $ix0 to addressing
// register $ar0 and add corresponding indexing register $ix3 to addressing
// register $ar3.
void DSPEmitterIR::lsnm(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x1;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_AR3, RAX, RegisterExtension::Zero);
  dsp_op_read_reg(sreg + DSP_REG_ACM0, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  dmem_write(tmp1, RAX, RCX);

  pushExtValueFromMem(dreg, DSP_REG_AR0);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX3, RCX, RegisterExtension::Sign);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar3_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR0, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0, RCX, RegisterExtension::Sign);
  const OpArg ar0_reg = m_gpr.GetReg(DSP_REG_AR0);
  MOVZX(32, 16, RAX, ar0_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar0_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SL $acS.m, $axD.D
// xxxx xxxx 10dd 001s
// Store value from register $acS.m to memory location pointed by register
// $ar0. Load register $axD.D with value from memory pointed by register
// $ar3. Increment both $ar0 and $ar3.
void DSPEmitterIR::sl(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x1;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_AR0, RAX, RegisterExtension::Zero);
  dsp_op_read_reg(sreg + DSP_REG_ACM0, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  dmem_write(tmp1, RAX, RCX);

  pushExtValueFromMem(dreg, DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar3_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR0, RDX, RegisterExtension::Zero);
  const OpArg ar0_reg = m_gpr.GetReg(DSP_REG_AR0);
  MOVZX(32, 16, RAX, ar0_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar0_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SLN $acS.m, $axD.D
// xxxx xxxx 10dd 011s
// Store value from register $acS.m to memory location pointed by register
// $ar0. Load register $axD.D with value from memory pointed by register
// $ar3. Add corresponding indexing register $ix0 to addressing register $ar0
// and increment $ar3.
void DSPEmitterIR::sln(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x1;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_AR0, RAX, RegisterExtension::Zero);
  dsp_op_read_reg(sreg + DSP_REG_ACM0, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  dmem_write(tmp1, RAX, RCX);

  pushExtValueFromMem(dreg, DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar3_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR0, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0, RCX, RegisterExtension::Sign);
  const OpArg ar0_reg = m_gpr.GetReg(DSP_REG_AR0);
  MOVZX(32, 16, RAX, ar0_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar0_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SLM $acS.m, $axD.D
// xxxx xxxx 10dd 101s
// Store value from register $acS.m to memory location pointed by register
// $ar0. Load register $axD.D with value from memory pointed by register
// $ar3. Add corresponding indexing register $ix3 to addressing register $ar3
// and increment $ar0.
void DSPEmitterIR::slm(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x1;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_AR0, RAX, RegisterExtension::Zero);
  dsp_op_read_reg(sreg + DSP_REG_ACM0, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  dmem_write(tmp1, RAX, RCX);

  pushExtValueFromMem(dreg, DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX3, RCX, RegisterExtension::Sign);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar3_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR0, RDX, RegisterExtension::Zero);
  const OpArg ar0_reg = m_gpr.GetReg(DSP_REG_AR0);
  MOVZX(32, 16, RAX, ar0_reg);

  increment_addr_reg(RAX, RDX, tmp1, RCX);

  MOV(32, ar0_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// SLMN $acS.m, $axD.D
// xxxx xxxx 10dd 111s
// Store value from register $acS.m to memory location pointed by register
// $ar0. Load register $axD.D with value from memory pointed by register
// $ar3. Add corresponding indexing register $ix0 to addressing register $ar0
// and add corresponding indexing register $ix3 to addressing register $ar3.
void DSPEmitterIR::slnm(const UDSPInstruction opc)
{
  u8 sreg = opc & 0x1;
  u8 dreg = ((opc >> 4) & 0x3) + DSP_REG_AXL0;

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_AR0, RAX, RegisterExtension::Zero);
  dsp_op_read_reg(sreg + DSP_REG_ACM0, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  dmem_write(tmp1, RAX, RCX);

  pushExtValueFromMem(dreg, DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX3, RCX, RegisterExtension::Sign);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar3_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.ReadReg(DSP_REG_WR0, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0, RCX, RegisterExtension::Sign);
  const OpArg ar0_reg = m_gpr.GetReg(DSP_REG_AR0);
  MOVZX(32, 16, RAX, ar0_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp1);

  MOV(32, ar0_reg, R(tmp1));
  m_gpr.PutReg(DSP_REG_AR0);

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

// LD $ax0.d, $ax1.r, @$arS
// xxxx xxxx 11dr 00ss
// example for "nx'ld $AX0.L, $AX1.L, @$AR3"
// Loads the word pointed by AR0 to AX0.H, then loads the word pointed by AR3
// to AX0.L.  Increments AR0 and AR3.  If AR0 and AR3 point into the same
// memory page (upper 6 bits of addr are the same -> games are not doing that!)
// then the value pointed by AR0 is loaded to BOTH AX0.H and AX0.L.  If AR0
// points into an invalid memory page (ie 0x2000), then AX0.H keeps its old
// value. (not implemented yet) If AR3 points into an invalid memory page, then
// AX0.L gets the same value as AX0.H. (not implemented yet)
void DSPEmitterIR::ld(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;
  u8 sreg = opc & 0x3;

  pushExtValueFromMem((dreg << 1) + DSP_REG_AXL0, sreg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  // 	if (IsSameMemArea(g_dsp.r[sreg], g_dsp.r[DSP_REG_AR3])) {
  m_gpr.ReadReg(sreg, RCX, RegisterExtension::None);
  m_gpr.ReadReg(DSP_REG_AR3, tmp1, RegisterExtension::None);
  XOR(16, R(ECX), R(tmp1));
  m_gpr.PutXReg(tmp1);
  DSPJitIRRegCache c(m_gpr);
  TEST(16, R(ECX), Imm16(0xfc00));
  FixupBranch not_equal = J_CC(CC_NE, true);
  pushExtValueFromMem2((rreg << 1) + DSP_REG_AXL1, sreg);
  m_gpr.FlushRegs(c);
  FixupBranch after = J(true);
  SetJumpTarget(not_equal);  // else
  pushExtValueFromMem2((rreg << 1) + DSP_REG_AXL1, DSP_REG_AR3);
  m_gpr.FlushRegs(c);
  SetJumpTarget(after);

  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_WR0 + sreg, RDX, RegisterExtension::Zero);
  const OpArg arx_reg = m_gpr.GetReg(DSP_REG_AR0 + sreg);
  MOVZX(32, 16, RAX, arx_reg);

  increment_addr_reg(RAX, RDX, tmp3, RCX);

  MOV(32, arx_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0 + sreg);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increment_addr_reg(RAX, RDX, tmp3, RCX);

  MOV(32, ar3_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.PutXReg(tmp3);
}

// LDAX $axR, @$arS
// xxxx xxxx 11sr 0011
void DSPEmitterIR::ldax(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;

  pushExtValueFromMem(rreg + DSP_REG_AXH0, sreg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  // if (IsSameMemArea(g_dsp.r[sreg], g_dsp.r[DSP_REG_AR3])) {
  m_gpr.ReadReg(sreg, RCX, RegisterExtension::None);
  m_gpr.ReadReg(DSP_REG_AR3, tmp1, RegisterExtension::None);
  XOR(16, R(ECX), R(tmp1));
  m_gpr.PutXReg(tmp1);
  DSPJitIRRegCache c(m_gpr);
  TEST(16, R(ECX), Imm16(0xfc00));
  FixupBranch not_equal = J_CC(CC_NE, true);
  pushExtValueFromMem2(rreg + DSP_REG_AXL0, sreg);
  m_gpr.FlushRegs(c);
  FixupBranch after = J(true);  // else
  SetJumpTarget(not_equal);
  pushExtValueFromMem2(rreg + DSP_REG_AXL0, DSP_REG_AR3);
  m_gpr.FlushRegs(c);
  SetJumpTarget(after);

  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_WR0 + sreg, RDX, RegisterExtension::Zero);
  const OpArg arx_reg = m_gpr.GetReg(DSP_REG_AR0 + sreg);
  MOVZX(32, 16, RAX, arx_reg);

  increment_addr_reg(RAX, RDX, tmp3, RCX);

  MOV(32, arx_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0 + sreg);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increment_addr_reg(RAX, RDX, tmp3, RCX);

  MOV(32, ar3_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.PutXReg(tmp3);
}

// LDN $ax0.d, $ax1.r, @$arS
// xxxx xxxx 11dr 01ss
void DSPEmitterIR::ldn(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;
  u8 sreg = opc & 0x3;

  pushExtValueFromMem((dreg << 1) + DSP_REG_AXL0, sreg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  // if (IsSameMemArea(g_dsp.r[sreg], g_dsp.r[DSP_REG_AR3])) {
  m_gpr.ReadReg(sreg, RCX, RegisterExtension::None);
  m_gpr.ReadReg(DSP_REG_AR3, tmp1, RegisterExtension::None);
  XOR(16, R(ECX), R(tmp1));
  m_gpr.PutXReg(tmp1);
  DSPJitIRRegCache c(m_gpr);
  TEST(16, R(ECX), Imm16(0xfc00));
  FixupBranch not_equal = J_CC(CC_NE, true);
  pushExtValueFromMem2((rreg << 1) + DSP_REG_AXL1, sreg);
  m_gpr.FlushRegs(c);
  FixupBranch after = J(true);
  SetJumpTarget(not_equal);  // else
  pushExtValueFromMem2((rreg << 1) + DSP_REG_AXL1, DSP_REG_AR3);
  m_gpr.FlushRegs(c);
  SetJumpTarget(after);

  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_WR0 + sreg, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0 + sreg, RCX, RegisterExtension::Sign);
  const OpArg arx_reg = m_gpr.GetReg(DSP_REG_AR0 + sreg);
  MOVZX(32, 16, RAX, arx_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp3);

  MOV(32, arx_reg, R(tmp3));
  m_gpr.PutReg(DSP_REG_AR0 + sreg);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increment_addr_reg(RAX, RDX, tmp3, RCX);

  MOV(32, ar3_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.PutXReg(tmp3);
}

// LDAXN $axR, @$arS
// xxxx xxxx 11sr 0111
void DSPEmitterIR::ldaxn(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;

  pushExtValueFromMem(rreg + DSP_REG_AXH0, sreg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  // if (IsSameMemArea(g_dsp.r[sreg], g_dsp.r[DSP_REG_AR3])) {
  m_gpr.ReadReg(sreg, RCX, RegisterExtension::None);
  m_gpr.ReadReg(DSP_REG_AR3, tmp1, RegisterExtension::None);
  XOR(16, R(ECX), R(tmp1));
  m_gpr.PutXReg(tmp1);
  DSPJitIRRegCache c(m_gpr);
  TEST(16, R(ECX), Imm16(0xfc00));
  FixupBranch not_equal = J_CC(CC_NE, true);
  pushExtValueFromMem2(rreg + DSP_REG_AXL0, sreg);
  m_gpr.FlushRegs(c);
  FixupBranch after = J(true);  // else
  SetJumpTarget(not_equal);
  pushExtValueFromMem2(rreg + DSP_REG_AXL0, DSP_REG_AR3);
  m_gpr.FlushRegs(c);
  SetJumpTarget(after);

  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_WR0 + sreg, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0 + sreg, RCX, RegisterExtension::Sign);
  const OpArg arx_reg = m_gpr.GetReg(DSP_REG_AR0 + sreg);
  MOVZX(32, 16, RAX, arx_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp3);

  MOV(32, arx_reg, R(tmp3));
  m_gpr.PutReg(DSP_REG_AR0 + sreg);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increment_addr_reg(RAX, RDX, tmp3, RCX);

  MOV(32, ar3_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.PutXReg(tmp3);
}

// LDM $ax0.d, $ax1.r, @$arS
// xxxx xxxx 11dr 10ss
void DSPEmitterIR::ldm(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;
  u8 sreg = opc & 0x3;

  pushExtValueFromMem((dreg << 1) + DSP_REG_AXL0, sreg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  // if (IsSameMemArea(g_dsp.r[sreg], g_dsp.r[DSP_REG_AR3])) {
  m_gpr.ReadReg(sreg, RCX, RegisterExtension::None);
  m_gpr.ReadReg(DSP_REG_AR3, tmp1, RegisterExtension::None);
  XOR(16, R(ECX), R(tmp1));
  m_gpr.PutXReg(tmp1);
  DSPJitIRRegCache c(m_gpr);
  TEST(16, R(ECX), Imm16(0xfc00));
  FixupBranch not_equal = J_CC(CC_NE, true);
  pushExtValueFromMem2((rreg << 1) + DSP_REG_AXL1, sreg);
  m_gpr.FlushRegs(c);
  FixupBranch after = J(true);
  SetJumpTarget(not_equal);  // else
  pushExtValueFromMem2((rreg << 1) + DSP_REG_AXL1, DSP_REG_AR3);
  m_gpr.FlushRegs(c);
  SetJumpTarget(after);

  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_WR0 + sreg, RDX, RegisterExtension::Zero);
  const OpArg arx_reg = m_gpr.GetReg(DSP_REG_AR0 + sreg);
  MOVZX(32, 16, RAX, arx_reg);

  increment_addr_reg(RAX, RDX, tmp3, RCX);

  MOV(32, arx_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0 + sreg);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX3, RCX, RegisterExtension::Sign);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp3);

  MOV(32, ar3_reg, R(tmp3));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.PutXReg(tmp3);
}

// LDAXM $axR, @$arS
// xxxx xxxx 11sr 1011
void DSPEmitterIR::ldaxm(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;

  pushExtValueFromMem(rreg + DSP_REG_AXH0, sreg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  // if (IsSameMemArea(g_dsp.r[sreg], g_dsp.r[DSP_REG_AR3])) {
  m_gpr.ReadReg(sreg, RCX, RegisterExtension::None);
  m_gpr.ReadReg(DSP_REG_AR3, tmp1, RegisterExtension::None);
  XOR(16, R(ECX), R(tmp1));
  m_gpr.PutXReg(tmp1);
  DSPJitIRRegCache c(m_gpr);
  TEST(16, R(ECX), Imm16(0xfc00));
  FixupBranch not_equal = J_CC(CC_NE, true);
  pushExtValueFromMem2(rreg + DSP_REG_AXL0, sreg);
  m_gpr.FlushRegs(c);
  FixupBranch after = J(true);  // else
  SetJumpTarget(not_equal);
  pushExtValueFromMem2(rreg + DSP_REG_AXL0, DSP_REG_AR3);
  m_gpr.FlushRegs(c);
  SetJumpTarget(after);

  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_WR0 + sreg, RDX, RegisterExtension::Zero);
  const OpArg arx_reg = m_gpr.GetReg(DSP_REG_AR0 + sreg);
  MOVZX(32, 16, RAX, arx_reg);

  increment_addr_reg(RAX, RDX, tmp3, RCX);

  MOV(32, arx_reg, R(RAX));
  m_gpr.PutReg(DSP_REG_AR0 + sreg);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX3, RCX, RegisterExtension::Sign);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp3);

  MOV(32, ar3_reg, R(tmp3));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.PutXReg(tmp3);
}

// LDNM $ax0.d, $ax1.r, @$arS
// xxxx xxxx 11dr 11ss
void DSPEmitterIR::ldnm(const UDSPInstruction opc)
{
  u8 dreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;
  u8 sreg = opc & 0x3;

  pushExtValueFromMem((dreg << 1) + DSP_REG_AXL0, sreg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  // if (IsSameMemArea(g_dsp.r[sreg], g_dsp.r[DSP_REG_AR3])) {
  m_gpr.ReadReg(sreg, RCX, RegisterExtension::None);
  m_gpr.ReadReg(DSP_REG_AR3, tmp1, RegisterExtension::None);
  XOR(16, R(ECX), R(tmp1));
  m_gpr.PutXReg(tmp1);
  DSPJitIRRegCache c(m_gpr);
  TEST(16, R(ECX), Imm16(0xfc00));
  FixupBranch not_equal = J_CC(CC_NE, true);
  pushExtValueFromMem2((rreg << 1) + DSP_REG_AXL1, sreg);
  m_gpr.FlushRegs(c);
  FixupBranch after = J(true);
  SetJumpTarget(not_equal);  // else
  pushExtValueFromMem2((rreg << 1) + DSP_REG_AXL1, DSP_REG_AR3);
  m_gpr.FlushRegs(c);
  SetJumpTarget(after);

  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_WR0 + sreg, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0 + sreg, RCX, RegisterExtension::Sign);
  const OpArg arx_reg = m_gpr.GetReg(DSP_REG_AR0 + sreg);
  MOVZX(32, 16, RAX, arx_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp3);

  MOV(32, arx_reg, R(tmp3));
  m_gpr.PutReg(DSP_REG_AR0 + sreg);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX3, RCX, RegisterExtension::Sign);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp3);

  MOV(32, ar3_reg, R(tmp3));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.PutXReg(tmp3);
}

// LDAXNM $axR, @$arS
// xxxx xxxx 11sr 1111
void DSPEmitterIR::ldaxnm(const UDSPInstruction opc)
{
  u8 sreg = (opc >> 5) & 0x1;
  u8 rreg = (opc >> 4) & 0x1;

  pushExtValueFromMem(rreg + DSP_REG_AXH0, sreg);

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  // if (IsSameMemArea(g_dsp.r[sreg], g_dsp.r[DSP_REG_AR3])) {
  m_gpr.ReadReg(sreg, RCX, RegisterExtension::None);
  m_gpr.ReadReg(DSP_REG_AR3, tmp1, RegisterExtension::None);
  XOR(16, R(ECX), R(tmp1));
  m_gpr.PutXReg(tmp1);
  DSPJitIRRegCache c(m_gpr);
  TEST(16, R(ECX), Imm16(0xfc00));
  FixupBranch not_equal = J_CC(CC_NE, true);
  pushExtValueFromMem2(rreg + DSP_REG_AXL0, sreg);
  m_gpr.FlushRegs(c);
  FixupBranch after = J(true);  // else
  SetJumpTarget(not_equal);
  pushExtValueFromMem2(rreg + DSP_REG_AXL0, DSP_REG_AR3);
  m_gpr.FlushRegs(c);
  SetJumpTarget(after);

  X64Reg tmp3 = m_gpr.GetFreeXReg();

  m_gpr.ReadReg(DSP_REG_WR0 + sreg, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX0 + sreg, RCX, RegisterExtension::Sign);
  const OpArg arx_reg = m_gpr.GetReg(DSP_REG_AR0 + sreg);
  MOVZX(32, 16, RAX, arx_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp3);

  MOV(32, arx_reg, R(tmp3));
  m_gpr.PutReg(DSP_REG_AR0 + sreg);

  m_gpr.ReadReg(DSP_REG_WR3, RDX, RegisterExtension::Zero);
  m_gpr.ReadReg(DSP_REG_IX3, RCX, RegisterExtension::Sign);
  const OpArg ar3_reg = m_gpr.GetReg(DSP_REG_AR3);
  MOVZX(32, 16, RAX, ar3_reg);

  increase_addr_reg(RAX, RDX, RCX, tmp3);

  MOV(32, ar3_reg, R(tmp3));
  m_gpr.PutReg(DSP_REG_AR3);

  m_gpr.PutXReg(tmp3);
}

// Push value from address in g_dsp.r[sreg] into EBX and stores the
// destinationindex in storeIndex
void DSPEmitterIR::pushExtValueFromMem(u16 dreg, u16 sreg)
{
  //	u16 addr = g_dsp.r[addr];

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  dsp_op_read_reg(sreg, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  dmem_read(tmp1, RAX);

  MOVZX(32, 16, EBX, R(EAX));

  m_store_index = dreg;

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

void DSPEmitterIR::pushExtValueFromMem2(u16 dreg, u16 sreg)
{
  //	u16 addr = g_dsp.r[addr];

  X64Reg tmp1 = m_gpr.GetFreeXReg();
  X64Reg tmp2 = m_gpr.GetFreeXReg();
  X64Reg tmp3 = m_gpr.GetFreeXReg();

  dsp_op_read_reg(sreg, tmp1, RegisterExtension::Zero, tmp2, tmp3, RAX);
  dmem_read(tmp1, RAX);

  SHL(32, R(EAX), Imm8(16));
  OR(32, R(EBX), R(EAX));

  m_store_index2 = dreg;

  m_gpr.PutXReg(tmp3);
  m_gpr.PutXReg(tmp2);
  m_gpr.PutXReg(tmp1);
}

void DSPEmitterIR::popExtValueToReg()
{
  // in practice, we rarely ever have a non-NX main op
  // with an extended op, so the OR here is either
  // not run (storeIndex == -1) or ends up OR'ing
  // EBX with 0 (becoming the MOV we have here)
  // nakee wants to keep it clean, so lets do that.
  // [nakeee] the or case never happens in real
  // [nakeee] it's just how the hardware works so we added it
  if (m_store_index != -1)
  {
    X64Reg tmp1 = m_gpr.GetFreeXReg();
    X64Reg tmp2 = m_gpr.GetFreeXReg();
    dsp_op_write_reg(m_store_index, RBX, tmp1, tmp2, RAX);
    m_gpr.PutXReg(tmp2);
    m_gpr.PutXReg(tmp1);
    if (m_store_index >= DSP_REG_ACM0 && m_store_index2 == -1)
    {
      TEST(32, R(EBX), Imm32(SR_40_MODE_BIT << 16));
      FixupBranch not_40bit = J_CC(CC_Z, true);
      DSPJitIRRegCache c(m_gpr);
      // if (g_dsp.r[DSP_REG_SR] & SR_40_MODE_BIT)
      //{
      // Sign extend into whole accum.
      // u16 val = g_dsp.r[reg];
      MOVSX(32, 16, EAX, R(EBX));
      SHR(32, R(EAX), Imm8(16));
      // g_dsp.r[reg - DSP_REG_ACM0 + DSP_REG_ACH0] = (val & 0x8000) ? 0xFFFF : 0x0000;
      // g_dsp.r[reg - DSP_REG_ACM0 + DSP_REG_ACL0] = 0;
      m_gpr.WriteReg(m_store_index - DSP_REG_ACM0 + DSP_REG_ACH0, R(RAX));
      m_gpr.WriteReg(m_store_index - DSP_REG_ACM0 + DSP_REG_ACL0, Imm16(0));
      //}
      m_gpr.FlushRegs(c);
      SetJumpTarget(not_40bit);
    }
  }

  m_store_index = -1;

  if (m_store_index2 != -1)
  {
    SHR(32, R(EBX), Imm8(16));
    X64Reg tmp1 = m_gpr.GetFreeXReg();
    X64Reg tmp2 = m_gpr.GetFreeXReg();
    dsp_op_write_reg(m_store_index2, RBX, tmp1, tmp2, RAX);
    m_gpr.PutXReg(tmp2);
    m_gpr.PutXReg(tmp1);
  }

  m_store_index2 = -1;
}

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
