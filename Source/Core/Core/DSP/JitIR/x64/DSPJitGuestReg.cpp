// Copyright (C) 2012 Dolphin Project.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 2.0.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License 2.0 for more details.

// A copy of the GPL 2.0 should have been included with the program.
// If not, see http://www.gnu.org/licenses/

// Official GIT repository and contact information can be found at
// http://code.google.com/p/dolphin-emu/

#include "Common/x64Emitter.h"
#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP
{
namespace JITIR
{
namespace x64
{
OpArg DSPEmitterIR::regMem(int reg)
{
  switch (reg)
  {
  case DSP_REG_AR0:
  case DSP_REG_AR1:
  case DSP_REG_AR2:
  case DSP_REG_AR3:
    return MDisp(
        R15, static_cast<int>(offsetof(SDSP, r.ar) + sizeof(SDSP::r.ar[0]) * (reg - DSP_REG_AR0)));
  case DSP_REG_IX0:
  case DSP_REG_IX1:
  case DSP_REG_IX2:
  case DSP_REG_IX3:
    return MDisp(
        R15, static_cast<int>(offsetof(SDSP, r.ix) + sizeof(SDSP::r.ix[0]) * (reg - DSP_REG_IX0)));
  case DSP_REG_WR0:
  case DSP_REG_WR1:
  case DSP_REG_WR2:
  case DSP_REG_WR3:
    return MDisp(
        R15, static_cast<int>(offsetof(SDSP, r.wr) + sizeof(SDSP::r.wr[0]) * (reg - DSP_REG_WR0)));
  case DSP_REG_ST0:
  case DSP_REG_ST1:
  case DSP_REG_ST2:
  case DSP_REG_ST3:
    return MDisp(
        R15, static_cast<int>(offsetof(SDSP, r.st) + sizeof(SDSP::r.st[0]) * (reg - DSP_REG_ST0)));
  case DSP_REG_ACH0:
  case DSP_REG_ACH1:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.ac[0].h) +
                                       sizeof(SDSP::r.ac[0]) * (reg - DSP_REG_ACH0)));
  case DSP_REG_CR:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.cr)));
  case DSP_REG_SR:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.sr)));
  case DSP_REG_PRODL:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.prod.l)));
  case DSP_REG_PRODM:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.prod.m)));
  case DSP_REG_PRODH:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.prod.h)));
  case DSP_REG_PRODM2:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.prod.m2)));
  case DSP_REG_AXL0:
  case DSP_REG_AXL1:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.ax[0].l) +
                                       sizeof(SDSP::r.ax[0]) * (reg - DSP_REG_AXL0)));
  case DSP_REG_AXH0:
  case DSP_REG_AXH1:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.ax[0].h) +
                                       sizeof(SDSP::r.ax[0]) * (reg - DSP_REG_AXH0)));
  case DSP_REG_ACL0:
  case DSP_REG_ACL1:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.ac[0].l) +
                                       sizeof(SDSP::r.ac[0]) * (reg - DSP_REG_ACL0)));
  case DSP_REG_ACM0:
  case DSP_REG_ACM1:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.ac[0].m) +
                                       sizeof(SDSP::r.ac[0]) * (reg - DSP_REG_ACM0)));
  case IROp::DSP_REG_AX0_ALL:
  case IROp::DSP_REG_AX1_ALL:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.ax[0].val) +
                                       sizeof(SDSP::r.ax[0]) * (reg - IROp::DSP_REG_AX0_ALL)));
  case IROp::DSP_REG_ACC0_ALL:
  case IROp::DSP_REG_ACC1_ALL:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.ac[0].val) +
                                       sizeof(SDSP::r.ac[0]) * (reg - IROp::DSP_REG_ACC0_ALL)));
  case IROp::DSP_REG_PROD_ALL:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.prod.val)));
  default:
    ASSERT_MSG(DSPLLE, 0, "cannot happen");
    return M(static_cast<void*>(nullptr));
  }
}

size_t DSPEmitterIR::regSize(int reg)
{
  switch (reg)
  {
  case DSP_REG_AR0:
  case DSP_REG_AR1:
  case DSP_REG_AR2:
  case DSP_REG_AR3:
  case DSP_REG_IX0:
  case DSP_REG_IX1:
  case DSP_REG_IX2:
  case DSP_REG_IX3:
  case DSP_REG_WR0:
  case DSP_REG_WR1:
  case DSP_REG_WR2:
  case DSP_REG_WR3:
  case DSP_REG_ST0:
  case DSP_REG_ST1:
  case DSP_REG_ST2:
  case DSP_REG_ST3:
  case DSP_REG_ACH0:
  case DSP_REG_ACH1:
  case DSP_REG_CR:
  case DSP_REG_SR:
  case DSP_REG_PRODL:
  case DSP_REG_PRODM:
  case DSP_REG_PRODH:
  case DSP_REG_PRODM2:
  case DSP_REG_AXL0:
  case DSP_REG_AXL1:
  case DSP_REG_AXH0:
  case DSP_REG_AXH1:
  case DSP_REG_ACL0:
  case DSP_REG_ACL1:
  case DSP_REG_ACM0:
  case DSP_REG_ACM1:
    return 2;
  case IROp::DSP_REG_AX0_ALL:
  case IROp::DSP_REG_AX1_ALL:
    return 4;
  case IROp::DSP_REG_ACC0_ALL:
  case IROp::DSP_REG_ACC1_ALL:
  case IROp::DSP_REG_PROD_ALL:
    return 8;
  default:
    _assert_msg_(DSPLLE, 0, "cannot happen");
    return 0;
  }
}

void DSPEmitterIR::iremit_LoadImmOp(IRInsn const& insn)
{
  // nothing to do here. the checkImmVRegs already put something
  // sensible here.
  if (insn.output.oparg.IsImm())
    return;

  u64 imm = insn.inputs[0].imm;
  if ((u32)imm == (u64)imm)
    // implicit zero extend with operations in 32bit mode
    MOV(32, insn.output.oparg, Imm32(imm));
  else if ((s32)imm == (s64)imm)
    // implicit sign extend with mov from 32bit imm in 64bit mode
    MOV(64, insn.output.oparg, Imm32(imm));
  else
    MOV(64, insn.output.oparg, Imm64(imm));
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LoadImmOp = {
    "LoadImmOp", &DSPEmitterIR::iremit_LoadImmOp, 0, 0, 0, 0, false, {}, {OpAnyReg | OpImmAny}};

void DSPEmitterIR::iremit_LoadGuestStackOp(IRInsn const& insn)
{
  X64Reg hreg = insn.output.oparg.GetSimpleReg();

  int reqs = m_vregs[insn.output.vreg].reqs;
  int greg = ir_to_regcache_reg(insn.inputs[0].guest_reg);

  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();

  dsp_reg_load_stack((StackRegister)(greg - DSP_REG_ST0), hreg, tmp1, tmp2, tmp3);
  switch (reqs & ExtendMask)
  {
  case ExtendNone:
    break;
  case ExtendSign16:
    MOVSX(64, 16, hreg, R(hreg));
    break;
  case ExtendZero16:
    MOVZX(64, 16, hreg, R(hreg));
    break;
  default:
    _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
    break;
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LoadGuestStackOp = {
    "LoadGuestStackOp", &DSPEmitterIR::iremit_LoadGuestStackOp, 0, 0, 0, 0, false, {},
    {OpAny64},          {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_LoadGuestSROp(IRInsn const& insn)
{
  X64Reg hreg = insn.output.oparg.GetSimpleReg();
  // for the moment, this is hardcoded here... the register fetcher
  // to be implemented will fix this again.
  MOV(16, R(hreg), insn.SR);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LoadGuestSROp = {
    "LoadGuestSROp", &DSPEmitterIR::iremit_LoadGuestSROp, 0, 0, 0, 0, false, {}, {OpAny64}, {}};

void DSPEmitterIR::iremit_LoadGuestACMOp(IRInsn const& insn)
{
  X64Reg hreg = insn.output.oparg.GetSimpleReg();
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  int reqs = m_vregs[insn.output.vreg].reqs;
  int greg = ir_to_regcache_reg(insn.inputs[0].guest_reg);

  OpArg mem = regMem(greg);

  if ((insn.constant_mask_SR & SR_40_MODE_BIT) && (insn.constant_val_SR & SR_40_MODE_BIT))
  {
    MOV(64, R(tmp1), mem);

    MOVSX(64, 32, hreg, R(tmp1));
    CMP(64, R(hreg), R(tmp1));
    FixupBranch no_saturate = J_CC(CC_Z);

    TEST(64, R(tmp1), R(tmp1));
    FixupBranch negative = J_CC(CC_LE);

    MOV(64, R(hreg), Imm32(0x7fff));  // this works for all extend modes
    FixupBranch done_positive = J();

    SetJumpTarget(negative);
    switch (reqs & ExtendMask)
    {
    case ExtendSign16:
      MOV(64, R(hreg), Imm32(0xffff8000));
      break;
    case ExtendZero16:
    case ExtendNone:
      MOV(32, R(hreg), Imm32(0x00008000));
      break;
    default:
      _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
      break;
    }
    FixupBranch done_negative = J();

    SetJumpTarget(no_saturate);

    // assume the cpu is smart enough to still have this in L1
    // and this being cheaper than SAR/SHR needed to convert tmp1
    mem.AddMemOffset(2);
    switch (reqs & ExtendMask)
    {
    case ExtendSign16:
      MOVSX(64, 16, hreg, mem);
      break;
    case ExtendZero16:
      MOVZX(64, 16, hreg, mem);
      break;
    case ExtendNone:
      MOV(16, R(hreg), mem);
      break;
    default:
      _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
      break;
    }

    SetJumpTarget(done_positive);
    SetJumpTarget(done_negative);
  }
  else if ((insn.constant_mask_SR & SR_40_MODE_BIT) && !(insn.constant_val_SR & SR_40_MODE_BIT))
  {
    // assume the cpu is smart enough to still have this in L1
    // and this being cheaper than SAR/SHR needed to convert tmp1
    mem.AddMemOffset(2);
    switch (reqs & ExtendMask)
    {
    case ExtendSign16:
      MOVSX(64, 16, hreg, mem);
      break;
    case ExtendZero16:
      MOVZX(64, 16, hreg, mem);
      break;
    case ExtendNone:
      MOV(16, R(hreg), mem);
      break;
    default:
      _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
      break;
    }
  }
  else
  {
    TEST(16, insn.SR, Imm16(SR_40_MODE_BIT));
    FixupBranch not_40bit = J_CC(CC_Z, true);

    MOV(64, R(tmp1), mem);

    MOVSX(64, 32, hreg, R(tmp1));
    CMP(64, R(hreg), R(tmp1));
    FixupBranch no_saturate = J_CC(CC_Z);

    TEST(64, R(tmp1), R(tmp1));
    FixupBranch negative = J_CC(CC_LE);

    MOV(64, R(hreg), Imm32(0x7fff));  // this works for all extend modes
    FixupBranch done_positive = J();

    SetJumpTarget(negative);
    switch (reqs & ExtendMask)
    {
    case ExtendSign16:
      MOV(64, R(hreg), Imm32(0xffff8000));
      break;
    case ExtendZero16:
    case ExtendNone:
      MOV(32, R(hreg), Imm32(0x00008000));
      break;
    default:
      _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
      break;
    }
    FixupBranch done_negative = J();

    SetJumpTarget(no_saturate);
    SetJumpTarget(not_40bit);

    // assume the cpu is smart enough to still have this in L1
    // and this being cheaper than SAR/SHR needed to convert tmp1
    mem.AddMemOffset(2);
    switch (reqs & ExtendMask)
    {
    case ExtendSign16:
      MOVSX(64, 16, hreg, mem);
      break;
    case ExtendZero16:
      MOVZX(64, 16, hreg, mem);
      break;
    case ExtendNone:
      MOV(16, R(hreg), mem);
      break;
    default:
      _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
      break;
    }

    SetJumpTarget(done_positive);
    SetJumpTarget(done_negative);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LoadGuestACMOp = {
    "LoadGuestACMOp",
    &DSPEmitterIR::iremit_LoadGuestACMOp,
    SR_40_MODE_BIT,
    0,
    0,
    0,
    false,
    {},
    {OpAny64},
    {{OpAny64}}};

void DSPEmitterIR::iremit_LoadGuestFastOp(IRInsn const& insn)
{
  X64Reg hreg = insn.output.oparg.GetSimpleReg();

  int reqs = m_vregs[insn.output.vreg].reqs;
  int greg = ir_to_regcache_reg(insn.inputs[0].guest_reg);

  const OpArg mem = regMem(greg);
  switch (regSize(greg))
  {
  case 2:
    switch (reqs & ExtendMask)
    {
    case ExtendSign16:
      MOVSX(64, 16, hreg, mem);
      break;
    case ExtendZero16:
      MOVZX(64, 16, hreg, mem);
      break;
    case ExtendNone:
      MOV(16, R(hreg), mem);
      break;
    default:
      _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
      break;
    }
    break;
  case 4:
    switch (reqs & ExtendMask)
    {
    case ExtendSign32:
      MOVSX(64, 32, hreg, mem);
      break;
    case ExtendZero32:  // implicit zero extend
    case ExtendNone:
      MOV(32, R(hreg), mem);
      break;
    default:
      _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
      break;
    }
    break;
  case 8:
    switch (reqs & ExtendMask)
    {
    case ExtendSign64:
    case ExtendZero64:
    case ExtendNone:
      MOV(64, R(hreg), mem);
      break;
    default:
      _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
      break;
    }
    break;
  default:
    _assert_msg_(DSPLLE, 0, "unsupported memory size");
    break;
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LoadGuestFastOp = {
    "LoadGuestFastOp", &DSPEmitterIR::iremit_LoadGuestFastOp, 0, 0, 0, 0, false, {}, {OpAny64}};

void DSPEmitterIR::iremit_LoadGuestProdOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  // s64 val   = (s8)(u8)g_dsp.r[DSP_REG_PRODH];
  const OpArg mem = regMem(IROp::DSP_REG_PROD_ALL);

  MOV(64, insn.output.oparg, mem);

  MOV(64, R(tmp1), insn.output.oparg);
  SHL(64, insn.output.oparg, Imm8(64 - 40));  // sign extend
  SAR(64, insn.output.oparg, Imm8(64 - 40));
  SHR(64, R(tmp1), Imm8(32));
  // imm gets sign extended, but upper 32bit are 0 already
  AND(64, R(tmp1), Imm32(0xffff0000));
  ADD(64, insn.output.oparg, R(tmp1));
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LoadGuestProdOp = {
    "LoadGuestProdOp", &DSPEmitterIR::iremit_LoadGuestProdOp, 0, 0, 0, 0, false, {}, {OpAny64},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_StoreGuestProdOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  OpArg mem = regMem(IROp::DSP_REG_PROD_ALL);

  if (insn.inputs[0].oparg.IsImm())
  {
    s64 imm = insn.inputs[0].oparg.AsImm64().Imm64() & 0x000000ffffffffffULL;

    if ((s32)imm == (s64)imm)
      MOV(64, mem, Imm32(imm));
    else
    {
      MOV(32, mem, Imm32(imm));
      mem.AddMemOffset(4);
      MOV(32, mem, Imm32(imm >> 32));
    }
  }
  else if (insn.inputs[0].oparg.IsSimpleReg())
  {
    MOV(64, R(tmp1), Imm64(0x000000ffffffffffULL));
    AND(64, R(tmp1), insn.inputs[0].oparg);

    MOV(64, mem, R(tmp1));
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::StoreGuestProdOp = {
    "StoreGuestProdOp",
    &DSPEmitterIR::iremit_StoreGuestProdOp,
    0,
    0,
    0,
    0,
    false,
    {{OpAny64}},  // not clobbered
    {},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_StoreGuestStackOp(IRInsn const& insn)
{
  int greg = ir_to_regcache_reg(insn.output.guest_reg);

  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();
  X64Reg tmp2 = insn.temps[1].oparg.GetSimpleReg();
  X64Reg tmp3 = insn.temps[2].oparg.GetSimpleReg();
  dsp_reg_store_stack((StackRegister)(greg - DSP_REG_ST0), insn.inputs[0].oparg, tmp1, tmp2, tmp3);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::StoreGuestStackOp = {
    "StoreGuestStackOp",
    &DSPEmitterIR::iremit_StoreGuestStackOp,
    0,
    0,
    0,
    0,
    false,
    {{OpAny64}},  // not clobbered
    {},
    {{OpAnyReg}, {OpAnyReg}, {OpAnyReg}}};

void DSPEmitterIR::iremit_StoreGuestSROp(IRInsn const& insn)
{
  MOV(16, insn.SR, insn.inputs[0].oparg);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::StoreGuestSROp = {
    "StoreGuestSROp",
    &DSPEmitterIR::iremit_StoreGuestSROp,
    0,
    0,
    0,
    0,
    false,
    {{OpAny64}},  // not clobbered
    {}};

void DSPEmitterIR::iremit_StoreGuestACMOp(IRInsn const& insn)
{
  int greg = ir_to_regcache_reg(insn.output.guest_reg);

  OpArg mem = regMem(greg);

  TEST(16, insn.SR, Imm16(SR_40_MODE_BIT));
  FixupBranch not_40bit = J_CC(CC_Z, true);

  if (insn.inputs[0].oparg.IsImm())
  {
    s16 val = insn.inputs[0].oparg.AsImm16().SImm16();
    MOV(64, mem, Imm32(((s32)val) << 16));
  }
  else if (insn.inputs[0].oparg.IsSimpleReg())
  {
    MOVSX(64, 16, insn.inputs[0].oparg.GetSimpleReg(), insn.inputs[0].oparg);
    SHL(64, insn.inputs[0].oparg, Imm8(16));
    MOV(64, mem, insn.inputs[0].oparg);
  }
  else
  {
    _assert_msg_(DSPLLE, 0, "StoreGuestACMOp only handles Imm and R");
  }

  FixupBranch is_40bit = J();
  SetJumpTarget(not_40bit);

  mem.AddMemOffset(2);
  MOV(16, mem, insn.inputs[0].oparg);

  SetJumpTarget(is_40bit);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::StoreGuestACMOp = {
    "StoreGuestACMOp",
    &DSPEmitterIR::iremit_StoreGuestACMOp,
    SR_40_MODE_BIT,
    0,
    0,
    0,
    false,
    {{OpAny64 | Clobbered}},
    {}};

void DSPEmitterIR::iremit_StoreGuestOp(IRInsn const& insn)
{
  int greg = ir_to_regcache_reg(insn.output.guest_reg);

  OpArg mem = regMem(greg);
  OpArg src = insn.inputs[0].oparg;

  if (src.IsImm())
  {
    size_t size = regSize(greg);
    if (greg == DSP_REG_ACH0 || greg == DSP_REG_ACH1)
    {
      src = Imm64((s64)(s16)src.AsImm64().Imm64());
      size = 4;
    }
    switch (size)
    {
    case 2:
      MOV(16, mem, src.AsImm16());
      break;
    case 4:
      MOV(32, mem, src.AsImm32());
      break;
    case 8:
    {
      u64 v = src.AsImm64().Imm64();
      if ((s32)v == (s64)v)
        MOV(64, mem, src.AsImm32());
      else
      {
        // two insn form
        MOV(32, mem, Imm32(v));
        mem.AddMemOffset(4);
        MOV(32, mem, Imm32(v >> 32));
      }
      break;
    }
    default:
      _assert_msg_(DSPLLE, 0, "unsupported memory size");
      break;
    }
  }
  else if (src.IsSimpleReg())
  {
    size_t size = regSize(greg);
    if (greg == DSP_REG_ACH0 || greg == DSP_REG_ACH1)
    {
      MOVSX(32, 8, src.GetSimpleReg(), src);
      size = 4;
    }
    switch (size)
    {
    case 2:
      MOV(16, mem, src);
      break;
    case 4:
      MOV(32, mem, src);
      break;
    case 8:
      MOV(64, mem, src);
      break;
    default:
      _assert_msg_(DSPLLE, 0, "unsupported memory size");
      break;
    }
  }
  else
    _assert_msg_(DSPLLE, 0, "must be imm or reg");
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::StoreGuestOp = {
    "StoreGuestOp",
    &DSPEmitterIR::iremit_StoreGuestOp,
    0,
    0,
    0,
    0,
    false,
    {{OpAny64 | Clobbered}},
    {}};

void DSPEmitterIR::addGuestLoadStore(IRNode* node, std::vector<IRNode*>& new_nodes)
{
  IRNode* n1 = node;
  IRInsnNode* in = dynamic_cast<IRInsnNode*>(n1);
  IRBranchNode* bn = dynamic_cast<IRBranchNode*>(n1);
  if (!in)
    return;
  IRInsn& insn = in->insn;
  std::list<IRInsn> nlist;
  // inject the loading ops
  for (unsigned int i = 0; i < NUM_INPUTS; i++)
  {
    if (!insn.emitter->inputs[i].reqs)
      continue;
    IRInsn p = {NULL, {insn.inputs[i]}, IROp::Vreg(insn.inputs[i].vreg)};
    if (insn.inputs[i].type == IROp::IMM)
    {
      p.emitter = &LoadImmOp;
      assignVRegs(p);
      nlist.push_back(p);
      continue;
    }
    if (insn.inputs[i].type != IROp::REG)
      continue;
    switch (insn.inputs[i].guest_reg)
    {
    case IROp::DSP_REG_PROD_ALL:
      p.emitter = &LoadGuestProdOp;
      break;
    case DSP_REG_ST0:
    case DSP_REG_ST1:
    case DSP_REG_ST2:
    case DSP_REG_ST3:
      p.emitter = &LoadGuestStackOp;
      break;
    case DSP_REG_SR:
      p.emitter = &LoadGuestSROp;
      p.needs_SR = 0xffff;
      break;
    case DSP_REG_ACM0:
    case DSP_REG_ACM1:
      if (!(insn.emitter->inputs[i].reqs & NoSaturate))
      {
        p.emitter = &LoadGuestACMOp;
        p.needs_SR |= SR_40_MODE_BIT;
        p.inputs[0].guest_reg = p.inputs[0].guest_reg - DSP_REG_ACM0 + IROp::DSP_REG_ACC0_ALL;
        break;
      }
    // fall through
    default:
      p.emitter = &LoadGuestFastOp;
      break;
    }
    assignVRegs(p);
    nlist.push_back(p);
    insn.inputs[i].type = IROp::VREG;
    insn.inputs[i].guest_reg = -1;
    insn.inputs[i].imm = -1;
  }

  // inserting in reverse order so the order
  // of the list based implementation is kept.
  for (auto nit = nlist.rbegin(); nit != nlist.rend(); nit++)
  {
    IRInsnNode* n2 = makeIRInsnNode(*nit);
    new_nodes.push_back(n2);
    in->insertBefore(n2);
  }

  nlist.clear();

  // inject the storing ops
  if (insn.emitter->output.reqs && insn.output.type == IROp::REG)
  {
    IRInsn p = {NULL, {IROp::Vreg(insn.output.vreg)}, insn.output};
    switch (insn.output.guest_reg)
    {
    case IROp::DSP_REG_PROD_ALL:
      p.emitter = &StoreGuestProdOp;
      break;
    case DSP_REG_SR:
      p.emitter = &StoreGuestSROp;
      p.modifies_SR = 0xffff;
      break;
    case DSP_REG_ST0:
    case DSP_REG_ST1:
    case DSP_REG_ST2:
    case DSP_REG_ST3:
      p.emitter = &StoreGuestStackOp;
      break;
    case DSP_REG_ACM0:
    case DSP_REG_ACM1:
      p.needs_SR |= SR_40_MODE_BIT;
      if (!(insn.emitter->output.reqs & NoACMExtend))
      {
        p.emitter = &StoreGuestACMOp;
        p.output.guest_reg = p.output.guest_reg - DSP_REG_ACM0 + IROp::DSP_REG_ACC0_ALL;
        break;
      }
    // fall through
    default:
      p.emitter = &StoreGuestOp;
      break;
    }
    assignVRegs(p);

    insn.output.type = IROp::VREG;
    insn.output.guest_reg = -1;

    nlist.push_back(p);
  }

  for (auto nit = nlist.rbegin(); nit != nlist.rend(); nit++)
  {
    IRInsnNode* n2 = makeIRInsnNode(*nit);
    new_nodes.push_back(n2);
    in->insertAfter(n2);
    if (bn)
    {
      IRInsnNode* n3 = new IRInsnNode();
      new_nodes.push_back(n3);
      n3->insn = *nit;
      bn->insertAfterOnBranch(n3);
    }
  }
}

void DSPEmitterIR::addGuestLoadStore(IRNode* node, IRBB* bb)
{
  std::vector<IRNode*> new_nodes;

  addGuestLoadStore(node, new_nodes);

  if (bb)
  {
    bb->nodes.insert(new_nodes.begin(), new_nodes.end());
    // check if node is the start_node or end_node of bb
    if (bb->start_node == node)
    {
      // find the node in new_nodes that has .prev.empty(),
      // make it new start_node
      for (auto n : new_nodes)
      {
        if (n->prev.empty())
        {
          bb->start_node = n;
          break;
        }
      }
    }
    if (bb->end_node == node)
    {
      // find the node in new_nodes that has .next.empty(),
      // make it new start_node
      for (auto n : new_nodes)
      {
        if (n->next.empty())
        {
          bb->end_node = n;
          break;
        }
      }
    }
  }
}

void DSPEmitterIR::addGuestLoadStore(IRBB* bb)
{
  std::vector<IRNode*> new_nodes;
  for (auto n : bb->nodes)
  {
    addGuestLoadStore(n, new_nodes);
  }
  bb->nodes.insert(new_nodes.begin(), new_nodes.end());

  // update start_node/end_node
  while (!bb->start_node->prev.empty())
    bb->start_node = *bb->start_node->prev.begin();
  while (!bb->end_node->next.empty())
    bb->end_node = *bb->end_node->next.begin();
}

}  // namespace x64
}  // namespace JITIR
}  // namespace DSP
