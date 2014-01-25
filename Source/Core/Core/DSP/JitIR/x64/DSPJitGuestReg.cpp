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
void DSPEmitterIR::iremit_LoadImmOp(IRInsn const& insn)
{
  // nothing to do here. the register allocator already put something
  // sensible here.
  if (insn.output.oparg.IsImm())
    return;

  // this can be optimised a bit, by using the implicit sign extension
  // from 32 to 64 bits
  MOV(64, insn.output.oparg, Imm64(insn.inputs[0].imm));
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

  int reqs = m_vregs[insn.output.vreg].reqs;
  int greg = ir_to_regcache_reg(insn.inputs[0].guest_reg);

  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  RegisterExtension extend;

  switch (reqs & ExtendMask)
  {
  case ExtendSign16:
    extend = RegisterExtension::Sign;
    break;
  case ExtendZero16:
    extend = RegisterExtension::Zero;
    break;
  case ExtendSign32:
    extend = RegisterExtension::Sign;
    break;
  case ExtendZero32:
    extend = RegisterExtension::Zero;
    break;
  case ExtendSign64:
    extend = RegisterExtension::Sign;
    break;
  case ExtendZero64:
    extend = RegisterExtension::Zero;
    break;
  case ExtendNone:
    extend = RegisterExtension::None;
    break;
  default:
    _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
    extend = RegisterExtension::None;
    break;
  }

  dsp_op_read_acm_reg(greg - DSP_REG_ACC0_64 + DSP_REG_ACM0, hreg, extend, insn.SR, tmp1);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LoadGuestACMOp = {
    "LoadGuestACMOp", &DSPEmitterIR::iremit_LoadGuestACMOp, 0, 0, 0, 0, false, {}, {OpAny64},
    {{OpAnyReg}}};

void DSPEmitterIR::iremit_LoadGuestFastOp(IRInsn const& insn)
{
  X64Reg hreg = insn.output.oparg.GetSimpleReg();

  int reqs = m_vregs[insn.output.vreg].reqs;
  int greg = ir_to_regcache_reg(insn.inputs[0].guest_reg);

  RegisterExtension extend;
  switch (reqs & ExtendMask)
  {
  case ExtendSign16:
    extend = RegisterExtension::Sign;
    break;
  case ExtendZero16:
    extend = RegisterExtension::Zero;
    break;
  case ExtendSign32:
    extend = RegisterExtension::Sign;
    break;
  case ExtendZero32:
    extend = RegisterExtension::Zero;
    break;
  case ExtendSign64:
    extend = RegisterExtension::Sign;
    break;
  case ExtendZero64:
    extend = RegisterExtension::Zero;
    break;
  case ExtendNone:
    extend = RegisterExtension::None;
    break;
  default:
    _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
    return;
  }
  m_gpr.ReadReg(greg, hreg, extend);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LoadGuestFastOp = {
    "LoadGuestFastOp", &DSPEmitterIR::iremit_LoadGuestFastOp, 0, 0, 0, 0, false, {}, {OpAny64}};

void DSPEmitterIR::iremit_LoadGuestProdOp(IRInsn const& insn)
{
  X64Reg tmp1 = insn.temps[0].oparg.GetSimpleReg();

  // s64 val   = (s8)(u8)g_dsp.r[DSP_REG_PRODH];
  const OpArg prod_reg = m_gpr.GetReg(DSP_REG_PROD_64);
  MOV(64, insn.output.oparg, prod_reg);
  m_gpr.PutReg(DSP_REG_PROD_64, false);

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

  const OpArg prod_reg = m_gpr.GetReg(DSP_REG_PROD_64, false);

  if (insn.inputs[0].oparg.IsImm())
  {
    s64 imm = insn.inputs[0].oparg.AsImm64().Imm64() & 0x000000ffffffffffULL;

    MOV(64, prod_reg, Imm64(imm));
  }
  else if (insn.inputs[0].oparg.IsSimpleReg())
  {
    MOV(64, R(tmp1), Imm64(0x000000ffffffffffULL));
    AND(64, R(tmp1), insn.inputs[0].oparg);

    MOV(64, prod_reg, R(tmp1));
  }
  m_gpr.PutReg(DSP_REG_PROD_64, true);
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

  DSPJitIRRegCache c(m_gpr);
  TEST(16, insn.SR, Imm16(SR_40_MODE_BIT));
  FixupBranch not_40bit = J_CC(CC_Z, true);

  if (insn.inputs[0].oparg.IsImm())
  {
    s16 val = insn.inputs[0].oparg.AsImm16().SImm16();
    // using automatic 32=>64 bit sign extension by cpu
    m_gpr.WriteReg(greg, Imm32(((s32)val) << 16));
  }
  else if (insn.inputs[0].oparg.IsSimpleReg())
  {
    MOVSX(64, 16, insn.inputs[0].oparg.GetSimpleReg(), insn.inputs[0].oparg);
    SHL(64, insn.inputs[0].oparg, Imm8(16));
    m_gpr.WriteReg(greg, insn.inputs[0].oparg);
  }
  else
  {
    _assert_msg_(DSPLLE, 0, "StoreGuestACMOp only handles Imm and R");
  }

  m_gpr.FlushRegs(c);
  FixupBranch is_40bit = J();
  SetJumpTarget(not_40bit);

  m_gpr.WriteReg(greg - DSP_REG_ACC0_64 + DSP_REG_ACM0, insn.inputs[0].oparg);

  m_gpr.FlushRegs(c);
  SetJumpTarget(is_40bit);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::StoreGuestACMOp = {
    "StoreGuestACMOp",
    &DSPEmitterIR::iremit_StoreGuestACMOp,
    0,
    0,
    0,
    0,
    false,
    {{OpAny64 | Clobbered}},
    {}};

void DSPEmitterIR::iremit_StoreGuestOp(IRInsn const& insn)
{
  int greg = ir_to_regcache_reg(insn.output.guest_reg);

  m_gpr.WriteReg(greg, insn.inputs[0].oparg);
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
