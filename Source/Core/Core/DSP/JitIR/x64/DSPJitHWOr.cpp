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
void DSPEmitterIR::iremit_GRegOrACCACCOp(IRInsn const& insn)
{
  OR(64, insn.inputs[0].oparg, insn.inputs[1].oparg);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::GRegOrACCACCOp = {
    "GRegOrACCACCOp",
    &DSPEmitterIR::iremit_GRegOrACCACCOp,
    0,
    0,
    0,
    0,
    false,
    {{OpAnyReg | SameAsOutput}, {OpAnyReg}},
    {OpAnyReg}};

void DSPEmitterIR::iremit_GRegOrACCACLOp(IRInsn const& insn)
{
  MOVZX(64, 16, insn.inputs[1].oparg.GetSimpleReg(), insn.inputs[1].oparg);
  OR(64, insn.inputs[0].oparg, insn.inputs[1].oparg);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::GRegOrACCACLOp = {
    "GRegOrACCACLOp",
    &DSPEmitterIR::iremit_GRegOrACCACLOp,
    0,
    0,
    0,
    0,
    false,
    {{OpAnyReg | SameAsOutput}, {OpAnyReg | Clobbered}},
    {OpAnyReg}};

void DSPEmitterIR::iremit_GRegOrACCACHOp(IRInsn const& insn)
{
  MOVSX(32, 8, insn.inputs[1].oparg.GetSimpleReg(), insn.inputs[1].oparg);
  SHL(64, insn.inputs[1].oparg, Imm8(32));
  OR(64, insn.inputs[0].oparg, insn.inputs[1].oparg);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::GRegOrACCACHOp = {
    "GRegOrACCACHOp",
    &DSPEmitterIR::iremit_GRegOrACCACHOp,
    0,
    0,
    0,
    0,
    false,
    {{OpAnyReg | SameAsOutput}, {OpAnyReg | Clobbered}},
    {OpAnyReg}};

void DSPEmitterIR::iremit_GRegOrACMACMOp(IRInsn const& insn)
{
  OR(16, insn.inputs[0].oparg, insn.inputs[1].oparg);
  // extending is handled by output stage
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::GRegOrACMACMOp = {
    "GRegOrACMACMOp",
    &DSPEmitterIR::iremit_GRegOrACMACMOp,
    0,
    0,
    0,
    0,
    false,
    {{OpAnyReg | SameAsOutput}, {OpAnyReg}},
    {OpAnyReg}};

void DSPEmitterIR::iremit_GRegOrACMACCOp(IRInsn const& insn)
{
  // conditionally extend, then do the OR
  int reqs = m_vregs[insn.inputs[0].vreg].reqs;

  if (!(reqs & NoACMExtend))
  {
    TEST(16, insn.SR, Imm16(SR_40_MODE_BIT));
    FixupBranch not_40bit = J_CC(CC_Z);
    MOVSX(64, 16, insn.inputs[0].oparg.GetSimpleReg(), insn.inputs[0].oparg);
    FixupBranch is_40bit = J();
    SetJumpTarget(not_40bit);
    MOVZX(64, 16, insn.inputs[0].oparg.GetSimpleReg(), insn.inputs[0].oparg);
    SetJumpTarget(is_40bit);
  }
  else
  {
    MOVZX(64, 16, insn.inputs[0].oparg.GetSimpleReg(), insn.inputs[0].oparg);
  }
  SHL(64, insn.inputs[0].oparg, Imm8(16));
  OR(64, insn.inputs[1].oparg, insn.inputs[0].oparg);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::GRegOrACMACCOp = {
    "GRegOrACMACCOp",
    &DSPEmitterIR::iremit_GRegOrACMACCOp,
    SR_40_MODE_BIT,
    0,
    0,
    0,
    false,
    {{OpAnyReg | Clobbered}, {OpAnyReg | SameAsOutput}},
    {OpAnyReg}};

void DSPEmitterIR::iremit_GRegOrACMACLOp(IRInsn const& insn)
{
  // conditionally extend, then do the OR
  int reqs = m_vregs[insn.inputs[0].vreg].reqs;

  MOVZX(64, 16, insn.inputs[1].oparg.GetSimpleReg(), insn.inputs[1].oparg);
  FixupBranch is_40bit;
  if (!(reqs & NoACMExtend))
  {
    TEST(16, insn.SR, Imm16(SR_40_MODE_BIT));
    FixupBranch not_40bit = J_CC(CC_Z);
    // this completely replaces the ACC
    MOVSX(64, 16, insn.output.oparg.GetSimpleReg(), insn.inputs[0].oparg);
    SHL(64, insn.output.oparg, Imm8(16));
    OR(64, insn.output.oparg, insn.inputs[1].oparg);
    is_40bit = J();
    SetJumpTarget(not_40bit);
  }
  // need to reintegrate ACH(high 32 bit)
  MOVZX(64, 16, insn.inputs[0].oparg.GetSimpleReg(), insn.inputs[0].oparg);
  SHL(64, insn.inputs[0].oparg, Imm8(16));
  OR(64, insn.inputs[0].oparg, insn.inputs[1].oparg);
  MOV(64, insn.inputs[1].oparg, Imm64(~0xffffffffULL));
  AND(64, insn.inputs[2].oparg, insn.inputs[1].oparg);
  OR(64, insn.inputs[2].oparg, insn.inputs[0].oparg);
  if (!(reqs & NoACMExtend))
  {
    SetJumpTarget(is_40bit);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::GRegOrACMACLOp = {
    "GRegOrACMACLOp",
    &DSPEmitterIR::iremit_GRegOrACMACLOp,
    SR_40_MODE_BIT,
    0,
    0,
    0,
    false,
    {{OpAnyReg | Clobbered}, {OpAnyReg | Clobbered}, {OpAnyReg | SameAsOutput}},
    {OpAnyReg}};

void DSPEmitterIR::iremit_GRegOrACMACHOp(IRInsn const& insn)
{
  // conditionally extend, then do the OR
  int reqs = m_vregs[insn.inputs[0].vreg].reqs;

  MOVSX(64, 8, insn.inputs[1].oparg.GetSimpleReg(), insn.inputs[1].oparg);
  SHL(64, insn.inputs[1].oparg, Imm8(32));
  FixupBranch is_40bit;
  if (!(reqs & NoACMExtend))
  {
    TEST(16, insn.SR, Imm16(SR_40_MODE_BIT));
    FixupBranch not_40bit = J_CC(CC_Z);
    // this completely replaces the ACC
    MOVSX(64, 16, insn.output.oparg.GetSimpleReg(), insn.inputs[0].oparg);
    SHL(64, insn.output.oparg, Imm8(16));
    OR(64, insn.output.oparg, insn.inputs[1].oparg);
    is_40bit = J();
    SetJumpTarget(not_40bit);
  }
  // need to reintegrate ACL(low 16 bit)
  MOVZX(64, 16, insn.inputs[0].oparg.GetSimpleReg(), insn.inputs[0].oparg);
  SHL(64, insn.inputs[0].oparg, Imm8(16));
  OR(64, insn.inputs[0].oparg, insn.inputs[1].oparg);
  AND(64, insn.inputs[2].oparg, Imm32(0xffff));
  OR(64, insn.inputs[2].oparg, insn.inputs[0].oparg);
  if (!(reqs & NoACMExtend))
  {
    SetJumpTarget(is_40bit);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::GRegOrACMACHOp = {
    "GRegOrACMACHOp",
    &DSPEmitterIR::iremit_GRegOrACMACHOp,
    SR_40_MODE_BIT,
    0,
    0,
    0,
    false,
    {{OpAnyReg | Clobbered}, {OpAnyReg | Clobbered}, {OpAnyReg | SameAsOutput}},
    {OpAnyReg}};

void DSPEmitterIR::iremit_GRegOrAXAXOp(IRInsn const& insn)
{
  OR(32, insn.inputs[0].oparg, insn.inputs[1].oparg);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::GRegOrAXAXOp = {
    "GRegOrAXAXOp", &DSPEmitterIR::iremit_GRegOrAXAXOp,      0,         0, 0, 0,
    false,          {{OpAnyReg | SameAsOutput}, {OpAnyReg}}, {OpAnyReg}};

void DSPEmitterIR::iremit_GRegOrAXAXLOp(IRInsn const& insn)
{
  MOVZX(32, 16, insn.inputs[1].oparg.GetSimpleReg(), insn.inputs[1].oparg);
  OR(32, insn.inputs[0].oparg, insn.inputs[1].oparg);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::GRegOrAXAXLOp = {
    "GRegOrAXAXLOp",
    &DSPEmitterIR::iremit_GRegOrAXAXLOp,
    0,
    0,
    0,
    0,
    false,
    {{OpAnyReg | SameAsOutput}, {OpAnyReg | Clobbered}},
    {OpAnyReg}};

void DSPEmitterIR::iremit_GRegOrAXAXHOp(IRInsn const& insn)
{
  SHL(32, insn.inputs[1].oparg, Imm8(16));
  OR(32, insn.inputs[0].oparg, insn.inputs[1].oparg);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::GRegOrAXAXHOp = {
    "GRegOrAXAXHOp",
    &DSPEmitterIR::iremit_GRegOrAXAXHOp,
    0,
    0,
    0,
    0,
    false,
    {{OpAnyReg | SameAsOutput}, {OpAnyReg | Clobbered}},
    {OpAnyReg}};

void DSPEmitterIR::iremit_GRegOr1616Op(IRInsn const& insn)
{
  OR(16, insn.inputs[0].oparg, insn.inputs[1].oparg);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::GRegOr1616Op = {
    "GRegOr1616Op", &DSPEmitterIR::iremit_GRegOr1616Op,      0,         0, 0, 0,
    false,          {{OpAnyReg | SameAsOutput}, {OpAnyReg}}, {OpAnyReg}};

void DSPEmitterIR::handleOverlappingOps(IRBB* bb)
{
  std::vector<IRNode*> new_nodes;

  // we execute very early, so the ilnode layout is:
  //     / insnnode \      .
  // node-->insnnode-->node .
  //     \ insnnode /      .
  for (auto n : bb->nodes)
  {
    if (n->next.size() <= 1)
      continue;
    // check if we need to to anything for the next
    // nodes. they are all supposed to be single
    // insn nodes at this point.

    // we need to check for two things:
    //* concurrent memory loads, the second one gets special treatment
    //  => special Load16Op, also taking the inputs[0] of the first
    //* concurrent guest register writes, all regs are ored
    //  => special OrOps, need to take into account ACMx vs ACCx etc.
    //     but there is only a small list of guests that _can_ conflict
    //* concurrent memory load/stores not handled, as i don't know if
    //  that does anything(but IMO it should, being accesses on the same
    //  memory bus)

    // concurrent loads
    IRInsnNode* firstLoad16Op = NULL;
    for (auto n2 : n->next)
    {
      IRInsnNode* in = dynamic_cast<IRInsnNode*>(n2);
      ASSERT_MSG(DSPLLE, in, "not an insn node");

      if (in->insn.emitter == &DSPEmitterIR::Load16Op)
      {
        if (!firstLoad16Op)
          firstLoad16Op = in;
        else
        {
          in->insn.emitter = &DSPEmitterIR::Load16ConcurrentOp;
          in->insn.inputs[1] = firstLoad16Op->insn.inputs[0];
        }
      }
    }

    // concurrent guest writes
    std::list<IRInsn*> oprefs;
    for (auto n2 : n->next)
    {
      IRInsnNode* in = dynamic_cast<IRInsnNode*>(n2);
      if (in->insn.output.type == IROp::REG)
        oprefs.push_back(&(in->insn));
    }

    std::list<IRInsn> gregOrOps;
    while (!oprefs.empty())
    {
      bool found = false;
      for (auto it1 = oprefs.begin(); it1 != oprefs.end(); it1++)
      {
        for (auto it2 = oprefs.begin(); it2 != oprefs.end(); it2++)
        {
          if (it1 == it2)
            continue;
          IRInsn p = {NULL, {(*it1)->output, (*it2)->output}, (*it1)->output};
          switch ((*it1)->output.guest_reg)
          {
          case DSP_REG_ACM0:
          case DSP_REG_ACM1:
            if ((*it2)->output.guest_reg == (*it1)->output.guest_reg)
              p.emitter = &DSPEmitterIR::GRegOrACMACMOp;
            else if ((*it2)->output.guest_reg ==
                     (*it1)->output.guest_reg - DSP_REG_ACM0 + IROp::DSP_REG_ACC0_ALL)
            {
              p.emitter = &DSPEmitterIR::GRegOrACMACCOp;
              p.output = p.inputs[1];
            }
            else if ((*it2)->output.guest_reg ==
                     (*it1)->output.guest_reg - DSP_REG_ACM0 + DSP_REG_ACL0)
            {
              p.emitter = &DSPEmitterIR::GRegOrACMACLOp;
              p.output.guest_reg = (*it1)->output.guest_reg - DSP_REG_ACM0 + IROp::DSP_REG_ACC0_ALL;
              p.inputs[2] = p.output;
            }
            else if ((*it2)->output.guest_reg ==
                     (*it1)->output.guest_reg - DSP_REG_ACM0 + DSP_REG_ACH0)
            {
              p.emitter = &DSPEmitterIR::GRegOrACMACHOp;
              p.output.guest_reg = (*it1)->output.guest_reg - DSP_REG_ACM0 + IROp::DSP_REG_ACC0_ALL;
              p.inputs[2] = p.output;
            }
            break;
          case IROp::DSP_REG_ACC0_ALL:
          case IROp::DSP_REG_ACC1_ALL:
            if ((*it2)->output.guest_reg == (*it1)->output.guest_reg)
              p.emitter = &DSPEmitterIR::GRegOrACCACCOp;
            else if ((*it2)->output.guest_reg ==
                     (*it1)->output.guest_reg - IROp::DSP_REG_ACC0_ALL + DSP_REG_ACL0)
              p.emitter = &DSPEmitterIR::GRegOrACCACLOp;
            else if ((*it2)->output.guest_reg ==
                     (*it1)->output.guest_reg - IROp::DSP_REG_ACC0_ALL + DSP_REG_ACH0)
              p.emitter = &DSPEmitterIR::GRegOrACCACHOp;
            break;
          case IROp::DSP_REG_AX0_ALL:
          case IROp::DSP_REG_AX1_ALL:
            if ((*it2)->output.guest_reg == (*it1)->output.guest_reg)
              p.emitter = &DSPEmitterIR::GRegOrAXAXOp;
            else if ((*it2)->output.guest_reg ==
                     (*it1)->output.guest_reg - IROp::DSP_REG_AX0_ALL + DSP_REG_AXL0)
              p.emitter = &DSPEmitterIR::GRegOrAXAXLOp;
            else if ((*it2)->output.guest_reg ==
                     (*it1)->output.guest_reg - IROp::DSP_REG_AX0_ALL + DSP_REG_AXH0)
              p.emitter = &DSPEmitterIR::GRegOrAXAXHOp;
            break;
          // no need to have this for prod. ext ops cannot write it.
          default:
            // these are 16bit vs 16bit
            if ((*it1)->output.guest_reg == (*it2)->output.guest_reg)
              p.emitter = &DSPEmitterIR::GRegOr1616Op;
            break;
          }
          if (p.emitter)
          {
            // complete the IRInsn, fix the inputs/
            // outputs
            p.output.vreg = -1;
            p.inputs[0].type = IROp::VREG;
            p.inputs[0].guest_reg = -1;
            p.inputs[1].type = IROp::VREG;
            p.inputs[1].guest_reg = -1;
            (*it1)->output.type = IROp::VREG;
            (*it1)->output.guest_reg = -1;
            (*it2)->output.guest_reg = IROp::VREG;
            assignVRegs(p);

            // drop the inputs from oprefs
            oprefs.erase(it1);
            oprefs.erase(it2);
            // and add the output func
            gregOrOps.push_back(p);
            oprefs.push_back(&gregOrOps.back());
            found = true;
            break;
          }
        }
        if (found)
          break;  // found something, restart
      }
      if (!found)
        break;  // there was nothing to do
    }

    // introduce the new nodes sequentially before the
    // joining node
    IRNode* nn = *(*n->next.begin())->next.begin();

    for (auto on : gregOrOps)
    {
      IRInsnNode* n2 = makeIRInsnNode(on);
      nn->insertBefore(n2);
      new_nodes.push_back(n2);
    }
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
