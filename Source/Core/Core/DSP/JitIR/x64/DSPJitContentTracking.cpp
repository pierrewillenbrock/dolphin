// Copyright (C) 2013 Dolphin Project.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 2.0.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License 2.0 for more details.

// A copy of the GPL 2.0 should have been included with the program.
// If not, see http://www.gnu.org/licenses/

// Official SVN repository and contact information can be found at
// http://code.google.com/p/dolphin-emu/

#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP
{
namespace JITIR
{
namespace x64
{
void DSPEmitterIR::analyseSRNeed(IRBB* bb)
{
  // initialize later_needs_SR to 0
  std::unordered_set<IRNode*> todo;
  for (auto n : bb->nodes)
  {
    todo.insert(n);
  }
  // iterate over all nodes until it doesn't change anymore
  while (!todo.empty())
  {
    IRNode* n = *todo.begin();
    todo.erase(todo.begin());
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (n == bb->end_node)
      continue;
    u16 later_needs_SR = 0;
    for (auto n2 : n->next)
    {
      u16 needs_SR = n2->later_needs_SR;
      IRInsnNode* in2 = dynamic_cast<IRInsnNode*>(n2);
      if (in2)
      {
        needs_SR &= ~in2->insn.modifies_SR;
        needs_SR |= in2->insn.needs_SR;
      }

      later_needs_SR |= needs_SR;
    }
    if (n->later_needs_SR != later_needs_SR)
    {
      n->later_needs_SR = later_needs_SR;
      if (in)
        in->insn.later_needs_SR = later_needs_SR;
      for (auto n2 : n->prev)
        todo.insert(n2);
    }
  }
}

void DSPEmitterIR::analyseSRNeed()
{
  // initialize later_needs_SR to 0
  std::unordered_set<IRBB*> todo;
  for (auto bb : m_bb_storage)
  {
    todo.insert(bb);
    for (auto n2 : bb->nodes)
    {
      IRInsnNode* in = dynamic_cast<IRInsnNode*>(n2);
      n2->later_needs_SR = 0;
      if (!in)
        continue;
      in->insn.later_needs_SR = 0;
    }
    analyseSRNeed(bb);
  }
  // iterate over all bbs until it doesn't change anymore
  while (!todo.empty())
  {
    IRBB* bb = *todo.begin();
    todo.erase(todo.begin());
    u16 later_needs_SR = 0;
    for (auto n2 : bb->next)
    {
      u16 needs_SR = n2->start_node->later_needs_SR;
      IRInsnNode* in2 = dynamic_cast<IRInsnNode*>(n2->start_node);
      if (in2)
      {
        needs_SR &= ~in2->insn.modifies_SR;
        needs_SR |= in2->insn.needs_SR;
      }

      later_needs_SR |= needs_SR;
    }
    if (bb->end_node->later_needs_SR != later_needs_SR)
    {
      bb->end_node->later_needs_SR = later_needs_SR;
      IRInsnNode* in = dynamic_cast<IRInsnNode*>(bb->end_node);
      if (in)
        in->insn.later_needs_SR = later_needs_SR;
      analyseSRNeed(bb);
      for (auto n2 : bb->prev)
        todo.insert(n2);
    }
  }
}

void DSPEmitterIR::analyseKnownSR(IRNode* node, u16& const_SR, u16& modified_SR, u16& value_SR)
{
  IRInsnNode* in = dynamic_cast<IRInsnNode*>(node);

  u16 c_SR = node->const_SR;
  u16 v_SR = node->value_SR;
  u16 m_SR = node->modified_SR;

  if (in)
  {
    c_SR &= ~in->insn.modifies_SR;
    c_SR |= in->insn.constant_mask_SR;
    v_SR &= ~in->insn.constant_mask_SR;
    v_SR |= in->insn.constant_val_SR;
  }
  // and for sanity reset all the bits we don't know about
  v_SR &= c_SR;

  // merge with the others
  modified_SR |= m_SR | ((value_SR & c_SR) ^ (v_SR & const_SR));

  const_SR |= c_SR;
  value_SR |= v_SR;
}

void DSPEmitterIR::analyseKnownSR(IRBB* bb)
{
  // iterate over all nodes until it doesn't change anymore
  std::unordered_set<IRNode*> todo;
  for (auto n : bb->nodes)
    todo.insert(n);
  while (!todo.empty())
  {
    IRNode* n = *todo.begin();
    todo.erase(todo.begin());
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    IRBranchNode* bn = dynamic_cast<IRBranchNode*>(n);
    u16 const_SR = n->const_SR;
    u16 value_SR = n->value_SR;
    u16 modified_SR = n->modified_SR;
    for (auto n2 : n->prev)
    {
      analyseKnownSR(n2, const_SR, modified_SR, value_SR);
    }
    const_SR &= ~modified_SR;
    value_SR &= ~modified_SR;

    if (const_SR != n->const_SR || value_SR != n->value_SR || modified_SR != n->modified_SR)
    {
      n->const_SR = const_SR;
      n->value_SR = value_SR;
      n->modified_SR = modified_SR;

      if (in)
      {
        in->insn.const_SR = const_SR;
        in->insn.value_SR = value_SR;
        in->insn.modified_SR = modified_SR;
      }

      std::unordered_set<IRInsnNode*> nexts;
      for (auto n2 : n->next)
        todo.insert(n2);

      if (bn)
      {
        for (auto n2 : bn->branch)
          todo.insert(n2);
      }
    }
  }
}

void DSPEmitterIR::analyseKnownSR()
{
  // iterate over all nodes until it doesn't change anymore
  std::unordered_set<IRBB*> todo;
  for (auto bb : m_bb_storage)
  {
    todo.insert(bb);
    analyseKnownSR(bb);
  }
  while (!todo.empty())
  {
    IRBB* bb = *todo.begin();
    todo.erase(todo.begin());
    u16 const_SR = bb->start_node->const_SR;
    u16 value_SR = bb->start_node->value_SR;
    u16 modified_SR = bb->start_node->modified_SR;
    for (auto n : bb->prev)
    {
      analyseKnownSR(n->end_node, const_SR, modified_SR, value_SR);
    }
    const_SR &= ~modified_SR;
    value_SR &= ~modified_SR;

    if (const_SR != bb->start_node->const_SR || value_SR != bb->start_node->value_SR ||
        modified_SR != bb->start_node->modified_SR)
    {
      bb->start_node->const_SR = const_SR;
      bb->start_node->value_SR = value_SR;
      bb->start_node->modified_SR = modified_SR;

      IRInsnNode* in = dynamic_cast<IRInsnNode*>(bb->start_node);
      if (in)
      {
        in->insn.const_SR = const_SR;
        in->insn.value_SR = value_SR;
        in->insn.modified_SR = modified_SR;
      }

      analyseKnownSR(bb);

      for (auto n : bb->next)
        todo.insert(n);
    }
  }
}

void DSPEmitterIR::analyseKnownRegs(IRNode* node, bool (&const_regs)[32], bool (&modified_regs)[32],
                                    u16 (&value_regs)[32], std::unordered_set<int>& const_vregs,
                                    std::unordered_set<int>& modified_vregs,
                                    std::unordered_map<int, u64>& value_vregs)
{
  IRInsnNode* in = dynamic_cast<IRInsnNode*>(node);

  if (in)
  {
    bool c_regs[32];
    bool m_regs[32];
    u16 v_regs[32];
    std::unordered_set<int> cnt_vregs = node->const_vregs;
    std::unordered_set<int> mod_vregs = node->modified_vregs;
    std::unordered_map<int, u64> var_vregs = node->value_vregs;

    memcpy(c_regs, node->const_regs, sizeof(c_regs));
    memcpy(m_regs, node->modified_regs, sizeof(m_regs));
    memcpy(v_regs, node->value_regs, sizeof(v_regs));

    if (in->insn.emitter == &DSPEmitterIR::LoadImmOp)
    {
      cnt_vregs.insert(in->insn.output.vreg);
      var_vregs[in->insn.output.vreg] = m_vregs[in->insn.output.vreg].imm;
    }
    else if (in->insn.output.type == IROp::VREG)
      cnt_vregs.erase(in->insn.output.vreg);

    if (in->insn.output.type == IROp::REG)
    {
      int reg = in->insn.output.guest_reg;
      // modification is a given.
      // now see if this is a guest store and the input is
      // constant
      if (in->insn.emitter == &DSPEmitterIR::StoreGuestOp &&
          in->insn.inputs[0].type == IROp::VREG && cnt_vregs.count(in->insn.inputs[0].vreg) > 0 &&
          reg < 32 && reg != DSP_REG_ACH0 && reg != DSP_REG_ACH1 && reg != DSP_REG_PRODL &&
          reg != DSP_REG_PRODH && reg != DSP_REG_PRODM && reg != DSP_REG_PRODM2)
      {
        c_regs[reg] = true;
        v_regs[reg] = var_vregs[in->insn.inputs[0].vreg];
        m_regs[reg] = false;
      }
      else
      {
        switch (reg)
        {
        case IROp::DSP_REG_ACC0_ALL:
        case IROp::DSP_REG_ACC1_ALL:
          m_regs[reg - IROp::DSP_REG_ACC0_ALL + DSP_REG_ACL0] = true;
          m_regs[reg - IROp::DSP_REG_ACC0_ALL + DSP_REG_ACM0] = true;
          m_regs[reg - IROp::DSP_REG_ACC0_ALL + DSP_REG_ACH0] = true;
          break;
        case IROp::DSP_REG_AX0_ALL:
        case IROp::DSP_REG_AX1_ALL:
          m_regs[reg - IROp::DSP_REG_AX0_ALL + DSP_REG_AXL0] = true;
          m_regs[reg - IROp::DSP_REG_AX0_ALL + DSP_REG_AXH0] = true;
          break;
        case IROp::DSP_REG_PROD_ALL:
          m_regs[DSP_REG_PRODL] = true;
          m_regs[DSP_REG_PRODM] = true;
          m_regs[DSP_REG_PRODH] = true;
          m_regs[DSP_REG_PRODM2] = true;
          break;
        default:
          m_regs[reg] = true;
        }
      }
      if (in->insn.emitter == &DSPEmitterIR::StoreGuestACMOp && reg < 32)
      {
        // when we know SR_MODE_40_BIT, we can to better
        // here.
        m_regs[reg - DSP_REG_ACM0 + DSP_REG_ACL0] = true;
        m_regs[reg - DSP_REG_ACM0 + DSP_REG_ACH0] = true;
      }
    }

    if (in->insn.emitter == &DSPEmitterIR::LoadGuestFastOp && in->insn.inputs[0].type == IROp::REG)
    {
      int reg = in->insn.inputs[0].guest_reg;
      if (reg < 32 && const_regs[reg] && reg != DSP_REG_ACH0 && reg != DSP_REG_ACH1 &&
          reg != DSP_REG_PRODL && reg != DSP_REG_PRODH && reg != DSP_REG_PRODM &&
          reg != DSP_REG_PRODM2)
      {
        cnt_vregs.insert(in->insn.output.vreg);
        var_vregs[in->insn.output.vreg] = v_regs[reg];
        mod_vregs.erase(in->insn.output.vreg);
      }
    }

    for (auto r : mod_vregs)
      cnt_vregs.erase(r);
    for (int i = 0; i < 32; i++)
      c_regs[i] &= !m_regs[i];

    // merge with the others
    for (int i = 0; i < 32; i++)
    {
      modified_regs[i] |= m_regs[i];
      if (const_regs[i] && c_regs[i] && value_regs[i] != v_regs[i])
        modified_regs[i] = true;
      const_regs[i] = const_regs[i] || c_regs[i];
      if (c_regs[i])
        value_regs[i] = v_regs[i];
    }
    for (auto r : mod_vregs)
      modified_vregs.insert(r);
    for (auto r : cnt_vregs)
    {
      if (const_vregs.count(r) > 0 && value_vregs[r] != var_vregs[r])
        modified_vregs.insert(r);
      const_vregs.insert(r);
      value_vregs[r] = var_vregs[r];
    }
  }
  else
  {
    // merge with the others
    for (int i = 0; i < 32; i++)
    {
      modified_regs[i] |= node->modified_regs[i];
      if (const_regs[i] && node->const_regs[i] && value_regs[i] != node->value_regs[i])
        modified_regs[i] = true;
      const_regs[i] = const_regs[i] || node->const_regs[i];
      if (node->const_regs[i])
        value_regs[i] = node->value_regs[i];
    }
    for (auto r : node->modified_vregs)
      modified_vregs.insert(r);
    for (auto r : node->const_vregs)
    {
      if (const_vregs.count(r) > 0 && value_vregs[r] != node->value_vregs[r])
        modified_vregs.insert(r);
      const_vregs.insert(r);
      value_vregs[r] = node->value_vregs[r];
    }
  }
}

void DSPEmitterIR::analyseKnownRegs(IRBB* bb)
{
  // iterate over all nodes until it doesn't change anymore
  std::unordered_set<IRNode*> todo = bb->nodes;
  while (!todo.empty())
  {
    IRNode* n = *todo.begin();
    todo.erase(todo.begin());

    // create copy of working structs
    bool const_regs[32];
    bool modified_regs[32];
    u16 value_regs[32];
    std::unordered_set<int> const_vregs = n->const_vregs;
    std::unordered_set<int> modified_vregs = n->modified_vregs;
    std::unordered_map<int, u64> value_vregs = n->value_vregs;

    memcpy(const_regs, n->const_regs, sizeof(const_regs));
    memcpy(modified_regs, n->modified_regs, sizeof(modified_regs));
    memcpy(value_regs, n->value_regs, sizeof(value_regs));

    for (auto n2 : n->prev)
    {
      analyseKnownRegs(n2, const_regs, modified_regs, value_regs, const_vregs, modified_vregs,
                       value_vregs);
    }

    // finish working structs
    for (int i = 0; i < 32; i++)
    {
      if (modified_regs[i])
      {
        value_regs[i] = 0;
        const_regs[i] = false;
      }
    }
    for (auto r : modified_vregs)
    {
      const_vregs.erase(r);
      value_vregs.erase(r);
    }

    // check for changes
    bool c = false;
    for (int i = 0; i < 32; i++)
    {
      if (n->const_regs[i] != const_regs[i] || n->modified_regs[i] != modified_regs[i] ||
          n->value_regs[i] != value_regs[i])
        c = true;
    }
    if (c || n->const_vregs != const_vregs || n->modified_vregs != modified_vregs ||
        n->value_vregs != value_vregs)
    {
      memcpy(n->const_regs, const_regs, sizeof(n->const_regs));
      memcpy(n->modified_regs, modified_regs, sizeof(n->modified_regs));
      memcpy(n->value_regs, value_regs, sizeof(n->value_regs));
      n->const_vregs = const_vregs;
      n->modified_vregs = modified_vregs;
      n->value_vregs = value_vregs;

      for (auto n2 : n->next)
        todo.insert(n2);
    }
  }
}

void DSPEmitterIR::analyseKnownRegs()
{
  // iterate over all nodes until it doesn't change anymore
  std::unordered_set<IRBB*> todo;
  for (auto bb : m_bb_storage)
  {
    todo.insert(bb);
    analyseKnownRegs(bb);
  }
  while (!todo.empty())
  {
    IRBB* bb = *todo.begin();
    todo.erase(todo.begin());

    // create copy of working structs
    bool const_regs[32];
    bool modified_regs[32];
    u16 value_regs[32];
    std::unordered_set<int> const_vregs = bb->start_node->const_vregs;
    std::unordered_set<int> modified_vregs = bb->start_node->modified_vregs;
    std::unordered_map<int, u64> value_vregs = bb->start_node->value_vregs;

    memcpy(const_regs, bb->start_node->const_regs, sizeof(const_regs));
    memcpy(modified_regs, bb->start_node->modified_regs, sizeof(modified_regs));
    memcpy(value_regs, bb->start_node->value_regs, sizeof(value_regs));

    for (auto n2 : bb->prev)
    {
      analyseKnownRegs(n2->end_node, const_regs, modified_regs, value_regs, const_vregs,
                       modified_vregs, value_vregs);
    }

    // finish working structs
    for (int i = 0; i < 32; i++)
    {
      if (modified_regs[i])
      {
        value_regs[i] = 0;
        const_regs[i] = false;
      }
    }
    for (auto vr : modified_vregs)
    {
      const_vregs.erase(vr);
      value_vregs.erase(vr);
    }

    // check for changes
    bool c = false;
    for (int i = 0; i < 32; i++)
    {
      if (bb->start_node->const_regs[i] != const_regs[i] ||
          bb->start_node->modified_regs[i] != modified_regs[i] ||
          bb->start_node->value_regs[i] != value_regs[i])
        c = true;
    }
    if (c || bb->start_node->const_vregs != const_vregs ||
        bb->start_node->modified_vregs != modified_vregs ||
        bb->start_node->value_vregs != value_vregs)
    {
      memcpy(bb->start_node->const_regs, const_regs, sizeof(bb->start_node->const_regs));
      memcpy(bb->start_node->modified_regs, modified_regs, sizeof(bb->start_node->modified_regs));
      memcpy(bb->start_node->value_regs, value_regs, sizeof(bb->start_node->value_regs));
      bb->start_node->const_vregs = const_vregs;
      bb->start_node->modified_vregs = modified_vregs;
      bb->start_node->value_vregs = value_vregs;

      analyseKnownRegs(bb);

      for (auto n : bb->next)
        todo.insert(n);
    }
  }

  /* update vregs */
  for (auto n : m_node_storage)
  {
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (!in)
      continue;
    IRInsn& insn = in->insn;
    if (insn.emitter == &DSPEmitterIR::LoadGuestFastOp && insn.inputs[0].type == IROp::REG)
    {
      int reg = insn.inputs[0].guest_reg;
      if (n->const_regs[reg] && reg < 32 && reg != DSP_REG_ACH0 && reg != DSP_REG_ACH1 &&
          reg != DSP_REG_PRODL && reg != DSP_REG_PRODH && reg != DSP_REG_PRODM &&
          reg != DSP_REG_PRODM2)
      {
        ERROR_LOG(DSPLLE, "constifiying reg %d", reg);
        m_vregs[insn.output.vreg].isImm = true;
        m_vregs[insn.output.vreg].imm = n->value_regs[reg];

        insn.emitter = &DSPEmitterIR::LoadImmOp;
        insn.inputs[0] = IROp::Imm(n->value_regs[reg]);
        insn.inputs[0].vreg = insn.output.vreg;
      }
    }
  }
}

}  // namespace x64
}  // namespace JITIR
}  // namespace DSP
