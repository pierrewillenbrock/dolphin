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

}  // namespace x64
}  // namespace JITIR
}  // namespace DSP
