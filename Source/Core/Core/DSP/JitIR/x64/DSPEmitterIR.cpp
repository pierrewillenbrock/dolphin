// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <set>

#include "Common/Assert.h"
#include "Common/BitSet.h"
#include "Common/ChunkFile.h"
#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"

#include "Core/DSP/DSPAnalyzer.h"
#include "Core/DSP/DSPCore.h"
#include "Core/DSP/DSPHost.h"
#include "Core/DSP/DSPTables.h"
#include "Core/DSP/JitIR/x64/DSPJitTables.h"

using namespace Gen;

namespace DSP::JITIR::x64
{
constexpr size_t COMPILED_CODE_SIZE = 2097152;
constexpr size_t MAX_BLOCK_SIZE = 250;
constexpr u16 DSP_IDLE_SKIP_CYCLES = 0x1000;

DSPEmitterIR::DSPEmitterIR(DSPCore& dsp)
    : m_blocks(MAX_BLOCKS), m_block_size(MAX_BLOCKS), m_dsp_core{dsp}
{
  x64::InitInstructionTables();
  AllocCodeSpace(COMPILED_CODE_SIZE);

  CompileStaticHelpers();

  // Clear all of the block references
  std::fill(m_blocks.begin(), m_blocks.end(), (DSPCompiledCode)m_stub_entry_point);
}

DSPEmitterIR::~DSPEmitterIR()
{
  FreeCodeSpace();
}

u16 DSPEmitterIR::RunCycles(u16 cycles)
{
  if (m_dsp_core.DSPState().external_interrupt_waiting.exchange(false, std::memory_order_acquire))
  {
    m_dsp_core.CheckExternalInterrupt();
    m_dsp_core.CheckExceptions();
  }

  m_cycles_left = cycles;
  auto exec_addr = (DSPCompiledCode)m_enter_dispatcher;
  exec_addr();

  if (m_dsp_core.DSPState().reset_dspjit_codespace)
    ClearIRAMandDSPJITCodespaceReset();

  return m_cycles_left;
}

void DSPEmitterIR::DoState(PointerWrap& p)
{
  p.Do(m_cycles_left);
}

void DSPEmitterIR::ClearIRAM()
{
  for (size_t i = 0; i < MAX_BLOCKS; i++)
  {
    m_blocks[i] = (DSPCompiledCode)m_stub_entry_point;
    m_block_size[i] = 0;
  }
  m_dsp_core.DSPState().reset_dspjit_codespace = true;
}

void DSPEmitterIR::ClearIRAMandDSPJITCodespaceReset()
{
  ClearCodeSpace();
  CompileStaticHelpers();

  for (size_t i = 0; i < MAX_BLOCKS; i++)
  {
    m_blocks[i] = (DSPCompiledCode)m_stub_entry_point;
    m_block_size[i] = 0;
  }
  m_dsp_core.DSPState().reset_dspjit_codespace = false;
}

bool DSPEmitterIR::FlagsNeeded(u16 address) const
{
  const auto& analyzer = m_dsp_core.DSPState().GetAnalyzer();

  return !analyzer.IsStartOfInstruction(address) || analyzer.IsUpdateSR(address);
}

int DSPEmitterIR::ir_to_regcache_reg(int reg)
{
  switch (reg)
  {
  case DSPEmitterIR::IROp::DSP_REG_ACC0_ALL:
    return DSP_REG_ACC0_64;
  case DSPEmitterIR::IROp::DSP_REG_ACC1_ALL:
    return DSP_REG_ACC1_64;
  case DSPEmitterIR::IROp::DSP_REG_AX0_ALL:
    return DSP_REG_AX0_32;
  case DSPEmitterIR::IROp::DSP_REG_AX1_ALL:
    return DSP_REG_AX1_32;
  case DSPEmitterIR::IROp::DSP_REG_PROD_ALL:
    return DSP_REG_PROD_64;
  case 0:
  case 1:
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
  case 7:
  case 8:
  case 9:
  case 10:
  case 11:
  case 12:
  case 13:
  case 14:
  case 15:
  case 16:
  case 17:
  case 18:
  case 19:
  case 20:
  case 21:
  case 22:
  case 23:
  case 24:
  case 25:
  case 26:
  case 27:
  case 28:
  case 29:
  case 30:
  case 31:
    return reg;
  default:
    _assert_msg_(DSPLLE, 0, "cannot convert il reg %d to regcache", reg);
    return -1;
  }
}

void DSPEmitterIR::DecodeInstruction(UDSPInstruction inst)
{
  const auto jit_decode_function = GetOp(inst);
  const DSPOPCTemplate* tinst = GetOpTemplate(inst);

  // Call extended
  if (tinst->extended)
  {
    const auto jit_ext_decode_function = GetExtOp(inst);
    (this->*jit_ext_decode_function)(inst);
  }

  (this->*jit_decode_function)(inst);

  ir_commit_parallel_nodes();
}

void DSPEmitterIR::assignVRegs(IRInsn& insn)
{
  for (unsigned int i = 0; i < NUM_TEMPS; i++)
  {
    if (insn.emitter->temps[i].reqs && insn.temps[i].vreg <= 0)
    {
      insn.temps[i].vreg = m_vregs.size();
      insn.temps[i].type = IROp::VREG;
      VReg vr = {insn.emitter->temps[i].reqs, 0, false, M((void*)0)};
      m_vregs.push_back(vr);
    }
  }

  bool output_is_input = false;
  for (unsigned int i = 0; i < NUM_INPUTS; i++)
  {
    if (insn.emitter->inputs[i].reqs && insn.inputs[i].vreg <= 0)
    {
      insn.inputs[i].vreg = m_vregs.size();
      VReg vr = {insn.emitter->inputs[i].reqs, 0, false, M((void*)0)};
      if (insn.inputs[i].type == IROp::IMM)
      {
        vr.imm = insn.inputs[i].imm;
        vr.isImm = true;  // can be changed to false if reqs+imm requires
      }
      m_vregs.push_back(vr);
      if (insn.emitter->inputs[i].reqs & SameAsOutput)
      {
        output_is_input = true;
        insn.output.vreg = insn.inputs[i].vreg;

        _assert_msg_(DSPLLE, !((insn.emitter->inputs[i].reqs ^ insn.emitter->output.reqs) & OpMask),
                     "Op type mismatch: %x != %x", insn.emitter->inputs[i].reqs & OpMask,
                     insn.emitter->output.reqs & OpMask);

        m_vregs[insn.inputs[i].vreg].reqs |= insn.emitter->output.reqs & NoACMExtend;
      }
    }
  }

  if (!output_is_input && insn.emitter->output.reqs && insn.output.vreg <= 0)
  {
    insn.output.vreg = m_vregs.size();
    VReg vr = {insn.emitter->output.reqs, 0, false, M((void*)0)};
    m_vregs.push_back(vr);
  }
}

void DSPEmitterIR::findInputOutput(IRNode* begin, IRNode* end, IRNode*& last,
                                   std::unordered_set<int>& input_gregs,
                                   std::unordered_set<int>& output_gregs)
{
  last = begin;
  for (IRNode* n = begin; n != end; n = *n->next.begin())
  {
    last = n;
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (in)
    {
      for (int i = 0; i < NUM_INPUTS; i++)
      {
        if (in->insn.inputs[i].type == IROp::REG)
        {
          switch (in->insn.inputs[i].guest_reg)
          {
          case IROp::DSP_REG_ACC0_ALL:
            input_gregs.insert(DSP_REG_ACH0);
            input_gregs.insert(DSP_REG_ACM0);
            input_gregs.insert(DSP_REG_ACL0);
            break;
          case IROp::DSP_REG_ACC1_ALL:
            input_gregs.insert(DSP_REG_ACH1);
            input_gregs.insert(DSP_REG_ACM1);
            input_gregs.insert(DSP_REG_ACL1);
            break;
          case IROp::DSP_REG_AX0_ALL:
            input_gregs.insert(DSP_REG_AXH0);
            input_gregs.insert(DSP_REG_AXL0);
            break;
          case IROp::DSP_REG_AX1_ALL:
            input_gregs.insert(DSP_REG_AXH1);
            input_gregs.insert(DSP_REG_AXL1);
            break;
          case IROp::DSP_REG_PROD_ALL:
            input_gregs.insert(DSP_REG_PRODL);
            input_gregs.insert(DSP_REG_PRODM);
            input_gregs.insert(DSP_REG_PRODH);
            input_gregs.insert(DSP_REG_PRODM2);
            break;
          default:
            input_gregs.insert(in->insn.inputs[i].guest_reg);
          }
        }
      }
      if (in->insn.output.type == IROp::REG)
      {
        switch (in->insn.output.guest_reg)
        {
        case IROp::DSP_REG_ACC0_ALL:
          output_gregs.insert(DSP_REG_ACH0);
          output_gregs.insert(DSP_REG_ACM0);
          output_gregs.insert(DSP_REG_ACL0);
          break;
        case IROp::DSP_REG_ACC1_ALL:
          output_gregs.insert(DSP_REG_ACH1);
          output_gregs.insert(DSP_REG_ACM1);
          output_gregs.insert(DSP_REG_ACL1);
          break;
        case IROp::DSP_REG_AX0_ALL:
          output_gregs.insert(DSP_REG_AXH0);
          output_gregs.insert(DSP_REG_AXL0);
          break;
        case IROp::DSP_REG_AX1_ALL:
          output_gregs.insert(DSP_REG_AXH1);
          output_gregs.insert(DSP_REG_AXL1);
          break;
        case IROp::DSP_REG_PROD_ALL:
          output_gregs.insert(DSP_REG_PRODL);
          output_gregs.insert(DSP_REG_PRODM);
          output_gregs.insert(DSP_REG_PRODH);
          output_gregs.insert(DSP_REG_PRODM2);
          break;
        default:
          output_gregs.insert(in->insn.output.guest_reg);
          break;
        }
      }
    }
  }
}

void DSPEmitterIR::deparallelize(IRNode* node)
{
  // skip all single ended nodes

  while (node->next.size() <= 1)
  {
    if (node->next.size() == 0)
      return;
    node = *node->next.begin();
  }

  IRNode* parallel_begin = node;
  // now, find the end of this parallel section.
  // we assume there are no branches in here, all the parallel
  // execution paths end in the same node, and don't contain
  // parallel sections of their own.

  IRInsnNode* in1 = dynamic_cast<IRInsnNode*>(parallel_begin);
  _assert_msg_(DSPLLE, !in1, "found insn in begin node of parallel section");
  // so, find the end node of this section by traversing one branch.
  IRNode* n1 = *parallel_begin->next.begin();
  while (n1->prev.size() <= 1)
  {
    _assert_msg_(DSPLLE, n1->prev.size() == 1, "found invalid node connection");
    _assert_msg_(DSPLLE, n1->next.size() == 1, "found parallel section inside another");
    IRBranchNode* bn = dynamic_cast<IRBranchNode*>(n1);
    _assert_msg_(DSPLLE, !bn, "found branch node in parallel section");
    n1 = *n1->next.begin();
  }

  IRNode* parallel_end = n1;
  IRInsnNode* in2 = dynamic_cast<IRInsnNode*>(parallel_end);
  ASSERT_MSG(DSPLLE, !in2, "found insn in end node of parallel section");

  // now check the rest of the paths

  for (IRNode* n2 : parallel_begin->next)
  {
    while (n2->prev.size() <= 1)
    {
      _assert_msg_(DSPLLE, n2->prev.size() == 1, "found invalid node connection");
      _assert_msg_(DSPLLE, n2->next.size() == 1, "found parallel section inside another");
      IRBranchNode* bn = dynamic_cast<IRBranchNode*>(n2);
      _assert_msg_(DSPLLE, !bn, "found branch node in parallel section");
      n2 = *n2->next.begin();
    }
    _assert_msg_(DSPLLE, n2 == parallel_end, "found multiple end points in parallel section");
  }

  _assert_msg_(DSPLLE, parallel_begin->next.size() == parallel_end->prev.size(),
               "parallel section begin and end don't match in out/in edge count");

  // try to figure out a sequence of IRInsns so no insn reads
  // the results of a previous one.
  //(we just assume here that no sane code reads and writes to
  // the same address in the same instruction, with two different
  // operations. this may be wrong.)
  // in general, this should be possible.
  // if not, we need to split the insn in a load/move or move/store
  // pair with a virtual register in between
  // also, we need to keep opcodes modifying high SR bits at the
  // end.

  // this is a bit of a hack, correct would be to resolve all
  // insns to
  // op-with-result-to-virtual-reg,move-result-to-real-reg pairs
  // and then, in a later pass, reshuffel if possible, assuming
  // memory _can be_ the same
  // that should probably all happen on a virtual reg level,
  // that only later on get resolved to real(host) reg level

  // we are not that far that we can actually _do_ virtual regs

  std::list<DSPEmitterParallelSectioninfo> sections;
  for (auto n : parallel_begin->next)
  {
    sections.push_back(DSPEmitterParallelSectioninfo(n));
    findInputOutput(sections.back().first, parallel_end, sections.back().last,
                    sections.back().input_gregs, sections.back().output_gregs);
  }

  std::list<DSPEmitterParallelSectioninfo> sorted_sections;

  bool fail = false;
  while (!fail && !sections.empty())
  {
    auto its1 = sections.begin();
    for (; its1 != sections.end(); its1++)
    {
      bool good = true;
      if (!its1->output_gregs.empty())
      {
        for (auto its2 = sections.begin(); its2 != sections.end(); its2++)
        {
          if (its1 == its2)
            continue;
          for (auto ogreg : its1->output_gregs)
          {
            for (auto igreg : its2->input_gregs)
            {
              if (ogreg == igreg)
                good = false;
            }
          }
        }
      }
      // check for writes to SR, because those must be last

      bool SRWrite = false;
      for (IRNode* n = its1->first; n != parallel_end; n = *n->next.begin())
      {
        IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
        if (in)
        {
          if (in->insn.emitter == &SBSetOp || in->insn.emitter == &SBClrOp)
            SRWrite = true;
        }
      }

      for (auto greg : its1->output_gregs)
      {
        if (greg == DSP_REG_SR)
          SRWrite = true;
      }

      if (SRWrite && sections.size() != 1)
        good = false;
      if (good)
        break;
    }
    if (its1 == sections.end())
    {
      fail = true;
      break;
    }
    sorted_sections.push_back(*its1);
    sections.erase(its1);
  }

  if (!fail)
  {
    // got a valid order, apply.
    IRNode* n3 = parallel_begin;
    for (auto s : sorted_sections)
    {
      parallel_begin->removeNext(s.first);
      n3->addNext(s.first);
      parallel_end->removePrev(s.last);
      n3 = s.last;
    }
    parallel_end->addPrev(n3);
    deparallelize(parallel_end);
    return;
  }

  WARN_LOG(DSPLLE, "Failed to deparallelize using the simple method, trying to separate the loads");
  dumpIRNodes();

  ASSERT_MSG(DSPLLE, 0, "not-yet-implemented section");
  exit(1);
}

void DSPEmitterIR::deparallelize(IRBB* bb)
{
  deparallelize(bb->start_node);
}

void DSPEmitterIR::checkImmVRegs()
{
  // check if we can assign the imms
  for (unsigned int i = 1; i < m_vregs.size(); i++)
  {
    if (m_vregs[i].isImm)
    {
      if ((m_vregs[i].reqs & OpImm) && (s16)m_vregs[i].imm == m_vregs[i].imm)
      {
        m_vregs[i].oparg = Imm16(m_vregs[i].imm);
      }
      else if ((m_vregs[i].reqs & OpImm) && (s32)m_vregs[i].imm == m_vregs[i].imm)
      {
        // imm32 gets sign extended to 64bit in 64bit ops
        // result of 32bit ops is zero extended to 64bit
        m_vregs[i].oparg = Imm32(m_vregs[i].imm);
      }
      else if (m_vregs[i].reqs & OpImm64)
      {
        m_vregs[i].oparg = Imm64(m_vregs[i].imm);
      }
      else
      {
        // demote to register
        m_vregs[i].isImm = false;
      }
    }
  }
}

void DSPEmitterIR::analyseVRegLifetime(IRBB* bb)
{
  // step 2: repeatedly go through all insns and fix up live depending on
  // the "future" paths
  std::unordered_set<IRNode*> todo;
  for (auto n : bb->nodes)
    todo.insert(n);
  while (!todo.empty())
  {
    IRNode* n = *todo.begin();
    todo.erase(todo.begin());

    std::unordered_set<int> live_vregs = n->live_vregs;
    std::unordered_set<IRNode*> nexts;
    for (auto n2 : n->next)
    {
      IRInsnNode* in2 = dynamic_cast<IRInsnNode*>(n2);
      std::unordered_set<int> l_vregs = n2->live_vregs;
      if (in2)
      {
        IRInsn& insn = in2->insn;
        {
          int vreg = insn.output.vreg;
          if (vreg > 0 && insn.output.type == IROp::VREG)
            l_vregs.erase(vreg);
        }
        for (unsigned int i = 0; i < NUM_TEMPS; i++)
        {
          int vreg = insn.temps[i].vreg;
          if (vreg > 0)
            l_vregs.erase(vreg);
        }
        for (unsigned int i = 0; i < NUM_INPUTS; i++)
        {
          int vreg = insn.inputs[i].vreg;
          if (vreg > 0 && insn.inputs[i].type == IROp::VREG)
            l_vregs.insert(vreg);
        }
      }
      for (auto vi3 : l_vregs)
        live_vregs.insert(vi3);
    }

    if (n->live_vregs != live_vregs)
    {
      n->live_vregs = live_vregs;
      for (auto n2 : n->prev)
        todo.insert(n2);
    }
  }
}

void DSPEmitterIR::analyseVRegLifetime()
{
  // rules are:
  // * a set of vregs is live for every IRInsn.
  // * no vreg is live before it is written to
  // * no vregs are live before any end-of-execution-path
  // * a vreg is live when read
  // * it is forbidden to have a backwards path from a vreg read to
  //   start of code, without a corresponding write in between.

  // step 1: mark all vregs live that are used in an insn
  for (auto n : m_node_storage)
  {
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (!in)
      continue;
    IRInsn& insn = in->insn;
    for (unsigned int i = 0; i < NUM_TEMPS; i++)
    {
      int vreg = insn.temps[i].vreg;
      if (vreg > 0)
        n->live_vregs.insert(vreg);
    }
    for (unsigned int i = 0; i < NUM_INPUTS; i++)
    {
      int vreg = insn.inputs[i].vreg;
      if (vreg > 0 && insn.inputs[i].type == IROp::VREG)
        n->live_vregs.insert(vreg);
    }
    {
      int vreg = insn.output.vreg;
      if (vreg > 0 && insn.output.type == IROp::VREG)
        n->live_vregs.insert(vreg);
    }
  }

  // step 2: repeatedly go through all insns and fix up live depending on
  // the "future" paths
  std::unordered_set<IRBB*> todo;
  for (auto bb : m_bb_storage)
  {
    todo.insert(bb);
    analyseVRegLifetime(bb);
  }
  while (!todo.empty())
  {
    IRBB* bb = *todo.begin();
    todo.erase(todo.begin());

    std::unordered_set<int> live_vregs = bb->end_node->live_vregs;
    for (auto ni : bb->next)
    {
      IRNode* n2 = ni->start_node;
      IRInsnNode* in2 = dynamic_cast<IRInsnNode*>(n2);
      std::unordered_set<int> l_vregs = n2->live_vregs;
      if (in2)
      {
        IRInsn& insn = in2->insn;
        {
          int vreg = insn.output.vreg;
          if (vreg > 0 && insn.output.type == IROp::VREG)
            l_vregs.erase(vreg);
        }
        for (unsigned int i = 0; i < NUM_TEMPS; i++)
        {
          int vreg = insn.temps[i].vreg;
          if (vreg > 0)
            l_vregs.erase(vreg);
        }
        for (unsigned int i = 0; i < NUM_INPUTS; i++)
        {
          int vreg = insn.inputs[i].vreg;
          if (vreg > 0 && insn.inputs[i].type == IROp::VREG)
            l_vregs.insert(vreg);
        }
      }
      for (auto vi3 : l_vregs)
        live_vregs.insert(vi3);
    }

    if (bb->end_node->live_vregs != live_vregs)
    {
      bb->end_node->live_vregs = live_vregs;
      analyseVRegLifetime(bb);
      for (auto bb2 : bb->prev)
        todo.insert(bb2);
    }
  }
}

void DSPEmitterIR::findLiveVRegs()
{
  for (auto n : m_node_storage)
  {
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (in)
    {
      for (auto vi1 : n->live_vregs)
      {
        for (auto vi2 : n->live_vregs)
          m_vregs[vi1].parallel_live_vregs.insert(vi2);
      }
    }
  }
}

void DSPEmitterIR::allocHostRegs()
{
  // at this point, parallel ops are not allowed.

  // todo: refactor the allocation into one function, getting passed
  // which reqs to handle and/or from which pool to choose

  // todo: make this handle sequential lists of branches and sequential lists etc(no parallels,
  // though)

  // allocate RAX/RCX
  for (unsigned int i1 = 1; i1 < m_vregs.size(); i1++)
  {
    if (m_vregs[i1].isImm)
      continue;
    if ((m_vregs[i1].reqs & OpAnyReg) != OpRAX && (m_vregs[i1].reqs & OpAnyReg) != OpRCX)
      continue;

    std::set<X64Reg> inuse;
    for (auto i2 : m_vregs[i1].parallel_live_vregs)
    {
      if (m_vregs[i2].oparg.IsSimpleReg())
        inuse.insert(m_vregs[i2].oparg.GetSimpleReg());
    }

    for (auto hreg : inuse)
      m_gpr.GetXReg(hreg);

    {
      X64Reg hreg = INVALID_REG;
      if ((m_vregs[i1].reqs & OpAnyReg) == OpRAX)
        hreg = RAX;
      if ((m_vregs[i1].reqs & OpAnyReg) == OpRCX)
        hreg = RCX;

      m_gpr.GetXReg(hreg);
      m_vregs[i1].oparg = R(hreg);

      m_gpr.PutXReg(hreg);
    }

    for (auto hreg : inuse)
      m_gpr.PutXReg(hreg);
  }

  // allocate the rest
  for (unsigned int i1 = 1; i1 < m_vregs.size(); i1++)
  {
    if (m_vregs[i1].isImm)
      continue;
    if (m_vregs[i1].oparg.IsSimpleReg())
      continue;

    std::set<X64Reg> inuse;
    for (auto i2 : m_vregs[i1].parallel_live_vregs)
    {
      if (m_vregs[i2].oparg.IsSimpleReg())
        inuse.insert(m_vregs[i2].oparg.GetSimpleReg());
    }

    for (auto hreg : inuse)
      m_gpr.GetXReg(hreg);

    {
      X64Reg hreg = m_gpr.GetFreeXReg();
      m_vregs[i1].oparg = R(hreg);
      m_gpr.PutXReg(hreg);
    }

    for (auto hreg : inuse)
      m_gpr.PutXReg(hreg);
  }
}

void DSPEmitterIR::updateInsnOpArgs(IRInsn& insn)
{
  for (unsigned int i = 0; i < NUM_TEMPS; i++)
  {
    if (insn.temps[i].vreg > 0)
      insn.temps[i].oparg = m_vregs[insn.temps[i].vreg].oparg;
  }
  for (unsigned int i = 0; i < NUM_INPUTS; i++)
  {
    if (insn.inputs[i].vreg > 0)
      insn.inputs[i].oparg = m_vregs[insn.inputs[i].vreg].oparg;
  }
  if (insn.output.vreg > 0)
    insn.output.oparg = m_vregs[insn.output.vreg].oparg;
}

void DSPEmitterIR::updateInsnOpArgs()
{
  for (auto n : m_node_storage)
  {
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (in)
      updateInsnOpArgs(in->insn);
  }
}

void DSPEmitterIR::EmitInsn(IRInsnNode* in)
{
  IRInsn& insn = in->insn;
  ASSERT_MSG(DSPLLE, insn.emitter->func, "unhandled IL Op %s", insn.emitter->name);

  ASSERT_MSG(DSPLLE, GetSpaceLeft() > 64, "code space too full");

  // mark the active vregs active, so we can tell the m_gpr about
  // them being put, if needed(dropAllRegs)
  for (auto i : in->live_vregs)
  {
    if (m_vregs[i].oparg.IsSimpleReg())
      m_vregs[i].active = true;
  }

  X64Reg sr_reg;
  if (insn.needs_SR || insn.modifies_SR)
  {
    for (auto i : in->live_vregs)
    {
      if (m_vregs[i].oparg.IsSimpleReg())
        m_gpr.GetXReg(m_vregs[i].oparg.GetSimpleReg());
    }

    sr_reg = m_gpr.GetFreeXReg();
    m_gpr.PutXReg(sr_reg);

    for (auto i : in->live_vregs)
    {
      if (m_vregs[i].oparg.IsSimpleReg())
        m_gpr.PutXReg(m_vregs[i].oparg.GetSimpleReg());
    }

    insn.SR = R(sr_reg);
  }
  else
    insn.SR = M_SDSP_r_sr();

  if ((insn.modifies_SR && insn.modifies_SR != 0xffff) || insn.needs_SR)
    MOV(16, insn.SR, M_SDSP_r_sr());

  (this->*insn.emitter->func)(insn);

  if (insn.modifies_SR)
    MOV(16, M_SDSP_r_sr(), insn.SR);

  // and now, drop it all again. this will not be needed when
  // we drop the m_gpr completely
  for (auto i : in->live_vregs)
  {
    if (m_vregs[i].oparg.IsSimpleReg())
      m_vregs[i].active = false;
  }
}

void DSPEmitterIR::EmitBB(IRBB* bb)
{
  IRNode* n = bb->start_node;
  while (1)
  {
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);

    if (in)
      EmitInsn(in);

    ASSERT_MSG(DSPLLE, n->next.size() < 2, "cannot handle parallel insns in emitter");

    if (n->next.empty())
      break;

    n = *(n->next.begin());
  }
}

void DSPEmitterIR::Compile(u16 start_addr)
{
  // Remember the current block address for later(WriteBlockLink and friends)
  m_start_address = start_addr;

  m_compile_pc = start_addr;
  m_block_size[start_addr] = 0;

  auto& analyzer = m_dsp_core.DSPState().GetAnalyzer();

  m_vregs.clear();
  VReg vr = {0, 0, false, M((void*)0)};
  m_vregs.push_back(vr);  // reserve vreg 0

  clearNodeStorage();

  // create start_bb and preliminary end_bb
  m_start_bb = new IRBB();
  m_bb_storage.push_back(m_start_bb);
  IRNode* start_bb_node = makeIRNode();
  m_start_bb->start_node = m_start_bb->end_node = start_bb_node;
  m_start_bb->nodes.insert(start_bb_node);

  m_end_bb = new IRBB();
  m_bb_storage.push_back(m_end_bb);
  IRNode* end_bb_node = makeIRNode();
  m_end_bb->start_node = m_end_bb->end_node = end_bb_node;
  m_end_bb->nodes.insert(end_bb_node);

  m_start_bb->setNextNonBranched(m_end_bb);

  m_parallel_nodes.clear();

  while (m_compile_pc < start_addr + MAX_BLOCK_SIZE)
  {
    if (analyzer.IsCheckExceptions(m_compile_pc))
    {
      IRInsn p = {&CheckExceptionsOp, {IROp::Imm(m_compile_pc)}};

      ir_add_branch(p);
    }

    const UDSPInstruction inst = m_dsp_core.DSPState().ReadIMEM(m_compile_pc);
    const DSPOPCTemplate* opcode = GetOpTemplate(inst);

    DecodeInstruction(inst);
    m_block_size[start_addr]++;

    m_compile_pc += opcode->size;

    if (analyzer.IsLoopEnd(m_compile_pc - 1))
    {
      ASSERT_MSG(DSPLLE, !opcode->branch, "jumps at end of loops are forbidden");
      // actually, we don't know what happens

      IRInsn p = {&HandleLoopOp, {IROp::Imm(m_compile_pc)}};

      ir_add_branch(p);
    }

    if (opcode->branch && opcode->uncond_branch)
      break;

    // End the block if we're before an idle skip address
    if (analyzer.IsIdleSkip(m_compile_pc))
    {
      break;
    }
  }

  IRInsn p = {&WriteBranchExitOp, {IROp::Imm(m_compile_pc)}};

  ir_add_op(p);

  ir_commit_parallel_nodes();

  // add final end_bb
  IRBB* new_end_bb = new IRBB();
  m_bb_storage.push_back(new_end_bb);
  IRNode* new_end_bb_node = makeIRNode();
  new_end_bb->start_node = new_end_bb->end_node = new_end_bb_node;
  new_end_bb->nodes.insert(new_end_bb_node);

  m_end_bb->setNextNonBranched(new_end_bb);
  m_end_bb = new_end_bb;

  // parsing done.
  dumpIRNodes();

  // if this is needed, it usually is some kind of coding error in the
  // ucode, but it may still rely on the effect
  for (auto bb : m_bb_storage)
    handleOverlappingOps(bb);

  dumpIRNodes();

  for (auto bb : m_bb_storage)
    addGuestLoadStore(bb);

  for (auto bb : m_bb_storage)
    deparallelize(bb);

  checkImmVRegs();

  // fills insns.*.live_vregs
  analyseVRegLifetime();

  dumpIRNodes();

  // if we keep the live_, *_refed_vregs correct, we can do load/store
  // removal here, and know all the time if we can afford to use another
  // host reg

  // fills vregs[*].parallel_live_vregs from insns.*.live_vreg
  findLiveVRegs();
  allocHostRegs();
  updateInsnOpArgs();

  dumpIRNodes();

  const u8* entryPoint = AlignCode16();

  enterJitCode();

  for (IRBB* bb = m_start_bb; bb != m_end_bb; bb = bb->nextNonBranched)
    EmitBB(bb);

  clearNodeStorage();

  m_blocks[start_addr] = (DSPCompiledCode)entryPoint;

  if (m_block_size[start_addr] == 0)
  {
    // just a safeguard, should never happen anymore.
    // if it does we might get stuck over in RunForCycles.
    ERROR_LOG_FMT(DSPLLE, "Block at {:#06x} has zero size", start_addr);
    m_block_size[start_addr] = 1;
  }
}

void DSPEmitterIR::CompileCurrentIR(DSPEmitterIR& emitter)
{
  emitter.Compile(emitter.m_dsp_core.DSPState().pc);
}

void DSPEmitterIR::CompileStaticHelpers()
{
  m_enter_dispatcher = AlignCode16();
  // We don't use floating point (high 16 bits).
  BitSet32 registers_used = ABI_ALL_CALLEE_SAVED & BitSet32(0xffff);
  ABI_PushRegistersAndAdjustStack(registers_used, 8);

  MOV(64, R(R15), ImmPtr(&m_dsp_core.DSPState()));

  const u8* dispatcherLoop = GetCodePtr();

  FixupBranch exceptionExit;
  if (Host::OnThread())
  {
    CMP(8, M_SDSP_external_interrupt_waiting(), Imm8(0));
    exceptionExit = J_CC(CC_NE);
  }

  // Check for DSP halt
  TEST(8, M_SDSP_cr(), Imm8(CR_HALT));
  FixupBranch _halt = J_CC(CC_NE);

  // Execute block. Cycles executed returned in EAX.
  MOVZX(64, 16, ECX, M_SDSP_pc());
  MOV(64, R(RBX), ImmPtr(m_blocks.data()));
  JMPptr(MComplex(RBX, RCX, SCALE_8, 0));

  m_return_dispatcher = AlignCode16();

  // Decrement cyclesLeft
  MOV(64, R(RCX), ImmPtr(&m_cycles_left));
  SUB(16, MatR(RCX), R(EAX));

  J_CC(CC_A, dispatcherLoop);

  // DSP gave up the remaining cycles.
  SetJumpTarget(_halt);
  if (Host::OnThread())
  {
    SetJumpTarget(exceptionExit);
  }
  // MOV(32, M(&cyclesLeft), Imm32(0));
  ABI_PopRegistersAndAdjustStack(registers_used, 8);
  RET();

  m_int3_loop = AlignCode16();
  INT3();
  JMP(m_int3_loop);

  m_unused_jump = J(true);

  m_stub_entry_point = AlignCode16();
  //there is no ABI_CallFunctionP, but this is how it would look like
  MOV(64, R(ABI_PARAM1), Imm64(reinterpret_cast<u64>(this)));
  ABI_CallFunction(CompileCurrentIR);
  XOR(32, R(EAX), R(EAX));  // Return 0 cycles executed
  JMP(m_return_dispatcher);
}

Gen::OpArg DSPEmitterIR::M_SDSP_pc()
{
  return MDisp(R15, static_cast<int>(offsetof(SDSP, pc)));
}

Gen::OpArg DSPEmitterIR::M_SDSP_exceptions()
{
  return MDisp(R15, static_cast<int>(offsetof(SDSP, exceptions)));
}

Gen::OpArg DSPEmitterIR::M_SDSP_cr()
{
  return MDisp(R15, static_cast<int>(offsetof(SDSP, cr)));
}

Gen::OpArg DSPEmitterIR::M_SDSP_external_interrupt_waiting()
{
  static_assert(decltype(SDSP::external_interrupt_waiting)::is_always_lock_free &&
                sizeof(SDSP::external_interrupt_waiting) == sizeof(u8));

  return MDisp(R15, static_cast<int>(offsetof(SDSP, external_interrupt_waiting)));
}

Gen::OpArg DSPEmitterIR::M_SDSP_r_st(size_t index)
{
  return MDisp(R15, static_cast<int>(offsetof(SDSP, r.st) + sizeof(SDSP::r.st[0]) * index));
}

Gen::OpArg DSPEmitterIR::M_SDSP_reg_stack_ptrs(size_t index)
{
  return MDisp(R15, static_cast<int>(offsetof(SDSP, reg_stack_ptrs) +
                                     sizeof(SDSP::reg_stack_ptrs[0]) * index));
}

Gen::OpArg DSPEmitterIR::M_SDSP_r_sr()
{
  return MDisp(R15, static_cast<int>(offsetof(SDSP, r.sr)));
}

void DSPEmitterIR::ir_finish_insn(IRInsn& insn)
{
  insn.original = m_dsp_core.DSPState().ReadIMEM(m_compile_pc);
  insn.addr = m_compile_pc;
  insn.cycle_count = m_block_size[m_start_address];
  insn.needs_SR |= insn.emitter->needs_SR;
  insn.modifies_SR |= insn.emitter->modifies_SR;
  insn.constant_mask_SR |= insn.emitter->constant_mask_SR;
  insn.constant_val_SR |= insn.emitter->constant_val_SR;
  insn.later_needs_SR = 0;
  insn.modified_SR = 0;
  insn.const_SR = 0;
  insn.value_SR = 0;
  memset(insn.value_regs, 0, sizeof(insn.value_regs));
  memset(insn.const_regs, 0, sizeof(insn.const_regs));
  memset(insn.modified_regs, 0, sizeof(insn.modified_regs));
  assignVRegs(insn);
}

void DSPEmitterIR::ir_add_op(IRInsn insn)
{
  IRInsnNode* n = makeIRInsnNode(insn);
  ir_add_irnodes(n, n);
}

void DSPEmitterIR::ir_add_branch(IRInsn insn)
{
  // commit anything thats supposed to be parallel
  ir_commit_parallel_nodes();

  IRBranchNode* bn = makeIRBranchNode(insn);
  ir_finish_irnodes(bn, bn);

  m_end_bb->end_node->addNext(bn);
  m_end_bb->end_node = bn;

  // generate new BB
  IRBB* new_end_bb = new IRBB();
  m_bb_storage.push_back(new_end_bb);
  IRNode* new_end_bb_node = makeIRNode();
  new_end_bb->start_node = new_end_bb->end_node = new_end_bb_node;
  new_end_bb->nodes.insert(new_end_bb_node);

  m_end_bb->next.insert(new_end_bb);
  m_end_bb->nextNonBranched = new_end_bb;
  new_end_bb->prev.insert(m_end_bb);
  m_end_bb = new_end_bb;
}

void DSPEmitterIR::ir_finish_irnodes(IRNode* first, IRNode* last)
{
  std::unordered_set<IRNode*> nodes;
  std::unordered_set<IRNode*> todo;
  todo.insert(first);
  while (!todo.empty())
  {
    IRNode* n = *(todo.begin());
    todo.erase(todo.begin());
    nodes.insert(n);

    for (auto n2 : n->next)
      todo.insert(n2);
  }

  for (auto n : nodes)
  {
    m_end_bb->nodes.insert(n);
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (!in)
      continue;
    ir_finish_insn(in->insn);
  }
}

void DSPEmitterIR::ir_add_irnodes(IRNode* first, IRNode* last)
{
  ir_finish_irnodes(first, last);
  m_parallel_nodes.push_back(std::make_pair(first, last));
}

void DSPEmitterIR::ir_commit_parallel_nodes()
{
  if (m_parallel_nodes.empty())
    return;
  // need to make sure there is always a "normal" IRNode at the
  // begin and end of parallel sections. at least for now.
  // ir_add_branch does this by itself to avoid the final added end node
  IRNode* new_end = makeIRNode();
  m_end_bb->nodes.insert(new_end);
  if (m_parallel_nodes.size() == 1)
  {
    m_end_bb->end_node->addNext(m_parallel_nodes[0].first);
    m_parallel_nodes[0].second->addNext(new_end);
  }
  else
  {
    for (auto pnp : m_parallel_nodes)
    {
      m_end_bb->end_node->addNext(pnp.first);
      new_end->addPrev(pnp.second);
    }
  }
  m_end_bb->end_node = new_end;
  m_parallel_nodes.clear();
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::InvalidOp = {"InvalidOp", NULL};

}  // namespace DSP::JITIR::x64
