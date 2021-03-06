// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <deque>
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

namespace std
{
template <>
struct hash<Gen::X64Reg>
{
  size_t operator()(const Gen::X64Reg& r) const
  {
    std::hash<int> hi;
    return hi((int)r);
  }
};
}

using namespace Gen;

namespace DSP::JITIR::x64
{
constexpr size_t COMPILED_CODE_SIZE = 2097152;
constexpr size_t MAX_BLOCK_SIZE = 250;
constexpr u16 DSP_IDLE_SKIP_CYCLES = 0x1000;

// Ordered in order of prefered use.
// All of these are actually available (note absence of R15 and RSP)
constexpr std::array<X64Reg, 14> s_allocation_order = {
    {R8, R9, R10, R11, R12, R13, R14, RSI, RDI, RBX, RCX, RDX, RAX, RBP}};

DSPEmitterIR::DSPEmitterIR(DSPCore& dsp) : m_blocks(MAX_BLOCKS), m_dsp_core{dsp}
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
  }
  m_dsp_core.DSPState().reset_dspjit_codespace = false;
}

bool DSPEmitterIR::FlagsNeeded(IRInsn const& insn) const
{
  // this actually catches a bit more than the analyzer,
  // probably calls, but i'm too lazy to look it up
  return (insn.later_needs_SR & insn.modifies_SR) != 0;
}

void DSPEmitterIR::DecodeInstruction(UDSPInstruction inst)
{
  const auto jit_decode_function = GetOp(inst);
  const DSPOPCTemplate* tinst = GetOpTemplate(inst);

  IRInsn p5 = {&CycleCountUpdateOp, {IROp::Imm(1)}};
  ir_add_op(p5);

  ir_commit_parallel_nodes();

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
      {
        insn.inputs[i].vreg = m_vregs.size();
        VReg vr = {insn.emitter->inputs[i].reqs, 0, false, M((void*)0)};
        if (insn.inputs[i].type == IROp::IMM)
        {
          vr.imm = insn.inputs[i].imm;
          vr.isImm = true;  // can be changed to false if reqs+imm requires
        }
        m_vregs.push_back(vr);
      }
      if (insn.emitter->inputs[i].reqs & SameAsOutput)
      {
        output_is_input = true;
        ASSERT_MSG(DSPLLE, !((insn.emitter->inputs[i].reqs ^ insn.emitter->output.reqs) & OpMask),
                   "Op type mismatch: %x != %x", insn.emitter->inputs[i].reqs & OpMask,
                   insn.emitter->output.reqs & OpMask);

        insn.output.vreg = m_vregs.size();
        VReg vr = {insn.emitter->output.reqs, 0, false, M((void*)0)};
        m_vregs.push_back(vr);
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

void DSPEmitterIR::connectLoops()
{
  // copy because we will remove things from m_end_bb->prev, invalidating
  // iterators
  std::unordered_set<IRBB*> todo = m_end_bb->prev;
  for (auto n : todo)
  {
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n->end_node);
    if (!in)
      continue;
    if (in->insn.emitter != &WriteBranchExitOp || in->insn.inputs[0].type != IROp::IMM)
      continue;
    // and now, the jump.
    u16 jump_pc = in->insn.inputs[0].imm;
    if (jump_pc == 0)
      continue;
    if (m_addr_info.find(jump_pc) == m_addr_info.end())
      continue;
    if (!m_addr_info[jump_pc].node)
      continue;
    // we replace the insn.

    IRInsn p = {&RegFixOp};
    assignVRegs(p);
    in->insn = p;

    // now hook up the target.
    IRBB* target = findAndSplitBB(m_addr_info[jump_pc].node);
    n->replaceNextNonBranched(target);
  }
}

void DSPEmitterIR::collectCycleCountUpdates(IRBB* bb, IRNode* node, IRNode* last_node,
                                            unsigned& cycle_count)
{
  IRNode* n = node;
  IRNode* ln = last_node;
  while (1)
  {
    ASSERT_MSG(DSPLLE, n->next.size() <= 1, "found parallel section");
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);

    if ((in &&
         (in->insn.emitter == &WriteBranchExitOp || in->insn.emitter == &CheckExceptionsUncondOp ||
          in->insn.emitter == &HandleLoopUnknownBeginOp || in->insn.emitter == &RetUncondOp ||
          in->insn.emitter == &RtiOp || in->insn.emitter == &HaltOp)) &&
        cycle_count > 0)
    {
      // end of execution. if we accumulated any number
      // of cycles (other than 0), we need to put a
      // update after the last node.

      // no need to split the bb for the update

      IRInsn p = {&CycleCountUpdateOp, {IROp::Imm(cycle_count)}};
      assignVRegs(p);
      IRInsnNode* in2 = makeIRInsnNode(p);

      bb->nodes.insert(in2);
      if (ln)
        ln->insertAfter(in2);
      else
      {
        in2->addNext(bb->start_node);
        bb->start_node = in2;
      }

      addGuestLoadStore(in2, bb);

      cycle_count = 0;
    }

    if (!in)
    {
      if (n->next.empty())
        return;
      ln = n;
      n = *n->next.begin();
      continue;
    }

    // collect and remove UpdateOps
    if (in->insn.emitter == &CycleCountUpdateOp)
    {
      // count and remove
      cycle_count += in->insn.inputs[0].imm;
      bb->nodes.erase(n);
      if (!n->next.empty())
      {
        IRNode* nn = *n->next.begin();

        ln->addNext(nn);

        n->removeNext(nn);
        n->removePrev(ln);
        n = nn;
        continue;
      }
      else
      {
        n->removePrev(ln);
        return;
      }
    }

    if (n->next.empty())
      return;
    ln = n;
    n = *n->next.begin();
  }
}

void DSPEmitterIR::collectCycleCountUpdates(std::unordered_set<IRBB*>& visited, IRBB* bb,
                                            IRBB* last_bb, unsigned cycle_count)
{
  IRBB* lbb = last_bb;
  while (1)
  {
    // insert the cycle count updates if needed
    if (bb->prev.size() > 1 && cycle_count > 0)
    {
      // branch/loop entry point. if we accumulated any number
      // of cycles (other than 0), we need to put a
      // update and check after the previous bb.

      // means: add another bb with the updateAndCheck,
      // then another bb with the exit branch

      IRBB* check_bb = new IRBB();
      m_bb_storage.push_back(check_bb);

      IRInsn p = {&CycleCountUpdateCheckOp, {IROp::Imm(cycle_count)}};
      assignVRegs(p);
      IRBranchNode* check_bn = makeIRBranchNode(p);
      check_bb->nodes.insert(check_bn);
      check_bb->start_node = check_bb->end_node = check_bn;

      addGuestLoadStore(check_bn, check_bb);

      ASSERT_MSG(DSPLLE, m_node_addr_map.find(bb->start_node) != m_node_addr_map.end(),
                 "no address for node %p", bb->start_node);
      u16 tgt_pc = m_node_addr_map[bb->start_node];

      // fill in the finishing insn
      IRBB* exit_bb = new IRBB();
      m_bb_storage.push_back(exit_bb);

      IRInsn p3 = {&CycleCountExitOp, {IROp::Imm(tgt_pc)}};
      assignVRegs(p3);
      IRInsnNode* exit_in = makeIRInsnNode(p3);

      exit_bb->nodes.insert(exit_in);
      exit_bb->start_node = exit_bb->end_node = exit_in;

      addGuestLoadStore(exit_in, exit_bb);

      // insert the bbs
      // hook non-branched exit from check_bb to bb
      check_bb->setNextNonBranched(bb);
      // hook lbb to check_bb, reusing whatever branch slot
      // was used for bb
      if (lbb->nextBranched == bb)
        lbb->replaceNextBranched(check_bb);
      if (lbb->nextNonBranched == bb)
        lbb->replaceNextNonBranched(check_bb);

      // the only exit from exit_bb goes to m_end_bb
      exit_bb->setNextNonBranched(m_end_bb);

      // check_bb branches to exit_bb
      check_bb->setNextBranched(exit_bb);

      cycle_count = 0;
    }

    // did we visit this merging bb already?
    if (bb->prev.size() > 1)
    {
      if (visited.find(bb) != visited.end())
        return;
      visited.insert(bb);
    }

    collectCycleCountUpdates(bb, bb->start_node, NULL, cycle_count);

    if (bb->nextBranched)
    {
      // execution branches. follow the branch with the
      // current cycle count.

      collectCycleCountUpdates(visited, bb->nextBranched, bb, cycle_count);
    }

    if (!bb->nextNonBranched)
      return;
    lbb = bb;
    bb = bb->nextNonBranched;
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

  {
    IRInsnNode* in1 = dynamic_cast<IRInsnNode*>(parallel_begin);
    if (in1)
      dumpIRNodes();
    ASSERT_MSG(DSPLLE, !in1, "found insn in begin node of parallel section(%p)", in1);
  }
  // so, find the end node of this section by traversing one branch.
  IRNode* n1 = *parallel_begin->next.begin();
  while (n1->prev.size() <= 1)
  {
    ASSERT_MSG(DSPLLE, n1->prev.size() == 1, "found invalid node connection");
    ASSERT_MSG(DSPLLE, n1->next.size() == 1, "found parallel section inside another");
    IRBranchNode* bn = dynamic_cast<IRBranchNode*>(n1);
    ASSERT_MSG(DSPLLE, !bn, "found branch node in parallel section");
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
      ASSERT_MSG(DSPLLE, n2->prev.size() == 1, "found invalid node connection");
      ASSERT_MSG(DSPLLE, n2->next.size() == 1, "found parallel section inside another");
      IRBranchNode* bn = dynamic_cast<IRBranchNode*>(n2);
      ASSERT_MSG(DSPLLE, !bn, "found branch node in parallel section");
      n2 = *n2->next.begin();
    }
    ASSERT_MSG(DSPLLE, n2 == parallel_end, "found multiple end points in parallel section");
  }

  // this is not exact match because there could be
  // branches joining our parallel_end
  if (parallel_begin->next.size() > parallel_end->prev.size())
    dumpIRNodes();
  ASSERT_MSG(DSPLLE, parallel_begin->next.size() <= parallel_end->prev.size(),
             "parallel section begin and end don't match in out/in edge count (%p -> %p)",
             parallel_begin, parallel_end);

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

void DSPEmitterIR::demoteGuestACMLoadStore()
{
  for (auto n : m_node_storage)
  {
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (in)
    {
      if (in->insn.emitter == &DSPEmitterIR::StoreGuestACMOp &&
          (in->insn.const_SR & SR_40_MODE_BIT) != 0 && (in->insn.value_SR & SR_40_MODE_BIT) == 0)
      {
        in->insn.emitter = &DSPEmitterIR::StoreGuestOp;
        in->insn.output.guest_reg =
            in->insn.output.guest_reg - IROp::DSP_REG_ACC0_ALL + DSP_REG_ACM0;
      }
      if (in->insn.emitter == &DSPEmitterIR::LoadGuestACMOp &&
          (in->insn.const_SR & SR_40_MODE_BIT) != 0 && (in->insn.value_SR & SR_40_MODE_BIT) == 0)
      {
        in->insn.emitter = &DSPEmitterIR::LoadGuestFastOp;
        in->insn.inputs[0].guest_reg =
            in->insn.inputs[0].guest_reg - IROp::DSP_REG_ACC0_ALL + DSP_REG_ACM0;
        in->insn.temps[0] = IROp::None();
      }
    }
  }
}

void DSPEmitterIR::dropNoOps()
{
  for (auto bb : m_bb_storage)
  {
    std::list<IRNode*> new_nodes;
    std::list<IRNode*> old_nodes;
    for (auto bbn : bb->nodes)
    {
      IRInsnNode* in = dynamic_cast<IRInsnNode*>(bbn);
      if (in)
      {
        if (in->insn.emitter->func == &DSPEmitterIR::iremit_NoOp)
        {
          for (int i = 0; i < NUM_INPUTS; i++)
          {
            if (in->insn.emitter->inputs[i].reqs & SameAsOutput)
            {
              merge_vregs(in->insn.inputs[i].vreg, in->insn.output.vreg);
            }
          }

          IRNode* n = makeIRNode();
          for (auto n2 : in->next)
            n->addNext(n2);
          for (auto n2 : in->prev)
            n->addPrev(n2);
          while (in->next.begin() != in->next.end())
            in->removeNext(*(in->next.begin()));
          while (in->prev.begin() != in->prev.end())
            in->removePrev(*(in->prev.begin()));
          new_nodes.push_back(n);
          old_nodes.push_back(in);
        }
      }
    }
    for (auto n : new_nodes)
      bb->nodes.insert(n);
    for (auto n : old_nodes)
      bb->nodes.erase(n);
  }
}

void DSPEmitterIR::checkImmVRegs()
{
  // check if we can assign the imms
  // will probably go into the whole "swap registers to fit requirements"
  // thingy
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
    // don't try to use same_hostReg as that does not extend
    // to the register _contents_. an operation can change
    // the value but still have SameAsOutput set
  }
}

void DSPEmitterIR::removeCheckExceptions()
{
  // must run after constant register evaluation,
  // but before StoreGuest*Op moves

  for (auto bb : m_bb_storage)
  {
    IRInsnNode* ein = dynamic_cast<IRInsnNode*>(bb->end_node);
    if (!ein)
      continue;
    if (ein->insn.emitter != &CheckExceptionsOp)
      continue;
    bool need_check = false;
    for (auto n : bb->nodes)
    {
      IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
      if (!in)
        continue;
      IRInsn& insn = in->insn;
      if (insn.emitter == &Load16Op || insn.emitter == &Load16ConcurrentOp)
      {
        // check if we can avoid the exception check
        if (!m_vregs[insn.inputs[0].vreg].isImm ||
            m_vregs[insn.inputs[0].vreg].imm == DSP_ACCELERATOR)
          need_check = true;
      }
      if (insn.emitter == &Load16SOp)
      {
        // check if we can avoid the exception check
        if ((!m_vregs[insn.inputs[1].vreg].isImm ||
             m_vregs[insn.inputs[1].vreg].imm == (DSP_ACCELERATOR >> 8)) &&
            m_vregs[insn.inputs[0].vreg].imm == (DSP_ACCELERATOR & 0xff))
          need_check = true;
      }
    }
    if (need_check)
      continue;
    // drop the check at the end
    IRNode* e = bb->end_node;
    bb->end_node = *e->prev.begin();
    bb->end_node->removeNext(e);
    bb->nodes.erase(e);
    // drop the link to the branch bb
    bb->next.erase(bb->nextBranched);
    bb->nextBranched->prev.erase(bb);
    bb->nextBranched = NULL;

    // todo: make it its own function? may be useful at other
    // points
    // try to join with the next bb
    IRBB* nbb = bb->nextNonBranched;
    if (nbb->prev.size() == 1)
    {
      // move the nodes over
      bb->nodes.insert(nbb->nodes.begin(), nbb->nodes.end());
      nbb->nodes.clear();
      // move the exits over
      bb->nextNonBranched = nbb->nextNonBranched;
      bb->nextBranched = nbb->nextBranched;
      bb->next = nbb->next;
      for (auto nn : nbb->next)
      {
        nn->prev.erase(nbb);
        nn->prev.insert(bb);
      }
      nbb->nextNonBranched = NULL;
      nbb->nextBranched = m_end_bb;
      nbb->next.clear();
      nbb->next.insert(m_end_bb);
      nbb->prev.clear();
      nbb->prev.insert(nbb);
      // connect nodes
      bb->end_node->addNext(nbb->start_node);
      bb->end_node = nbb->end_node;

      // we stole its node. make it a new one.
      IRNode* nnode = makeIRNode();
      nbb->start_node = nbb->end_node = nnode;
      nbb->nodes.insert(nnode);
    }
  }
}

void DSPEmitterIR::moveGuestLoadUp(IRBB* bb, IRInsnNode* node)
{
  // for now, we only move the node to the start of its bb.

  // assumes deparellelized bb

  // for all LoadGuest*Op, inputs[0] is the original input. no
  // vreg assigned. output is a vreg
  std::unordered_set<int> colliding_guests;
  colliding_guests.insert(node->insn.inputs[0].guest_reg);
  switch (node->insn.inputs[0].guest_reg)
  {
  case DSP_REG_PRODL:
  case DSP_REG_PRODM:
  case DSP_REG_PRODH:
  case DSP_REG_PRODM2:
    colliding_guests.insert(IROp::DSP_REG_PROD_ALL);
    break;
  case DSP_REG_AXL0:
  case DSP_REG_AXH0:
    colliding_guests.insert(IROp::DSP_REG_AX0_ALL);
    break;
  case DSP_REG_AXL1:
  case DSP_REG_AXH1:
    colliding_guests.insert(IROp::DSP_REG_AX1_ALL);
    break;
  case DSP_REG_ACL0:
  case DSP_REG_ACM0:
  case DSP_REG_ACH0:
    colliding_guests.insert(IROp::DSP_REG_ACC0_ALL);
    break;
  case DSP_REG_ACL1:
  case DSP_REG_ACM1:
  case DSP_REG_ACH1:
    colliding_guests.insert(IROp::DSP_REG_ACC1_ALL);
    break;
  case IROp::DSP_REG_PROD_ALL:
    colliding_guests.insert(DSP_REG_PRODL);
    colliding_guests.insert(DSP_REG_PRODM);
    colliding_guests.insert(DSP_REG_PRODH);
    colliding_guests.insert(DSP_REG_PRODM2);
    break;
  case IROp::DSP_REG_AX0_ALL:
    colliding_guests.insert(DSP_REG_AXL0);
    colliding_guests.insert(DSP_REG_AXH0);
    break;
  case IROp::DSP_REG_AX1_ALL:
    colliding_guests.insert(DSP_REG_AXL1);
    colliding_guests.insert(DSP_REG_AXH1);
    break;
  case IROp::DSP_REG_ACC0_ALL:
    colliding_guests.insert(DSP_REG_ACL0);
    colliding_guests.insert(DSP_REG_ACM0);
    colliding_guests.insert(DSP_REG_ACH0);
    break;
  case IROp::DSP_REG_ACC1_ALL:
    colliding_guests.insert(DSP_REG_ACL1);
    colliding_guests.insert(DSP_REG_ACM1);
    colliding_guests.insert(DSP_REG_ACH1);
    break;
  }

  IRNode* tcnode = node;

  // and now, we just scan backwards.
  while (1)
  {
    ASSERT_MSG(DSPLLE, tcnode->prev.size() <= 1, "too many previous nodes at this point");

    if (tcnode->prev.empty())
      break;

    IRNode* n = *(tcnode->prev.begin());
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (in)
    {
      if (in->insn.emitter == &StoreGuestProdOp || in->insn.emitter == &StoreGuestOp ||
          in->insn.emitter == &StoreGuestACMOp)
      {
        if (colliding_guests.find(in->insn.output.guest_reg) != colliding_guests.end())
          break;
      }
    }
    // iterate.
    tcnode = n;
  }
  if (tcnode == node)
    // no move possible
    return;
  ASSERT_MSG(DSPLLE, node->prev.size() <= 1, "too many previous nodes at this point");
  ASSERT_MSG(DSPLLE, node->next.size() <= 1, "too many next nodes at this point");
  // remove original node from its position
  if (node->next.empty())
    // was last node
    bb->end_node = *(node->prev.begin());
  else
  {
    (*node->prev.begin())->addNext(*node->next.begin());
    (*node->next.begin())->removePrev(node);
  }
  (*node->prev.begin())->removeNext(node);

  // insert before tcnode.
  if (tcnode->prev.empty())
    bb->start_node = node;
  else
  {
    node->addPrev(*tcnode->prev.begin());
    tcnode->removePrev(*tcnode->prev.begin());
  }
  tcnode->addPrev(node);
}

void DSPEmitterIR::moveGuestLoadsUp(IRBB* bb)
{
  std::deque<IRBB*> todo;
  std::unordered_set<IRBB*> done;
  todo.push_back(bb);
  // bb is used for scanning through the instructions to find
  // load/stores.
  while (!todo.empty())
  {
    IRBB* bb2 = todo.front();
    todo.pop_front();

    if (done.find(bb2) != done.end())
      continue;

    done.insert(bb2);
    for (auto n : bb2->next)
      todo.push_back(n);
    std::unordered_set<IRInsnNode*> inodes;
    for (auto n : bb2->nodes)
    {
      IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
      if (!in)
        continue;
      inodes.insert(in);
    }
    for (auto in : inodes)
    {
      if (in->insn.emitter == &LoadGuestProdOp || in->insn.emitter == &LoadGuestFastOp ||
          in->insn.emitter == &LoadGuestACMOp)
        moveGuestLoadUp(bb2, in);
    }
  }
}

void DSPEmitterIR::moveGuestStoreDown(IRBB* bb, IRInsnNode* node)
{
  // for now, we only move the node to the end of its bb.

  // assumes deparellelized bb

  // for all StoreGuest*Op, output is the original output. no
  // vreg assigned. input[0] is a vreg
  std::unordered_set<int> colliding_guests;
  colliding_guests.insert(node->insn.output.guest_reg);
  switch (node->insn.output.guest_reg)
  {
  case DSP_REG_PRODL:
  case DSP_REG_PRODM:
  case DSP_REG_PRODH:
  case DSP_REG_PRODM2:
    colliding_guests.insert(IROp::DSP_REG_PROD_ALL);
    break;
  case DSP_REG_AXL0:
  case DSP_REG_AXH0:
    colliding_guests.insert(IROp::DSP_REG_AX0_ALL);
    break;
  case DSP_REG_AXL1:
  case DSP_REG_AXH1:
    colliding_guests.insert(IROp::DSP_REG_AX1_ALL);
    break;
  case DSP_REG_ACL0:
  case DSP_REG_ACM0:
  case DSP_REG_ACH0:
    colliding_guests.insert(IROp::DSP_REG_ACC0_ALL);
    break;
  case DSP_REG_ACL1:
  case DSP_REG_ACM1:
  case DSP_REG_ACH1:
    colliding_guests.insert(IROp::DSP_REG_ACC1_ALL);
    break;
  case IROp::DSP_REG_PROD_ALL:
    colliding_guests.insert(DSP_REG_PRODL);
    colliding_guests.insert(DSP_REG_PRODM);
    colliding_guests.insert(DSP_REG_PRODH);
    colliding_guests.insert(DSP_REG_PRODM2);
    break;
  case IROp::DSP_REG_AX0_ALL:
    colliding_guests.insert(DSP_REG_AXL0);
    colliding_guests.insert(DSP_REG_AXH0);
    break;
  case IROp::DSP_REG_AX1_ALL:
    colliding_guests.insert(DSP_REG_AXL1);
    colliding_guests.insert(DSP_REG_AXH1);
    break;
  case IROp::DSP_REG_ACC0_ALL:
    colliding_guests.insert(DSP_REG_ACL0);
    colliding_guests.insert(DSP_REG_ACM0);
    colliding_guests.insert(DSP_REG_ACH0);
    break;
  case IROp::DSP_REG_ACC1_ALL:
    colliding_guests.insert(DSP_REG_ACL1);
    colliding_guests.insert(DSP_REG_ACM1);
    colliding_guests.insert(DSP_REG_ACH1);
    break;
  }

  IRNode* tcnode = node;

  // and now, we just scan forwards.
  while (1)
  {
    ASSERT_MSG(DSPLLE, tcnode->prev.size() <= 1, "too many previous nodes at this point");

    if (tcnode->next.empty())
      break;

    IRNode* n = *(tcnode->next.begin());
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (in)
    {
      if (in->insn.emitter == &StoreGuestProdOp || in->insn.emitter == &StoreGuestOp ||
          in->insn.emitter == &StoreGuestACMOp)
      {
        if (colliding_guests.find(in->insn.output.guest_reg) != colliding_guests.end())
          break;
      }
      if (in->insn.emitter == &LoadGuestProdOp || in->insn.emitter == &LoadGuestFastOp ||
          in->insn.emitter == &LoadGuestACMOp)
      {
        if (colliding_guests.find(in->insn.inputs[0].guest_reg) != colliding_guests.end())
          break;
      }
    }
    // iterate.
    tcnode = n;
  }
  if (tcnode == node)
    // no move possible
    return;
  ASSERT_MSG(DSPLLE, node->prev.size() <= 1, "too many previous nodes at this point");
  ASSERT_MSG(DSPLLE, node->next.size() <= 1, "too many next nodes at this point");
  // remove original node from its position
  if (node->prev.empty())
    // was first node
    bb->start_node = *(node->next.begin());
  else
  {
    (*node->prev.begin())->addNext(*node->next.begin());
    (*node->next.begin())->removePrev(node);
  }
  (*node->prev.begin())->removeNext(node);

  // cannot insert after end_node
  if (tcnode->next.empty())
    tcnode = *(tcnode->prev.begin());

  node->addNext(*tcnode->next.begin());
  tcnode->removeNext(*tcnode->next.begin());
  tcnode->addNext(node);
}

void DSPEmitterIR::moveGuestStoresDown(IRBB* bb)
{
  std::deque<IRBB*> todo;
  std::unordered_set<IRBB*> done;
  todo.push_back(bb);
  // bb is used for scanning through the instructions to find
  // load/stores.
  while (!todo.empty())
  {
    IRBB* bb2 = todo.front();
    todo.pop_front();

    if (done.find(bb2) != done.end())
      continue;

    done.insert(bb2);
    for (auto n : bb2->prev)
      todo.push_back(n);
    IRNode* n = bb2->end_node;
    while (n)
    {
      IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
      IRNode* nn = NULL;
      if (!n->prev.empty())
        nn = *n->prev.begin();
      if (in)
      {
        if (in->insn.emitter == &StoreGuestProdOp || in->insn.emitter == &StoreGuestOp ||
            in->insn.emitter == &StoreGuestACMOp)
          moveGuestStoreDown(bb2, in);
      }
      n = nn;
    }
  }
}

void DSPEmitterIR::extractSameHostReg()
{
  for (auto n : m_node_storage)
  {
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (in)
    {
      IRInsn& insn = in->insn;
      for (int i = 0; i < NUM_INPUTS; i++)
      {
        if (insn.emitter->inputs[i].reqs & SameAsOutput)
        {
          m_vregs[insn.output.vreg].same_hostreg_vregs.insert(insn.inputs[i].vreg);
          m_vregs[insn.inputs[i].vreg].same_hostreg_vregs.insert(insn.output.vreg);
        }
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
      std::unordered_set<int> l_vregs = (n2)->live_vregs;
      if (in2)
      {
        IRInsn& insn = in2->insn;
        {
          int vreg = insn.output.vreg;
          if (vreg > 0 && insn.output.type == IROp::VREG)
          {
            l_vregs.erase(vreg);
            for (auto vr : m_vregs[vreg].same_hostreg_vregs)
            {
              l_vregs.erase(vr);
            }
          }
        }
        for (unsigned int i = 0; i < NUM_TEMPS; i++)
        {
          int vreg = insn.temps[i].vreg;
          if (vreg > 0)
          {
            l_vregs.erase(vreg);
            for (auto vr : m_vregs[vreg].same_hostreg_vregs)
              l_vregs.erase(vr);
          }
        }
        for (unsigned int i = 0; i < NUM_INPUTS; i++)
        {
          int vreg = insn.inputs[i].vreg;
          if (vreg > 0 && insn.inputs[i].type == IROp::VREG)
          {
            l_vregs.insert(vreg);
            for (auto vr : m_vregs[vreg].same_hostreg_vregs)
              l_vregs.insert(vr);
          }
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
      {
        n->live_vregs.insert(vreg);
        for (auto vr : m_vregs[vreg].same_hostreg_vregs)
        {
          n->live_vregs.insert(vr);
        }
      }
    }
    for (unsigned int i = 0; i < NUM_INPUTS; i++)
    {
      int vreg = insn.inputs[i].vreg;
      if (vreg > 0 && insn.inputs[i].type == IROp::VREG)
      {
        n->live_vregs.insert(vreg);
        for (auto vr : m_vregs[vreg].same_hostreg_vregs)
        {
          n->live_vregs.insert(vr);
        }
      }
    }
    {
      int vreg = insn.output.vreg;
      if (vreg > 0 && insn.output.type == IROp::VREG)
      {
        n->live_vregs.insert(vreg);
        for (auto vr : m_vregs[vreg].same_hostreg_vregs)
          n->live_vregs.insert(vr);
      }
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
          {
            l_vregs.erase(vreg);
            for (auto vr : m_vregs[vreg].same_hostreg_vregs)
              l_vregs.erase(vr);
          }
        }
        for (unsigned int i = 0; i < NUM_TEMPS; i++)
        {
          int vreg = insn.temps[i].vreg;
          if (vreg > 0)
          {
            l_vregs.erase(vreg);
            for (auto vr : m_vregs[vreg].same_hostreg_vregs)
              l_vregs.erase(vr);
          }
        }
        for (unsigned int i = 0; i < NUM_INPUTS; i++)
        {
          int vreg = insn.inputs[i].vreg;
          if (vreg > 0 && insn.inputs[i].type == IROp::VREG)
          {
            l_vregs.insert(vreg);
            for (auto vr : m_vregs[vreg].same_hostreg_vregs)
              l_vregs.insert(vr);
          }
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

static X64Reg findHostReg(std::unordered_set<X64Reg> inuse)
{
  for (X64Reg x : s_allocation_order)
  {
    if (inuse.count(x) == 0)
    {
      return x;
    }
  }

  ASSERT_MSG(DSPLLE, 0, "could not allocate register");

  return INVALID_REG;
}

bool DSPEmitterIR::allocHostRegs(bool strict)
{
  // todo: refactor the allocation into one function, getting passed
  // which reqs to handle and/or from which pool to choose

  std::unordered_set<int> vreg_todo;
  std::list<int> vreg_allocation_order;
  bool success = false;

  for (unsigned int i = 1; i < m_vregs.size(); i++)
  {
    if (m_vregs[i].isImm)
      continue;
    vreg_todo.insert(i);
  }

  // allocate RAX/RCX
  for (auto it = vreg_todo.begin(); it != vreg_todo.end();)
  {
    if ((m_vregs[*it].reqs & OpAnyReg) == OpRAX || (m_vregs[*it].reqs & OpAnyReg) == OpRCX)
    {
      vreg_allocation_order.push_back(*it);
      it = vreg_todo.erase(it);
    }
    else
      it++;
  }

  // allocate the rest
  for (auto it = vreg_todo.begin(); it != vreg_todo.end();)
  {
    vreg_allocation_order.push_back(*it);
    it = vreg_todo.erase(it);
  }

  for (auto i : vreg_allocation_order)
  {
    if (m_vregs[i].oparg.IsSimpleReg() && m_vregs[i].oparg.GetSimpleReg() != INVALID_REG)
      continue;

    std::unordered_set<X64Reg> inuse;

    for (auto k : m_vregs[i].parallel_live_vregs)
    {
      if (m_vregs[k].oparg.IsSimpleReg())
        inuse.insert(m_vregs[k].oparg.GetSimpleReg());
      for (auto l : m_vregs[k].same_hostreg_vregs)
      {
        if (m_vregs[l].oparg.IsSimpleReg())
          inuse.insert(m_vregs[l].oparg.GetSimpleReg());
      }
    }

    X64Reg hreg = INVALID_REG;
    if ((m_vregs[i].reqs & OpAnyReg) == OpRAX)
      hreg = RAX;
    else if ((m_vregs[i].reqs & OpAnyReg) == OpRCX)
      hreg = RCX;
    else
      hreg = findHostReg(inuse);

    ASSERT_MSG(DSPLLE, !strict || hreg != INVALID_REG, "could not allocate host reg");
    ASSERT_MSG(DSPLLE, !strict || inuse.count(hreg) == 0, "register is already in use");
    OpArg o;
    if (hreg == INVALID_REG || inuse.count(hreg) != 0)
      o = M((void*)0);
    else
      o = R(hreg);

    m_vregs[i].oparg = o;
    for (auto vr : m_vregs[i].same_hostreg_vregs)
      m_vregs[vr].oparg = o;
  }

  success = true;
  for (unsigned int i = 1; i < m_vregs.size(); i++)
  {
    if (m_vregs[i].isImm)
      continue;
    if (strict)
    {
      ASSERT_MSG(DSPLLE,
                 m_vregs[i].oparg.IsSimpleReg() && m_vregs[i].oparg.GetSimpleReg() != INVALID_REG,
                 "Allocation for VReg %d failed", i);
      ASSERT_MSG(DSPLLE,
                 (m_vregs[i].reqs & OpAnyReg) != OpRAX || m_vregs[i].oparg.GetSimpleReg() == RAX,
                 "OpRAX requirement not met on VReg %d", i);
      ASSERT_MSG(DSPLLE,
                 (m_vregs[i].reqs & OpAnyReg) != OpRCX || m_vregs[i].oparg.GetSimpleReg() == RCX,
                 "OpRCX requirement not met on VReg %d", i);
    }
    if (!m_vregs[i].oparg.IsSimpleReg())
      success = false;
    if (m_vregs[i].oparg.GetSimpleReg() == INVALID_REG)
      success = false;
    if ((m_vregs[i].reqs & OpAnyReg) == OpRAX && m_vregs[i].oparg.GetSimpleReg() != RAX)
      success = false;
    if ((m_vregs[i].reqs & OpAnyReg) == OpRCX && m_vregs[i].oparg.GetSimpleReg() != RCX)
      success = false;
  }
  return success;
}

bool DSPEmitterIR::allocateHostRegs(bool strict)
{
  // clear information going to be collected below
  for (auto& vr : m_vregs)
  {
    vr.same_hostreg_vregs.clear();
    vr.parallel_live_vregs.clear();
  }

  for (auto n : m_node_storage)
  {
    n->live_vregs.clear();
  }

  // fills sameHostReg information in m_vregs
  extractSameHostReg();

  // fills insns.*.live_vregs
  analyseVRegLifetime();

  // if we keep the live_vregs correct, we can do load/store
  // removal here, and know all the time if we can afford to use another
  // host reg

  // fills m_vregs[*].parallel_live_vregs from insns.*.live_vreg
  findLiveVRegs();

  return allocHostRegs(strict);
}

void DSPEmitterIR::iremit_UnSpillOp(IRInsn const& insn)
{
  u64 spill_idx = insn.inputs[0].imm;
  int reqs = m_vregs[insn.output.vreg].reqs;
  // spilling from stack. stack grows downward, so it is
  // rsp+spill_idx
  OpArg dst = insn.output.oparg;
  OpArg src = MDisp(RSP, spill_idx * 8);
  switch (reqs & ExtendMask)
  {
  case ExtendSign16:
    MOVSX(64, 16, dst.GetSimpleReg(), src);
    break;
  case ExtendZero16:
    MOVZX(64, 16, dst.GetSimpleReg(), src);
    break;
  case ExtendSign32:
    MOVSX(64, 32, dst.GetSimpleReg(), src);
    break;
  case ExtendZero32:  // implicit zero extend
    MOV(32, dst, src);
    break;
  case ExtendSign64:
  case ExtendZero64:
  case ExtendNone:
    MOV(64, dst, src);
    break;
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::UnSpillOp = {
    "UnSpillOp", &DSPEmitterIR::iremit_UnSpillOp, 0, 0, 0, 0, false, {{OpImmAny}}, {OpAnyReg}};

void DSPEmitterIR::iremit_SpillOp(IRInsn const& insn)
{
  u64 spill_idx = insn.inputs[1].imm;
  // spilling from stack. stack grows downward, so it is
  // rsp+spill_idx
  MOV(64, MDisp(RSP, spill_idx * 8), insn.inputs[0].oparg);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::SpillOp = {
    "SpillOp", &DSPEmitterIR::iremit_SpillOp, 0, 0, 0, 0, false, {{OpAnyReg}, {OpImmAny}},
};

void DSPEmitterIR::spillVReg(int vreg)
{
  ASSERT_MSG(DSPLLE, !m_vregs[vreg].isImm, "trying to spill immediate VReg %d", vreg);
  // find all instructions actually using this vreg
  for (auto bb : m_bb_storage)
  {
    std::unordered_set<IRInsnNode*> todo;
    for (auto n : bb->nodes)
    {
      IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
      if (in)
        todo.insert(in);
    }
    for (auto in : todo)
    {
      IRInsn& insn = in->insn;
      bool output_is_input = false;
      for (int i = 0; i < NUM_INPUTS; i++)
      {
        if (insn.inputs[i].type == IROp::VREG && insn.inputs[i].vreg == vreg)
        {
          int vreg_idx = m_vregs.size();
          insn.inputs[i].vreg = vreg_idx;
          VReg vr = {insn.emitter->inputs[i].reqs, 0, false, M((void*)0)};
          m_vregs.push_back(vr);

          output_is_input = insn.emitter->inputs[i].reqs & SameAsOutput;
          if (output_is_input)
            insn.output.vreg = vreg_idx;

          IRInsn p = {&UnSpillOp, {IROp::Imm(m_spill_count)}, IROp::Vreg(vreg_idx)};
          assignVRegs(p);
          // should probably let something else
          // do this, but we run late so we
          // can get away with hardcoding
          m_vregs[p.inputs[0].vreg].isImm = true;
          m_vregs[p.inputs[0].vreg].oparg = Imm16(m_vregs[p.inputs[0].vreg].imm);
          IRInsnNode* in2 = makeIRInsnNode(p);

          bb->nodes.insert(in2);
          in->insertBefore(in2);
          addGuestLoadStore(in2, bb);
        }
      }
      for (int i = 0; i < NUM_TEMPS; i++)
      {
        ASSERT_MSG(DSPLLE, insn.temps[i].vreg != vreg, "trying to spill temporary %d!!", i);
      }

      if (insn.output.type == IROp::VREG && insn.output.vreg == vreg)
      {
        int vreg_idx;
        if (!output_is_input)
        {
          vreg_idx = m_vregs.size();
          insn.output.vreg = vreg_idx;
          VReg vr = {insn.emitter->output.reqs, 0, false, M((void*)0)};
          m_vregs.push_back(vr);
        }
        else
          vreg_idx = insn.output.vreg;

        IRInsn p = {
            &SpillOp, {IROp::Vreg(vreg_idx), IROp::Imm(m_spill_count)},
        };
        assignVRegs(p);
        // should probably let something else
        // do this, but we run late so we
        // can get away with hardcoding
        m_vregs[p.inputs[1].vreg].isImm = true;
        m_vregs[p.inputs[1].vreg].oparg = Imm16(m_vregs[p.inputs[1].vreg].imm);
        IRInsnNode* in2 = makeIRInsnNode(p);

        bb->nodes.insert(in2);
        in->insertAfter(in2);
        addGuestLoadStore(in2, bb);
      }
    }
  }

  m_spill_count++;
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

  // mark the active vregs active(pre/postABICall)
  for (auto i : in->live_vregs)
  {
    if (m_vregs[i].oparg.IsSimpleReg())
      m_vregs[i].active = true;
  }

  if (insn.needs_SR || insn.modifies_SR)
  {
    std::unordered_set<X64Reg> inuse;
    for (auto i : in->live_vregs)
    {
      if (m_vregs[i].oparg.IsSimpleReg())
        inuse.insert(m_vregs[i].oparg.GetSimpleReg());
    }

    X64Reg sr_reg = findHostReg(inuse);
    insn.SR = R(sr_reg);
  }
  else
    insn.SR = M_SDSP_r_sr();

  if ((insn.modifies_SR && insn.modifies_SR != 0xffff) || insn.needs_SR)
    MOV(16, insn.SR, M_SDSP_r_sr());

  (this->*insn.emitter->func)(insn);

  if (insn.modifies_SR)
    MOV(16, M_SDSP_r_sr(), insn.SR);

  // and now, drop it all again.(pre/postABICall)
  for (auto i : in->live_vregs)
  {
    if (m_vregs[i].oparg.IsSimpleReg())
      m_vregs[i].active = false;
  }
}

bool DSPEmitterIR::EmitBB(IRBB* bb)
{
  ASSERT_MSG(DSPLLE, GetSpaceLeft() > 64, "code space too full");
  if (bb->code)
  {
    // we already emitted this bb, so just do a JMP there.
    JMP(bb->code, true);
    return false;
  }
  bb->code = GetCodePtr();

  // for fixup of the final branch
  m_branch_todo.push_back(bb);

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
  return true;
}

void DSPEmitterIR::Compile(u16 start_addr)
{
  m_compile_pc = start_addr;

  auto& analyzer = m_dsp_core.DSPState().GetAnalyzer();

  m_vregs.clear();
  {
    VReg vr = {0, 0, false, M((void*)0)};
    m_vregs.push_back(vr);  // reserve vreg 0
  }

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
    if (reinterpret_cast<IRInsnNode*>(m_end_bb->end_node) || m_end_bb->end_node->prev.size() > 1)
    {
      IRNode* tn = makeIRNode();
      m_end_bb->nodes.insert(tn);
      m_end_bb->end_node->addNext(tn);
      m_end_bb->end_node = tn;
    }

    m_addr_info[m_compile_pc].node = m_end_bb->end_node;
    m_node_addr_map[m_end_bb->end_node] = m_compile_pc;

    // this is supposed to happen after
    // any op possibly raising an exception _or_ enabling
    // the SR_INT_ENABLE flag
    // exceptions can be raised by external(then latency is
    // acceptable) or internally by reading from the accelerator
    //(which is a memory mapped register)
    // this implements a check whenever memory is read
    // todo: move into load/store functions, so we can actually
    // verify if the access goes to a register
    // todo: check if we can actually do the whole interrupt
    // setup in gdsp_ifx_read or dsp_read_accelerator
    // from the interpreter side, things look good. jit?
    if (analyzer.IsCheckExceptions(m_compile_pc))
    {
      IRInsn p = {&CheckExceptionsOp};

      IRInsn p2 = {&CheckExceptionsUncondOp, {IROp::Imm(m_compile_pc)}};
      IRInsnNode* in = makeIRInsnNode(p2);

      ir_add_branch(p, in, in);

      ir_commit_parallel_nodes();
    }

    const UDSPInstruction inst = m_dsp_core.DSPState().ReadIMEM(m_compile_pc);
    const DSPOPCTemplate* opcode = GetOpTemplate(inst);

    DecodeInstruction(inst);

    m_compile_pc += opcode->size;

    if (analyzer.IsLoopEnd(m_compile_pc - 1))
    {
      ASSERT_MSG(DSPLLE, !opcode->branch, "jumps at end of loops are forbidden");
      // actually, we don't know what happens

      IRInsn p = {&HandleLoopOp, {IROp::Imm(m_compile_pc)}};

      IRBB* branch_bb = new IRBB();
      m_bb_storage.push_back(branch_bb);
      IRNode* branch_bb_node = makeIRNode();
      branch_bb->start_node = branch_bb->end_node = branch_bb_node;
      branch_bb->nodes.insert(branch_bb_node);

      u16 pred_loop_begin = m_addr_info[m_compile_pc].loop_begin;
      if (pred_loop_begin < 0xfffe)
      {
        IRInsn p2 = {&HandleLoopJumpBeginOp, {IROp::Imm(pred_loop_begin)}};

        IRInsn p3 = {
            &HandleLoopUnknownBeginOp,
        };
        IRInsnNode* in3 = makeIRInsnNode(p3);

        IRBB* nbb = ir_add_branch(branch_bb, p2, in3, in3);

        IRInsn p4 = {&WriteBranchExitOp, {IROp::Imm(pred_loop_begin)}};
        IRInsnNode* in4 = makeIRInsnNode(p4);

        ir_finish_irnodes(nbb, in4, in4);
        nbb->end_node->addNext(in4);
        nbb->end_node = in4;
      }
      else
      {
        IRInsn p3 = {
            &HandleLoopUnknownBeginOp,
        };
        IRInsnNode* in3 = makeIRInsnNode(p3);

        ir_finish_irnodes(branch_bb, in3, in3);
        branch_bb->end_node->addNext(in3);
        branch_bb->end_node = in3;
      }

      m_end_bb = ir_add_branch(m_end_bb, p, branch_bb);

      ir_commit_parallel_nodes();

      // todo: branch should be inverted(i.E. the
      // path preparing the jump back should be directly
      // after the loop-body, while the path out of the
      // loop should be somewhere else)
    }

    if (opcode->branch && opcode->uncond_branch)
      break;

    // End the block if we're before an idle skip address
    if (analyzer.IsIdleSkip(m_compile_pc))
    {
      break;
    }
  }

  {
    // add a WriteBranchExitOp if needed
    IRInsnNode* in = reinterpret_cast<IRInsnNode*>(m_end_bb->end_node);
    if (!in || in->insn.emitter != &WriteBranchExitOp)
    {
      IRInsn p = {&WriteBranchExitOp, {IROp::Imm(m_compile_pc)}};

      ir_add_op(p);

      ir_commit_parallel_nodes();
    }
  }

  // add final end_bb
  IRBB* new_end_bb = new IRBB();
  m_bb_storage.push_back(new_end_bb);
  IRNode* new_end_bb_node = makeIRNode();
  new_end_bb->start_node = new_end_bb->end_node = new_end_bb_node;
  new_end_bb->nodes.insert(new_end_bb_node);

  m_end_bb->setNextNonBranched(new_end_bb);
  m_end_bb = new_end_bb;

  // collect all dangling ends. is simple.
  for (auto bb : m_bb_storage)
  {
    if (!bb->nextNonBranched && bb != m_end_bb)
      bb->setNextNonBranched(m_end_bb);
  }

  connectLoops();

  // parsing done.
  dumpIRNodes();

  // if this is needed, it usually is some kind of coding error in the
  // ucode, but it may still rely on the effect
  for (auto bb : m_bb_storage)
    handleOverlappingOps(bb);

  for (auto bb : m_bb_storage)
    addGuestLoadStore(bb);

  for (auto bb : m_bb_storage)
    deparallelize(bb);

  dropNoOps();

  // fills in needs_SR
  analyseSRNeed();

  bool known_regs[32] = {false};
  u16 known_val_regs[32] = {0};
  // the assumption here is that these registers are statically assigned.
  // This breaks if guests have code fragments that run with changing
  // register values.
  // mp3player seems to do that with wr.
  for (int i = 0; i < 32; i++)
  {
    if ((i >= DSP_REG_WR0 && i <= DSP_REG_WR3 && false) || (i == DSP_REG_CR))
    {
      known_regs[i] = true;
      known_val_regs[i] = m_dsp_core.ReadRegister(i);
    }
  }

  m_start_bb->start_node->value_SR = m_dsp_core.DSPState().r.sr & 0xff00;
  m_start_bb->start_node->const_SR = 0xff00;
  memcpy(m_start_bb->start_node->value_regs, known_val_regs,
         sizeof(m_start_bb->start_node->value_regs));
  memcpy(m_start_bb->start_node->const_regs, known_regs,
         sizeof(m_start_bb->start_node->const_regs));
  analyseKnownSR();

  std::unordered_set<IRBB*> visited;
  collectCycleCountUpdates(visited, m_start_bb);

  // uses known SR to change some Load/Store back to the Fast variety
  demoteGuestACMLoadStore();

  analyseKnownRegs();

  checkImmVRegs();

  removeCheckExceptions();

  dumpIRNodes();
  //	moveGuestLoadsUp(start_bb);
  //	moveGuestStoresDown(end_bb);
  dumpIRNodes();
  // todo: coalesce load/store pairs handling the same guest register
  // then, move the remaining stores again(yeah, need it twice)
  // then, coalesce store/store pairs handling the same guest register
  //  (should be all at the same place, now)

  // todo: reorder guest load/stores
  // look through all BBs what registers are read/written
  //(cannot move stores over reads/writes and loads over writes)
  //* move loads as far "up" in the graph as possible
  //* move stores as far "down" in the graph as possibe
  // * follow both paths if a branch is encountered
  // * insert register merges(IRNodes that convert a vreg to another one,
  //   depending on from where the BB is entered) and required loads
  //   if control flow merges
  // loop handling:
  // * passing through a loop all bbs must not touch the guest
  // * when we track back to the same bb, loads and stores can be omitted
  //   if the other operation(store or load) is absent
  //
  // to conserve registers, later we need to
  //* move stores as far "up" in the graph as possible, but keeping
  //  them out of loops(can be skipped if the vreg is not used)
  //* move loads as far "down" in the graph as possible, but keeping
  //  them out of loops(can be skipped if the vreg is not used)

  m_spill_count = 0;
  while (!allocateHostRegs(false))
  {
    //* find candidate for spilling
    /*
        auto most_live_vregs = node_storage.begin();
        for(auto n : node_storage)
        {
          if ((*most_live_vregs)->live_vregs.size() <
              n->live_vregs.size())
            most_live_vregs = n;
        }
    */
    for (auto& vr : m_vregs)
    {
      vr.live_count = 0;
      vr.refed_count = 0;
    }

    for (auto n : m_node_storage)
    {
      for (auto vr : n->live_vregs)
        m_vregs[vr].live_count++;
      IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
      if (!in)
        continue;
      for (unsigned int i = 0; i < NUM_INPUTS; i++)
        m_vregs[in->insn.inputs[i].vreg].refed_count++;
      for (unsigned int i = 0; i < NUM_TEMPS; i++)
        m_vregs[in->insn.temps[i].vreg].refed_count++;
      m_vregs[in->insn.output.vreg].refed_count++;
    }

    int best_vreg = 1;
    for (unsigned int i = 2; i < m_vregs.size(); i++)
    {
      if (m_vregs[i].refed_count == 0 || m_vregs[i].isImm)
        continue;
      if (m_vregs[best_vreg].refed_count == 0 ||
          float(m_vregs[best_vreg].live_count) / float(m_vregs[best_vreg].refed_count) <
              float(m_vregs[i].live_count) / float(m_vregs[i].refed_count))
        best_vreg = i;
    }

    ERROR_LOG(DSPLLE, "spilling vreg %d(live for %d, refed %d times)", best_vreg,
              m_vregs[best_vreg].live_count, m_vregs[best_vreg].refed_count);
    // idea:
    // * find the instruction with most live vregs
    // * only take these vregs into account
    // * order the vregs by lifetime and usage during lifetime
    //   (perhaps like (lifetime in instructions having this
    //      vreg active)/(instructions using this vreg)  )
    // * take a high ranking vreg and spill(see below)

    //* split the vreg usage, insert spill ops
    // for spill to work, we need to know which vreg
    spillVReg(best_vreg);
    dumpIRNodes();
  }
  dumpIRNodes();
  allocateHostRegs(true);

  updateInsnOpArgs();

  // emit code
  dumpIRNodes();

  const u8* entryPoint = AlignCode16();

  enterJitCode();
  // prepare spill area
  if (m_spill_count > 0)
    SUB(64, R(RSP), Imm32(((m_spill_count + 1) / 2) * 16));

  // todo: future plan: create some kind of BB-scheduler that
  // produces a somewhat optimial order of the BBs to schedule,
  // then emit BBs in that order and fix up branches on the way,
  // including using the branch inversion feature

  for (IRBB* bb = m_start_bb; bb != m_end_bb; bb = bb->nextNonBranched)
  {
    if (!EmitBB(bb))
      break;
  }

  while (!m_branch_todo.empty())
  {
    IRBB* bb = m_branch_todo.front();
    m_branch_todo.pop_front();
    IRBranchNode* bn = dynamic_cast<IRBranchNode*>(bb->end_node);
    if (bn)
    {
      if (bb->nextBranched->code)
      {
        // already know where this is going
        SetJumpTarget(bn->insn.branchTaken, bb->nextBranched->code);
      }
      else
      {
        // fix the original branch
        SetJumpTarget(bn->insn.branchTaken);
        // emit all the bbs hanging there
        for (IRBB* bb2 = bb->nextBranched; bb2 != m_end_bb; bb2 = bb2->nextNonBranched)
          if (!EmitBB(bb2))
            break;
      }
    }
  }

  clearNodeStorage();

  m_blocks[start_addr] = (DSPCompiledCode)entryPoint;
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

  m_reenter_dispatcher = GetCodePtr();

  FixupBranch exceptionExit;
  if (Host::OnThread())
  {
    CMP(8, M_SDSP_external_interrupt_waiting(), Imm8(0));
    exceptionExit = J_CC(CC_NE);
  }

  // Check for DSP halt
  TEST(8, M_SDSP_cr(), Imm8(CR_HALT));
  FixupBranch _halt = J_CC(CC_NE);

  // Execute block. Block is responsible for updating m_cycles_left
  // and jumping to either m_reenter_dispatcher or m_return_dispatcher
  MOVZX(64, 16, ECX, M_SDSP_pc());
  MOV(64, R(RBX), ImmPtr(m_blocks.data()));
  JMPptr(MComplex(RBX, RCX, SCALE_8, 0));

  m_return_dispatcher = AlignCode16();

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
  JMP(m_reenter_dispatcher);
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
  insn.needs_SR |= insn.emitter->needs_SR;
  insn.modifies_SR |= insn.emitter->modifies_SR;
  insn.constant_mask_SR |= insn.emitter->constant_mask_SR;
  insn.constant_val_SR |= insn.emitter->constant_val_SR;
  insn.later_needs_SR = 0;
  insn.modified_SR = 0;
  insn.const_SR = 0;
  insn.value_SR = 0;
  assignVRegs(insn);
}

void DSPEmitterIR::ir_add_op(IRInsn insn)
{
  IRInsnNode* n = makeIRInsnNode(insn);
  ir_add_irnodes(n, n);
}

DSPEmitterIR::IRBB* DSPEmitterIR::ir_add_branch(IRBB* bb, IRInsn insn, IRBB* branch_bb)
{
  // commit anything thats supposed to be parallel
  ir_commit_parallel_nodes();

  IRBranchNode* bn = makeIRBranchNode(insn);
  ir_finish_irnodes(bb, bn, bn);

  bb->end_node->addNext(bn);
  bb->end_node = bn;

  if (branch_bb)
    bb->setNextBranched(branch_bb);

  // generate new BB
  IRBB* new_bb = new IRBB();
  m_bb_storage.push_back(new_bb);
  IRNode* new_bb_node = makeIRNode();
  new_bb->start_node = new_bb->end_node = new_bb_node;
  new_bb->nodes.insert(new_bb_node);

  bb->setNextNonBranched(new_bb);
  return new_bb;
}

DSPEmitterIR::IRBB* DSPEmitterIR::ir_add_branch(IRBB* bb, IRInsn insn, IRNode* first, IRNode* last)
{
  IRBB* branch_bb = NULL;
  if (first || last)
  {
    // generate branched bb
    branch_bb = new IRBB();
    m_bb_storage.push_back(branch_bb);
    IRNode* branch_bb_node = makeIRNode();
    branch_bb->start_node = branch_bb->end_node = branch_bb_node;
    branch_bb->nodes.insert(branch_bb_node);

    ir_finish_irnodes(branch_bb, first, last);

    branch_bb->end_node->addNext(first);
    branch_bb->end_node = last;
  }

  return ir_add_branch(bb, insn, branch_bb);
}

void DSPEmitterIR::ir_add_branch(IRInsn insn, IRNode* first, IRNode* last)
{
  m_end_bb = ir_add_branch(m_end_bb, insn, first, last);
}

void DSPEmitterIR::ir_finish_irnodes(IRBB* bb, IRNode* first, IRNode* last)
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
    bb->nodes.insert(n);
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (!in)
      continue;
    ir_finish_insn(in->insn);
  }
}

void DSPEmitterIR::ir_add_irnodes(IRBB* bb, IRNode* first, IRNode* last)
{
  ir_finish_irnodes(bb, first, last);
  m_parallel_nodes.push_back(std::make_pair(first, last));
}

void DSPEmitterIR::ir_add_irnodes(IRNode* first, IRNode* last)
{
  ir_add_irnodes(m_end_bb, first, last);
}

void DSPEmitterIR::ir_commit_parallel_nodes(IRBB* bb)
{
  if (m_parallel_nodes.empty())
    return;
  IRInsnNode* in = reinterpret_cast<IRInsnNode*>(bb->end_node);
  if (in &&
      (in->insn.emitter == &WriteBranchExitOp || in->insn.emitter == &CheckExceptionsUncondOp ||
       in->insn.emitter == &HandleLoopUnknownBeginOp || in->insn.emitter == &RetUncondOp ||
       in->insn.emitter == &RtiOp || in->insn.emitter == &HaltOp))
  {
    // no need to add anything; it will not be executed.
    m_parallel_nodes.clear();
    return;
  }

  // need to make sure there is always a "normal" IRNode at the
  // begin and end of parallel sections. at least for now.
  // ir_add_branch does this by itself to avoid the final added end node
  if (m_parallel_nodes.size() == 1)
  {
    bb->end_node->addNext(m_parallel_nodes[0].first);
    bb->end_node = m_parallel_nodes[0].second;
  }
  else
  {
    if (in)
    {
      IRNode* tn = makeIRNode();
      bb->nodes.insert(tn);
      bb->end_node->addNext(tn);
      bb->end_node = tn;
    }
    IRNode* new_end = makeIRNode();
    bb->nodes.insert(new_end);
    for (auto pnp : m_parallel_nodes)
    {
      bb->end_node->addNext(pnp.first);
      new_end->addPrev(pnp.second);
    }
    bb->end_node = new_end;
  }
  m_parallel_nodes.clear();
}

void DSPEmitterIR::ir_commit_parallel_nodes()
{
  ir_commit_parallel_nodes(m_end_bb);
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::InvalidOp = {"InvalidOp", NULL};

void DSPEmitterIR::iremit_NoOp(IRInsn const& insn)
{
  // really, does nothing
}

void DSPEmitterIR::iremit_RegFixOp(IRInsn const& insn)
{
  // not done, yet. different options, actually.
  //* replace with multiple ops
  //* create a single op that handles all register conversions
  // do we even have registers in flight at the moment?
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::RegFixOp = {"RegFixOp",
                                                                &DSPEmitterIR::iremit_RegFixOp};

// done: need a better way to track registers
/*
  state: we track registers(vregs) as host-register placeholders.

  idea: track registers as constant-value entities. extra attributes:
  same-hostreg-as(possibly a list?) needed for insns with the SameAsOutput
  requirement.
  alternatively, have hostreg groups. also, same-hostreg-as can be
  collected later.
  need to resolve conflicts between register requirements somehow.(RCX/RAX etc)
  perhaps also have a same-value-as list?


  NOTE: it looks like we don't actually need this, and the old vreg system
  is simpler to manipulate

 */

}  // namespace DSP::JITIR::x64
