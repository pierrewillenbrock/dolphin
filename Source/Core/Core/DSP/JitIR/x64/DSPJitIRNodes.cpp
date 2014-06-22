
#include <iomanip>
#include <sstream>
#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

namespace DSP
{
namespace JITIR
{
namespace x64
{
void DSPEmitterIR::IRNode::addNext(IRNode* node)
{
  next.insert(node);
  node->prev.insert(this);
}

void DSPEmitterIR::IRNode::addPrev(IRNode* node)
{
  prev.insert(node);
  node->next.insert(this);
}

void DSPEmitterIR::IRNode::removeNext(IRNode* node)
{
  next.erase(node);
  node->prev.erase(this);
}

void DSPEmitterIR::IRNode::removePrev(IRNode* node)
{
  prev.erase(node);
  node->next.erase(this);
}

void DSPEmitterIR::IRNode::insertBefore(IRNode* first, IRNode* last)
{
  for (auto n : prev)
  {
    // check if we are on *its branch(if any) or next
    IRBranchNode* bn = dynamic_cast<IRBranchNode*>(n);
    if (bn && bn->branch.find(this) != bn->branch.end())
    {
      bn->addNextOnBranch(first);
    }
    else
    {
      n->addNext(first);
    }
  }
  while (!prev.empty())
  {
    (*prev.begin())->removeNext(this);
  }
  addPrev(last);
}

void DSPEmitterIR::IRNode::insertBefore(IRNode* node)
{
  insertBefore(node, node);
}

void DSPEmitterIR::IRNode::insertAfter(IRNode* first, IRNode* last)
{
  for (auto n : next)
    last->addNext(n);
  while (!next.empty())
    removeNext(*next.begin());
  addNext(first);
}

void DSPEmitterIR::IRNode::insertAfter(IRNode* node)
{
  insertAfter(node, node);
}

void DSPEmitterIR::IRBranchNode::addNextOnBranch(IRNode* node)
{
  branch.insert(node);
  node->prev.insert(this);
}

void DSPEmitterIR::IRBranchNode::removeNext(IRNode* node)
{
  branch.erase(node);
  DSPEmitterIR::IRNode::removeNext(node);
}

void DSPEmitterIR::IRBranchNode::insertAfterOnBranch(IRNode* first, IRNode* last)
{
  for (auto n : branch)
    last->addNext(n);
  while (!branch.empty())
    removeNext(*branch.begin());
  addNextOnBranch(first);
}

void DSPEmitterIR::IRBranchNode::insertAfterOnBranch(IRNode* node)
{
  insertAfterOnBranch(node, node);
}

void DSPEmitterIR::clearNodeStorage()
{
  m_addr_info.clear();
  for (auto node : m_node_storage)
    delete node;
  m_node_storage.clear();
  for (auto bb : m_bb_storage)
    delete bb;
  m_bb_storage.clear();
  m_start_bb = NULL;
  m_end_bb = NULL;
}

std::string DSPEmitterIR::dumpIRNodeInsn(DSPEmitterIR::IRInsn const& insn) const
{
  std::stringstream buf;

  const char* name = insn.emitter->name;
  buf << "0x" << std::hex << std::setw(4) << std::setfill('0') << insn.addr << ": Insn " << name
      << " " << std::dec;

  switch (insn.output.type)
  {
  case DSPEmitterIR::IROp::REG:
    buf << "r" << insn.output.guest_reg;
    break;
  case DSPEmitterIR::IROp::VREG:
    buf << "v" << insn.output.vreg;
    break;
  default:
    break;
  }

  buf << " = ";

  for (unsigned i = 0;
       i < DSPEmitterIR::NUM_INPUTS && insn.inputs[i].type != DSPEmitterIR::IROp::INVALID; i++)
  {
    if (i > 0)
      buf << ", ";
    switch (insn.inputs[i].type)
    {
    case DSPEmitterIR::IROp::REG:
      buf << "r" << insn.inputs[i].guest_reg;
      break;
    case DSPEmitterIR::IROp::IMM:
      buf << "#0x" << std::hex << std::setw(4) << std::setfill('0') << insn.inputs[i].imm
          << std::dec;
      buf << "(v" << insn.inputs[i].vreg << ")";
      break;
    case DSPEmitterIR::IROp::VREG:
      buf << "v" << insn.inputs[i].vreg;
      break;
    default:
      break;
    }
  }

  buf << "\\n";

  buf << "later_needs_SR=0x" << std::hex << std::setw(4) << std::setfill('0') << insn.later_needs_SR
      << " const_SR=0x" << std::setw(4) << std::setfill('0') << insn.const_SR << "\\nvalue_SR=0x"
      << std::setw(4) << std::setfill('0') << insn.value_SR << std::dec
      << " known_WR:" << (insn.const_regs[DSP_REG_WR0] ? 1 : 0)
      << (insn.const_regs[DSP_REG_WR1] ? 1 : 0) << (insn.const_regs[DSP_REG_WR2] ? 1 : 0)
      << (insn.const_regs[DSP_REG_WR3] ? 1 : 0) << "\\n live vregs:";

  for (auto vr : insn.live_vregs)
  {
    buf << " " << vr;
  }

  return buf.str();
}

void DSPEmitterIR::dumpIRNodes() const
{
  static int ctr = 0;
  ERROR_LOG(DSPLLE, "IRNode dump");
  ERROR_LOG(DSPLLE, "!-----------------------------------------------!");
  ERROR_LOG(DSPLLE, "digraph Nodes {");
  ERROR_LOG(DSPLLE, "compound=true;");
  ERROR_LOG(DSPLLE, "node [fontname=Arial];");
  ERROR_LOG(DSPLLE, "graph [fontname=Arial];");
  ERROR_LOG(DSPLLE, "fontname=Arial;");
  for (auto n : m_node_storage)
  {
    IRBranchNode* bn = dynamic_cast<IRBranchNode*>(n);
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (bn)
    {
      std::stringstream buf2;
      buf2 << "N_" << ctr << bn;
      std::string name = buf2.str();
      std::string branched = buf2.str() + "_BR";
      std::string fallthrough = buf2.str() + "_FT";

      ERROR_LOG(DSPLLE, "%s [ label=\"%s\\n%s\" shape=rectangle ];", name.c_str(), name.c_str(),
                dumpIRNodeInsn(bn->insn).c_str());

      ERROR_LOG(DSPLLE, "%s -> %s [weight=40];", name.c_str(), branched.c_str());
      ERROR_LOG(DSPLLE, "%s -> %s [weight=100];", name.c_str(), fallthrough.c_str());
      for (auto n2 : bn->branch)
      {
        std::stringstream buf3;
        buf3 << "N_" << ctr << n2;
        std::string nm = buf3.str();
        ERROR_LOG(DSPLLE, "%s -> %s [weight=100];", branched.c_str(), nm.c_str());
      }
      for (auto n2 : bn->next)
      {
        std::stringstream buf3;
        buf3 << "N_" << ctr << n2;
        std::string nm = buf3.str();
        ERROR_LOG(DSPLLE, "%s -> %s [weight=100];", fallthrough.c_str(), nm.c_str());
      }
    }
    else if (in)
    {
      std::stringstream buf2;
      buf2 << "N_" << ctr << in;
      std::string name = buf2.str();
      ERROR_LOG(DSPLLE, "%s [ label=\"%s\\n%s\" shape=rectangle ];", name.c_str(), name.c_str(),
                dumpIRNodeInsn(in->insn).c_str());
      for (auto n2 : in->next)
      {
        std::stringstream buf3;
        buf3 << "N_" << ctr << n2;
        std::string nm = buf3.str();
        ERROR_LOG(DSPLLE, "%s -> %s [weight=100];", name.c_str(), nm.c_str());
      }
    }
    else
    {
      std::stringstream buf2;
      buf2 << "N_" << ctr << n;
      std::string name = buf2.str();
      for (auto n2 : n->next)
      {
        std::stringstream buf3;
        buf3 << "N_" << ctr << n2;
        std::string nm = buf3.str();
        ERROR_LOG(DSPLLE, "%s -> %s [weight=100];", name.c_str(), nm.c_str());
      }
    }
  }

  for (auto aip : m_addr_info)
  {
    IRNode* n = aip.second.node;
    std::stringstream buf2;
    buf2 << "N_" << ctr << n;
    std::string node_name = buf2.str();
    buf2.str("");
    buf2 << "A_" << std::hex << std::setw(4) << std::setfill('0') << aip.first;
    std::string name = buf2.str();

    if (n)
      ERROR_LOG(DSPLLE, "%s -> %s;", name.c_str(), node_name.c_str());
    buf2.str("");
    if (aip.second.loop_begin == 0xfffe)
      buf2 << " loop:multiple";
    else if (aip.second.loop_begin != 0xffff)
      buf2 << " loop:" << std::hex << std::setw(4) << std::setfill('0') << aip.second.loop_begin;
    ERROR_LOG(DSPLLE, "%s [ label=\"%s\\n%s\" ];", name.c_str(), name.c_str(), buf2.str().c_str());
  }

  for (auto bb : m_bb_storage)
  {
    std::stringstream buf2;
    buf2 << "BB_" << ctr << bb;
    std::string name = buf2.str();
    ERROR_LOG(DSPLLE, "subgraph cluster_%s {", name.c_str());
    ERROR_LOG(DSPLLE, "label=%s;", name.c_str());
    ERROR_LOG(DSPLLE, "%s [shape=point style=invis];", name.c_str());
    if (m_start_bb == bb)
      ERROR_LOG(DSPLLE, "color=green;");
    if (m_end_bb == bb)
      ERROR_LOG(DSPLLE, "color=red;");
    for (auto n : bb->nodes)
    {
      std::stringstream buf3;
      buf3 << "N_" << ctr << n;
      std::string node_name = buf3.str();
      std::string node_opts;
      if (bb->start_node == n && bb->end_node == n)
      {
        node_opts += " color=yellow";
      }
      else if (bb->start_node == n)
      {
        node_opts += " color=green";
      }
      else if (bb->end_node == n)
      {
        node_opts += " color=red";
      }
      ERROR_LOG(DSPLLE, "%s [%s];", node_name.c_str(), node_opts.c_str());
      IRBranchNode* bn = dynamic_cast<IRBranchNode*>(n);
      if (bn)
      {
        std::string branched = buf2.str() + "_BR";
        std::string fallthrough = buf2.str() + "_FT";

        ERROR_LOG(DSPLLE, "%s [%s];", branched.c_str(), node_opts.c_str());
        ERROR_LOG(DSPLLE, "%s [%s];", fallthrough.c_str(), node_opts.c_str());
      }
    }
    ERROR_LOG(DSPLLE, "};");
    for (auto n : bb->next)
    {
      std::stringstream buf3;
      buf3 << "BB_" << ctr << n;
      std::string other_name = buf3.str();
      std::string opts;
      if (bb->nextBranched == n)
        opts += " color=blue";
      ERROR_LOG(DSPLLE, "%s -> %s [ltail=cluster_%s, lhead=cluster_%s%s];", name.c_str(),
                other_name.c_str(), name.c_str(), other_name.c_str(), opts.c_str());
    }
  }

  ERROR_LOG(DSPLLE, "}");
  ERROR_LOG(DSPLLE, "!-----------------------------------------------!");
  ctr++;
}

}  // namespace x64
}  // namespace JITIR
}  // namespace DSP
