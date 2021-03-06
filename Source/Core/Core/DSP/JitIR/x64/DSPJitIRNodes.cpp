
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
DSPEmitterIR::IRNode::IRNode()
    : code(NULL), later_needs_SR(0), const_SR(0), modified_SR(0), value_SR(0)
{
  memset(const_regs, 0, sizeof(const_regs));
  memset(modified_regs, 0, sizeof(modified_regs));
  memset(value_regs, 0, sizeof(value_regs));
}

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
    // there cannot be a branch node as a prev of another node
    ASSERT_MSG(DSPLLE, !dynamic_cast<IRBranchNode*>(n),
               "branch nodes are not allowed in node.prev");

    (n)->addNext(first);
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

/* locates the IRBB for the given IRNode and splits it if needed to satisfy
 * bb->start_node == at
 */
DSPEmitterIR::IRBB* DSPEmitterIR::findAndSplitBB(IRNode* at)
{
  IRNode* bb_start_node = NULL;
  std::unordered_set<IRNode*> nodes_for_move;
  std::unordered_set<IRNode*> todo_nodes;
  if (at->prev.empty())
    bb_start_node = at;
  for (auto n : at->prev)
    todo_nodes.insert(n);
  while (!todo_nodes.empty())
  {
    IRNode* n = *todo_nodes.begin();
    todo_nodes.erase(todo_nodes.begin());
    if (n->prev.empty())
      bb_start_node = n;

    nodes_for_move.insert(n);
    for (auto n2 : n->prev)
      todo_nodes.insert(n2);
  }
  ASSERT_MSG(DSPLLE, bb_start_node, "could not find start node relative to %p", at);
  IRBB* old_bb = NULL;
  for (auto bb : m_bb_storage)
    if (bb->start_node == bb_start_node)
      old_bb = bb;
  ASSERT_MSG(DSPLLE, old_bb, "could not find bb for node %p (via node %p)", at, bb_start_node);
  if (!nodes_for_move.empty())
  {
    IRBB* new_bb = new IRBB();
    m_bb_storage.push_back(new_bb);

    new_bb->start_node = bb_start_node;
    if (at->prev.size() > 1)
    {
      IRNode* new_end_node = makeIRNode();
      while (!at->prev.empty())
      {
        IRNode* n = *at->prev.begin();
        at->removePrev(n);
        n->addNext(new_end_node);
      }
      new_bb->end_node = new_end_node;
    }
    else
    {
      new_bb->end_node = *at->prev.begin();
      at->removePrev(*at->prev.begin());
    }

    for (auto n : nodes_for_move)
    {
      old_bb->nodes.erase(n);
      new_bb->nodes.insert(n);
    }

    while (!old_bb->prev.empty())
    {
      IRBB* bb = *old_bb->prev.begin();
      if (bb->nextNonBranched == old_bb)
        bb->replaceNextNonBranched(new_bb);
      if (bb->nextBranched == old_bb)
        bb->replaceNextBranched(new_bb);
    }

    new_bb->setNextNonBranched(old_bb);

    old_bb->start_node = at;
  }

  return old_bb;
}

static std::string opargToString(Gen::OpArg const& oparg)
{
  std::stringstream buf;
  if (oparg.IsSimpleReg())
  {
    buf << "R(";
    switch (oparg.GetSimpleReg())
    {
    case Gen::RAX:
      buf << "RAX";
      break;
    case Gen::RBX:
      buf << "RBX";
      break;
    case Gen::RCX:
      buf << "RCX";
      break;
    case Gen::RDX:
      buf << "RDX";
      break;
    case Gen::RSP:
      buf << "RSP";
      break;
    case Gen::RBP:
      buf << "RBP";
      break;
    case Gen::RSI:
      buf << "RSI";
      break;
    case Gen::RDI:
      buf << "RDI";
      break;
    case Gen::R8:
      buf << "R8";
      break;
    case Gen::R9:
      buf << "R9";
      break;
    case Gen::R10:
      buf << "R10";
      break;
    case Gen::R11:
      buf << "R11";
      break;
    case Gen::R12:
      buf << "R12";
      break;
    case Gen::R13:
      buf << "R13";
      break;
    case Gen::R14:
      buf << "R14";
      break;
    case Gen::R15:
      buf << "R15";
      break;
    default:
      buf << "unknown";
      break;
    }
    buf << ")";
    return buf.str();
  }
  if (oparg.IsImm())
  {
    buf << "Imm(" << std::hex << oparg.AsImm64().Imm64() << std::dec << ")";
    return buf.str();
  }
  return "complex";
}

std::string DSPEmitterIR::dumpIRNodeInsn(DSPEmitterIR::IRInsnNode* in) const
{
  DSPEmitterIR::IRInsn const& insn = in->insn;
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
    buf << "(" << opargToString(m_vregs[insn.output.vreg].oparg) << ")";
    buf << " o(" << opargToString(insn.output.oparg) << ")";
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
      buf << "(v" << insn.inputs[i].vreg;
      buf << "(" << opargToString(m_vregs[insn.inputs[i].vreg].oparg) << ")";
      buf << " o(" << opargToString(insn.inputs[i].oparg) << "))";
      break;
    case DSPEmitterIR::IROp::VREG:
      buf << "(v" << insn.inputs[i].vreg;
      buf << "(" << opargToString(m_vregs[insn.inputs[i].vreg].oparg) << ")";
      buf << " o(" << opargToString(insn.inputs[i].oparg) << ")";
      break;
    default:
      break;
    }
  }

  buf << "\\n";

  buf << "temps: ";
  for (unsigned i = 0;
       i < DSPEmitterIR::NUM_TEMPS && insn.temps[i].type != DSPEmitterIR::IROp::INVALID; i++)
  {
    if (i > 0)
      buf << ", ";
    switch (insn.temps[i].type)
    {
    case DSPEmitterIR::IROp::VREG:
      buf << "(v" << insn.temps[i].vreg;
      buf << "(" << opargToString(m_vregs[insn.temps[i].vreg].oparg) << ")";
      buf << " o(" << opargToString(insn.temps[i].oparg) << ")";
      break;
    default:
      buf << "unhandled type";
      break;
    }
  }
  buf << "\\n";

  buf << "later_needs_SR=0x" << std::hex << std::setw(4) << std::setfill('0') << insn.later_needs_SR
      << " const_SR=0x" << std::setw(4) << std::setfill('0') << insn.const_SR << "\\nvalue_SR=0x"
      << std::setw(4) << std::setfill('0') << insn.value_SR << std::dec
      << " known_WR:" << (in->const_regs[DSP_REG_WR0] ? 1 : 0)
      << (in->const_regs[DSP_REG_WR1] ? 1 : 0) << (in->const_regs[DSP_REG_WR2] ? 1 : 0)
      << (in->const_regs[DSP_REG_WR3] ? 1 : 0) << "\\n live vregs:";

  for (auto vr : in->live_vregs)
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
                dumpIRNodeInsn(bn).c_str());

      ERROR_LOG(DSPLLE, "%s -> %s [weight=40];", name.c_str(), branched.c_str());
      ERROR_LOG(DSPLLE, "%s -> %s [weight=100];", name.c_str(), fallthrough.c_str());
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
                dumpIRNodeInsn(in).c_str());
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

  for (unsigned i = 1; i < m_vregs.size(); i++)
  {
    std::stringstream buf2;
    buf2 << "V_" << i;
    std::string name = buf2.str();

    buf2.str("");
    buf2 << "reqs:";
    if ((m_vregs[i].reqs & OpMask) == OpAny64)
      buf2 << " Any64";
    else if ((m_vregs[i].reqs & OpMask) == OpAny)
      buf2 << " Any";
    else
    {
      if ((m_vregs[i].reqs & OpAnyReg) == OpAnyReg)
        buf2 << " AnyReg";
      else if (m_vregs[i].reqs & OpRAX)
        buf2 << " RAX";
      else if (m_vregs[i].reqs & OpRCX)
        buf2 << " RCX";
      if (m_vregs[i].reqs & OpMem)
        buf2 << " Mem";
      if ((m_vregs[i].reqs & OpImmAny) == OpImmAny)
        buf2 << " ImmAny";
      else if (m_vregs[i].reqs & OpImm)
        buf2 << " Imm";
      else if (m_vregs[i].reqs & OpImm64)
        buf2 << " Imm64";
    }
    buf2 << "(" << std::hex << m_vregs[i].reqs << std::dec << ")";

    buf2 << "\\noparg: " << opargToString(m_vregs[i].oparg);
    if (m_vregs[i].isImm)
      buf2 << "\\nimm: " << std::hex << m_vregs[i].imm << std::dec;

    ERROR_LOG(DSPLLE, "%s [ label=\"%s\\n%s\" ];", name.c_str(), name.c_str(), buf2.str().c_str());

    for (auto vr : m_vregs[i].same_hostreg_vregs)
    {
      buf2.str("");
      buf2 << "V_" << vr;
      std::string othername = buf2.str();
      ERROR_LOG(DSPLLE, "%s -> %s [color=red];", name.c_str(), othername.c_str());
    }
    for (auto vr : m_vregs[i].parallel_live_vregs)
    {
      buf2.str("");
      buf2 << "V_" << vr;
      std::string othername = buf2.str();
      ERROR_LOG(DSPLLE, "%s -> %s [color=green];", name.c_str(), othername.c_str());
    }
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
        std::string branched = buf3.str() + "_BR";
        std::string fallthrough = buf3.str() + "_FT";

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

void DSPEmitterIR::merge_vregs(int vreg1, int vreg2)
{
  // find all references to vreg2 and replace them with ones to vreg1
  // but first, merge requirements.
  m_vregs[vreg1].reqs &= m_vregs[vreg2].reqs;
  // merge same_hostreg_vregs and parallel_live_vregs
  for (auto i : m_vregs[vreg2].same_hostreg_vregs)
  {
    m_vregs[i].same_hostreg_vregs.insert(vreg1);
    for (auto j : m_vregs[vreg2].same_hostreg_vregs)
      m_vregs[i].same_hostreg_vregs.insert(j);
  }
  for (auto i : m_vregs[vreg2].parallel_live_vregs)
  {
    m_vregs[i].parallel_live_vregs.insert(vreg1);
    for (auto j : m_vregs[vreg2].parallel_live_vregs)
      m_vregs[i].parallel_live_vregs.insert(j);
  }
  // adjust instructions
  for (auto n : m_node_storage)
  {
    IRInsnNode* in = dynamic_cast<IRInsnNode*>(n);
    if (!in)
      continue;
    for (int i = 0; i < NUM_INPUTS; i++)
      if (in->insn.inputs[i].vreg == vreg2)
        in->insn.inputs[i].vreg = vreg1;
    for (int i = 0; i < NUM_TEMPS; i++)
      if (in->insn.temps[i].vreg == vreg2)
        in->insn.temps[i].vreg = vreg1;
    if (in->insn.output.vreg == vreg2)
      in->insn.output.vreg = vreg1;
  }
  m_vregs[vreg2].same_hostreg_vregs.clear();
  m_vregs[vreg2].parallel_live_vregs.clear();
}
}  // namespace x64
}  // namespace JITIR
}  // namespace DSP
