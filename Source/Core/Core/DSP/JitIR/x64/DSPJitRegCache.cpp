// Copyright 2011 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/DSP/JitIR/x64/DSPJitRegCache.h"

#include <cinttypes>
#include <cstddef>

#include "Common/Assert.h"
#include "Common/Logging/Log.h"

#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP::JITIR::x64
{
// Ordered in order of prefered use.
// Not all of these are actually available
constexpr std::array<X64Reg, 15> s_allocation_order = {
    {R8, R9, R10, R11, R12, R13, R14, R15, RSI, RDI, RBX, RCX, RDX, RAX, RBP}};

static Gen::OpArg GetRegisterPointer(size_t reg)
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
  case DSP_REG_AX0_32:
  case DSP_REG_AX1_32:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.ax[0].val) +
                                       sizeof(SDSP::r.ax[0]) * (reg - DSP_REG_AX0_32)));
  case DSP_REG_ACC0_64:
  case DSP_REG_ACC1_64:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.ac[0].val) +
                                       sizeof(SDSP::r.ac[0]) * (reg - DSP_REG_ACC0_64)));
  case DSP_REG_PROD_64:
    return MDisp(R15, static_cast<int>(offsetof(SDSP, r.prod.val)));
  default:
    ASSERT_MSG(DSPLLE, 0, "cannot happen");
    return M(static_cast<void*>(nullptr));
  }
}

DSPJitIRRegCache::DSPJitIRRegCache(DSPEmitterIR& emitter) : m_emitter(emitter)
{
  for (size_t i = 0; i < m_regs.size(); i++)
  {
    m_regs[i].mem = GetRegisterPointer(i);
    m_regs[i].size = 0;
  }

  for (unsigned int i = 0; i < 32; i++)
  {
    m_regs[i].size = 2;
  }

  m_regs[DSP_REG_ST0].size = 0;
  m_regs[DSP_REG_ST1].size = 0;
  m_regs[DSP_REG_ST2].size = 0;
  m_regs[DSP_REG_ST3].size = 0;

  // special composite registers
  for (unsigned int i = 0; i < 2; i++)
  {
    m_regs[i + DSP_REG_ACC0_64].size = 8;
  }
  m_regs[DSP_REG_PROD_64].size = 8;

  for (unsigned int i = 0; i < 2; i++)
  {
    m_regs[i + DSP_REG_AX0_32].size = 4;
  }
}

}  // namespace DSP::JITIR::x64
