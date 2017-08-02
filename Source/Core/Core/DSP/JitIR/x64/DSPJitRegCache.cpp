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

DSPJitIRRegCache::DSPJitIRRegCache(DSPEmitterIR& emitter)
    : m_emitter(emitter), m_is_temporary(false), m_is_merged(false)
{
  for (X64CachedReg& xreg : m_xregs)
  {
    xreg.guest_reg = DSP_REG_STATIC;
  }

  ResetXRegs();

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

DSPJitIRRegCache::DSPJitIRRegCache(const DSPJitIRRegCache& cache)
    : m_regs(cache.m_regs), m_xregs(cache.m_xregs), m_emitter(cache.m_emitter),
      m_is_temporary(true), m_is_merged(false)
{
}

DSPJitIRRegCache& DSPJitIRRegCache::operator=(const DSPJitIRRegCache& cache)
{
  ASSERT_MSG(DSPLLE, &m_emitter == &cache.m_emitter, "emitter does not match");
  ASSERT_MSG(DSPLLE, m_is_temporary, "register cache not temporary??");
  m_is_merged = false;

  m_xregs = cache.m_xregs;
  m_regs = cache.m_regs;

  return *this;
}

DSPJitIRRegCache::~DSPJitIRRegCache()
{
  ASSERT_MSG(DSPLLE, !m_is_temporary || m_is_merged, "temporary cache not merged");
}

void DSPJitIRRegCache::FlushRegs(DSPJitIRRegCache& cache, bool emit)
{
  cache.m_is_merged = true;

  // sync the freely used xregs
  if (!emit)
  {
    for (size_t i = 0; i < m_xregs.size(); i++)
    {
      if (cache.m_xregs[i].guest_reg == DSP_REG_USED && m_xregs[i].guest_reg == DSP_REG_NONE)
      {
        m_xregs[i].guest_reg = DSP_REG_USED;
      }
      if (cache.m_xregs[i].guest_reg == DSP_REG_NONE && m_xregs[i].guest_reg == DSP_REG_USED)
      {
        m_xregs[i].guest_reg = DSP_REG_NONE;
      }
    }
  }

  // consistency checks
  for (size_t i = 0; i < m_xregs.size(); i++)
  {
    ASSERT_MSG(DSPLLE, m_xregs[i].guest_reg == cache.m_xregs[i].guest_reg,
               "cache and current xreg guest_reg mismatch for %zu", i);
  }
}

void DSPJitIRRegCache::LoadRegs(bool emit)
{
}

void DSPJitIRRegCache::SaveRegs()
{
}

void DSPJitIRRegCache::PushRegs(X64Reg returnreg)
{
  int push_count = 0;
  for (size_t i = 0; i < m_xregs.size(); i++)
  {
    if (m_xregs[i].guest_reg != DSP_REG_NONE && m_xregs[i].guest_reg != DSP_REG_STATIC &&
        static_cast<X64Reg>(i) != returnreg)
      push_count++;
  }

  // hardcoding alignment to 16 bytes
  if (push_count & 1)
  {
    m_emitter.SUB(64, R(RSP), Imm32(8));
  }

  for (size_t i = 0; i < m_xregs.size(); i++)
  {
    if (m_xregs[i].guest_reg != DSP_REG_NONE && m_xregs[i].guest_reg != DSP_REG_STATIC &&
        static_cast<X64Reg>(i) != returnreg)
    {
      m_emitter.PUSH(static_cast<X64Reg>(i));
    }
  }
}

void DSPJitIRRegCache::PopRegs(X64Reg returnreg)
{
  if (returnreg != INVALID_REG && returnreg != RAX)
    m_emitter.MOV(64, R(returnreg), R(RAX));

  int push_count = 0;
  for (int i = static_cast<int>(m_xregs.size() - 1); i >= 0; i--)
  {
    if (m_xregs[i].guest_reg != DSP_REG_NONE && m_xregs[i].guest_reg != DSP_REG_STATIC &&
        static_cast<X64Reg>(i) != returnreg)
    {
      push_count++;

      m_emitter.POP(static_cast<X64Reg>(i));
    }
  }

  // hardcoding alignment to 16 bytes
  if (push_count & 1)
  {
    m_emitter.ADD(64, R(RSP), Imm32(8));
  }
}

X64Reg DSPJitIRRegCache::SpillXReg()
{
  return INVALID_REG;
}

X64Reg DSPJitIRRegCache::FindFreeXReg() const
{
  for (X64Reg x : s_allocation_order)
  {
    if (m_xregs[x].guest_reg == DSP_REG_NONE)
    {
      return x;
    }
  }

  return INVALID_REG;
}

X64Reg DSPJitIRRegCache::FindSpillFreeXReg()
{
  X64Reg reg = FindFreeXReg();
  if (reg == INVALID_REG)
  {
    reg = SpillXReg();
  }
  return reg;
}

X64Reg DSPJitIRRegCache::GetFreeXReg()
{
  X64Reg reg = FindSpillFreeXReg();

  ASSERT_MSG(DSPLLE, reg != INVALID_REG, "could not find register");
  if (reg == INVALID_REG)
  {
    m_emitter.INT3();
  }

  m_xregs[reg].guest_reg = DSP_REG_USED;
  return reg;
}

void DSPJitIRRegCache::GetXReg(X64Reg reg)
{
  if (m_xregs[reg].guest_reg == DSP_REG_STATIC)
  {
    ERROR_LOG_FMT(DSPLLE, "Trying to get statically used XReg {}", reg);
    return;
  }

  ASSERT_MSG(DSPLLE, m_xregs[reg].guest_reg == DSP_REG_NONE, "register already in use");
  m_xregs[reg].guest_reg = DSP_REG_USED;
}

void DSPJitIRRegCache::PutXReg(X64Reg reg)
{
  if (m_xregs[reg].guest_reg == DSP_REG_STATIC)
  {
    ERROR_LOG_FMT(DSPLLE, "Trying to put statically used XReg {}", reg);
    return;
  }

  ASSERT_MSG(DSPLLE, m_xregs[reg].guest_reg == DSP_REG_USED, "PutXReg without get(Free)XReg");

  m_xregs[reg].guest_reg = DSP_REG_NONE;
}

void DSPJitIRRegCache::ResetXRegs()
{
  m_xregs[RAX].guest_reg = DSP_REG_NONE;
  m_xregs[RCX].guest_reg = DSP_REG_NONE;
  m_xregs[RDX].guest_reg = DSP_REG_NONE;

  m_xregs[RBX].guest_reg = DSP_REG_NONE;

  m_xregs[RSP].guest_reg = DSP_REG_STATIC;  // stack pointer

  m_xregs[RBP].guest_reg = DSP_REG_NONE;

  m_xregs[RSI].guest_reg = DSP_REG_NONE;
  m_xregs[RDI].guest_reg = DSP_REG_NONE;

  m_xregs[R8].guest_reg = DSP_REG_NONE;
  m_xregs[R9].guest_reg = DSP_REG_NONE;
  m_xregs[R10].guest_reg = DSP_REG_NONE;
  m_xregs[R11].guest_reg = DSP_REG_NONE;
  m_xregs[R12].guest_reg = DSP_REG_NONE;
  m_xregs[R13].guest_reg = DSP_REG_NONE;
  m_xregs[R14].guest_reg = DSP_REG_NONE;
  m_xregs[R15].guest_reg = DSP_REG_STATIC;
}

}  // namespace DSP::JITIR::x64
