// Copyright 2011 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <array>

#include "Common/x64Emitter.h"

namespace DSP::JITIR::x64
{
class DSPEmitterIR;

enum DSPJitRegSpecial
{
  DSP_REG_AX0_32 = 32,
  DSP_REG_AX1_32 = 33,
  DSP_REG_ACC0_64 = 34,
  DSP_REG_ACC1_64 = 35,
  DSP_REG_PROD_64 = 36,
  DSP_REG_MAX_MEM_BACKED = 36,

  DSP_REG_USED = 253,
  DSP_REG_STATIC = 254,
  DSP_REG_NONE = 255
};

enum class RegisterExtension
{
  Sign,
  Zero,
  None
};

class DSPJitIRRegCache
{
public:
  explicit DSPJitIRRegCache(DSPEmitterIR& emitter);

  // Find a free host reg, spill if used, reserve
  Gen::X64Reg GetFreeXReg();
  // Spill a specific host reg if used, reserve
  void GetXReg(Gen::X64Reg reg);
  // Unreserve the given host reg
  void PutXReg(Gen::X64Reg reg);

  void ResetXRegs();

  size_t RegSize(int reg) { return m_regs[reg].size; }
  Gen::OpArg RegMem(int reg) { return m_regs[reg].mem; }

private:
  struct DynamicReg
  {
    Gen::OpArg mem;
    size_t size;
  };

  std::array<DynamicReg, 37> m_regs;

  DSPEmitterIR& m_emitter;
};

}  // namespace DSP::JITIR::x64
