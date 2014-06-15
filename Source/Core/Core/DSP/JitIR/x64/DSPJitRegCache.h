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

  // For branching into multiple control flows
  DSPJitIRRegCache(const DSPJitIRRegCache& cache);
  DSPJitIRRegCache& operator=(const DSPJitIRRegCache& cache);

  ~DSPJitIRRegCache();

  // Merge must be done _before_ leaving the code branch, so we can fix
  // up any differences in state
  void FlushRegs(DSPJitIRRegCache& cache, bool emit = true);
  /* since some use cases are non-trivial, some examples:

     //this does not modify the final state of gpr
     <code using gpr>
     FixupBranch b = JCC();
       DSPJitIRRegCache c = gpr;
       <code using c>
       gpr.FlushRegs(c);
     SetBranchTarget(b);
     <code using gpr>

     //this does not modify the final state of gpr
     <code using gpr>
     DSPJitIRRegCache c = gpr;
     FixupBranch b1 = JCC();
       <code using gpr>
       gpr.FlushRegs(c);
       FixupBranch b2 = JMP();
     SetBranchTarget(b1);
       <code using gpr>
       gpr.FlushRegs(c);
     SetBranchTarget(b2);
     <code using gpr>

     //this allows gpr to be modified in the second branch
     //and fixes gpr according to the results form in the first branch
     <code using gpr>
     DSPJitIRRegCache c = gpr;
     FixupBranch b1 = JCC();
       <code using c>
       FixupBranch b2 = JMP();
     SetBranchTarget(b1);
       <code using gpr>
       gpr.FlushRegs(c);
     SetBranchTarget(b2);
     <code using gpr>

     //this does not modify the final state of gpr
     <code using gpr>
     u8* b = GetCodePtr();
       DSPJitIRRegCache c = gpr;
       <code using gpr>
       gpr.FlushRegs(c);
       JCC(b);
     <code using gpr>

     this all is not needed when gpr would not be used at all in the
     conditional branch
   */

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
  struct X64CachedReg
  {
    size_t guest_reg;  // Including DSPJitRegSpecial
  };

  struct DynamicReg
  {
    Gen::OpArg mem;
    size_t size;
  };

  // Find a free host reg
  Gen::X64Reg FindFreeXReg() const;
  Gen::X64Reg SpillXReg();
  Gen::X64Reg FindSpillFreeXReg();

  std::array<DynamicReg, 37> m_regs;
  std::array<X64CachedReg, 16> m_xregs;

  DSPEmitterIR& m_emitter;
  bool m_is_temporary;
  bool m_is_merged;
};

}  // namespace DSP::JITIR::x64
