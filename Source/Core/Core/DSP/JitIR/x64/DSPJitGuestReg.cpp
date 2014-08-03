// Copyright (C) 2012 Dolphin Project.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 2.0.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License 2.0 for more details.

// A copy of the GPL 2.0 should have been included with the program.
// If not, see http://www.gnu.org/licenses/

// Official GIT repository and contact information can be found at
// http://code.google.com/p/dolphin-emu/

#include "Common/x64Emitter.h"
#include "Core/DSP/DSPCore.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

using namespace Gen;

namespace DSP
{
namespace JITIR
{
namespace x64
{
void DSPEmitterIR::iremit_LoadImmOp(IRInsn const& insn)
{
  // nothing to do here. the register allocator already put something
  // sensible here.
  if (insn.output.oparg.IsImm())
    return;

  // this can be optimised a bit, by using the implicit sign extension
  // from 32 to 64 bits
  MOV(64, insn.output.oparg, Imm64(insn.inputs[0].imm));
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LoadImmOp = {
    "LoadImmOp", NULL, 0, 0, 0, 0, false, {}, {OpAnyReg | OpImmAny}};

void DSPEmitterIR::iremit_LoadGuestOp(IRInsn const& insn)
{
  X64Reg hreg = insn.output.oparg.GetSimpleReg();

  int reqs = m_vregs[insn.output.vreg].reqs;
  int greg = ir_to_regcache_reg(insn.inputs[0].guest_reg);

  RegisterExtension extend;

  switch (reqs & ExtendMask)
  {
  case ExtendSign16:
    extend = RegisterExtension::Sign;
    break;
  case ExtendZero16:
    extend = RegisterExtension::Zero;
    break;
  case ExtendSign32:
    extend = RegisterExtension::Sign;
    break;
  case ExtendZero32:
    extend = RegisterExtension::Zero;
    break;
  case ExtendSign64:
    extend = RegisterExtension::Sign;
    break;
  case ExtendZero64:
    extend = RegisterExtension::Zero;
    break;
  case ExtendNone:
    extend = RegisterExtension::None;
    break;
  default:
    _assert_msg_(DSPLLE, 0, "unrecognized extend requirement");
    extend = RegisterExtension::None;
    break;
  }

  if (insn.inputs[0].guest_reg == IROp::DSP_REG_PROD_ALL)
  {
    X64Reg tmp1 = m_gpr.GetFreeXReg();
    get_long_prod(hreg, tmp1);
    m_gpr.PutXReg(tmp1);
  }
  else if (insn.inputs[0].guest_reg == DSP_REG_SR)
  {
    MOV(16, R(hreg), insn.SR);
  }
  else if (((insn.inputs[0].guest_reg != DSP_REG_ACM0 &&
             insn.inputs[0].guest_reg != DSP_REG_ACM1) ||
            (reqs & NoSaturate)) &&
           insn.inputs[0].guest_reg != DSP_REG_ST0 && insn.inputs[0].guest_reg != DSP_REG_ST1 &&
           insn.inputs[0].guest_reg != DSP_REG_ST2 && insn.inputs[0].guest_reg != DSP_REG_ST3)
  {
    m_gpr.ReadReg(greg, hreg, extend);
  }
  else
  {
    X64Reg tmp1 = m_gpr.GetFreeXReg();
    X64Reg tmp2 = m_gpr.GetFreeXReg();
    X64Reg tmp3 = m_gpr.GetFreeXReg();
    dsp_op_read_reg(greg, hreg, extend, insn.SR, tmp1, tmp2, tmp3);
    m_gpr.PutXReg(tmp3);
    m_gpr.PutXReg(tmp2);
    m_gpr.PutXReg(tmp1);
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::LoadGuestOp = {
    "LoadGuestOp",
    NULL,
    0,
    0,
    0,
    0,  // todo: actually, we need a bit of SR. in some cases(...)
    false,
    {},
    {OpAny64}};

void DSPEmitterIR::iremit_StoreGuestOp(IRInsn const& insn)
{
  int reqs = m_vregs[insn.inputs[0].vreg].reqs;
  int greg = ir_to_regcache_reg(insn.output.guest_reg);

  if ((reqs & OpAnyReg) && insn.inputs[0].oparg.IsSimpleReg())
  {
    X64Reg hreg = insn.inputs[0].oparg.GetSimpleReg();
    if (insn.output.guest_reg == IROp::DSP_REG_PROD_ALL)
    {
      X64Reg tmp1 = m_gpr.GetFreeXReg();
      set_long_prod(hreg, tmp1);
      m_gpr.PutXReg(tmp1);
    }
    else
    {
      if (greg >= DSP_REG_ST0 && greg <= DSP_REG_ST3)
      {
        X64Reg tmp1 = m_gpr.GetFreeXReg();
        X64Reg tmp2 = m_gpr.GetFreeXReg();
        X64Reg tmp3 = m_gpr.GetFreeXReg();
        dsp_reg_store_stack((StackRegister)(greg - DSP_REG_ST0), hreg, tmp1, tmp2, tmp3);
        m_gpr.PutXReg(tmp3);
        m_gpr.PutXReg(tmp2);
        m_gpr.PutXReg(tmp1);
      }
      else if (greg == DSP_REG_SR)
      {
        MOV(16, insn.SR, R(hreg));
      }
      else
      {
        m_gpr.WriteReg(greg, R(hreg));
      }
      if (!(reqs & NoACMExtend))
      {
        X64Reg tmp1 = m_gpr.GetFreeXReg();
        dsp_conditional_extend_accum(greg, insn.SR, tmp1);
        m_gpr.PutXReg(tmp1);
      }
    }
  }
  else if ((reqs & OpImmAny) && insn.inputs[0].oparg.IsImm())
  {
    if (insn.output.guest_reg == IROp::DSP_REG_PROD_ALL)
    {
      X64Reg tmp1 = m_gpr.GetFreeXReg();
      X64Reg tmp2 = m_gpr.GetFreeXReg();
      MOV(64, R(tmp2), insn.inputs[0].oparg);
      set_long_prod(tmp2, tmp1);
      m_gpr.PutXReg(tmp2);
      m_gpr.PutXReg(tmp1);
    }
    else
    {
      if (greg >= DSP_REG_ST0 && greg <= DSP_REG_ST3)
      {
        X64Reg tmp1 = m_gpr.GetFreeXReg();
        X64Reg tmp2 = m_gpr.GetFreeXReg();
        X64Reg tmp3 = m_gpr.GetFreeXReg();
        X64Reg tmp4 = m_gpr.GetFreeXReg();
        MOV(64, R(tmp4), insn.inputs[0].oparg);
        dsp_reg_store_stack((StackRegister)(greg - DSP_REG_ST0), tmp4, tmp1, tmp2, tmp3);
        m_gpr.PutXReg(tmp4);
        m_gpr.PutXReg(tmp3);
        m_gpr.PutXReg(tmp2);
        m_gpr.PutXReg(tmp1);
      }
      else if (greg == DSP_REG_SR)
      {
        MOV(16, insn.SR, insn.inputs[0].oparg);
      }
      else
      {
        m_gpr.WriteReg(greg, insn.inputs[0].oparg);
      }
      if (!(insn.emitter->output.reqs & NoACMExtend))
      {
        dsp_conditional_extend_accum_imm(greg, insn.inputs[0].oparg.AsImm64().Imm64(), insn.SR);
      }
    }
  }
}

struct DSPEmitterIR::IREmitInfo const DSPEmitterIR::StoreGuestOp = {
    "StoreGuestOp",
    NULL,
    0,
    0,
    0,
    0,  // todo: actually, we need a bit of SR. in some cases(...)
    false,
    {{OpAny64 | Clobbered}},
    {}};

}  // namespace x64
}  // namespace JITIR
}  // namespace DSP
