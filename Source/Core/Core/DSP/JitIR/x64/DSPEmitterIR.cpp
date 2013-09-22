// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

#include <algorithm>
#include <cstddef>
#include <cstring>

#include "Common/Assert.h"
#include "Common/BitSet.h"
#include "Common/ChunkFile.h"
#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"

#include "Core/DSP/DSPAnalyzer.h"
#include "Core/DSP/DSPCore.h"
#include "Core/DSP/DSPHost.h"
#include "Core/DSP/DSPTables.h"
#include "Core/DSP/Interpreter/DSPIntTables.h"
#include "Core/DSP/Interpreter/DSPInterpreter.h"
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
  for (size_t i = 0; i < DSP_IRAM_SIZE; i++)
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

static void CheckExceptionsThunk(DSPCore& dsp)
{
  dsp.CheckExceptions();
}

// Must go out of block if exception is detected
void DSPEmitterIR::checkExceptions(u32 retval)
{
  // Check for interrupts and exceptions
  TEST(8, M_SDSP_exceptions(), Imm8(0xff));
  FixupBranch skipCheck = J_CC(CC_Z, true);

  MOV(16, M_SDSP_pc(), Imm16(m_compile_pc));

  DSPJitIRRegCache c(m_gpr);
  m_gpr.SaveRegs();
  ABI_CallFunctionP(CheckExceptionsThunk, &m_dsp_core);
  MOV(32, R(EAX), Imm32(retval));
  JMP(m_return_dispatcher, true);
  m_gpr.LoadRegs(false);
  m_gpr.FlushRegs(c, false);

  SetJumpTarget(skipCheck);
}

bool DSPEmitterIR::FlagsNeeded() const
{
  const auto& analyzer = m_dsp_core.DSPState().GetAnalyzer();

  return !analyzer.IsStartOfInstruction(m_compile_pc) || analyzer.IsUpdateSR(m_compile_pc);
}

static void FallbackThunk(Interpreter::Interpreter& interpreter, UDSPInstruction inst)
{
  (interpreter.*Interpreter::GetOp(inst))(inst);
}

void DSPEmitterIR::FallBackToInterpreter(UDSPInstruction inst)
{
  const DSPOPCTemplate* const op_template = GetOpTemplate(inst);

  if (op_template->reads_pc)
  {
    // Increment PC - we shouldn't need to do this for every instruction. only for branches and end
    // of block.
    // Fallbacks to interpreter need this for fetching immediate values

    MOV(16, M_SDSP_pc(), Imm16(m_compile_pc + 1));
  }

  // Fall back to interpreter
  const auto interpreter_function = Interpreter::GetOp(inst);

  m_gpr.PushRegs();
  ASSERT_MSG(DSPLLE, interpreter_function != nullptr, "No function for %04x", inst);
  ABI_CallFunctionPC(FallbackThunk, &m_dsp_core.GetInterpreter(), inst);
  m_gpr.PopRegs();
}

static void FallbackExtThunk(Interpreter::Interpreter& interpreter, UDSPInstruction inst)
{
  (interpreter.*Interpreter::GetExtOp(inst))(inst);
}

static void ApplyWriteBackLogThunk(Interpreter::Interpreter& interpreter)
{
  interpreter.ApplyWriteBackLog();
}

void DSPEmitterIR::EmitInstruction(UDSPInstruction inst)
{
  const DSPOPCTemplate* const op_template = GetOpTemplate(inst);
  bool ext_is_jit = false;

  // Call extended
  if (op_template->extended)
  {
    const auto jit_decode_function = GetExtOp(inst);

    if (jit_decode_function)
    {
      (this->*jit_decode_function)(inst);
      ext_is_jit = true;
    }
    else
    {
      // Fall back to interpreter
      m_gpr.PushRegs();
      ABI_CallFunctionPC(FallbackExtThunk, &m_dsp_core.GetInterpreter(), inst);
      m_gpr.PopRegs();
      INFO_LOG_FMT(DSPLLE, "Instruction not JITed(ext part): {:04x}", inst);
      ext_is_jit = false;
    }
  }

  // Main instruction
  const auto jit_decode_function = GetOp(inst);
  if (jit_decode_function)
  {
    (this->*jit_decode_function)(inst);
  }
  else
  {
    FallBackToInterpreter(inst);
    INFO_LOG_FMT(DSPLLE, "Instruction not JITed(main part): {:04x}", inst);
  }

  // Backlog
  if (op_template->extended)
  {
    if (!ext_is_jit)
    {
      // need to call the online cleanup function because
      // the writeBackLog gets populated at runtime
      m_gpr.PushRegs();
      ABI_CallFunctionP(ApplyWriteBackLogThunk, &m_dsp_core.GetInterpreter());
      m_gpr.PopRegs();
    }
    else
    {
      popExtValueToReg();
    }
  }
}

void DSPEmitterIR::Compile(u16 start_addr)
{
  // Remember the current block address for later
  m_start_address = start_addr;

  const u8* entryPoint = AlignCode16();

  m_gpr.LoadRegs();

  m_compile_pc = start_addr;
  bool fixup_pc = false;
  m_block_size[start_addr] = 0;

  auto& analyzer = m_dsp_core.DSPState().GetAnalyzer();
  while (m_compile_pc < start_addr + MAX_BLOCK_SIZE)
  {
    if (analyzer.IsCheckExceptions(m_compile_pc))
      checkExceptions(m_block_size[start_addr]);

    const UDSPInstruction inst = m_dsp_core.DSPState().ReadIMEM(m_compile_pc);
    const DSPOPCTemplate* opcode = GetOpTemplate(inst);

    EmitInstruction(inst);

    m_block_size[start_addr]++;
    m_compile_pc += opcode->size;

    fixup_pc = true;

    // Handle loop condition, only if current instruction was flagged as a loop destination
    // by the analyzer.
    if (analyzer.IsLoopEnd(static_cast<u16>(m_compile_pc - 1u)))
    {
      MOVZX(32, 16, EAX, M_SDSP_r_st(2));
      TEST(32, R(EAX), R(EAX));
      FixupBranch rLoopAddressExit = J_CC(CC_LE, true);

      MOVZX(32, 16, EAX, M_SDSP_r_st(3));
      TEST(32, R(EAX), R(EAX));
      FixupBranch rLoopCounterExit = J_CC(CC_LE, true);

      if (!opcode->branch)
      {
        // branch insns update the g_dsp.pc
        MOV(16, M_SDSP_pc(), Imm16(m_compile_pc));
      }

      // These functions branch and therefore only need to be called in the
      // end of each block and in this order
      DSPJitIRRegCache c(m_gpr);
      HandleLoop();
      m_gpr.SaveRegs();
      if (!Host::OnThread() && analyzer.IsIdleSkip(start_addr))
      {
        MOV(16, R(EAX), Imm16(DSP_IDLE_SKIP_CYCLES));
      }
      else
      {
        MOV(16, R(EAX), Imm16(m_block_size[start_addr]));
      }
      JMP(m_return_dispatcher, true);
      m_gpr.LoadRegs(false);
      m_gpr.FlushRegs(c, false);

      SetJumpTarget(rLoopAddressExit);
      SetJumpTarget(rLoopCounterExit);
    }

    if (opcode->branch)
    {
      // don't update g_dsp.pc -- the branch insn already did
      fixup_pc = false;
      if (opcode->uncond_branch)
      {
        break;
      }

      const auto jit_decode_function = GetOp(inst);
      if (!jit_decode_function)
      {
        // look at g_dsp.pc if we actually branched
        MOV(16, R(AX), M_SDSP_pc());
        CMP(16, R(AX), Imm16(m_compile_pc));
        FixupBranch rNoBranch = J_CC(CC_Z, true);

        DSPJitIRRegCache c(m_gpr);
        // don't update g_dsp.pc -- the branch insn already did
        m_gpr.SaveRegs();
        if (!Host::OnThread() && analyzer.IsIdleSkip(start_addr))
        {
          MOV(16, R(EAX), Imm16(DSP_IDLE_SKIP_CYCLES));
        }
        else
        {
          MOV(16, R(EAX), Imm16(m_block_size[start_addr]));
        }
        JMP(m_return_dispatcher, true);
        m_gpr.LoadRegs(false);
        m_gpr.FlushRegs(c, false);

        SetJumpTarget(rNoBranch);
      }
    }

    // End the block if we're before an idle skip address
    if (analyzer.IsIdleSkip(m_compile_pc))
    {
      break;
    }
  }

  if (fixup_pc)
  {
    MOV(16, M_SDSP_pc(), Imm16(m_compile_pc));
  }

  m_blocks[start_addr] = (DSPCompiledCode)entryPoint;

  if (m_block_size[start_addr] == 0)
  {
    // just a safeguard, should never happen anymore.
    // if it does we might get stuck over in RunForCycles.
    ERROR_LOG_FMT(DSPLLE, "Block at {:#06x} has zero size", start_addr);
    m_block_size[start_addr] = 1;
  }

  m_gpr.SaveRegs();
  if (!Host::OnThread() && analyzer.IsIdleSkip(start_addr))
  {
    MOV(16, R(EAX), Imm16(DSP_IDLE_SKIP_CYCLES));
  }
  else
  {
    MOV(16, R(EAX), Imm16(m_block_size[start_addr]));
  }
  JMP(m_return_dispatcher, true);
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

}  // namespace DSP::JITIR::x64
