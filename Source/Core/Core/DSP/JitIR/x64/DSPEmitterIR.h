// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <array>
#include <cstddef>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Common/CommonTypes.h"
#include "Common/x64ABI.h"
#include "Common/x64Emitter.h"

#include "Core/DSP/DSPCommon.h"
#include "Core/DSP/Jit/DSPEmitterBase.h"
#include "Core/DSP/JitIR/x64/DSPJitRegCache.h"

class PointerWrap;

namespace DSP
{
enum class StackRegister;

namespace JITIR::x64
{
class DSPEmitterIR final : public JIT::DSPEmitter, public Gen::X64CodeBlock
{
public:
  explicit DSPEmitterIR(DSPCore& dsp);
  ~DSPEmitterIR() override;

  u16 RunCycles(u16 cycles) override;
  void DoState(PointerWrap& p) override;
  void ClearIRAM() override;

  // ******* Parsers *******
  // Ext commands
  void ir_l(UDSPInstruction opc);
  void ir_ln(UDSPInstruction opc);
  void ir_ls(UDSPInstruction opc);
  void ir_lsn(UDSPInstruction opc);
  void ir_lsm(UDSPInstruction opc);
  void ir_lsnm(UDSPInstruction opc);
  void ir_sl(UDSPInstruction opc);
  void ir_sln(UDSPInstruction opc);
  void ir_slm(UDSPInstruction opc);
  void ir_slnm(UDSPInstruction opc);
  void ir_s(UDSPInstruction opc);
  void ir_sn(UDSPInstruction opc);
  void ir_ld(UDSPInstruction opc);
  void ir_ldax(UDSPInstruction opc);
  void ir_ldn(UDSPInstruction opc);
  void ir_ldaxn(UDSPInstruction opc);
  void ir_ldm(UDSPInstruction opc);
  void ir_ldaxm(UDSPInstruction opc);
  void ir_ldnm(UDSPInstruction opc);
  void ir_ldaxnm(UDSPInstruction opc);
  void ir_mv(UDSPInstruction opc);
  void ir_dr(UDSPInstruction opc);
  void ir_ir(UDSPInstruction opc);
  void ir_nr(UDSPInstruction opc);
  void ir_nop(const UDSPInstruction opc) {}

  // Commands
  void ir_dar(UDSPInstruction opc);
  void ir_iar(UDSPInstruction opc);
  void ir_subarn(UDSPInstruction opc);
  void ir_addarn(UDSPInstruction opc);
  void ir_sbclr(UDSPInstruction opc);
  void ir_sbset(UDSPInstruction opc);
  void ir_srbith(UDSPInstruction opc);
  void ir_lri(UDSPInstruction opc);
  void ir_lris(UDSPInstruction opc);
  void ir_mrr(UDSPInstruction opc);
  // NX
  // 1000 -000 xxxx xxxx
  // No operation, but can be extended with extended opcode.
  // This opcode is supposed to do nothing - it's used if you want to use
  // an opcode extension but not do anything.
  void ir_nx(UDSPInstruction opc) {}

  // Branch
  void ir_jcc(UDSPInstruction opc);
  void ir_jmprcc(UDSPInstruction opc);
  void ir_call(UDSPInstruction opc);
  void ir_callr(UDSPInstruction opc);
  void ir_ifcc(UDSPInstruction opc);
  void ir_ret(UDSPInstruction opc);
  void ir_rti(UDSPInstruction opc);
  void ir_halt(UDSPInstruction opc);
  void ir_loop(UDSPInstruction opc);
  void ir_loopi(UDSPInstruction opc);
  void ir_bloop(UDSPInstruction opc);
  void ir_bloopi(UDSPInstruction opc);

  // Load/Store
  void ir_srs(UDSPInstruction opc);
  void ir_lrs(UDSPInstruction opc);
  void ir_lr(UDSPInstruction opc);
  void ir_sr(UDSPInstruction opc);
  void ir_si(UDSPInstruction opc);
  void ir_lrr(UDSPInstruction opc);
  void ir_lrrd(UDSPInstruction opc);
  void ir_lrri(UDSPInstruction opc);
  void ir_lrrn(UDSPInstruction opc);
  void ir_srr(UDSPInstruction opc);
  void ir_srrd(UDSPInstruction opc);
  void ir_srri(UDSPInstruction opc);
  void ir_srrn(UDSPInstruction opc);
  void ir_ilrr(UDSPInstruction opc);
  void ir_ilrrd(UDSPInstruction opc);
  void ir_ilrri(UDSPInstruction opc);
  void ir_ilrrn(UDSPInstruction opc);

  // Arithmetic
  void ir_clr(UDSPInstruction opc);
  void ir_clrl(UDSPInstruction opc);
  void ir_andcf(UDSPInstruction opc);
  void ir_andf(UDSPInstruction opc);
  void ir_tst(UDSPInstruction opc);
  void ir_tstaxh(UDSPInstruction opc);
  void ir_cmp(UDSPInstruction opc);
  void ir_cmpar(UDSPInstruction opc);
  void ir_cmpi(UDSPInstruction opc);
  void ir_cmpis(UDSPInstruction opc);
  void ir_xorr(UDSPInstruction opc);
  void ir_andr(UDSPInstruction opc);
  void ir_orr(UDSPInstruction opc);
  void ir_andc(UDSPInstruction opc);
  void ir_orc(UDSPInstruction opc);
  void ir_xorc(UDSPInstruction opc);
  void ir_notc(UDSPInstruction opc);
  void ir_xori(UDSPInstruction opc);
  void ir_andi(UDSPInstruction opc);
  void ir_ori(UDSPInstruction opc);
  void ir_addr(UDSPInstruction opc);
  void ir_addax(UDSPInstruction opc);
  void ir_add(UDSPInstruction opc);
  void ir_addp(UDSPInstruction opc);
  void ir_addaxl(UDSPInstruction opc);
  void ir_addi(UDSPInstruction opc);
  void ir_addis(UDSPInstruction opc);
  void ir_incm(UDSPInstruction opc);
  void ir_inc(UDSPInstruction opc);
  void ir_subr(UDSPInstruction opc);
  void ir_subax(UDSPInstruction opc);
  void ir_sub(UDSPInstruction opc);
  void ir_subp(UDSPInstruction opc);
  void ir_decm(UDSPInstruction opc);
  void ir_dec(UDSPInstruction opc);
  void ir_neg(UDSPInstruction opc);
  void ir_abs(UDSPInstruction opc);
  void ir_movr(UDSPInstruction opc);
  void ir_movax(UDSPInstruction opc);
  void ir_mov(UDSPInstruction opc);
  void ir_lsl16(UDSPInstruction opc);
  void ir_lsr16(UDSPInstruction opc);
  void ir_asr16(UDSPInstruction opc);
  void ir_lsl(UDSPInstruction opc);
  void ir_lsr(UDSPInstruction opc);
  void ir_asl(UDSPInstruction opc);
  void ir_asr(UDSPInstruction opc);
  void ir_lsrn(UDSPInstruction opc);
  void ir_asrn(UDSPInstruction opc);
  void ir_lsrnrx(UDSPInstruction opc);
  void ir_asrnrx(UDSPInstruction opc);
  void ir_lsrnr(UDSPInstruction opc);
  void ir_asrnr(UDSPInstruction opc);

  // Multipliers
  void ir_clrp(UDSPInstruction opc);
  void ir_tstprod(UDSPInstruction opc);
  void ir_movp(UDSPInstruction opc);
  void ir_movnp(UDSPInstruction opc);
  void ir_movpz(UDSPInstruction opc);
  void ir_addpaxz(UDSPInstruction opc);
  void ir_mulaxh(UDSPInstruction opc);
  void ir_mul(UDSPInstruction opc);
  void ir_mulac(UDSPInstruction opc);
  void ir_mulmv(UDSPInstruction opc);
  void ir_mulmvz(UDSPInstruction opc);
  void ir_mulx(UDSPInstruction opc);
  void ir_mulxac(UDSPInstruction opc);
  void ir_mulxmv(UDSPInstruction opc);
  void ir_mulxmvz(UDSPInstruction opc);
  void ir_mulc(UDSPInstruction opc);
  void ir_mulcac(UDSPInstruction opc);
  void ir_mulcmv(UDSPInstruction opc);
  void ir_mulcmvz(UDSPInstruction opc);
  void ir_maddx(UDSPInstruction opc);
  void ir_msubx(UDSPInstruction opc);
  void ir_maddc(UDSPInstruction opc);
  void ir_msubc(UDSPInstruction opc);
  void ir_madd(UDSPInstruction opc);
  void ir_msub(UDSPInstruction opc);

private:
  class IRInsn;
  typedef void (DSPEmitterIR::*IREmitter)(IRInsn const& insn);
  enum IROpReqs
  {
    OpRAX = 0x0001,     // i can take R(RAX) (usually used alone)
    OpRCX = 0x0002,     // i can take R(RCX) (usually used alone)
    OpRDX = 0x0004,     // i can take R(RDX) (usually used alone)
    OpAnyReg = 0x000f,  // i can take any R
    OpMem = 0x0010,     // i can take M(and derivatives)
    OpImm = 0x0020,     // i can take Imm32,16,8
    OpImm64 = 0x0040,   // i can take Imm64
    OpImmAny = 0x0060,  // i can take Imm64,32,16,8
    OpAny = 0x003f,     // i can take any OpArg(excluding Imm64)
    OpAny64 = 0x007f,   // i can take any OpArg(including Imm64)
    OpMask = 0x007f,
    ExtendNone = 0x0000,
    ExtendSign16 = 0x0100,  // if its in a reg, sign extend it for me
    ExtendZero16 = 0x0180,  // if its in a reg, zero extend it for me
    ExtendSign32 = 0x0200,
    ExtendZero32 = 0x0280,
    ExtendSign64 = 0x0300,  // these two can be reused(and made same
    ExtendZero64 = 0x0380,  // ExtendNone) if needed
    ExtendMask = 0x0380,
    Shift16 = 0x0400,       // if its in a reg, shift it for me
    Clobbered = 0x0800,     // if its in a reg, content gets destroyed
    SameAsOutput = 0x1000,  // this inputs host reg should be the same
    // as the output
    NoSaturate = 0x2000,   // don't saturate when retrieving the reg
    NoACMExtend = 0x4000,  // don't extend ACM on write
  };
  struct IROpInfo
  {
    int reqs;
  };
  enum
  {
    NUM_INPUTS = 3,
    NUM_TEMPS = 4
  };
  struct IREmitInfo
  {
    const char* name;
    IREmitter func;

    u16 needs_SR;          // bitmask of SR bits used by this instruction
    u16 modifies_SR;       // bitmask of SR bits (potentially) modifed
    u16 constant_mask_SR;  // bits that get assigned a constant value
    u16 constant_val_SR;   // the values assigned above
    bool modifies_PC;

    IROpInfo inputs[NUM_INPUTS];
    IROpInfo output;
    IROpInfo temps[NUM_TEMPS];
  };
  class IROp
  {
  public:
    enum
    {
      DSP_REG_ACC0_ALL = 0x20,
      DSP_REG_ACC1_ALL = 0x21,
      DSP_REG_AX0_ALL = 0x22,
      DSP_REG_AX1_ALL = 0x23,
      DSP_REG_PROD_ALL = 0x24,
    };

    // overall plan:
    // 1) do this per insn(done)
    // 2) drop redundant store_greg/load_greg insns, and unify
    // the vregs
    // other todo:
    // implement the ORing of parallel output to the same insn
    // handle fetching from the same memory segment
    // things that should be thought of:
    //* kill m_gpr
    //** what about insn.SR? => make it vreg?
    //* switch insn "tree" to real graph
    //* what to do with "active" vregs that need to be stored
    //  when a branch happens(CallOp, JmpOp, HandleLoop,
    //  CheckException, ... about everything calling
    //  WriteBranchExit)
    //* kill redundant StoreGuest/LoadGuest pairs, potentially with
    //  new converter ops in between
    //* kill noops with input==output
    //* do something sane when we run out of hregs for vregs
    //* avoid the above in deparallelizing
    //* cycle counter => possible solution: add to global var on
    //  leaving the BB. probably something for the "unsaved" part
    //  of g_dsp
    static IROp R(int reg)
    {
      IROp op = {0};
      op.guest_reg = reg;
      op.type = REG;
      return op;
    }
    static IROp Imm(int64_t imm)
    {
      IROp op = {0};
      op.type = IMM;
      op.imm = imm;
      return op;
    }
    static IROp Vreg(int num)
    {
      IROp op = {0};
      op.type = VREG;
      op.vreg = num;
      return op;
    }
    static IROp None()
    {
      IROp op = {0};
      op.guest_reg = -1;
      return op;
    }

    int guest_reg;
    int64_t imm;
    Gen::OpArg oparg;
    enum
    {
      INVALID = 0,
      REG,
      IMM,
      VREG,
    } type;
    int vreg;
  };
  class IRInsn
  {
  public:
    IREmitInfo const* emitter;
    IROp inputs[NUM_INPUTS];
    IROp output;
    IROp temps[NUM_TEMPS];  // need to check how many we actually need

    u16 needs_SR;  // these do the same as, and are ORed to EmitInfos
    u16 modifies_SR;
    u16 constant_mask_SR;
    u16 constant_val_SR;

    UDSPInstruction original;
    u16 addr;         // filled in by ir_add_insn helper
    u16 cycle_count;  // cycles done in this block

    Gen::OpArg SR;  // may be M, especially if the insn doesn't use it

    u16 later_needs_SR;  // SR bits needed here or later
    // bits derived from earlier code
    u16 const_SR;            // known constant value
    u16 modified_SR;         // unpredictably modified
    u16 value_SR;            // value for fixed SR bits
    bool const_regs[32];     // known constant value
    bool modified_regs[32];  // unpredictably modified
    u16 value_regs[32];      // value for fixed regs
    std::unordered_set<int> const_vregs;
    std::unordered_set<int> modified_vregs;
    std::unordered_map<int, u64> value_vregs;

    mutable Gen::FixupBranch branchTaken;

    std::unordered_set<int> live_vregs;
    std::unordered_set<int> first_refed_vregs;
    std::unordered_set<int> last_refed_vregs;
  };
  /*
    We want some kind of graph.

    Use case 1:
      Iterate over all IRInsns. any order. IRInsns can only be modified.
      (not created, destroyed or moved)

    Use case 2:
      Add a bunch of IRInsns during parsing. These IRInsns run in
      parallel.

    Use case 3:
      Split out the guest register accesses: All IRInsns are split
      into multiple IRInsns that run sequentally.

    Use case 4:
      Early on, some insns running in parallel may have side effects
            on each other. This relationship needs to stay recoverable.

    Use case 5:
      Branches have two execution paths, that get executed exclusively.
      The branch needs to be able to differentiate them, i.E. there is
      a branch A and a branch B.
      For now, these don't ever join back to the main flow, and are
      really simple, but that will change once inter-BB optimization
      is added.

    Use case 6:
      At some point, the whole graph should only represent
      register dependencies and branches.(todo: figure out how branches
      and the dependency graph interact.
      "start"-node for providing all dependencies on the code running
      earlier, multiple "end"-nodes depending on the results of
      the respective branches. what about the branch-points? just
      regular instructions, but with some way to determine which subset
      of the children is the positive part? two sets of children!)

    Idea:
      IRNode, base class.
              contains:
                set of IRNodes it depends on
                set of IRNodes depending on this
      IRInsnNode, derives from ILNone.
        contains:
                the IRInsn
      IRBranchNode, derives from IRInsnNode.
              contains:
                set of IRNodes depending on this in case of branch
      all of these are kept on a single vector

   */
  class IRNode
  {
  public:
    IRNode() : code(nullptr) {}
    virtual ~IRNode() {}

    void addNext(IRNode* node);
    void addPrev(IRNode* node);
    void removeNext(IRNode* node);
    void removePrev(IRNode* node);
    void insertBefore(IRNode* first, IRNode* last);
    void insertBefore(IRNode* node);
    void insertAfter(IRNode* first, IRNode* last);
    void insertAfter(IRNode* node);

    std::unordered_set<IRNode*> prev;
    std::unordered_set<IRNode*> next;
    const u8* code;
  };
  class IRInsnNode : public IRNode
  {
  public:
    IRInsn insn;
  };
  class IRBranchNode : public IRInsnNode
  {
  public:
    virtual void addNextOnBranch(IRNode* node);
    virtual void removeNext(IRNode* node);
    virtual void insertAfterOnBranch(IRNode* first, IRNode* last);
    void insertAfterOnBranch(IRNode* node);

    std::unordered_set<IRNode*> branch;
  };
  class IRBB
  {  // BasicBlocks
  public:
    IRBB()
        : code(nullptr), nextNonBranched(nullptr), nextBranched(nullptr), start_node(nullptr),
          end_node(nullptr)
    {
    }

    void setNextNonBranched(IRBB* bb)
    {
      next.insert(bb);
      bb->prev.insert(this);
      nextNonBranched = bb;
    }
    void setNextBranched(IRBB* bb)
    {
      next.insert(bb);
      bb->prev.insert(this);
      nextBranched = bb;
    }
    void replaceNextNonBranched(IRBB* bb)
    {
      next.erase(nextNonBranched);
      nextNonBranched->prev.erase(this);
      next.insert(bb);
      bb->prev.insert(this);
      nextNonBranched = bb;
    }
    void replaceNextBranched(IRBB* bb)
    {
      next.erase(nextBranched);
      nextBranched->prev.erase(this);
      next.insert(bb);
      bb->prev.insert(this);
      nextBranched = bb;
    }

    const u8* code;

    std::unordered_set<IRBB*> prev;
    std::unordered_set<IRBB*> next;
    std::unordered_set<IRNode*> nodes;
    IRBB* nextNonBranched;  // used for emitting
    IRBB* nextBranched;     // may be nullptr

    IRNode* start_node;  // possibly containing parallel sections
    IRNode* end_node;    // may(mostly will) be a branch insn, and the
    // only branch insn in this BB
  };
  class AddressInfo
  {
  public:
    AddressInfo() : node(nullptr), loop_begin(0xffff) {}

    IRNode* node;
    u16 loop_begin;
  };
  class VReg
  {
  public:
    int reqs;
    int64_t imm;
    bool isImm;
    Gen::OpArg oparg;
    bool active;
    std::unordered_set<int> parallel_live_vregs;
  };
  struct DSPEmitterParallelSectioninfo
  {
    DSPEmitterParallelSectioninfo(DSPEmitterIR::IRNode* aFirst) : first(aFirst) {}

    DSPEmitterIR::IRNode* first;
    DSPEmitterIR::IRNode* last;
    std::unordered_set<int> input_gregs;
    std::unordered_set<int> output_gregs;
  };

  using DSPCompiledCode = u32 (*)();
  using Block = const u8*;

  // The emitter emits calls to this function. It's present here
  // within the class itself to allow access to member variables.
  static void CompileCurrentIR(DSPEmitterIR& emitter);
  static u16 ReadIFXRegisterHelper(DSPEmitterIR& emitter, u16 address);
  static void WriteIFXRegisterHelper(DSPEmitterIR& emitter, u16 address, u16 value);

  void ClearIRAMandDSPJITCodespaceReset();

  void CompileStaticHelpers();
  void Compile(u16 start_addr);

  bool FlagsNeeded(u16 address) const;

  void WriteBranchExit(u16 execd_cycles, bool keepGpr = true);

  void IRReJitConditional(u8 cond, DSPEmitterIR::IRInsn const& insn,
                          void (DSPEmitterIR::*conditional_fn)(DSPEmitterIR::IRInsn const& insn,
                                                               Gen::X64Reg tmp1, Gen::X64Reg tmp2),
                          Gen::X64Reg tmp1, Gen::X64Reg tmp2);

  void irr_ret(IRInsn const& insn, Gen::X64Reg tmp1, Gen::X64Reg tmp2);
  void irr_jmp(IRInsn const& insn, Gen::X64Reg tmp1, Gen::X64Reg tmp2);
  void irr_call(IRInsn const& insn, Gen::X64Reg tmp1, Gen::X64Reg tmp2);

  void Update_SR_Register(Gen::X64Reg val, Gen::X64Reg tmp1);

  void get_long_prod(Gen::X64Reg long_prod, Gen::X64Reg tmp1);
  void set_long_prod(Gen::X64Reg host_sreg, Gen::X64Reg tmp1);
  void round_long(Gen::X64Reg long_acc);

  // CC helpers
  void Update_SR_Register64(Gen::X64Reg val, Gen::X64Reg tmp1);
  void Update_SR_Register64_Carry(Gen::X64Reg new_val, Gen::X64Reg old_val,
                                  Gen::X64Reg add_nsub_val, bool subtraction = false);
  void Update_SR_Register16(Gen::X64Reg val);
  void Update_SR_Register16_OverS32(Gen::X64Reg val, Gen::X64Reg acc);

  // Register helpers
  void setCompileSR(u16 bit);
  void clrCompileSR(u16 bit);
  void checkExceptions(u32 retval, u16 pc);

  // Memory helper functions
  void increment_addr_reg(Gen::X64Reg ar, Gen::X64Reg wr, Gen::X64Reg tmp1, Gen::X64Reg tmp4);
  void decrement_addr_reg(Gen::X64Reg ar_in, Gen::X64Reg wr, Gen::X64Reg ar_out, Gen::X64Reg tmp4);
  void increase_addr_reg(Gen::X64Reg ar_in, Gen::X64Reg wr, Gen::X64Reg ix, Gen::X64Reg ar_out);
  void decrease_addr_reg(Gen::X64Reg ar_in, Gen::X64Reg wr, Gen::X64Reg ix, Gen::X64Reg ar_out);
  void imem_read(Gen::X64Reg address, Gen::X64Reg host_dreg);
  void dmem_read(Gen::X64Reg address, Gen::X64Reg host_dreg);
  void dmem_read_imm(u16 addr, Gen::X64Reg host_dreg);
  void dmem_write(Gen::X64Reg value, Gen::X64Reg destaddr, Gen::X64Reg tmp1);
  void dmem_write_imm(u16 addr, Gen::X64Reg value, Gen::X64Reg tmp1);

  // Command helpers
  void dsp_reg_stack_push(StackRegister stack_reg, Gen::X64Reg tmp1, Gen::X64Reg tmp2,
                          Gen::X64Reg tmp3);
  void dsp_reg_stack_pop(StackRegister stack_reg, Gen::X64Reg tmp1, Gen::X64Reg tmp2,
                         Gen::X64Reg tmp3);
  void dsp_reg_store_stack(StackRegister stack_reg, Gen::X64Reg host_sreg, Gen::X64Reg tmp1,
                           Gen::X64Reg tmp2, Gen::X64Reg tmp3);
  void dsp_reg_load_stack(StackRegister stack_reg, Gen::X64Reg host_dreg, Gen::X64Reg tmp1,
                          Gen::X64Reg tmp2, Gen::X64Reg tmp3);
  void dsp_reg_store_stack_imm(StackRegister stack_reg, u16 val, Gen::X64Reg tmp1, Gen::X64Reg tmp2,
                               Gen::X64Reg tmp3);
  void dsp_op_write_reg(int reg, Gen::X64Reg host_sreg, Gen::X64Reg tmp1, Gen::X64Reg tmp2,
                        Gen::X64Reg tmp3);
  void dsp_op_write_reg_imm(int reg, u16 val, Gen::X64Reg tmp1, Gen::X64Reg tmp2, Gen::X64Reg tmp3);
  void dsp_conditional_extend_accum(int reg, Gen::X64Reg tmp1);
  void dsp_conditional_extend_accum_imm(int reg, u16 val);
  void dsp_op_read_reg_dont_saturate(int reg, Gen::X64Reg host_dreg, RegisterExtension extend,
                                     Gen::X64Reg tmp1, Gen::X64Reg tmp2,
                                     Gen::X64Reg tmp3);  // only used in the loop emitter
  void dsp_op_read_reg(int reg, Gen::X64Reg host_dreg, RegisterExtension extend, Gen::X64Reg tmp1,
                       Gen::X64Reg tmp2, Gen::X64Reg tmp3);

  // SDSP memory offset helpers
  Gen::OpArg M_SDSP_pc();
  Gen::OpArg M_SDSP_exceptions();
  Gen::OpArg M_SDSP_cr();
  Gen::OpArg M_SDSP_external_interrupt_waiting();
  Gen::OpArg M_SDSP_r_st(size_t index);
  Gen::OpArg M_SDSP_reg_stack_ptrs(size_t index);

  // Multiplier helpers
  void multiply(Gen::X64Reg dst, Gen::X64Reg mul);
  void multiply_uu(Gen::X64Reg dst, Gen::X64Reg mul);
  void multiply_us(Gen::X64Reg dst, Gen::X64Reg mul);

  // helper called by the ir_ emitters, all ops added for the same opc are
  //(at first) executed in parallel
  //(at first, because a later pass deparallelizes them, as good as
  // possible. if deparallelizing is impossible, it may need to emit
  // extra moves, and then deparallelize. we'll see)
  void ir_add_op(IRInsn insn);
  void ir_commit_parallel_nodes();

  static int ir_to_regcache_reg(int reg);

  void DecodeInstruction(UDSPInstruction inst);
  void deparallelize(IRNode* node);
  void deparallelize(IRBB* bb);

  void EmitBB(IRBB* bb);
  void EmitInsn(IRInsn& insn);

  static constexpr size_t MAX_BLOCKS = 0x10000;

  void clearNodeStorage();
  std::string dumpIRNodeInsn(DSPEmitterIR::IRInsn const& insn) const;
  void dumpIRNodes() const;
  IRNode* makeIRNode()
  {
    IRNode* n = new IRNode();
    m_node_storage.push_back(n);
    return n;
  }
  IRInsnNode* makeIRInsnNode(IRInsn const& insn)
  {
    IRInsnNode* n = new IRInsnNode();
    m_node_storage.push_back(n);
    n->insn = insn;
    return n;
  }
  IRBranchNode* makeIRBranchNode(IRInsn const& insn)
  {
    IRBranchNode* n = new IRBranchNode();
    m_node_storage.push_back(n);
    n->insn = insn;
    return n;
  }
  static void findInputOutput(IRNode* begin, IRNode* end, IRNode*& last,
                              std::unordered_set<int>& input_gregs,
                              std::unordered_set<int>& output_gregs);

  // ******* Emitters *******

  // Load store
  void
  iremit_Mov16Op(IRInsn const& insn);  // 16 => 16  //fits better here than in arith, no SR changes
  void iremit_Load16Op(IRInsn const& insn);
  void iremit_ILoad16Op(IRInsn const& insn);
  void iremit_Store16Op(IRInsn const& insn);
  void iremit_Load16SOp(IRInsn const& insn);
  void iremit_Store16SOp(IRInsn const& insn);
  // AR arith
  void iremit_AddAOp(IRInsn const& insn);
  void iremit_SubAOp(IRInsn const& insn);
  // acc arith
  void iremit_MovToAccOp(IRInsn const& insn);  // moves ax to acc, 32=>40, changes SR
  void
  iremit_MovROp(IRInsn const& insn);  // 16 => extended to 40. possibly already covered., changes SR
  void iremit_Mov40Op(IRInsn const& insn);
  void iremit_RoundOp(IRInsn const& insn);
  void iremit_AndCFOp(IRInsn const& insn);
  void iremit_AndFOp(IRInsn const& insn);
  void iremit_Tst40Op(IRInsn const& insn);
  void iremit_Tst16Op(IRInsn const& insn);
  void iremit_Cmp40Op(IRInsn const& insn);
  void iremit_Cmp16Op(IRInsn const& insn);
  void iremit_XorOp(IRInsn const& insn);
  void iremit_AndOp(IRInsn const& insn);
  void iremit_OrOp(IRInsn const& insn);
  void iremit_NotOp(IRInsn const& insn);
  void iremit_Add16Op(IRInsn const& insn);  // 0:16 fix point logic
  void iremit_Add32Op(IRInsn const& insn);  // 0:32 fix point
  void iremit_Add40Op(IRInsn const& insn);  // 8:32 fix point
  void iremit_AddPOp(IRInsn const& insn);   // prod is not precalculated
  void iremit_AddUOp(IRInsn const& insn);   // unsigned integer logic
  void iremit_Sub16Op(IRInsn const& insn);  // 0:16 fix point logic
  void iremit_Sub32Op(IRInsn const& insn);  // 0:32 fix point
  void iremit_Sub40Op(IRInsn const& insn);  // 8:32 fix point
  void iremit_SubPOp(IRInsn const& insn);   // prod is not precalculated
  void iremit_SubUOp(IRInsn const& insn);   // unsigned integer logic
  void iremit_NegOp(IRInsn const& insn);
  void iremit_AbsOp(IRInsn const& insn);
  void iremit_LslOp(IRInsn const& insn);
  void iremit_AslOp(IRInsn const& insn);
  // multiply
  void iremit_ClrPOp(IRInsn const& insn);
  void iremit_TstPOp(IRInsn const& insn);
  void iremit_MovPOp(IRInsn const& insn);
  void iremit_MovNPOp(IRInsn const& insn);
  void iremit_MovPZOp(IRInsn const& insn);  // similar to MovROp in the storing part. also, RoundOp.
  void iremit_AddPAxZOp(IRInsn const& insn);  // another one of the xxxZ series, similar to MovROp
  void iremit_MulOp(IRInsn const& insn);      // signed*signed
  void iremit_MulUUOp(IRInsn const& insn);    // unsigned*unsigned
  void iremit_MulSUOp(IRInsn const& insn);    // signed*unsigned
  void iremit_MulUSOp(IRInsn const& insn);    // unsigned*signed
  void iremit_MAddOp(IRInsn const& insn);     // signed*signed
  void iremit_MSubOp(IRInsn const& insn);     // signed*signed
  // SR bits
  void iremit_SBSetOp(IRInsn const& insn);
  void iremit_SBClrOp(IRInsn const& insn);
  // Branches
  void iremit_LoopOp(IRInsn const& insn);
  void iremit_HaltOp(IRInsn const& insn);
  void iremit_RetOp(IRInsn const& insn);
  void iremit_RtiOp(IRInsn const& insn);
  void iremit_JmpOp(IRInsn const& insn);
  void iremit_CallOp(IRInsn const& insn);

  void iremit_UpdatePCOp(IRInsn const& insn);
  void iremit_HandleLoopOp(IRInsn const& insn);
  void iremit_CheckExceptionsOp(IRInsn const& insn);

  // ******* Information Structs for Emitters *******

  static IREmitInfo const InvalidOp;
  // Load store
  static IREmitInfo const Mov16Op;  // 16 => 16  //fits better here than in arith, no SR changes
  static IREmitInfo const Load16Op;
  static IREmitInfo const ILoad16Op;
  static IREmitInfo const Store16Op;
  static IREmitInfo const Load16SOp;
  static IREmitInfo const Store16SOp;
  // AR arith
  static IREmitInfo const AddAOp;
  static IREmitInfo const SubAOp;
  // acc arith
  static IREmitInfo const MovToAccOp;  // moves ax to acc, 32=>40, changes SR
  static IREmitInfo const MovROp;  // 16 => extended to 40. possibly already covered., changes SR
  static IREmitInfo const Mov40Op;
  static IREmitInfo const RoundOp;
  static IREmitInfo const AndCFOp;
  static IREmitInfo const AndFOp;
  static IREmitInfo const Tst40Op;
  static IREmitInfo const Tst16Op;
  static IREmitInfo const Cmp40Op;
  static IREmitInfo const Cmp16Op;
  static IREmitInfo const XorOp;
  static IREmitInfo const AndOp;
  static IREmitInfo const OrOp;
  static IREmitInfo const NotOp;
  static IREmitInfo const Add16Op;  // 0:16 fix point logic
  static IREmitInfo const Add32Op;  // 0:32 fix point
  static IREmitInfo const Add40Op;  // 8:32 fix point
  static IREmitInfo const AddPOp;   // prod is not precalculated
  static IREmitInfo const AddUOp;   // unsigned integer logic
  static IREmitInfo const Sub16Op;  // 0:16 fix point logic
  static IREmitInfo const Sub32Op;  // 0:32 fix point
  static IREmitInfo const Sub40Op;  // 8:32 fix point
  static IREmitInfo const SubPOp;   // prod is not precalculated
  static IREmitInfo const SubUOp;   // unsigned integer logic
  static IREmitInfo const NegOp;
  static IREmitInfo const AbsOp;
  static IREmitInfo const LslOp;
  static IREmitInfo const AslOp;
  // multiply
  static IREmitInfo const ClrPOp;
  static IREmitInfo const TstPOp;
  static IREmitInfo const MovPOp;
  static IREmitInfo const MovNPOp;
  static IREmitInfo const MovPZOp;    // similar to MovROp in the storing part. also, RoundOp.
  static IREmitInfo const AddPAxZOp;  // another one of the xxxZ series, similar to MovROp
  static IREmitInfo const MulOp;      // signed*signed
  static IREmitInfo const MulUUOp;    // unsigned*unsigned
  static IREmitInfo const MulSUOp;    // signed*unsigned
  static IREmitInfo const MulUSOp;    // unsigned*signed
  static IREmitInfo const MAddOp;     // signed*signed
  static IREmitInfo const MSubOp;     // signed*signed
  // SR bits
  static IREmitInfo const SBSetOp;
  static IREmitInfo const SBClrOp;
  // Branches
  static IREmitInfo const LoopOp;
  static IREmitInfo const HaltOp;
  static IREmitInfo const RetOp;
  static IREmitInfo const RtiOp;
  static IREmitInfo const JmpOp;
  static IREmitInfo const CallOp;

  static IREmitInfo const UpdatePCOp;
  static IREmitInfo const HandleLoopOp;
  static IREmitInfo const CheckExceptionsOp;

  DSPJitIRRegCache m_gpr{*this};

  // during parsing: PC of the instruction being parsed
  u16 m_compile_pc;
  u16 m_start_address;

  std::vector<DSPCompiledCode> m_blocks;
  std::vector<u16> m_block_size;

  u16 m_cycles_left = 0;

  // CALL this to start the dispatcher
  const u8* m_enter_dispatcher;
  // JMP here to leave dispatcher
  const u8* m_return_dispatcher;
  // JMP here enters compiler
  const u8* m_stub_entry_point;
  const u8* m_int3_loop;
  // placeholder for when we need a branch, but don't actually branch
  Gen::FixupBranch m_unused_jump;

  // IRNode* and IRBB* get passed around and stored everywhere, so we need to keep their address
  // constant. Every IRNode and IRBB gets referenced from m_node_storage and m_bb_storage exactly
  // once so we can easily dispose of them again.
  std::vector<IRNode*> m_node_storage;
  std::vector<IRBB*> m_bb_storage;
  std::vector<std::pair<IRNode*, IRNode*>> m_parallel_nodes;
  IRBB* m_start_bb;
  IRBB* m_end_bb;
  std::list<IRBB*> m_branch_todo;
  std::unordered_map<u16, AddressInfo> m_addr_info;
  std::unordered_map<IRNode*, u16> m_node_addr_map;
  std::vector<VReg> m_vregs;

  DSPCore& m_dsp_core;
};

}  // namespace JITIR::x64
}  // namespace DSP
