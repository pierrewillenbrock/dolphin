// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/DSP/JitIR/x64/DSPJitTables.h"

#include <array>

#include "Common/CommonTypes.h"
#include "Core/DSP/DSPTables.h"
#include "Core/DSP/JitIR/x64/DSPEmitterIR.h"

namespace DSP::JITIR::x64
{
struct JITOpInfo
{
  u16 opcode;
  u16 opcode_mask;
  JITIRDecodeFunction decode_function;
};

// clang-format off
const std::array<JITOpInfo, 214> s_opcodes =
{{
  {0x0000, 0xfffc, &DSPEmitterIR::nop},

  {0x0004, 0xfffc, &DSPEmitterIR::dar},
  {0x0008, 0xfffc, &DSPEmitterIR::iar},
  {0x000c, 0xfffc, &DSPEmitterIR::subarn},
  {0x0010, 0xfff0, &DSPEmitterIR::addarn},

  {0x0021, 0xffff, &DSPEmitterIR::halt},

  {0x02d0, 0xffff, &DSPEmitterIR::ret},
  {0x02d1, 0xffff, &DSPEmitterIR::ret},
  {0x02d2, 0xffff, &DSPEmitterIR::ret},
  {0x02d3, 0xffff, &DSPEmitterIR::ret},
  {0x02d4, 0xffff, &DSPEmitterIR::ret},
  {0x02d5, 0xffff, &DSPEmitterIR::ret},
  {0x02d6, 0xffff, &DSPEmitterIR::ret},
  {0x02d7, 0xffff, &DSPEmitterIR::ret},
  {0x02d8, 0xffff, &DSPEmitterIR::ret},
  {0x02d9, 0xffff, &DSPEmitterIR::ret},
  {0x02da, 0xffff, &DSPEmitterIR::ret},
  {0x02db, 0xffff, &DSPEmitterIR::ret},
  {0x02dc, 0xffff, &DSPEmitterIR::ret},
  {0x02dd, 0xffff, &DSPEmitterIR::ret},
  {0x02de, 0xffff, &DSPEmitterIR::ret},
  {0x02df, 0xffff, &DSPEmitterIR::ret},

  {0x02ff, 0xffff, &DSPEmitterIR::rti},

  {0x02b0, 0xffff, &DSPEmitterIR::call},
  {0x02b1, 0xffff, &DSPEmitterIR::call},
  {0x02b2, 0xffff, &DSPEmitterIR::call},
  {0x02b3, 0xffff, &DSPEmitterIR::call},
  {0x02b4, 0xffff, &DSPEmitterIR::call},
  {0x02b5, 0xffff, &DSPEmitterIR::call},
  {0x02b6, 0xffff, &DSPEmitterIR::call},
  {0x02b7, 0xffff, &DSPEmitterIR::call},
  {0x02b8, 0xffff, &DSPEmitterIR::call},
  {0x02b9, 0xffff, &DSPEmitterIR::call},
  {0x02ba, 0xffff, &DSPEmitterIR::call},
  {0x02bb, 0xffff, &DSPEmitterIR::call},
  {0x02bc, 0xffff, &DSPEmitterIR::call},
  {0x02bd, 0xffff, &DSPEmitterIR::call},
  {0x02be, 0xffff, &DSPEmitterIR::call},
  {0x02bf, 0xffff, &DSPEmitterIR::call},

  {0x0270, 0xffff, &DSPEmitterIR::ifcc},
  {0x0271, 0xffff, &DSPEmitterIR::ifcc},
  {0x0272, 0xffff, &DSPEmitterIR::ifcc},
  {0x0273, 0xffff, &DSPEmitterIR::ifcc},
  {0x0274, 0xffff, &DSPEmitterIR::ifcc},
  {0x0275, 0xffff, &DSPEmitterIR::ifcc},
  {0x0276, 0xffff, &DSPEmitterIR::ifcc},
  {0x0277, 0xffff, &DSPEmitterIR::ifcc},
  {0x0278, 0xffff, &DSPEmitterIR::ifcc},
  {0x0279, 0xffff, &DSPEmitterIR::ifcc},
  {0x027a, 0xffff, &DSPEmitterIR::ifcc},
  {0x027b, 0xffff, &DSPEmitterIR::ifcc},
  {0x027c, 0xffff, &DSPEmitterIR::ifcc},
  {0x027d, 0xffff, &DSPEmitterIR::ifcc},
  {0x027e, 0xffff, &DSPEmitterIR::ifcc},
  {0x027f, 0xffff, &DSPEmitterIR::ifcc},

  {0x0290, 0xffff, &DSPEmitterIR::jcc},
  {0x0291, 0xffff, &DSPEmitterIR::jcc},
  {0x0292, 0xffff, &DSPEmitterIR::jcc},
  {0x0293, 0xffff, &DSPEmitterIR::jcc},
  {0x0294, 0xffff, &DSPEmitterIR::jcc},
  {0x0295, 0xffff, &DSPEmitterIR::jcc},
  {0x0296, 0xffff, &DSPEmitterIR::jcc},
  {0x0297, 0xffff, &DSPEmitterIR::jcc},
  {0x0298, 0xffff, &DSPEmitterIR::jcc},
  {0x0299, 0xffff, &DSPEmitterIR::jcc},
  {0x029a, 0xffff, &DSPEmitterIR::jcc},
  {0x029b, 0xffff, &DSPEmitterIR::jcc},
  {0x029c, 0xffff, &DSPEmitterIR::jcc},
  {0x029d, 0xffff, &DSPEmitterIR::jcc},
  {0x029e, 0xffff, &DSPEmitterIR::jcc},
  {0x029f, 0xffff, &DSPEmitterIR::jcc},

  {0x1700, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x1701, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x1702, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x1703, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x1704, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x1705, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x1706, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x1707, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x1708, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x1709, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x170a, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x170b, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x170c, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x170d, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x170e, 0xff1f, &DSPEmitterIR::jmprcc},
  {0x170f, 0xff1f, &DSPEmitterIR::jmprcc},

  {0x1710, 0xff1f, &DSPEmitterIR::callr},
  {0x1711, 0xff1f, &DSPEmitterIR::callr},
  {0x1712, 0xff1f, &DSPEmitterIR::callr},
  {0x1713, 0xff1f, &DSPEmitterIR::callr},
  {0x1714, 0xff1f, &DSPEmitterIR::callr},
  {0x1715, 0xff1f, &DSPEmitterIR::callr},
  {0x1716, 0xff1f, &DSPEmitterIR::callr},
  {0x1717, 0xff1f, &DSPEmitterIR::callr},
  {0x1718, 0xff1f, &DSPEmitterIR::callr},
  {0x1719, 0xff1f, &DSPEmitterIR::callr},
  {0x171a, 0xff1f, &DSPEmitterIR::callr},
  {0x171b, 0xff1f, &DSPEmitterIR::callr},
  {0x171c, 0xff1f, &DSPEmitterIR::callr},
  {0x171d, 0xff1f, &DSPEmitterIR::callr},
  {0x171e, 0xff1f, &DSPEmitterIR::callr},
  {0x171f, 0xff1f, &DSPEmitterIR::callr},

  {0x1200, 0xff00, &DSPEmitterIR::sbclr},
  {0x1300, 0xff00, &DSPEmitterIR::sbset},

  {0x1400, 0xfec0, &DSPEmitterIR::lsl},
  {0x1440, 0xfec0, &DSPEmitterIR::lsr},
  {0x1480, 0xfec0, &DSPEmitterIR::asl},
  {0x14c0, 0xfec0, &DSPEmitterIR::asr},

  // These two were discovered by ector
  {0x02ca, 0xffff, &DSPEmitterIR::lsrn},
  {0x02cb, 0xffff, &DSPEmitterIR::asrn},

  {0x0080, 0xffe0, &DSPEmitterIR::lri},
  {0x00c0, 0xffe0, &DSPEmitterIR::lr},
  {0x00e0, 0xffe0, &DSPEmitterIR::sr},

  {0x1c00, 0xfc00, &DSPEmitterIR::mrr},

  {0x1600, 0xff00, &DSPEmitterIR::si},

  {0x0400, 0xfe00, &DSPEmitterIR::addis},
  {0x0600, 0xfe00, &DSPEmitterIR::cmpis},
  {0x0800, 0xf800, &DSPEmitterIR::lris},

  {0x0200, 0xfeff, &DSPEmitterIR::addi},
  {0x0220, 0xfeff, &DSPEmitterIR::xori},
  {0x0240, 0xfeff, &DSPEmitterIR::andi},
  {0x0260, 0xfeff, &DSPEmitterIR::ori},
  {0x0280, 0xfeff, &DSPEmitterIR::cmpi},

  {0x02a0, 0xfeff, &DSPEmitterIR::andf},
  {0x02c0, 0xfeff, &DSPEmitterIR::andcf},

  {0x0210, 0xfefc, &DSPEmitterIR::ilrr},
  {0x0214, 0xfefc, &DSPEmitterIR::ilrrd},
  {0x0218, 0xfefc, &DSPEmitterIR::ilrri},
  {0x021c, 0xfefc, &DSPEmitterIR::ilrrn},

  // LOOPS
  {0x0040, 0xffe0, &DSPEmitterIR::loop},
  {0x0060, 0xffe0, &DSPEmitterIR::bloop},
  {0x1000, 0xff00, &DSPEmitterIR::loopi},
  {0x1100, 0xff00, &DSPEmitterIR::bloopi},

  // Load and store value pointed by indexing reg and increment; LRR/SRR variants
  {0x1800, 0xff80, &DSPEmitterIR::lrr},
  {0x1880, 0xff80, &DSPEmitterIR::lrrd},
  {0x1900, 0xff80, &DSPEmitterIR::lrri},
  {0x1980, 0xff80, &DSPEmitterIR::lrrn},

  {0x1a00, 0xff80, &DSPEmitterIR::srr},
  {0x1a80, 0xff80, &DSPEmitterIR::srrd},
  {0x1b00, 0xff80, &DSPEmitterIR::srri},
  {0x1b80, 0xff80, &DSPEmitterIR::srrn},

  // 2
  {0x2000, 0xf800, &DSPEmitterIR::lrs},
  {0x2800, 0xf800, &DSPEmitterIR::srs},

  // opcodes that can be extended

  // 3 - main opcode defined by 9 bits, extension defined by last 7 bits!!
  {0x3000, 0xfc80, &DSPEmitterIR::xorr},
  {0x3400, 0xfc80, &DSPEmitterIR::andr},
  {0x3800, 0xfc80, &DSPEmitterIR::orr},
  {0x3c00, 0xfe80, &DSPEmitterIR::andc},
  {0x3e00, 0xfe80, &DSPEmitterIR::orc},
  {0x3080, 0xfe80, &DSPEmitterIR::xorc},
  {0x3280, 0xfe80, &DSPEmitterIR::notc},
  {0x3480, 0xfc80, &DSPEmitterIR::lsrnrx},
  {0x3880, 0xfc80, &DSPEmitterIR::asrnrx},
  {0x3c80, 0xfe80, &DSPEmitterIR::lsrnr},
  {0x3e80, 0xfe80, &DSPEmitterIR::asrnr},

  // 4
  {0x4000, 0xf800, &DSPEmitterIR::addr},
  {0x4800, 0xfc00, &DSPEmitterIR::addax},
  {0x4c00, 0xfe00, &DSPEmitterIR::add},
  {0x4e00, 0xfe00, &DSPEmitterIR::addp},

  // 5
  {0x5000, 0xf800, &DSPEmitterIR::subr},
  {0x5800, 0xfc00, &DSPEmitterIR::subax},
  {0x5c00, 0xfe00, &DSPEmitterIR::sub},
  {0x5e00, 0xfe00, &DSPEmitterIR::subp},

  // 6
  {0x6000, 0xf800, &DSPEmitterIR::movr},
  {0x6800, 0xfc00, &DSPEmitterIR::movax},
  {0x6c00, 0xfe00, &DSPEmitterIR::mov},
  {0x6e00, 0xfe00, &DSPEmitterIR::movp},

  // 7
  {0x7000, 0xfc00, &DSPEmitterIR::addaxl},
  {0x7400, 0xfe00, &DSPEmitterIR::incm},
  {0x7600, 0xfe00, &DSPEmitterIR::inc},
  {0x7800, 0xfe00, &DSPEmitterIR::decm},
  {0x7a00, 0xfe00, &DSPEmitterIR::dec},
  {0x7c00, 0xfe00, &DSPEmitterIR::neg},
  {0x7e00, 0xfe00, &DSPEmitterIR::movnp},

  // 8
  {0x8000, 0xf700, &DSPEmitterIR::nx},
  {0x8100, 0xf700, &DSPEmitterIR::clr},
  {0x8200, 0xff00, &DSPEmitterIR::cmp},
  {0x8300, 0xff00, &DSPEmitterIR::mulaxh},
  {0x8400, 0xff00, &DSPEmitterIR::clrp},
  {0x8500, 0xff00, &DSPEmitterIR::tstprod},
  {0x8600, 0xfe00, &DSPEmitterIR::tstaxh},
  {0x8a00, 0xff00, &DSPEmitterIR::srbith},
  {0x8b00, 0xff00, &DSPEmitterIR::srbith},
  {0x8c00, 0xff00, &DSPEmitterIR::srbith},
  {0x8d00, 0xff00, &DSPEmitterIR::srbith},
  {0x8e00, 0xff00, &DSPEmitterIR::srbith},
  {0x8f00, 0xff00, &DSPEmitterIR::srbith},

  // 9
  {0x9000, 0xf700, &DSPEmitterIR::mul},
  {0x9100, 0xf700, &DSPEmitterIR::asr16},
  {0x9200, 0xf600, &DSPEmitterIR::mulmvz},
  {0x9400, 0xf600, &DSPEmitterIR::mulac},
  {0x9600, 0xf600, &DSPEmitterIR::mulmv},

  // A-B
  {0xa000, 0xe700, &DSPEmitterIR::mulx},
  {0xa100, 0xf700, &DSPEmitterIR::abs},
  {0xa200, 0xe600, &DSPEmitterIR::mulxmvz},
  {0xa400, 0xe600, &DSPEmitterIR::mulxac},
  {0xa600, 0xe600, &DSPEmitterIR::mulxmv},
  {0xb100, 0xf700, &DSPEmitterIR::tst},

  // C-D
  {0xc000, 0xe700, &DSPEmitterIR::mulc},
  {0xc100, 0xe700, &DSPEmitterIR::cmpar},
  {0xc200, 0xe600, &DSPEmitterIR::mulcmvz},
  {0xc400, 0xe600, &DSPEmitterIR::mulcac},
  {0xc600, 0xe600, &DSPEmitterIR::mulcmv},

  // E
  {0xe000, 0xfc00, &DSPEmitterIR::maddx},
  {0xe400, 0xfc00, &DSPEmitterIR::msubx},
  {0xe800, 0xfc00, &DSPEmitterIR::maddc},
  {0xec00, 0xfc00, &DSPEmitterIR::msubc},

  // F
  {0xf000, 0xfe00, &DSPEmitterIR::lsl16},
  {0xf200, 0xfe00, &DSPEmitterIR::madd},
  {0xf400, 0xfe00, &DSPEmitterIR::lsr16},
  {0xf600, 0xfe00, &DSPEmitterIR::msub},
  {0xf800, 0xfc00, &DSPEmitterIR::addpaxz},
  {0xfc00, 0xfe00, &DSPEmitterIR::clrl},
  {0xfe00, 0xfe00, &DSPEmitterIR::movpz},
}};

constexpr std::array<JITOpInfo, 25> s_opcodes_ext
{{
  {0x0000, 0x00fc, &DSPEmitterIR::nop},

  {0x0004, 0x00fc, &DSPEmitterIR::dr},
  {0x0008, 0x00fc, &DSPEmitterIR::ir},
  {0x000c, 0x00fc, &DSPEmitterIR::nr},
  {0x0010, 0x00f0, &DSPEmitterIR::mv},

  {0x0020, 0x00e4, &DSPEmitterIR::s},
  {0x0024, 0x00e4, &DSPEmitterIR::sn},

  {0x0040, 0x00c4, &DSPEmitterIR::l},
  {0x0044, 0x00c4, &DSPEmitterIR::ln},

  {0x0080, 0x00ce, &DSPEmitterIR::ls},
  {0x0082, 0x00ce, &DSPEmitterIR::sl},
  {0x0084, 0x00ce, &DSPEmitterIR::lsn},
  {0x0086, 0x00ce, &DSPEmitterIR::sln},
  {0x0088, 0x00ce, &DSPEmitterIR::lsm},
  {0x008a, 0x00ce, &DSPEmitterIR::slm},
  {0x008c, 0x00ce, &DSPEmitterIR::lsnm},
  {0x008e, 0x00ce, &DSPEmitterIR::slnm},

  {0x00c3, 0x00cf, &DSPEmitterIR::ldax},
  {0x00c7, 0x00cf, &DSPEmitterIR::ldaxn},
  {0x00cb, 0x00cf, &DSPEmitterIR::ldaxm},
  {0x00cf, 0x00cf, &DSPEmitterIR::ldaxnm},

  {0x00c0, 0x00cc, &DSPEmitterIR::ld},
  {0x00c4, 0x00cc, &DSPEmitterIR::ldn},
  {0x00c8, 0x00cc, &DSPEmitterIR::ldm},
  {0x00cc, 0x00cc, &DSPEmitterIR::ldnm},
}};
// clang-format on

namespace
{
std::array<JITIRDecodeFunction, 65536> s_op_table;
std::array<JITIRDecodeFunction, 256> s_ext_op_table;
bool s_tables_initialized = false;
}  // Anonymous namespace

JITIRDecodeFunction GetOp(UDSPInstruction inst)
{
  return s_op_table[inst];
}

JITIRDecodeFunction GetExtOp(UDSPInstruction inst)
{
  const bool has_seven_bit_extension = (inst >> 12) == 0x3;

  if (has_seven_bit_extension)
    return s_ext_op_table[inst & 0x7F];

  return s_ext_op_table[inst & 0xFF];
}

void InitInstructionTables()
{
  if (s_tables_initialized)
    return;

  // ext op table
  for (size_t i = 0; i < s_ext_op_table.size(); i++)
  {
    s_ext_op_table[i] = nullptr;

    const auto iter = FindByOpcode(static_cast<UDSPInstruction>(i), s_opcodes_ext);
    if (iter == s_opcodes_ext.cend())
      continue;

    s_ext_op_table[i] = iter->decode_function;
  }

  // op table
  for (size_t i = 0; i < s_op_table.size(); i++)
  {
    s_op_table[i] = nullptr;

    const auto iter = FindByOpcode(static_cast<UDSPInstruction>(i), s_opcodes);
    if (iter == s_opcodes.cend())
      continue;

    s_op_table[i] = iter->decode_function;
  }

  s_tables_initialized = true;
}
}  // namespace DSP::JITIR::x64
