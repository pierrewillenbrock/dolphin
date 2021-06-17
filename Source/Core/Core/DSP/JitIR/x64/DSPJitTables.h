// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Core/DSP/DSPCommon.h"

namespace DSP::JITIR::x64
{
class DSPEmitterIR;

using JITIRDecodeFunction = void (DSPEmitterIR::*)(UDSPInstruction);

JITIRDecodeFunction GetOp(UDSPInstruction inst);
JITIRDecodeFunction GetExtOp(UDSPInstruction inst);
void InitInstructionTables();
}  // namespace DSP::JITIR::x64
