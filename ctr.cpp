// Copyright 2022 Tenstorrent Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "DecodedInst.hpp"
#include "Hart.hpp"

using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execSctrclr(const DecodedInst* di)
{
  // smctr.adoc L17: Smctr alone ("encompasses all newly added CSRs, instructions, and
  // behavior modifications for a hart across all privilege levels") already includes
  // SCTRCLR -- there is no separate machine-level clear instruction, so a Smctr-only
  // hart must still be able to execute this one. Same defect-class fix as
  // CsRegs::updateCtrImplemented (sctrctl/sctrdepth/sctrstatus/CTR entries): either
  // extension alone suffices, not Ssctr exclusively.
  if (not (isRvsmctr() or isRvssctr()))
    {
      illegalInst(di);
      return;
    }

  using PM = PrivilegeMode;

  if (privMode_ != PM::Machine)
    {
      // smctr.adoc State Enable Access Control: "When mstateen0.CTR=0 and the privilege
      // mode is less privileged than M-mode, ... Execution of the SCTRCLR instruction"
      // raises an illegal-instruction exception. This mstateen0-only gate applies to
      // every mode below M (S, U, VS, VU alike) and takes priority over, and can
      // override, the ordinary U/VU-mode privilege-floor verdict below (no virtual-
      // instruction carve-out is given for VU-mode here). Deliberately probe
      // PM::Supervisor/non-virtual (the same "would HS-mode be allowed" idiom
      // Hart::checkCsrAccess already uses for its own hs-qualified decisions) rather
      // than privMode_/virtMode_ directly, so this reflects mstateen0.CTR alone and
      // not any sstateen0/hstateen0 layering that only matters to the CSR side of CTR.
      if (not csRegs_.isStateEnabled(CsrNumber::SCTRSTATUS, PM::Supervisor, false))
        {
          illegalInst(di);
          return;
        }
    }

  // Ordinary privilege floor: SCTRCLR is a supervisor-or-higher instruction.
  if (privMode_ == PM::User)
    {
      if (virtMode_)
        virtualInst(di);
      else
        illegalInst(di);
      return;
    }

  // mstateen0.CTR is already confirmed 1 above (for any non-M mode, including here).
  // hstateen0.CTR further restricts VS-mode specifically: "When mstateen0.CTR=1 and
  // hstateen0.CTR=0, ... VS-mode execution of SCTRCLR raise[s] a virtual-instruction
  // exception" (smctr.adoc State Enable Access Control). ctrStateEnabled() ANDs both
  // bits together for Supervisor/virtMode_, so with mstateen0.CTR already known to be 1,
  // its failure here can only mean hstateen0.CTR=0.
  if (virtMode_ and not ctrStateEnabled())
    {
      virtualInst(di);
      return;
    }

  csRegs_.clearCtrEntries();
  csRegs_.resetCtrCycleCounter();
}


template <typename URV>
unsigned
Hart<URV>::ctrTransferType(const DecodedInst& di) const
{
  // Caller (maybeRecordCtrTransfer) has already confirmed di.isBranch(). Mapping per
  // smctr.adoc L580-595/L605-624; reuses the already-implemented DecodedInst
  // classification helpers rather than re-deriving rd/rs1 link-register logic here.
  if (di.isConditionalBranch())
    return lastBranchTaken_ ? 5 /* Taken branch */ : 4 /* Not-taken branch */;

  if (di.isCoroutineSwap())
    return 12;  // Co-routine swap.

  if (di.isCall())
    return di.isBranchToRegister() ? 8 /* Indirect call */ : 9 /* Direct call */;

  if (di.isReturn())
    return 13;  // Function return.

  // Ordinary unconditional jump (not call/return/co-routine-swap).
  if (di.isBranchToRegister())
    return (di.op0() == 0) ? 10 /* Indirect jump w/o linkage */
                            : 14 /* Other indirect jump w/ linkage */;

  // TODO (Zcmt/Zcmp, smctr.adoc Table 9 L608-L622): CM.JALT and CM.JT are Zcmt opcodes;
  // CM.POPRET/CM.POPRETZ are Zcmp opcodes. Neither extension is implemented in swerv-iss
  // today (no RvExtension::Zcmt/Zcmp, no InstId/InstEntry/Decoder.cpp registration), and
  // neither is a Smctr/Ssctr prerequisite, so this switch correctly never sees those
  // InstIds yet -- the decoder cannot produce them. When Zcmt/Zcmp support is eventually
  // added to swerv-iss, extend this function (gated on isRvzcmt()/isRvzcmp() being
  // implemented and enabled) with: CM.JALT -> type 9 (Direct call), CM.JT -> type 11
  // (Direct jump without linkage), CM.POPRET/CM.POPRETZ -> type 13 (Function return).
  return (di.op0() == 0) ? 11 /* Direct jump w/o linkage */
                          : 15 /* Other direct jump w/ linkage */;
}


template <typename URV>
bool
Hart<URV>::ctrRecordingEnabled(PrivilegeMode pm, bool virt) const
{
  // smctr.adoc L495: mctrctl.M, sctrctl.S/U, and vsctrctl.{S,U} are used to determine
  // whether the source/target mode is enabled -- there is no single register that
  // defines all of M/HS/U/VS/VU's enable bits. Smctr and Ssctr are independently
  // toggled (CsRegs::enableSmctr/enableSsctr, via CsRegs::updateCtrImplemented), and
  // sctrctl is tied storage shared with mctrctl (CsRegs.cpp:
  // "sctrctl->tie(mctrctl->valuePtr_)"), so EITHER extension alone must make the S/U
  // (non-virtual) enable bits readable. Try SCTRCTL first, then fall back to MCTRCTL --
  // same idiom as ctrTypeInhibited/ctrRasEmulationActiveForFilter below. VS/VU reads
  // vsctrctl, which is independent (non-tied) storage, so no analogous fallback applies
  // there.
  using PM = PrivilegeMode;

  URV val = 0;
  bool ok = false;
  if (pm == PM::Machine)
    ok = peekCsr(CsrNumber::MCTRCTL, val);
  else if (pm == PM::Supervisor or pm == PM::User)
    {
      if (virt)
        ok = peekCsr(CsrNumber::VSCTRCTL, val);
      else
        ok = peekCsr(CsrNumber::SCTRCTL, val) or peekCsr(CsrNumber::MCTRCTL, val);
    }
  else
    return false;  // PM::Reserved: never a real mode.

  if (not ok)
    return false;

  MctrctlFields f{uint64_t(val)};
  if (pm == PM::Machine)
    return f.bits_.M != 0;
  if (pm == PM::Supervisor)
    return f.bits_.S != 0;
  return f.bits_.U != 0;
}


template <typename URV>
bool
Hart<URV>::ctrRasEmulationActive(PrivilegeMode pm, bool virt) const
{
  // smctr.adoc L689: mirrors ctrRecordingEnabled's exact CSR-selection rule (M-mode reads
  // MCTRCTL, S/U non-virtual reads SCTRCTL falling back to MCTRCTL, S/U virtual reads
  // VSCTRCTL), but extracts the RASEMU bit (bit 7) instead of the per-mode enable bit.
  // See ctrRecordingEnabled's comment for why the SCTRCTL->MCTRCTL fallback is required.
  using PM = PrivilegeMode;

  URV val = 0;
  bool ok = false;
  if (pm == PM::Machine)
    ok = peekCsr(CsrNumber::MCTRCTL, val);
  else if (pm == PM::Supervisor or pm == PM::User)
    {
      if (virt)
        ok = peekCsr(CsrNumber::VSCTRCTL, val);
      else
        ok = peekCsr(CsrNumber::SCTRCTL, val) or peekCsr(CsrNumber::MCTRCTL, val);
    }
  else
    return false;  // PM::Reserved: never a real mode.

  if (not ok)
    return false;

  MctrctlFields f{uint64_t(val)};
  return f.bits_.RASEMU != 0;
}


template <typename URV>
bool
Hart<URV>::ctrRasEmulationActiveForFilter(bool virt) const
{
  // smctr.adoc L498 (same register-selection rule ctrTypeInhibited uses): RASEMU is one of
  // the "all other xctrctl fields" the note assigns to vsctrctl whenever either side of a
  // trap/trap-return transition touches VS/VU (virt == filterVirt, the OR of source/target
  // virt), else to mctrctl/sctrctl's shared (tied) storage. Smctr and Ssctr are
  // independently toggled (CsRegs::enableSmctr/enableSsctr set MCTRCTL's and SCTRCTL's
  // implemented_ flags separately), so EITHER extension alone must make this field
  // readable: a legal Ssctr-only configuration (Smctr disabled) leaves MCTRCTL
  // unimplemented while the filter bits remain live and readable through SCTRCTL's
  // narrower mask, and a legal Smctr-only configuration (Ssctr disabled) leaves SCTRCTL
  // unimplemented while the same tied bits remain live and readable through MCTRCTL's
  // mask instead (verified identical: both masks cover the same filter-bit bank). Try
  // SCTRCTL first, then fall back to MCTRCTL; only when neither view is readable (neither
  // extension enabled) does the defensive "not RAS-emulating" default apply.
  if (not virt)
    {
      URV val = 0;
      if (peekCsr(CsrNumber::SCTRCTL, val) or peekCsr(CsrNumber::MCTRCTL, val))
        return MctrctlFields{uint64_t(val)}.bits_.RASEMU != 0;
      return false;  // Defensive: neither view is readable.
    }

  URV val = 0;
  if (not peekCsr(CsrNumber::VSCTRCTL, val))
    return false;

  MctrctlFields f{uint64_t(val)};
  return f.bits_.RASEMU != 0;
}


template <typename URV>
bool
Hart<URV>::ctrTypeInhibited(unsigned type, bool virt) const
{
  // smctr.adoc L498: for all fields other than the per-mode enable/external-trap-enable/
  // freeze-arming bits, the value in vsctrctl is used whenever the currently-relevant
  // mode is VS/VU; otherwise mctrctl/sctrctl's shared (tied) storage is used. Smctr and
  // Ssctr are independently toggled (CsRegs::enableSmctr/enableSsctr set MCTRCTL's and
  // SCTRCTL's implemented_ flags separately), so EITHER extension alone must make this
  // field readable: a legal Ssctr-only configuration (Smctr disabled) leaves MCTRCTL
  // unimplemented while the filter bits remain live and readable through SCTRCTL's
  // narrower mask, and a legal Smctr-only configuration (Ssctr disabled) leaves SCTRCTL
  // unimplemented while the same tied bits remain live and readable through MCTRCTL's
  // mask instead (verified identical: both masks cover the same filter-bit bank). Try
  // SCTRCTL first, then fall back to MCTRCTL; only when neither view is readable (neither
  // extension enabled) does the defensive "treat unreadable CSR as inhibiting everything"
  // default apply.
  URV val = 0;
  bool ok = virt ? peekCsr(CsrNumber::VSCTRCTL, val)
                 : (peekCsr(CsrNumber::SCTRCTL, val) or peekCsr(CsrNumber::MCTRCTL, val));
  if (not ok)
    return true;  // Defensive: treat an unreadable CSR as inhibiting everything.

  MctrctlFields f{uint64_t(val)};

  switch (type)
    {
    case 1:  return f.bits_.EXCINH  != 0;  // Exception (used by ctrRecordTrapEntry).
    case 2:  return f.bits_.INTRINH != 0;  // Interrupt (used by ctrRecordTrapEntry).
    case 3:  return f.bits_.TRETINH != 0;  // Trap return (used by ctrRecordTrapReturn).
    case 4:  return f.bits_.NTBREN  == 0;  // Not-taken branch: inverted polarity.
    case 5:  return f.bits_.TKBRINH    != 0;
    case 8:  return f.bits_.INDCALLINH != 0;
    case 9:  return f.bits_.DIRCALLINH != 0;
    case 10: return f.bits_.INDJMPINH  != 0;
    case 11: return f.bits_.DIRJMPINH  != 0;
    case 12: return f.bits_.CORSWAPINH != 0;
    case 13: return f.bits_.RETINH     != 0;
    case 14: return f.bits_.INDLJMPINH != 0;
    case 15: return f.bits_.DIRLJMPINH != 0;
    default: return true;  // Reserved/undefined type: defensively inhibit.
    }
}


template <typename URV>
bool
Hart<URV>::ctrFrozen() const
{
  // sctrstatus is not per-mode: a single FROZEN bit (bit 31) applies uniformly
  // (smctr.adoc L251/L728).
  URV val = 0;
  if (not peekCsr(CsrNumber::SCTRSTATUS, val))
    return false;
  return ((val >> 31) & 1) != 0;
}


template <typename URV>
bool
Hart<URV>::ctrFreezeArmed(PrivilegeMode targetMode, bool targetVirt, bool lcofi) const
{
  // smctr.adoc L720-L752: the freeze-arming bit lives in the xctrctl register OWNED by
  // the trap's TARGET: mctrctl for an M-mode target, sctrctl for an HS-mode target, and
  // vsctrctl's OWN (independent) copy for a VS-mode target -- never sctrctl's for VS.
  // For the HS target, sctrctl is tied storage shared with mctrctl, so (as in
  // ctrRecordingEnabled above) try SCTRCTL first, then fall back to MCTRCTL -- a legal
  // Smctr-only configuration leaves SCTRCTL unimplemented while the same tied
  // BPFRZ/LCOFIFRZ bits remain live and readable through MCTRCTL's mask instead.
  using PM = PrivilegeMode;

  URV val = 0;
  bool ok = false;
  if (targetMode == PM::Machine)
    ok = peekCsr(CsrNumber::MCTRCTL, val);
  else if (targetMode == PM::Supervisor)
    {
      if (targetVirt)
        ok = peekCsr(CsrNumber::VSCTRCTL, val);
      else
        ok = peekCsr(CsrNumber::SCTRCTL, val) or peekCsr(CsrNumber::MCTRCTL, val);
    }
  else
    return false;  // Freeze-arming is only defined for M/HS/VS trap targets.

  if (not ok)
    return false;

  // VSCTRCTL's low-bit layout differs from MCTRCTL/SCTRCTL's (no M/MTE bits), but
  // BPFRZ (bit 11) and LCOFIFRZ (bit 12) sit at the identical bit positions in both
  // layouts (smctr.adoc L132-140 wavedrom), so MctrctlFields is valid for either
  // register for just these two bits.
  MctrctlFields f{uint64_t(val)};
  return lcofi ? f.bits_.LCOFIFRZ != 0 : f.bits_.BPFRZ != 0;
}


template <typename URV>
bool
Hart<URV>::ctrQualifies(unsigned type, PrivilegeMode pm, bool virt) const
{
  // Debug Mode unconditionally inhibits recording (smctr.adoc L472). Re-checked here
  // (in addition to maybeRecordCtrTransfer's own check) so this function -- currently
  // called only from maybeRecordCtrTransfer -- is correct on its own regardless of
  // caller. Trap/trap-return recording (ctrRecordTrapEntry/ctrRecordTrapReturn) does NOT
  // call this function; it re-implements its own Debug-Mode/type-inhibit/FROZEN checks
  // directly, since its qualification rules differ (separate source/target enable
  // states, RAS-emulation, external-trap opt-in).
  if (debugMode_)
    return false;

  if (not ctrRecordingEnabled(pm, virt))
    return false;

  if (ctrTypeInhibited(type, virt))
    return false;

  if (ctrFrozen())
    return false;

  return true;
}


template <typename URV>
void
Hart<URV>::advanceCtrCycleCounter()
{
  if (not (isRvsmctr() or isRvssctr()))
    return;

  // Debug Mode is a hard override (smctr.adoc L472), mirroring ctrQualifies/
  // maybeRecordCtrTransfer: CTR recording is categorically inactive in Debug Mode, so
  // CtrCycleCounter -- which only counts cycles while CTR is active -- must not advance
  // then either.
  if (debugMode_)
    return;

  if (ctrRecordingEnabled(privMode_, virtMode_) and not ctrFrozen())
    csRegs_.incrementCtrCycleCounter();
}


template <typename URV>
void
Hart<URV>::recordCtrTransfer(unsigned type, URV sourcePc, URV targetPc, bool misp, bool rasemuCall)
{
  // Thin wrapper kept as Hart's own stable CTR-recording entry point: Hart never
  // reaches into CsRegs's private ctrEntries_/WRPTR backing store directly (mirrors
  // how Hart already goes through csRegs_.write()/poke()/peek() rather than touching
  // regs_ itself). ctrRecordTrapEntry/ctrRecordTrapReturn (called from
  // Hart::initiateTrap/execMret/execSret/execMnret) also call this exact function
  // directly, independent of maybeRecordCtrTransfer's ordinary-instruction path.
  //
  // rasemuCall (smctr.adoc L720-723): while xctrctl.RASEMU=1, a return that pops a call
  // entry would need to add the popped entry's CC back into CtrCycleCounter to keep the
  // elapsed-cycle count meaningful across the still-open parent call's interval. Rather
  // than implement that restoration in CsRegs::ctrPopReturn (which only invalidates the
  // entry and never reads its cc/ccv back out), this build takes the spec's explicitly
  // sanctioned simpler alternative: "an implementation may opt to record CCV=0 for all
  // calls... when RASEMU=1". maybeRecordCtrTransfer's RASEMU call-push site passes true
  // here so CsRegs::ctrPushEntry forces ccv=false for every RAS-emulation call record,
  // instead of the ordinary ctrCcv_-derived value it uses for non-RASEMU pushes.
  csRegs_.ctrPushEntry(uint64_t(sourcePc), uint64_t(targetPc), type, misp, rasemuCall);
}


template <typename URV>
bool
Hart<URV>::ctrExternalTrapEnabled(PrivilegeMode source, bool sourceVirt,
                                   PrivilegeMode target, bool targetVirt) const
{
  // smctr.adoc L519-532 (External Trap Enable Requirements table): eight explicit
  // (source,target) pairs, each requiring the target mode's own xTE bit plus every
  // strictly-intervening mode's xTE bit. Written as eight explicit cases (rather than one
  // generic total-order "intervening modes" computation) because the privilege ordering is
  // not a single chain here -- it is two chains (non-virtual U<S<M and virtual VU<VS) that
  // only re-join at M, and the spec's own table (e.g. VU->HS requiring both sctrctl.STE and
  // vsctrctl.STE) reflects that VS acts as "intervening" for a VU source even though VS is
  // never itself a trap target from VU in the non-virtual chain's sense -- mirroring the
  // table directly avoids silently inventing a wrong generic rule for this asymmetry.
  using PM = PrivilegeMode;
  auto isM  = [](PM pm, bool v) { return pm == PM::Machine and not v; };
  auto isHs = [](PM pm, bool v) { return pm == PM::Supervisor and not v; };
  auto isU  = [](PM pm, bool v) { return pm == PM::User and not v; };
  auto isVs = [](PM pm, bool v) { return pm == PM::Supervisor and v; };
  auto isVu = [](PM pm, bool v) { return pm == PM::User and v; };

  bool needMte = false, needSte = false, needVsSte = false;

  // U and VS sources need the same enables for the same target, so each pair of rows
  // below with identical outcomes is one branch.
  bool uOrVs = isU(source, sourceVirt) or isVs(source, sourceVirt);

  if (uOrVs and isHs(target, targetVirt))
    needSte = true;                                    // U -> S, VS -> HS: sctrctl.STE
  else if (uOrVs and isM(target, targetVirt))
    { needMte = true; needSte = true; }                // U -> M, VS -> M: mctrctl.MTE, sctrctl.STE
  else if (isHs(source, sourceVirt) and isM(target, targetVirt))
    needMte = true;                                    // S -> M:  mctrctl.MTE
  else if (isVu(source, sourceVirt) and isVs(target, targetVirt))
    needVsSte = true;                                  // VU -> VS: vsctrctl.STE
  else if (isVu(source, sourceVirt) and isHs(target, targetVirt))
    { needSte = true; needVsSte = true; }              // VU -> HS: sctrctl.STE, vsctrctl.STE
  else if (isVu(source, sourceVirt) and isM(target, targetVirt))
    { needMte = true; needSte = true; needVsSte = true; } // VU -> M
  else
    return false;  // Not one of the eight defined pairs -- not architecturally reachable.

  if (needMte)
    {
      URV val = 0;
      if (not peekCsr(CsrNumber::MCTRCTL, val))
        return false;
      if (MctrctlFields{uint64_t(val)}.bits_.MTE == 0)
        return false;
    }

  if (needSte)
    {
      // sctrctl is tied storage shared with mctrctl; a legal Smctr-only configuration
      // leaves SCTRCTL unimplemented while the same tied STE bit remains live and
      // readable through MCTRCTL's mask instead -- try SCTRCTL first, then fall back to
      // MCTRCTL, same idiom as ctrRecordingEnabled/ctrFreezeArmed above.
      URV val = 0;
      if (not (peekCsr(CsrNumber::SCTRCTL, val) or peekCsr(CsrNumber::MCTRCTL, val)))
        return false;
      if (MctrctlFields{uint64_t(val)}.bits_.STE == 0)
        return false;
    }

  if (needVsSte)
    {
      URV val = 0;
      if (not peekCsr(CsrNumber::VSCTRCTL, val))
        return false;
      if (MctrctlFields{uint64_t(val)}.bits_.STE == 0)
        return false;
    }

  return true;
}


template <typename URV>
void
Hart<URV>::ctrRecordTrapEntry(bool interrupt, PrivilegeMode origMode, bool origVirt,
                               PrivilegeMode nextMode, bool nextVirt,
                               URV sourcePc, URV targetPc)
{
  // Debug Mode unconditionally inhibits recording (smctr.adoc L472), re-checked here so
  // this function is correct whether or not its caller (Hart::initiateTrap) has already
  // excluded Debug Mode.
  if (debugMode_)
    return;

  bool srcEn = ctrRecordingEnabled(origMode, origVirt);
  bool tgtEn = ctrRecordingEnabled(nextMode, nextVirt);

  unsigned type = interrupt ? 2 /* Interrupt */ : 1 /* Exception */;

  // smctr.adoc L498/L502 (Virtualization Mode Transitions note): for the transfer-type
  // filter bits (and RASEMU/Custom), vsctrctl's copy is authoritative whenever either side
  // of the transition touches VS/VU -- the spec's own worked example traps VU (source,
  // virt) into HS (target, not virt) and states the filter configuration comes from
  // vsctrctl, not sctrctl, even though the target (HS) is not virtualized. Combining both
  // sides with OR correctly reduces to vsctrctl whenever exactly one side is virtual (the
  // boundary-crossing case the note covers) and also when both sides are virtual (VU<->VS,
  // where sctrctl/mctrctl simply do not define those modes' bits at all), and to
  // mctrctl/sctrctl when neither side is virtual.
  bool filterVirt = origVirt or nextVirt;

  // smctr.adoc L695: while RAS-emulation mode is active, every transfer type other than
  // call/return/co-routine-swap (TYPE 8/9/12/13) is inhibited, unconditionally and
  // regardless of the transfer-type filter/external-trap-enable bits (which L696 says are
  // not even consulted in this mode). A trap entry is always TYPE 1 (exception) or TYPE 2
  // (interrupt), neither of which is one of those four, so it is never recorded while
  // RASEMU is set in the relevant (filterVirt-selected) register -- checked uniformly here,
  // ahead of the srcEn/tgtEn classification below, since RASEMU inhibits every branch of
  // that classification (ee/ed/de) identically, including the external-trap opt-in path
  // that otherwise deliberately bypasses the ordinary type-filter check.
  if (ctrRasEmulationActiveForFilter(filterVirt))
    return;

  if (srcEn and tgtEn)
    {
      // smctr.adoc L482/ctr_trap_ee: recorded normally.
      if (ctrTypeInhibited(type, filterVirt))
        return;
      if (ctrFrozen())
        return;
      recordCtrTransfer(type, sourcePc, targetPc, /*misp=*/false);
      return;
    }

  if (srcEn and not tgtEn)
    {
      // smctr.adoc L482b/ctr_trap_ed: external trap -- not recorded by default; opt-in via
      // ctrExternalTrapEnabled(). L548: deliberately independent of EXCINH/INTRINH, so the
      // type-filter check is skipped entirely here (FROZEN still applies).
      if (not ctrExternalTrapEnabled(origMode, origVirt, nextMode, nextVirt))
        return;
      if (ctrFrozen())
        return;
      recordCtrTransfer(type, sourcePc, /*targetPc=*/URV(0), /*misp=*/false);  // L534
      return;
    }

  if (not srcEn and tgtEn)
    {
      // smctr.adoc L484/ctr_trap_de: recorded, but ctrsource.PC is forced to 0.
      if (ctrTypeInhibited(type, filterVirt))
        return;
      if (ctrFrozen())
        return;
      recordCtrTransfer(type, /*sourcePc=*/URV(0), targetPc, /*misp=*/false);
      return;
    }

  // smctr.adoc L484b/ctr_trap_dd: both disabled -- not recorded.
}


template <typename URV>
void
Hart<URV>::ctrRecordTrapReturn(PrivilegeMode srcMode, bool srcVirt,
                                PrivilegeMode tgtMode, bool tgtVirt,
                                URV sourcePc, URV targetPc)
{
  if (debugMode_)
    return;

  bool srcEn = ctrRecordingEnabled(srcMode, srcVirt);
  bool tgtEn = ctrRecordingEnabled(tgtMode, tgtVirt);

  // smctr.adoc L465c/L488/ctr_trapret_de and L488b/ctr_trapret_dd: a disabled-source trap
  // return is NOT recorded at all, regardless of the target's enable state -- this is the
  // deliberate asymmetry versus trap entry (where a disabled source still gets a
  // ctrsource.PC-zeroed record). Get this case out of the way first.
  if (not srcEn)
    return;

  unsigned type = 3;  // Trap return.
  bool filterVirt = srcVirt or tgtVirt;  // Same OR-rule as ctrRecordTrapEntry; see there.

  // smctr.adoc L695: TYPE 3 (trap return) is not one of the four types RAS-emulation mode
  // still records (call/return/co-routine-swap, TYPE 8/9/12/13), so a trap return is never
  // recorded while RASEMU is set in the relevant (filterVirt-selected) register -- same
  // unconditional inhibit, and same register-selection rule, as ctrRecordTrapEntry.
  if (ctrRasEmulationActiveForFilter(filterVirt))
    return;

  if (ctrTypeInhibited(type, filterVirt))
    return;
  if (ctrFrozen())
    return;

  // smctr.adoc L465/L486/ctr_trapret_ee: both enabled -> record normally.
  // smctr.adoc L465b/L486b/ctr_trapret_ed: enabled source, disabled target -> record with
  // ctrtarget.PC forced to 0.
  URV finalTargetPc = tgtEn ? targetPc : URV(0);
  recordCtrTransfer(type, sourcePc, finalTargetPc, /*misp=*/false);
}


template <typename URV>
void
Hart<URV>::maybeRecordCtrTransfer(const DecodedInst& di)
{
  // Fast path: this hook runs unconditionally on every retired instruction across all
  // four top-level execution loops, so avoid any further work at all (including the
  // peekCsr lookups ctrQualifies would otherwise perform) on a hart where neither CTR
  // extension is enabled -- ctrQualifies would reach the same "not qualified" verdict
  // via ctrRecordingEnabled's failed peekCsr either way, this just gets there cheaper.
  if (not isRvsmctr() and not isRvssctr())
    return;

  // Debug Mode is a hard override, evaluated before anything else (smctr.adoc L472):
  // neither transfers executed in Debug Mode, nor the transfer into/out of Debug Mode
  // itself, are ever recorded.
  if (debugMode_)
    return;

  if (not di.isBranch())
    return;  // Not a candidate control transfer at all.

  unsigned type = ctrTransferType(di);

  PrivilegeMode pm = privMode_;
  bool virt = virtMode_;

  if (ctrRasEmulationActive(pm, virt))
    {
      // smctr.adoc L689-L696: RAS-emulation mode is a complete replacement of the normal
      // recording algorithm, not an additional filter. While active, the transfer-type
      // filter bits (xctrctl[47:32]) and the external-trap-enable bits (xTE) are ignored
      // entirely (L696) -- they are not merely overridden, they are never consulted.
      // Debug Mode has already been excluded above; the only qualification gates that
      // still apply here are privilege-mode-enable and FROZEN (ctrQualifies's other two
      // checks, reused individually since ctrQualifies itself would also apply
      // ctrTypeInhibited, which must NOT run in this mode).
      if (not ctrRecordingEnabled(pm, virt) or ctrFrozen())
        return;

      if (type == 8 or type == 9)
        {
          // Indirect call / direct call: recorded exactly as normal (L691), except CCV
          // is forced to 0 for every RAS-emulation call record (smctr.adoc L722-723's
          // sanctioned simplification -- see recordCtrTransfer's rasemuCall doc comment).
          recordCtrTransfer(type, currPc_, pc_, /*misp=*/false, /*rasemuCall=*/true);
        }
      else if (type == 13)
        {
          // Function return: pop the most recent call instead of pushing (L692/L692b).
          csRegs_.ctrPopReturn();
        }
      else if (type == 12)
        {
          // Co-routine swap: overwrite logical entry 0 in place, WRPTR unchanged (L693).
          csRegs_.ctrOverwriteEntry0(uint64_t(currPc_), uint64_t(pc_), type);
        }
      // Every other transfer type is simply not recorded while RASEMU=1 (L695): fall
      // through and do nothing.
      return;
    }

  if (not ctrQualifies(type, pm, virt))
    return;

  // misp (branch misprediction) is permanently unimplemented in this build -- always 0.
  recordCtrTransfer(type, currPc_, pc_, /*misp=*/false);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
