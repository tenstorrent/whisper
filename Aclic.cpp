#include "Aclic.hpp"
#include <stdexcept>
#include <cassert>

using namespace TT_ACLIC;

Aclic::Aclic(const AclicParams& params)
    : numSources_(params.numSources),
      hasSupervisorDomain_(params.hasSupervisorDomain),
      ipriolen_(params.ipriolen)
{
    if (numSources_ == 0 || numSources_ > 1023)
        throw std::runtime_error("ACLIC numSources must be in range 1-1023\n");
    if (ipriolen_ == 0 || ipriolen_ > 8)
        throw std::runtime_error("ACLIC ipriolen must be in range 1-8\n");

    unsigned n = numSources_ + 1;  // index 0 unused
    m_sourcecfg_.assign(n, 0);
    s_sourcecfg_.assign(n, 0);
    source_states_.assign(n, false);
    m_pending_.assign(n, false);
    m_enabled_.assign(n, false);
    m_iprio_.assign(n, 0);
    s_pending_.assign(n, false);
    s_enabled_.assign(n, false);
    s_iprio_.assign(n, 0);
}

void
Aclic::reset()
{
    unsigned n = numSources_ + 1;
    m_sourcecfg_.assign(n, 0);
    s_sourcecfg_.assign(n, 0);
    source_states_.assign(n, false);
    m_pending_.assign(n, false);
    m_enabled_.assign(n, false);
    m_iprio_.assign(n, 0);
    s_pending_.assign(n, false);
    s_enabled_.assign(n, false);
    s_iprio_.assign(n, 0);
    mithreshold_ = 0;
    sithreshold_ = 0;
    mPreemptmsk_ = 0;
}


void
Aclic::setMithreshold(uint8_t val)
{
    auto mask = static_cast<uint8_t>((1u << ipriolen_) - 1);
    mithreshold_ = val & mask;
    updateDelivery(true);
}


void
Aclic::setSithreshold(uint8_t val)
{
    if (!hasSupervisorDomain_)
        return;
    auto mask = static_cast<uint8_t>((1u << ipriolen_) - 1);
    sithreshold_ = val & mask;
    updateDelivery(false);
}

void
Aclic::setDeliveryCallback(const DeliveryCallback& cb)
{
    deliveryCb_ = cb;
}

void
Aclic::updateDelivery(bool isMachine)
{
    if (!deliveryCb_)
        return;
    deliveryCb_(isMachine, topInterrupt(isMachine) != 0);
}

unsigned
Aclic::topInterrupt(bool isMachine, unsigned* prio, bool ignoreThreshold) const
{
    const auto& pending = isMachine ? m_pending_ : s_pending_;
    const auto& enabled = isMachine ? m_enabled_ : s_enabled_;
    const auto& iprio   = isMachine ? m_iprio_   : s_iprio_;
    // eithreshold is excluded from ACLIC scope per spec; mithreshold only controls
    // delivery (MIP.MEIP). For xtopei reads (ignoreThreshold=true), bypass it.
    unsigned threshold  = ignoreThreshold ? 0 : (isMachine ? mithreshold_ : sithreshold_);

    unsigned bestId = 0;
    unsigned bestPrio = ~0u;

    // NIPPRIO_MASK: low mPreemptmsk_ bits of priority are ignored for threshold
    // comparison.  NIPPRIO_MASK = ~(2^mPreemptmsk_ - 1).  When mPreemptmsk_=0
    // this evaluates to ~0u (all bits participate -- normal behaviour).
    unsigned nipprio_mask = ~((1u << mPreemptmsk_) - 1u);

    for (unsigned i = 1; i <= numSources_; ++i) {
        if (!pending[i] || !enabled[i]) continue;
        // Skip sources that don't belong to this domain.  Stale pending/enabled
        // bits can persist across delegation transitions (D bit changes).
        if (!isSourceActive(i, isMachine)) continue;
        // Internal iprio=0 is an uninitialised sentinel; treat as 1 (the minimum
        // legal WARL value) so arbitration matches what software reads back.
        unsigned p = (iprio[i] != 0) ? static_cast<unsigned>(iprio[i]) : 1u;
        // When threshold is nonzero, suppress sources where masked priority >=
        // masked threshold (spec: IPRIO >= xithreshold & NIPPRIO_MASK).
        if (threshold != 0 && (p & nipprio_mask) >= (threshold & nipprio_mask)) continue;
        if (p < bestPrio || (p == bestPrio && i < bestId)) {
            bestPrio = p;
            bestId = i;
        }
    }
    if (prio)
        *prio = (bestId != 0) ? bestPrio : 0;
    return bestId;
}

void
Aclic::applySourcecfg(unsigned i, uint16_t val, bool supervisorDomain)
{
    if (i == 0 || i > numSources_) return;
    if (supervisorDomain) {
        // S-domain is a leaf: no children to delegate to.
        // Per APLIC spec: writing D=1 to a leaf domain's sourcecfg zeros
        // the entire register (WARL: D=1 is not a legal value for a leaf).
        if (val & uint16_t(1u << 10))
            val = 0;
        else
            val &= 0x7;
        s_sourcecfg_[i] = val;
        if (val == 0) {  // SM=0 (or zeroed by D=1): clear S-domain state
            s_pending_[i] = false;
            s_enabled_[i] = false;
            s_iprio_[i]   = 0;
        }
    } else {
        // M-domain: D=1 is only valid when a supervisor domain exists.
        // Without a child domain, D=1 is an invalid configuration and the
        // entire register is zeroed (same WARL behavior as D=1 on a leaf).
        if (val & uint16_t(1u << 10)) {
            if (!hasSupervisorDomain_)
                val = 0;
        }
        bool wasDelegate = m_sourcecfg_[i] & uint16_t(1u << 10);
        m_sourcecfg_[i] = val;
        unsigned sm = val & 0x7;
        bool delegate = val & uint16_t(1u << 10);
        if (sm == 0 && !delegate) {
            // Truly Inactive (SM=0, D=0): clear all state in both domains.
            m_pending_[i] = false;
            s_pending_[i] = false;
            m_enabled_[i] = false;
            s_enabled_[i] = false;
            m_iprio_[i]   = 0;
            s_iprio_[i]   = 0;
        } else if (delegate && !wasDelegate) {
            // Delegation transition M->S: clear stale M-domain state.
            // The source now belongs to S-domain; any leftover M-domain
            // pending/enabled bits must not cause spurious MEIP.
            m_pending_[i] = false;
            m_enabled_[i] = false;
            m_iprio_[i]   = 0;
            // For level-triggered sources, re-evaluate pending in the new domain
            // based on current hardware state (the pin is still asserted/deasserted).
            unsigned effSm = (s_sourcecfg_[i] & 0x7) != 0 ? (s_sourcecfg_[i] & 0x7) : sm;
            if (effSm == 6)      s_pending_[i] = source_states_[i];
            else if (effSm == 7) s_pending_[i] = !source_states_[i];
        } else if (!delegate && wasDelegate) {
            // Delegation transition S->M: clear stale S-domain state.
            // The source now belongs to M-domain; any leftover S-domain
            // pending/enabled bits must not cause spurious SEIP.
            s_pending_[i] = false;
            s_enabled_[i] = false;
            s_iprio_[i]   = 0;
            // Re-evaluate pending in M-domain for level-triggered sources.
            if (sm == 6)      m_pending_[i] = source_states_[i];
            else if (sm == 7) m_pending_[i] = !source_states_[i];
        }
    }
}

bool
Aclic::isSourceActive(unsigned src, bool isMachine) const
{
    // D=1 in m_sourcecfg_ means the source is delegated to the S-domain.
    bool delegated = hasSupervisorDomain_ && ((m_sourcecfg_[src] >> 10) & 1);
    // The effective source mode: S-domain may override via sireg2; if not written
    // (s_sourcecfg_ SM=0), fall back to the SM written by M-domain via mireg2.
    unsigned sm = (s_sourcecfg_[src] & 0x7) != 0 ? (s_sourcecfg_[src] & 0x7)
                                                    : (m_sourcecfg_[src] & 0x7);
    if (isMachine)
        return sm != 0 && !delegated;
    else
        return delegated && sm != 0;
}

void
Aclic::setSourceState(unsigned i, bool state)
{
    if (i == 0 || i > numSources_) return;
    bool prev = source_states_[i];
    source_states_[i] = state;

    bool delegate = hasSupervisorDomain_ && ((m_sourcecfg_[i] >> 10) & 1);
    // Effective SM: S-domain override (sireg2) takes priority; fall back to M-domain SM.
    unsigned sm = (s_sourcecfg_[i] & 0x7) != 0 ? (s_sourcecfg_[i] & 0x7)
                                                  : (m_sourcecfg_[i] & 0x7);
    bool isMachine = !delegate;
    auto& pending = delegate ? s_pending_ : m_pending_;

    switch (sm) {
    case 0: // Inactive
    case 1: // Detached
        return;
    case 4: // Edge1 (rising edge triggers pending)
        if (!prev && state) {
            pending[i] = true;
            updateDelivery(isMachine);
        }
        break;
    case 5: // Edge0 (falling edge triggers pending)
        if (prev && !state) {
            pending[i] = true;
            updateDelivery(isMachine);
        }
        break;
    case 6: // Level1 (active high level)
        pending[i] = state;
        updateDelivery(isMachine);
        break;
    case 7: // Level0 (active low level)
        pending[i] = !state;
        updateDelivery(isMachine);
        break;
    default:
        break;
    }
}

// ---- eip helpers ----

template<typename URV>
bool
Aclic::readEip(bool isMachine, URV k, URV& value) const
{
    constexpr unsigned XLEN = sizeof(URV) * 8;
    const auto& pending = isMachine ? m_pending_ : s_pending_;
    URV result = 0;
    for (unsigned j = 0; j < XLEN; ++j) {
        unsigned src = static_cast<unsigned>(k) * XLEN + j;
        if (src == 0 || src > numSources_) continue;
        if (!isSourceActive(src, isMachine)) continue;  // Inactive in this domain: read-only zero
        if (pending[src])
            result |= URV(1) << j;
    }
    value = result;
    return true;
}

template<typename URV>
bool
Aclic::writeEip(bool isMachine, URV k, URV value)
{
    // Direct assign: written value becomes new pending state for sources in this word.
    // Exception: Level1 (SM=6) and Level0 (SM=7) sources in direct delivery mode have
    // their pending bit driven solely by the rectified input value (AIA spec section 4.7).
    // Their pending bit cannot be set or cleared by any register write - including claims
    // (MTOPEI/STOPEI), setip, or in_clrip.  Only setSourceState() (i.e. the hardware
    // input changing) may modify them.
    constexpr unsigned XLEN = sizeof(URV) * 8;
    auto& pending = isMachine ? m_pending_ : s_pending_;
    for (unsigned j = 0; j < XLEN; ++j) {
        unsigned src = static_cast<unsigned>(k) * XLEN + j;
        if (src == 0 || src > numSources_) continue;
        if (!isSourceActive(src, isMachine)) continue;  // Inactive in this domain: read-only zero
        unsigned sm = (s_sourcecfg_[src] & 0x7) != 0 ? (s_sourcecfg_[src] & 0x7)
                                                       : (m_sourcecfg_[src] & 0x7);
        if (sm == 6 || sm == 7) continue;   // Level1, Level0: pending follows rectified input only
        pending[src] = (value >> j) & 1;
    }
    updateDelivery(isMachine);
    return true;
}

// ---- eie helpers ----

template<typename URV>
bool
Aclic::readEie(bool isMachine, URV k, URV& value) const
{
    constexpr unsigned XLEN = sizeof(URV) * 8;
    const auto& enabled = isMachine ? m_enabled_ : s_enabled_;
    URV result = 0;
    for (unsigned j = 0; j < XLEN; ++j) {
        unsigned src = static_cast<unsigned>(k) * XLEN + j;
        if (src == 0 || src > numSources_) continue;
        if (!isSourceActive(src, isMachine)) continue;  // Inactive in this domain: read-only zero
        if (enabled[src])
            result |= URV(1) << j;
    }
    value = result;
    return true;
}

template<typename URV>
bool
Aclic::writeEie(bool isMachine, URV k, URV value)
{
    constexpr unsigned XLEN = sizeof(URV) * 8;
    auto& enabled = isMachine ? m_enabled_ : s_enabled_;
    for (unsigned j = 0; j < XLEN; ++j) {
        unsigned src = static_cast<unsigned>(k) * XLEN + j;
        if (src == 0 || src > numSources_) continue;
        if (!isSourceActive(src, isMachine)) continue;  // Inactive in this domain: read-only zero
        enabled[src] = (value >> j) & 1;
    }
    updateDelivery(isMachine);
    return true;
}

// ---- acliciprio helpers (via xireg) ----
// Register k covers sources k*4+0 .. k*4+(BYTES-1), one byte per source.
// Stride is always 4 per xiselect step (spec aclic.adoc lines 388, 391).
// Source 0 (target[0]) is read-only zero; sources beyond numSources_ are ignored.
// For RV64 the full 8 bytes are used; odd k is rejected (illegal instruction)
// before reaching here (enforced in CsRegs).

template<typename URV>
bool
Aclic::readIprio(bool isMachine, URV k, URV& value) const
{
    constexpr unsigned BYTES = sizeof(URV);  // 4 (RV32) or 8 (RV64)
    const auto& iprio = isMachine ? m_iprio_ : s_iprio_;
    URV result = 0;
    for (unsigned b = 0; b < BYTES; ++b) {
        unsigned src = static_cast<unsigned>(k) * 4 + b;
        if (src == 0) continue;   // target[0] is read-only zero
        if (src > numSources_) break;
        if (!isSourceActive(src, isMachine)) continue;  // Inactive in this domain: read-only zero
        // IPRIO is WARL: only values 1..2^IPRIOLEN-1 are legal (AIA spec 4.5.16).
        // Internally 0 means "use source number as priority"; snap to 1 on reads.
        uint8_t p = iprio[src];
        result |= URV(p != 0 ? p : 1) << (b * 8);
    }
    value = result;
    return true;
}

template<typename URV>
bool
Aclic::writeIprio(bool isMachine, URV k, URV value)
{
    constexpr unsigned BYTES = sizeof(URV);
    auto& iprio = isMachine ? m_iprio_ : s_iprio_;
    auto maxPrio = static_cast<uint8_t>((1u << ipriolen_) - 1);
    for (unsigned b = 0; b < BYTES; ++b) {
        unsigned src = static_cast<unsigned>(k) * 4 + b;
        if (src == 0) continue;   // target[0] is read-only zero
        if (src > numSources_) break;
        if (!isSourceActive(src, isMachine)) continue;  // Inactive in this domain: read-only zero
        auto prio = static_cast<uint8_t>((value >> (b * 8)) & 0xFF);
        prio &= maxPrio;  // mask to valid ipriolen bits
        // IPRIO is WARL: 0 is not a legal value; snap to 1 (AIA spec 4.5.16).
        iprio[src] = (prio != 0) ? prio : 1;
    }
    return true;
}

// ---- aclicsourcecfg helpers (via xireg2 and xireg3) ----
// xireg2 at xiselect k covers sources k*4+0 .. k*4+(FIELDS-1), 16 bits each.
// xireg3 at xiselect k covers sources k*4+FIELDS .. k*4+(2*FIELDS-1), 16 bits each.
// FIELDS = sizeof(URV)/2 (2 for RV32, 4 for RV64).
// Stride in source space is always 4 per xiselect step (spec aclic.adoc lines 546, 550).
// Source 0 is read-only zero (aclicsourcecfg[0][15:0] per spec line 547).
// For RV64 odd k is rejected before reaching here (enforced in CsRegs).

template<typename URV>
bool
Aclic::readSourcecfgPacked(URV k, URV& value, bool supervisorDomain) const
{
    constexpr unsigned FIELDS = sizeof(URV) / 2;  // 2 (RV32) or 4 (RV64)
    const auto& cfg = supervisorDomain ? s_sourcecfg_ : m_sourcecfg_;
    URV result = 0;
    for (unsigned f = 0; f < FIELDS; ++f) {
        unsigned src = static_cast<unsigned>(k) * 4 + f;
        if (src == 0) continue;   // aclicsourcecfg[0][15:0] is read-only zero
        if (src > numSources_) break;
        result |= URV(cfg[src]) << (f * 16);
    }
    value = result;
    return true;
}

template<typename URV>
bool
Aclic::writeSourcecfgPacked(URV k, URV value, bool supervisorDomain)
{
    constexpr unsigned FIELDS = sizeof(URV) / 2;
    for (unsigned f = 0; f < FIELDS; ++f) {
        unsigned src = static_cast<unsigned>(k) * 4 + f;
        if (src == 0) continue;   // aclicsourcecfg[0][15:0] is read-only zero
        if (src > numSources_) break;
        auto cfg = static_cast<uint16_t>((value >> (f * 16)) & 0xFFFF);
        applySourcecfg(src, cfg, supervisorDomain);
    }
    updateDelivery(true);
    updateDelivery(false);
    return true;
}

template<typename URV>
bool
Aclic::readSourcecfgPacked3(URV k, URV& value, bool supervisorDomain) const
{
    constexpr unsigned FIELDS = sizeof(URV) / 2;
    const auto& cfg = supervisorDomain ? s_sourcecfg_ : m_sourcecfg_;
    URV result = 0;
    for (unsigned f = 0; f < FIELDS; ++f) {
        unsigned src = static_cast<unsigned>(k) * 4 + FIELDS + f;
        if (src > numSources_) break;
        result |= URV(cfg[src]) << (f * 16);
    }
    value = result;
    return true;
}

template<typename URV>
bool
Aclic::writeSourcecfgPacked3(URV k, URV value, bool supervisorDomain)
{
    constexpr unsigned FIELDS = sizeof(URV) / 2;
    for (unsigned f = 0; f < FIELDS; ++f) {
        unsigned src = static_cast<unsigned>(k) * 4 + FIELDS + f;
        if (src > numSources_) break;
        auto cfg = static_cast<uint16_t>((value >> (f * 16)) & 0xFFFF);
        applySourcecfg(src, cfg, supervisorDomain);
    }
    updateDelivery(true);
    updateDelivery(false);
    return true;
}

// ---- Public CSR access methods ----

template<typename URV>
bool
Aclic::readMireg(URV sel, URV& value) const
{
    if (sel >= 0x80 && sel <= 0xBF)
        return readEip(true, sel - URV(0x80), value);
    if (sel >= 0xC0 && sel <= 0xFF)
        return readEie(true, sel - URV(0xC0), value);
    if (sel >= 0x1000 && sel <= 0x10FF)
        return readIprio(true, sel - URV(0x1000), value);
    return false;
}

template<typename URV>
bool
Aclic::writeMireg(URV sel, URV value)
{
    if (sel >= 0x80 && sel <= 0xBF)
        return writeEip(true, sel - URV(0x80), value);
    if (sel >= 0xC0 && sel <= 0xFF)
        return writeEie(true, sel - URV(0xC0), value);
    if (sel >= 0x1000 && sel <= 0x10FF)
        return writeIprio(true, sel - URV(0x1000), value);
    return false;
}

template<typename URV>
bool
Aclic::readMireg2(URV sel, URV& value) const
{
    if (sel >= 0x1000 && sel <= 0x10FF)
        return readSourcecfgPacked(sel - URV(0x1000), value, /*supervisorDomain=*/false);
    return false;
}

template<typename URV>
bool
Aclic::writeMireg2(URV sel, URV value)
{
    if (sel >= 0x1000 && sel <= 0x10FF)
        return writeSourcecfgPacked(sel - URV(0x1000), value, /*supervisorDomain=*/false);
    return false;
}

template<typename URV>
bool
Aclic::readMireg3(URV sel, URV& value) const
{
    if (sel >= 0x1000 && sel <= 0x10FF)
        return readSourcecfgPacked3(sel - URV(0x1000), value, /*supervisorDomain=*/false);
    return false;
}

template<typename URV>
bool
Aclic::writeMireg3(URV sel, URV value)
{
    if (sel >= 0x1000 && sel <= 0x10FF)
        return writeSourcecfgPacked3(sel - URV(0x1000), value, /*supervisorDomain=*/false);
    return false;
}

template<typename URV>
bool
Aclic::readSireg(URV sel, URV& value) const
{
    if (!hasSupervisorDomain_) return false;
    if (sel >= 0x80 && sel <= 0xBF)
        return readEip(false, sel - URV(0x80), value);
    if (sel >= 0xC0 && sel <= 0xFF)
        return readEie(false, sel - URV(0xC0), value);
    if (sel >= 0x1000 && sel <= 0x10FF)
        return readIprio(false, sel - URV(0x1000), value);
    return false;
}

template<typename URV>
bool
Aclic::writeSireg(URV sel, URV value)
{
    if (!hasSupervisorDomain_) return false;
    if (sel >= 0x80 && sel <= 0xBF)
        return writeEip(false, sel - URV(0x80), value);
    if (sel >= 0xC0 && sel <= 0xFF)
        return writeEie(false, sel - URV(0xC0), value);
    if (sel >= 0x1000 && sel <= 0x10FF)
        return writeIprio(false, sel - URV(0x1000), value);
    return false;
}

template<typename URV>
bool
Aclic::readSireg2(URV sel, URV& value) const
{
    if (!hasSupervisorDomain_) return false;
    if (sel >= 0x1000 && sel <= 0x10FF)
        return readSourcecfgPacked(sel - URV(0x1000), value, /*supervisorDomain=*/true);
    return false;
}

template<typename URV>
bool
Aclic::writeSireg2(URV sel, URV value)
{
    if (!hasSupervisorDomain_) return false;
    if (sel >= 0x1000 && sel <= 0x10FF)
        return writeSourcecfgPacked(sel - URV(0x1000), value, /*supervisorDomain=*/true);
    return false;
}

template<typename URV>
bool
Aclic::readSireg3(URV sel, URV& value) const
{
    if (!hasSupervisorDomain_) return false;
    if (sel >= 0x1000 && sel <= 0x10FF)
        return readSourcecfgPacked3(sel - URV(0x1000), value, /*supervisorDomain=*/true);
    return false;
}

template<typename URV>
bool
Aclic::writeSireg3(URV sel, URV value)
{
    if (!hasSupervisorDomain_) return false;
    if (sel >= 0x1000 && sel <= 0x10FF)
        return writeSourcecfgPacked3(sel - URV(0x1000), value, /*supervisorDomain=*/true);
    return false;
}

// Explicit instantiations
template bool Aclic::readMireg<uint32_t>(uint32_t, uint32_t&) const;
template bool Aclic::readMireg<uint64_t>(uint64_t, uint64_t&) const;
template bool Aclic::writeMireg<uint32_t>(uint32_t, uint32_t);
template bool Aclic::writeMireg<uint64_t>(uint64_t, uint64_t);
template bool Aclic::readMireg2<uint32_t>(uint32_t, uint32_t&) const;
template bool Aclic::readMireg2<uint64_t>(uint64_t, uint64_t&) const;
template bool Aclic::writeMireg2<uint32_t>(uint32_t, uint32_t);
template bool Aclic::writeMireg2<uint64_t>(uint64_t, uint64_t);
template bool Aclic::readMireg3<uint32_t>(uint32_t, uint32_t&) const;
template bool Aclic::readMireg3<uint64_t>(uint64_t, uint64_t&) const;
template bool Aclic::writeMireg3<uint32_t>(uint32_t, uint32_t);
template bool Aclic::writeMireg3<uint64_t>(uint64_t, uint64_t);
template bool Aclic::readSireg<uint32_t>(uint32_t, uint32_t&) const;
template bool Aclic::readSireg<uint64_t>(uint64_t, uint64_t&) const;
template bool Aclic::writeSireg<uint32_t>(uint32_t, uint32_t);
template bool Aclic::writeSireg<uint64_t>(uint64_t, uint64_t);
template bool Aclic::readSireg2<uint32_t>(uint32_t, uint32_t&) const;
template bool Aclic::readSireg2<uint64_t>(uint64_t, uint64_t&) const;
template bool Aclic::writeSireg2<uint32_t>(uint32_t, uint32_t);
template bool Aclic::writeSireg2<uint64_t>(uint64_t, uint64_t);
template bool Aclic::readSireg3<uint32_t>(uint32_t, uint32_t&) const;
template bool Aclic::readSireg3<uint64_t>(uint64_t, uint64_t&) const;
template bool Aclic::writeSireg3<uint32_t>(uint32_t, uint32_t);
template bool Aclic::writeSireg3<uint64_t>(uint64_t, uint64_t);
