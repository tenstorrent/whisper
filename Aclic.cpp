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
Aclic::setMithreshold(uint16_t val)
{
    // Smnip: mithreshold.iprio is IPRIOLEN+1 bits.  An "all enabled" threshold
    // value (>= 2^IPRIOLEN) reads back unchanged but is permitted.
    auto mask = static_cast<uint16_t>((1u << (ipriolen_ + 1)) - 1);
    mithreshold_ = val & mask;
    updateDelivery(true);
}


void
Aclic::setSithreshold(uint16_t val)
{
    if (!hasSupervisorDomain_)
        return;
    auto mask = static_cast<uint16_t>((1u << (ipriolen_ + 1)) - 1);
    sithreshold_ = val & mask;
    updateDelivery(false);
}


uint32_t
Aclic::miconfigMask() const
{
    // Bits implemented per spec.  Clamp mnipbits/snipbits to [0, ipriolen].
    uint32_t m = 0;
    m |= (1u << 2);                          // mnipen
    m |= (1u << 4);                          // mipu
    m |= 0xFu << 8;                          // mnipbits[11:8]
    if (hasSupervisorDomain_)
      {
        m |= (1u << 3);                      // snipen
        m |= (1u << 5);                      // sipu
        m |= 0xFu << 16;                     // snipbits[19:16]
      }
    return m;
}


void
Aclic::setSiconfig(uint32_t v)
{
    // siconfig is the S-mode-accessible subset of miconfig.
    // Bits siconfig owns: snipen [3], sipu [5], snipbits [19:16].
    uint32_t sMask = (1u << 3) | (1u << 5) | (0xFu << 16);
    sMask &= miconfigMask();
    miconfig_ = (miconfig_ & ~sMask) | (v & sMask);
}


uint32_t
Aclic::getSiconfig() const
{
    uint32_t sMask = (1u << 3) | (1u << 5) | (0xFu << 16);
    return miconfig_ & sMask;
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

// ---- Source domain helpers ----

bool
Aclic::isDelegated(unsigned src) const
{
    return hasSupervisorDomain_ && ((m_sourcecfg_[src] >> 10) & 1);
}

unsigned
Aclic::effectiveSm(unsigned src) const
{
    unsigned sSm = s_sourcecfg_[src] & 0x7;
    return sSm != 0 ? sSm : (m_sourcecfg_[src] & 0x7);
}

bool
Aclic::isSourceActive(unsigned src, bool isMachine) const
{
    bool delegated = isDelegated(src);
    unsigned sm = effectiveSm(src);
    if (isMachine)
        return sm != 0 && !delegated;
    return delegated && sm != 0;
}

void
Aclic::evalLevelPending(unsigned i, unsigned sm, bool isMachine)
{
    auto& pending = isMachine ? m_pending_ : s_pending_;
    if (sm == 6)
        pending[i] = source_states_[i];
    else if (sm == 7)
        pending[i] = !source_states_[i];
}

void
Aclic::clearAllState(unsigned i)
{
    s_sourcecfg_[i] = 0;
    m_pending_[i] = false;
    s_pending_[i] = false;
    m_enabled_[i] = false;
    s_enabled_[i] = false;
    m_iprio_[i]   = 0;
    s_iprio_[i]   = 0;
}

void
Aclic::clearDomainState(unsigned i, bool isMachine)
{
    if (isMachine) {
        m_pending_[i] = false;
        m_enabled_[i] = false;
        m_iprio_[i]   = 0;
    } else {
        s_pending_[i] = false;
        s_enabled_[i] = false;
        s_iprio_[i]   = 0;
    }
}

// ---- Source configuration ----

void
Aclic::applySourcecfg(unsigned i, uint16_t val, bool supervisorDomain)
{
    if (i == 0 || i > numSources_) return;

    // D=1 is invalid when there is no child domain to delegate to.
    // Per APLIC spec, the entire register is zeroed (WARL).
    bool hasD = val & uint16_t(1u << 10);
    if (hasD && (supervisorDomain || !hasSupervisorDomain_))
        val = 0;

    if (supervisorDomain) {
        val &= 0x7;  // S-domain is a leaf: SM bits only
        s_sourcecfg_[i] = val;
        if (val == 0)
            clearDomainState(i, false);
        return;
    }

    // M-domain write.
    bool wasDelegate = isDelegated(i);
    m_sourcecfg_[i] = val;
    unsigned sm = val & 0x7;
    bool delegate = hasD;

    if (sm == 0 && !delegate) {
        // Truly Inactive: source doesn't exist in either domain.
        clearAllState(i);
    } else if (delegate && !wasDelegate) {
        // M->S delegation: source moves to S-domain.
        // Clear stale M-domain state; re-evaluate pending in S-domain.
        clearDomainState(i, /*isMachine=*/true);
        evalLevelPending(i, effectiveSm(i), /*isMachine=*/false);
    } else if (!delegate && wasDelegate) {
        // S->M delegation: source moves to M-domain.
        // Clear S-domain pending/enabled/iprio but preserve s_sourcecfg_
        // (it will be hidden by readSourcecfgPacked and may be needed if
        // the source is re-delegated later).
        clearDomainState(i, /*isMachine=*/false);
        evalLevelPending(i, effectiveSm(i), /*isMachine=*/true);
    }
}

void
Aclic::setSourceState(unsigned i, bool state)
{
    if (i == 0 || i > numSources_) return;
    bool prev = source_states_[i];
    source_states_[i] = state;

    bool delegate = isDelegated(i);
    unsigned sm = effectiveSm(i);
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

unsigned
Aclic::topInterrupt(bool isMachine, unsigned* prio, bool ignoreThreshold) const
{
    const auto& pending = isMachine ? m_pending_ : s_pending_;
    const auto& enabled = isMachine ? m_enabled_ : s_enabled_;
    const auto& iprio   = isMachine ? m_iprio_   : s_iprio_;
    // 9-bit threshold (IPRIOLEN+1).  When >= 2^IPRIOLEN, no external source is
    // suppressed by threshold gating (spec §Smnip allows enabling all).
    unsigned threshold  = ignoreThreshold ? 0 : (isMachine ? mithreshold_ : sithreshold_);
    unsigned maxExternal = 1u << ipriolen_;
    if (threshold >= maxExternal)
        threshold = 0;  // 0 means "no threshold" in the loop below

    unsigned bestId = 0;
    unsigned bestPrio = ~0u;

    // Spec §Smnip "Operation": NIPPRIO_MASK = ~(2^(IPRIOLEN - xnipbits) - 1).
    // miconfig.{mnipbits,snipbits} controls how many MSBs of the priority
    // participate in nested preemption grouping.  When xnipen=0 or xnipbits=0,
    // threshold gating is a plain priority comparison (mask = ~0).
    unsigned xnipbits = isMachine ? getMnipBits() : getSnipBits();
    bool xnipen      = isMachine ? isMnipEnabled() : isSnipEnabled();
    unsigned nipprio_mask = ~0u;
    if (xnipen and xnipbits > 0 and xnipbits <= ipriolen_)
        nipprio_mask = ~((1u << (ipriolen_ - xnipbits)) - 1u);

    for (unsigned i = 1; i <= numSources_; ++i) {
        if (!pending[i] || !enabled[i]) continue;
        // Filter out sources that don't belong to this domain (stale bits
        // can persist across delegation transitions).
        if (!isSourceActive(i, isMachine)) continue;
        // iprio=0 is an uninitialised sentinel; treat as 1 (the minimum
        // legal WARL value) so arbitration matches what software reads back.
        unsigned p = (iprio[i] != 0) ? static_cast<unsigned>(iprio[i]) : 1u;
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
        if (!isSourceActive(src, isMachine)) continue;
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
    // Level1 (SM=6) and Level0 (SM=7) sources have pending driven solely by
    // the rectified hardware input (AIA spec section 4.7).  Their pending bit
    // cannot be modified by register writes, only by setSourceState().
    constexpr unsigned XLEN = sizeof(URV) * 8;
    auto& pending = isMachine ? m_pending_ : s_pending_;
    for (unsigned j = 0; j < XLEN; ++j) {
        unsigned src = static_cast<unsigned>(k) * XLEN + j;
        if (src == 0 || src > numSources_) continue;
        if (!isSourceActive(src, isMachine)) continue;
        unsigned sm = effectiveSm(src);
        if (sm == 6 || sm == 7) continue;
        pending[src] = (value >> j) & 1;
    }
    updateDelivery(isMachine);
    return true;
}

void
Aclic::tryClearPending(bool isMachine, unsigned src)
{
    if (src == 0 || src > numSources_) return;
    if (!isSourceActive(src, isMachine)) return;
    unsigned sm = effectiveSm(src);
    if (sm == 6 || sm == 7) return;  // Level-sensitive: clearing is not possible
    auto& pending = isMachine ? m_pending_ : s_pending_;
    if (!pending[src]) return;
    pending[src] = false;
    updateDelivery(isMachine);
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
        if (!isSourceActive(src, isMachine)) continue;
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
        if (!isSourceActive(src, isMachine)) continue;
        enabled[src] = (value >> j) & 1;
    }
    updateDelivery(isMachine);
    return true;
}

// ---- acliciprio helpers (via xireg) ----
// Register k covers sources k*4+0 .. k*4+(BYTES-1), one byte per source.
// Stride is always 4 per xiselect step.  Source 0 is read-only zero.
// For RV64, odd k is rejected before reaching here (enforced in CsRegs).

template<typename URV>
bool
Aclic::readIprio(bool isMachine, URV k, URV& value) const
{
    constexpr unsigned BYTES = sizeof(URV);
    const auto& iprio = isMachine ? m_iprio_ : s_iprio_;
    URV result = 0;
    for (unsigned b = 0; b < BYTES; ++b) {
        unsigned src = static_cast<unsigned>(k) * 4 + b;
        if (src == 0) continue;
        if (src > numSources_) break;
        if (!isSourceActive(src, isMachine)) continue;
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
        if (src == 0) continue;
        if (src > numSources_) break;
        if (!isSourceActive(src, isMachine)) continue;
        auto prio = static_cast<uint8_t>((value >> (b * 8)) & 0xFF);
        prio &= maxPrio;
        iprio[src] = (prio != 0) ? prio : 1;  // WARL: 0 snaps to 1
    }
    return true;
}

// ---- aclicsourcecfg helpers (via xireg2 and xireg3) ----
// xireg2 covers sources k*4+0 .. k*4+(FIELDS-1), 16 bits each.
// xireg3 covers sources k*4+FIELDS .. k*4+(2*FIELDS-1).
// FIELDS = sizeof(URV)/2.  Source 0 is read-only zero.
// For RV64, odd k is rejected before reaching here (enforced in CsRegs).

template<typename URV>
bool
Aclic::readSourcecfgPacked(URV k, URV& value, bool supervisorDomain) const
{
    constexpr unsigned FIELDS = sizeof(URV) / 2;
    const auto& cfg = supervisorDomain ? s_sourcecfg_ : m_sourcecfg_;
    URV result = 0;
    for (unsigned f = 0; f < FIELDS; ++f) {
        unsigned src = static_cast<unsigned>(k) * 4 + f;
        if (src == 0) continue;
        if (src > numSources_) break;
        // S-domain view: non-delegated sources read as 0.
        if (supervisorDomain && !isDelegated(src)) continue;
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
        if (src == 0) continue;
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
        if (supervisorDomain && !isDelegated(src)) continue;
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
Aclic::readMireg4(URV sel, URV& value) const
{
    // mireg4 is reserved for miconfig (Smnip) at miselect = 0x1000.
    // Other selectors in 0x1000-0x10FF raise illegal instruction at the caller.
    if (sel == URV(0x1000)) {
        value = URV(miconfig_);
        return true;
    }
    return false;
}

template<typename URV>
bool
Aclic::writeMireg4(URV sel, URV value)
{
    if (sel == URV(0x1000)) {
        setMiconfig(static_cast<uint32_t>(value));
        return true;
    }
    return false;
}

template<typename URV>
bool
Aclic::readSireg4(URV sel, URV& value) const
{
    if (!hasSupervisorDomain_) return false;
    if (sel == URV(0x1000)) {
        value = URV(getSiconfig());
        return true;
    }
    return false;
}

template<typename URV>
bool
Aclic::writeSireg4(URV sel, URV value)
{
    if (!hasSupervisorDomain_) return false;
    if (sel == URV(0x1000)) {
        setSiconfig(static_cast<uint32_t>(value));
        return true;
    }
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
template bool Aclic::readMireg4<uint32_t>(uint32_t, uint32_t&) const;
template bool Aclic::readMireg4<uint64_t>(uint64_t, uint64_t&) const;
template bool Aclic::writeMireg4<uint32_t>(uint32_t, uint32_t);
template bool Aclic::writeMireg4<uint64_t>(uint64_t, uint64_t);
template bool Aclic::readSireg4<uint32_t>(uint32_t, uint32_t&) const;
template bool Aclic::readSireg4<uint64_t>(uint64_t, uint64_t&) const;
template bool Aclic::writeSireg4<uint32_t>(uint32_t, uint32_t);
template bool Aclic::writeSireg4<uint64_t>(uint64_t, uint64_t);
