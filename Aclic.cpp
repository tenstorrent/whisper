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
        throw std::runtime_error("ACLIC numSources must be in range 1–1023\n");
    if (ipriolen_ == 0 || ipriolen_ > 8)
        throw std::runtime_error("ACLIC ipriolen must be in range 1–8\n");

    unsigned n = numSources_ + 1;  // index 0 unused
    sourcecfg_.assign(n, 0);
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
    sourcecfg_.assign(n, 0);
    source_states_.assign(n, false);
    m_pending_.assign(n, false);
    m_enabled_.assign(n, false);
    m_iprio_.assign(n, 0);
    s_pending_.assign(n, false);
    s_enabled_.assign(n, false);
    s_iprio_.assign(n, 0);
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
    bool anyPending = false;
    const auto& pending = isMachine ? m_pending_ : s_pending_;
    const auto& enabled = isMachine ? m_enabled_ : s_enabled_;
    for (unsigned i = 1; i <= numSources_; ++i)
        if (pending[i] && enabled[i]) { anyPending = true; break; }
    deliveryCb_(isMachine, anyPending);
}

unsigned
Aclic::topInterrupt(bool isMachine) const
{
    const auto& pending = isMachine ? m_pending_ : s_pending_;
    const auto& enabled = isMachine ? m_enabled_ : s_enabled_;
    const auto& iprio   = isMachine ? m_iprio_   : s_iprio_;

    unsigned bestId = 0;
    unsigned bestPrio = ~0u;

    for (unsigned i = 1; i <= numSources_; ++i) {
        if (!pending[i] || !enabled[i]) continue;
        // iprio=0 means use source number as effective priority (APLIC convention)
        unsigned prio = (iprio[i] == 0) ? i : static_cast<unsigned>(iprio[i]);
        if (prio < bestPrio || (prio == bestPrio && i < bestId)) {
            bestPrio = prio;
            bestId = i;
        }
    }
    return bestId;
}

void
Aclic::applySourcecfg(unsigned i, uint16_t val)
{
    if (i == 0 || i > numSources_) return;
    // Delegate bit (bit 10) only valid if hasSupervisorDomain
    if (!hasSupervisorDomain_)
        val &= ~uint16_t(1u << 10);
    sourcecfg_[i] = val;
    // If mode becomes inactive or detached, clear pending
    unsigned sm = val & 0x7;
    if (sm == 0 || sm == 1) {  // Inactive or Detached
        m_pending_[i] = false;
        s_pending_[i] = false;
        // No need to call updateDelivery here; done by caller if needed
    }
}

void
Aclic::setSourceState(unsigned i, bool state)
{
    if (i == 0 || i > numSources_) return;
    bool prev = source_states_[i];
    source_states_[i] = state;

    unsigned sm = sourcecfg_[i] & 0x7;
    bool delegate = hasSupervisorDomain_ && ((sourcecfg_[i] >> 10) & 1);
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
    // Direct assign: written value becomes new pending state for sources in this word
    constexpr unsigned XLEN = sizeof(URV) * 8;
    auto& pending = isMachine ? m_pending_ : s_pending_;
    for (unsigned j = 0; j < XLEN; ++j) {
        unsigned src = static_cast<unsigned>(k) * XLEN + j;
        if (src == 0 || src > numSources_) continue;
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
        enabled[src] = (value >> j) & 1;
    }
    updateDelivery(isMachine);
    return true;
}

// ---- acliciprio helpers (via xireg) ----
// Register k covers sources k*(XLEN/8)+1 .. k*(XLEN/8)+(XLEN/8)
// Each byte is the iprio for one source.

template<typename URV>
bool
Aclic::readIprio(bool isMachine, URV k, URV& value) const
{
    constexpr unsigned BYTES = sizeof(URV);  // 4 (RV32) or 8 (RV64)
    const auto& iprio = isMachine ? m_iprio_ : s_iprio_;
    URV result = 0;
    for (unsigned b = 0; b < BYTES; ++b) {
        unsigned src = static_cast<unsigned>(k) * BYTES + b + 1;
        if (src > numSources_) break;
        result |= URV(iprio[src]) << (b * 8);
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
    uint8_t maxPrio = static_cast<uint8_t>((1u << ipriolen_) - 1);
    for (unsigned b = 0; b < BYTES; ++b) {
        unsigned src = static_cast<unsigned>(k) * BYTES + b + 1;
        if (src > numSources_) break;
        uint8_t prio = static_cast<uint8_t>((value >> (b * 8)) & 0xFF);
        iprio[src] = prio & maxPrio;  // mask to valid ipriolen bits
    }
    return true;
}

// ---- aclicsourcecfg helpers (via xireg2) ----
// Register k covers 2 sources (RV32) or 4 sources (RV64), each 16-bit field.
// For RV32: sources k*2+1, k*2+2
// For RV64: sources k*4+1, k*4+2, k*4+3, k*4+4

template<typename URV>
bool
Aclic::readSourcecfgPacked(URV k, URV& value) const
{
    constexpr unsigned FIELDS = sizeof(URV) / 2;  // 2 (RV32) or 4 (RV64)
    URV result = 0;
    for (unsigned f = 0; f < FIELDS; ++f) {
        unsigned src = static_cast<unsigned>(k) * FIELDS + f + 1;
        if (src > numSources_) break;
        result |= URV(sourcecfg_[src]) << (f * 16);
    }
    value = result;
    return true;
}

template<typename URV>
bool
Aclic::writeSourcecfgPacked(URV k, URV value)
{
    constexpr unsigned FIELDS = sizeof(URV) / 2;
    for (unsigned f = 0; f < FIELDS; ++f) {
        unsigned src = static_cast<unsigned>(k) * FIELDS + f + 1;
        if (src > numSources_) break;
        uint16_t cfg = static_cast<uint16_t>((value >> (f * 16)) & 0xFFFF);
        applySourcecfg(src, cfg);
    }
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
        return readSourcecfgPacked(sel - URV(0x1000), value);
    return false;
}

template<typename URV>
bool
Aclic::writeMireg2(URV sel, URV value)
{
    if (sel >= 0x1000 && sel <= 0x10FF)
        return writeSourcecfgPacked(sel - URV(0x1000), value);
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
        return readSourcecfgPacked(sel - URV(0x1000), value);
    return false;
}

template<typename URV>
bool
Aclic::writeSireg2(URV sel, URV value)
{
    if (!hasSupervisorDomain_) return false;
    if (sel >= 0x1000 && sel <= 0x10FF)
        return writeSourcecfgPacked(sel - URV(0x1000), value);
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
template bool Aclic::readSireg<uint32_t>(uint32_t, uint32_t&) const;
template bool Aclic::readSireg<uint64_t>(uint64_t, uint64_t&) const;
template bool Aclic::writeSireg<uint32_t>(uint32_t, uint32_t);
template bool Aclic::writeSireg<uint64_t>(uint64_t, uint64_t);
template bool Aclic::readSireg2<uint32_t>(uint32_t, uint32_t&) const;
template bool Aclic::readSireg2<uint64_t>(uint64_t, uint64_t&) const;
template bool Aclic::writeSireg2<uint32_t>(uint32_t, uint32_t);
template bool Aclic::writeSireg2<uint64_t>(uint64_t, uint64_t);
