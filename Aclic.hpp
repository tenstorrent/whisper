#pragma once

#include <vector>
#include <functional>
#include <cstdint>

namespace TT_ACLIC {

struct AclicParams {
    unsigned numSources = 32;          // 1-1023
    bool hasSupervisorDomain = false;
    unsigned ipriolen = 8;             // 1-8
};

class Aclic {
public:
    explicit Aclic(const AclicParams& params);

    void reset();

    unsigned numSources() const { return numSources_; }
    bool hasSupervisorDomain() const { return hasSupervisorDomain_; }
    unsigned ipriolen() const { return ipriolen_; }

    // Threshold CSRs (mithreshold/sithreshold). Per spec §Smnip the field is
    // IPRIOLEN+1 bits wide (9-bit max).  Values are masked to (ipriolen+1) bits.
    void setMithreshold(uint16_t val);
    void setSithreshold(uint16_t val);
    uint16_t getMithreshold() const { return mithreshold_; }
    uint16_t getSithreshold() const { return sithreshold_; }

    // miconfig (Smnip/Smip). Bits per spec §Smnip "New Interrupt Configuration":
    //   [19:16] snipbits, [11:8] mnipbits, [5] sipu, [4] mipu, [3] snipen, [2] mnipen
    // siconfig is a subset alias of miconfig (snipbits, sipu, snipen).
    void setMiconfig(uint32_t v) { miconfig_ = v & miconfigMask(); }
    uint32_t getMiconfig() const { return miconfig_; }
    // siconfig writes leak through to miconfig's S-mode-controlled fields.
    void setSiconfig(uint32_t v);
    uint32_t getSiconfig() const;
    bool isMnipEnabled() const { return (miconfig_ >> 2) & 1; }
    bool isSnipEnabled() const { return (miconfig_ >> 3) & 1; }
    bool isMipuEnabled() const { return (miconfig_ >> 4) & 1; }
    bool isSipuEnabled() const { return (miconfig_ >> 5) & 1; }
    unsigned getMnipBits() const { return (miconfig_ >> 8) & 0xF; }
    unsigned getSnipBits() const { return (miconfig_ >> 16) & 0xF; }

private:
    uint32_t miconfigMask() const;
public:

    // CSR indirect access via miselect/miregN (machine domain)
    template<typename URV> bool readMireg(URV sel, URV& value) const;
    template<typename URV> bool writeMireg(URV sel, URV value);
    template<typename URV> bool readMireg2(URV sel, URV& value) const;
    template<typename URV> bool writeMireg2(URV sel, URV value);
    template<typename URV> bool readMireg3(URV sel, URV& value) const;
    template<typename URV> bool writeMireg3(URV sel, URV value);
    // mireg4 at miselect=0x1000 accesses miconfig (Smnip).  Other selectors raise illegal.
    template<typename URV> bool readMireg4(URV sel, URV& value) const;
    template<typename URV> bool writeMireg4(URV sel, URV value);

    // CSR indirect access via siselect/siregN (supervisor domain)
    template<typename URV> bool readSireg(URV sel, URV& value) const;
    template<typename URV> bool writeSireg(URV sel, URV value);
    template<typename URV> bool readSireg2(URV sel, URV& value) const;
    template<typename URV> bool writeSireg2(URV sel, URV value);
    template<typename URV> bool readSireg3(URV sel, URV& value) const;
    template<typename URV> bool writeSireg3(URV sel, URV value);
    // sireg4 at siselect=0x1000 accesses siconfig (subset alias of miconfig).
    template<typename URV> bool readSireg4(URV sel, URV& value) const;
    template<typename URV> bool writeSireg4(URV sel, URV value);

    // External interrupt source assertion
    void setSourceState(unsigned i, bool state);

    // Callback: called when top-priority interrupt delivery state changes
    using DeliveryCallback = std::function<void(bool isMachine, bool pending)>;
    void setDeliveryCallback(const DeliveryCallback& cb);

    // Return highest-priority (lowest iprio, then lowest source number)
    // pending+enabled source id for machine (isMachine=true) or supervisor domain.
    // Returns 0 if none. If prio is non-null, *prio is set to the effective
    // priority of the winning source (0 when return value is 0).
    // If ignoreThreshold is true, mithreshold/sithreshold is not applied (used
    // for xtopei reads: eithreshold is excluded from ACLIC scope per spec).
    unsigned topInterrupt(bool isMachine, unsigned* prio = nullptr,
                          bool ignoreThreshold = false) const;

    // Clear the pending bit for source src in the given domain.
    // No-op for Level1/Level0 sources whose pending bit is driven by the
    // hardware input level and cannot meaningfully be cleared while it holds.
    // Used by xtopei claim writes and IVT hardware dispatch.
    void tryClearPending(bool isMachine, unsigned src);

private:
    // ---- Per-source domain state ----
    // M-domain sourcecfg: bits[2:0] = SM (source mode), bit[10] = D (delegate to S).
    // S-domain sourcecfg: bits[2:0] = SM only (leaf domain, no D bit).
    // When D=1 in m_sourcecfg_, the source belongs to S-domain.
    // Pending/enabled/iprio are maintained per-domain; only the owning domain's
    // arrays are meaningful (the other may contain stale values).
    std::vector<uint16_t> m_sourcecfg_;
    std::vector<uint16_t> s_sourcecfg_;
    std::vector<bool>     source_states_; // current hardware input state
    std::vector<bool>     m_pending_;
    std::vector<bool>     m_enabled_;
    std::vector<uint8_t>  m_iprio_;
    std::vector<bool>     s_pending_;
    std::vector<bool>     s_enabled_;
    std::vector<uint8_t>  s_iprio_;

    unsigned numSources_;
    bool hasSupervisorDomain_;
    unsigned ipriolen_;
    uint16_t mithreshold_ = 0;     // 9-bit (IPRIOLEN+1) per Smnip spec
    uint16_t sithreshold_ = 0;     // 9-bit
    uint32_t miconfig_ = 0;        // Phase 4: replaces mipreemptcfg
    uint8_t mPreemptmsk_ = 0;      // DEPRECATED — kept until all callers migrated
    DeliveryCallback deliveryCb_;

    void updateDelivery(bool isMachine);

    // ---- Source domain helpers ----

    // True if source is delegated to S-domain (D=1 in m_sourcecfg_).
    bool isDelegated(unsigned src) const;

    // Effective source mode for source src.  S-domain SM (sireg2) takes
    // priority when non-zero; otherwise falls back to M-domain SM (mireg2).
    unsigned effectiveSm(unsigned src) const;

    // True if source is active (SM != 0) in the given domain.
    // M-domain: not delegated and effective SM != 0.
    // S-domain: delegated and effective SM != 0.
    bool isSourceActive(unsigned src, bool isMachine) const;

    // Set the pending bit for source i in the owning domain based on
    // current hardware state and SM (Level1/Level0 only).
    void evalLevelPending(unsigned i, unsigned sm, bool isMachine);

    // ---- Source configuration ----

    // Validate and apply a sourcecfg write for source i.  supervisorDomain
    // must be true when the write originates from S-level (sireg2/sireg3).
    void applySourcecfg(unsigned i, uint16_t val, bool supervisorDomain);

    // Clear all per-source state in both domains.
    void clearAllState(unsigned i);

    // Clear per-source state in the given domain only.
    void clearDomainState(unsigned i, bool isMachine);

    // ---- Register access helpers ----

    template<typename URV> bool readEip(bool isMachine, URV k, URV& value) const;
    template<typename URV> bool writeEip(bool isMachine, URV k, URV value);

    template<typename URV> bool readEie(bool isMachine, URV k, URV& value) const;
    template<typename URV> bool writeEie(bool isMachine, URV k, URV value);

    template<typename URV> bool readIprio(bool isMachine, URV k, URV& value) const;
    template<typename URV> bool writeIprio(bool isMachine, URV k, URV value);

    template<typename URV> bool readSourcecfgPacked(URV k, URV& value, bool supervisorDomain) const;
    template<typename URV> bool writeSourcecfgPacked(URV k, URV value, bool supervisorDomain);

    template<typename URV> bool readSourcecfgPacked3(URV k, URV& value, bool supervisorDomain) const;
    template<typename URV> bool writeSourcecfgPacked3(URV k, URV value, bool supervisorDomain);
};

} // namespace TT_ACLIC
