#pragma once

#include <vector>
#include <functional>
#include <cstdint>

namespace TT_ACLIC {

struct AclicParams {
    unsigned numSources = 32;          // 1–1023
    bool hasSupervisorDomain = false;
    unsigned ipriolen = 8;             // 1–8
};

class Aclic {
public:
    explicit Aclic(const AclicParams& params);

    void reset();

    unsigned numSources() const { return numSources_; }
    bool hasSupervisorDomain() const { return hasSupervisorDomain_; }

    // CSR indirect access via miselect/miregN (machine domain)
    template<typename URV> bool readMireg(URV sel, URV& value) const;
    template<typename URV> bool writeMireg(URV sel, URV value);
    template<typename URV> bool readMireg2(URV sel, URV& value) const;
    template<typename URV> bool writeMireg2(URV sel, URV value);

    // CSR indirect access via siselect/siregN (supervisor domain)
    template<typename URV> bool readSireg(URV sel, URV& value) const;
    template<typename URV> bool writeSireg(URV sel, URV value);
    template<typename URV> bool readSireg2(URV sel, URV& value) const;
    template<typename URV> bool writeSireg2(URV sel, URV value);

    // External interrupt source assertion
    void setSourceState(unsigned i, bool state);

    // Callback: called when top-priority interrupt delivery state changes
    using DeliveryCallback = std::function<void(bool isMachine, bool pending)>;
    void setDeliveryCallback(const DeliveryCallback& cb);

    // Return highest-priority (lowest iprio, then lowest source number)
    // pending+enabled source id for machine (isMachine=true) or supervisor domain.
    // Returns 0 if none.
    unsigned topInterrupt(bool isMachine) const;

private:
    // Per-source state (index 0 unused; 1..numSources_)
    std::vector<uint16_t> sourcecfg_;   // bits[2:0]=SM, bit[10]=D(delegate to S)
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
    DeliveryCallback deliveryCb_;

    void updateDelivery(bool isMachine);

    // Helpers for eip (pending bit arrays), sel = 0x80..0xBF
    template<typename URV> bool readEip(bool isMachine, URV k, URV& value) const;
    template<typename URV> bool writeEip(bool isMachine, URV k, URV value);

    // Helpers for eie (enable bit arrays), sel = 0xC0..0xFF
    template<typename URV> bool readEie(bool isMachine, URV k, URV& value) const;
    template<typename URV> bool writeEie(bool isMachine, URV k, URV value);

    // Helpers for acliciprio (packed iprio bytes), sel = 0x1000..0x10FF, via xireg
    template<typename URV> bool readIprio(bool isMachine, URV k, URV& value) const;
    template<typename URV> bool writeIprio(bool isMachine, URV k, URV value);

    // Helpers for aclicsourcecfg (packed 16-bit sourcecfg fields), sel = 0x1000..0x10FF, via xireg2
    template<typename URV> bool readSourcecfgPacked(URV k, URV& value) const;
    template<typename URV> bool writeSourcecfgPacked(URV k, URV value);

    // Validate and apply a sourcecfg write for source i
    void applySourcecfg(unsigned i, uint16_t val);
};

} // namespace TT_ACLIC
