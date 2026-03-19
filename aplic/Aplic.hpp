#pragma once

#include <string>
#include <span>
#include <vector>
#include <optional>
#include <memory>
#include <cassert>

#include "Domain.hpp"

namespace TT_APLIC {

class Aplic
{
public:

    /// Constructor. Define an APLIC with the given number of harts, number of
    /// interrupt sources, and domain parameters.
    Aplic(unsigned num_harts, unsigned num_sources, std::span<const DomainParams> domain_params_list);

    /// Return the root domain of this APLIC.
    std::shared_ptr<Domain> root() const { return root_; }

    /// Return the number of harts associated with this APLIC.
    unsigned numHarts() const { return num_harts_; }

    /// Return the number of interrupt sources associated with this APLIC.
    unsigned numSources() const { return num_sources_; }

    /// Return a pointer to the associated domain with the given name or a null pointer if
    /// no such domain.
    std::shared_ptr<Domain> findDomainByName(std::string_view name) const;

    /// Return a pointer to the associated domain with the given address or a null
    /// pointer if no such domain.
    std::shared_ptr<Domain> findDomainByAddr(uint64_t addr) const;

    /// Reset this APLIC.
    void reset();

    /// Return true if given address is within the address range of this APLIC.
    bool containsAddr(uint64_t addr) const;

    /// Read the APLIC register at the given address placing the value in data and
    /// returning true if successful and false otherwise in which case data is left
    /// unmodified. Given address must be absolute and not an offset from the base address
    /// of the APLIC. Size is the read size in byes. Fail if addr is not word aligned or
    /// if size is not 4 (word).
    bool read(uint64_t addr, size_t size, uint32_t& data) const;

    /// Write given data to the APLIC register at the given address and returning true if
    /// successful and false otherwise in which case no register is modified. Given
    /// address must be absolute and not an offset from the base address of the
    /// APLIC. Size is the write size in byes. Fail if addr is not word aligned or if size
    /// is not 4 (word).
    bool write(uint64_t addr, size_t size, uint32_t data) const;

    /// Define the callback to be invoked by this APLIC to deliver interrupts marked for
    /// direct (wired) deliver.
    void setDirectCallback(const DirectDeliveryCallback& callback);

    /// Define the callback to be invoked by this APLIC to deliver interrupts marked for
    /// MSI deliver.
    void setMsiCallback(const MsiDeliveryCallback& callback);

    /// Return the state of the ith source.
    bool getSourceState(unsigned i) const { return source_states_.at(i); }

    /// Set the state of the ith source.
    void setSourceState(unsigned i, bool state);

    /// Mark the ith source as being delivered using message signaled interrupt.
    bool forwardViaMsi(unsigned i);

    bool autoForwardViaMsi = true;

private:
    std::shared_ptr<Domain> createDomain(const DomainParams& params);

    unsigned num_harts_;
    unsigned num_sources_;
    std::shared_ptr<Domain> root_;
    std::vector<std::shared_ptr<Domain>> domains_;
    std::vector<bool> source_states_;
    DirectDeliveryCallback direct_callback_ = nullptr;
    MsiDeliveryCallback msi_callback_ = nullptr;
};

}
