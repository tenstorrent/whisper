#pragma once

#include "IommuStructures.hpp"
#include <array>
#include <iostream>

using namespace TT_IOMMU;

namespace IOMMU {


class MemoryManager {
public:
    MemoryManager() {
        // Initialize free page lists
        for (uint32_t gscid = 0; gscid < MAX_GSCID; gscid++) {
            next_free_gpage_.at(gscid) = 0;
        }
    }

    // Get free physical page numbers (equivalent to get_free_ppn)
    uint64_t getFreePhysicalPages(uint64_t num_pages) {
        uint64_t free_ppn = next_free_page_;

        // Align to requested number of pages
        if (free_ppn & (num_pages - 1)) {
            free_ppn = free_ppn + (num_pages - 1);
            free_ppn = free_ppn & ~(num_pages - 1);
        }

        next_free_page_ = free_ppn + num_pages;

        std::cout << "[MEM_MGR] Allocated " << num_pages << " physical pages starting at PPN 0x"
                  << std::hex << free_ppn << std::dec << '\n';

        return free_ppn;
    }

    // Get free guest physical page numbers (equivalent to get_free_gppn)
    uint64_t getFreeGuestPages(uint64_t num_pages, const Iohgatp& iohgatp) {
        uint16_t gscid = iohgatp.bits_.gcsid_;
        if (gscid >= MAX_GSCID) {
            std::cerr << "[MEM_MGR] Invalid GSCID: " << gscid << '\n';
            return 0;
        }

        uint64_t free_gppn = next_free_gpage_.at(gscid);

        // Align to requested number of pages
        if (free_gppn & (num_pages - 1)) {
            free_gppn = free_gppn + (num_pages - 1);
            free_gppn = free_gppn & ~(num_pages - 1);
        }

        next_free_gpage_.at(gscid) = free_gppn + num_pages;

        std::cout << "[MEM_MGR] Allocated " << num_pages << " guest pages for GSCID "
                  << gscid << " starting at GPPN 0x" << std::hex << free_gppn << std::dec << '\n';

        return free_gppn;
    }

    // Reset allocation state
    void reset() {
        for (uint32_t gscid = 0; gscid < MAX_GSCID; gscid++) {
            next_free_gpage_.at(gscid) = 0;
        }
        next_free_page_ = 0;
        std::cout << "[MEM_MGR] Reset page allocation state" << '\n';
    }

    // Get allocation statistics
    void printStats() const {
        std::cout << "[MEM_MGR] Next free PPN: 0x" << std::hex << next_free_page_ << std::dec << '\n';
        std::cout << "[MEM_MGR] Active GSCIDs with allocations:" << '\n';
        for (uint32_t gscid = 0; gscid < MAX_GSCID; gscid++) {
            if (next_free_gpage_.at(gscid) > 0) {
                std::cout << "  GSCID " << gscid << ": next GPPN 0x"
                          << std::hex << next_free_gpage_.at(gscid) << std::dec << '\n';
            }
        }
    }

private:
    static constexpr uint32_t MAX_GSCID = 65536;

    uint64_t next_free_page_ = 0;
    std::array<uint64_t, MAX_GSCID> next_free_gpage_{};
};

} // namespace IOMMU

