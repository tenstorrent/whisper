#pragma once

#include "IommuStructures.hpp"
#include "MemoryManager.hpp"
#include "MemoryModel.hpp"
#include <array>
#include <iostream>
#include <functional>

using namespace TT_IOMMU;

namespace IOMMU {

class TableBuilder {
public:
    using MemoryReadFunc = std::function<bool(uint64_t, unsigned, uint64_t&, bool&)>;
    using MemoryWriteFunc = std::function<bool(uint64_t, unsigned, uint64_t)>;

    TableBuilder(MemoryManager& memMgr, MemoryReadFunc readFunc,
                 MemoryWriteFunc writeFunc)
      : mem_mgr_(memMgr), read_func_(std::move(readFunc)), write_func_(std::move(writeFunc)) {}

    // Build Device Directory Table entry (adapted from add_dev_context)
    uint64_t addDeviceContext(const ExtendedDeviceContext& dc, uint32_t device_id,
                             const Ddtp& ddtp, bool msi_flat = false) {
        std::array<uint16_t, 3> ddi{};
        uint8_t dc_size = 0;

        // Calculate device directory indexes based on MSI format
        if (!msi_flat) {
            ddi.at(0) = get_bits(6, 0, device_id);
            ddi.at(1) = get_bits(15, 7, device_id);
            ddi.at(2) = get_bits(23, 16, device_id);
            dc_size = BASE_FORMAT_DC_SIZE;
        } else {
            ddi.at(0) = get_bits(5, 0, device_id);
            ddi.at(1) = get_bits(14, 6, device_id);
            ddi.at(2) = get_bits(23, 15, device_id);
            dc_size = EXT_FORMAT_DC_SIZE;
        }

        uint64_t addr = ddtp.fields.ppn * PAGESIZE;
        uint8_t levels = ddtp.levels();

        if (levels == 0) {
            std::cerr << "[TABLE] Invalid DDT mode\n";
            return 0;
        }

        // Walk down the directory levels
        for (int i = levels - 1; i > 0; i--) {
            Ddte ddte;
            bool corrupted = false;
            uint64_t entry_addr = addr + (ddi.at(i) * uint64_t(8));

            if (!read_func_(entry_addr, 8, ddte.value_, corrupted)) {
                std::cerr << "[TABLE] Failed to read DDTE at 0x" << std::hex << entry_addr << '\n';
                return 0;
            }

            if (ddte.bits_.v_ == 0) {
                // Allocate new page for next level
                ddte.bits_.v_ = 1;
                ddte.bits_.ppn_ = mem_mgr_.getFreePhysicalPages(1);

                if (!write_func_(entry_addr, 8, ddte.value_)) {
                    std::cerr << "[TABLE] Failed to write DDTE at 0x" << std::hex << entry_addr << '\n';
                    return 0;
                }

                std::cout << "[TABLE] Created DDT level " << i << " entry at 0x"
                          << std::hex << entry_addr << " -> PPN 0x" << ddte.bits_.ppn_ << std::dec << '\n';
            }

            addr = ddte.bits_.ppn_ * PAGESIZE;
        }

        // Write device context at leaf level
        uint64_t dc_addr = addr + (ddi.at(0) * uint64_t(dc_size));
        bool ok = true;
        ok = ok and write_func_(dc_addr + 0,  8, dc.tc_);
        ok = ok and write_func_(dc_addr + 8,  8, dc.iohgatp_);
        ok = ok and write_func_(dc_addr + 16, 8, dc.ta_);
        ok = ok and write_func_(dc_addr + 24, 8, dc.fsc_);
        if (msi_flat)
          {
            ok = ok and write_func_(dc_addr + 32, 8, dc.msiptp_);
            ok = ok and write_func_(dc_addr + 40, 8, dc.msimask_);
            ok = ok and write_func_(dc_addr + 48, 8, dc.msipat_);
            ok = ok and write_func_(dc_addr + 56, 8, 0);  // reserved field
          }

        if (not ok) {
            std::cerr << "[TABLE] Failed to write device context at 0x" << std::hex << dc_addr << '\n';
            return 0;
        }

        std::cout << "[TABLE] Added device context for device_id 0x" << std::hex << device_id
                  << " at address 0x" << dc_addr << std::dec << '\n';

        return dc_addr;
    }

    // Build Process Directory Table entry (adapted from add_process_context)
    uint64_t addProcessContext(const ExtendedDeviceContext& dc, bool gxl, const ProcessContext& pc,
                              uint32_t process_id) {
        std::array<uint16_t, 3> pdi{};

        pdi.at(0) = get_bits(7, 0, process_id);
        pdi.at(1) = get_bits(16, 8, process_id);
        pdi.at(2) = get_bits(19, 17, process_id);

        // FSC holds PDTP when PDTV=1
        TT_IOMMU::Fsc fsc(dc.fsc_);
        uint8_t levels = 0;
        auto pdtpMode = static_cast<TT_IOMMU::PdtpMode>(fsc.bits_.mode_);
        switch (pdtpMode) {
            case TT_IOMMU::PdtpMode::Pd20: levels = 3; break;
            case TT_IOMMU::PdtpMode::Pd17: levels = 2; break;
            case TT_IOMMU::PdtpMode::Pd8: levels = 1; break;
            default:
                std::cerr << "[TABLE] Invalid PDT mode\n";
                return 0;
        }

        uint64_t addr = fsc.bits_.ppn_ * PAGESIZE;

        // Walk down the process directory levels
        TT_IOMMU::Iohgatp iohgatp(dc.iohgatp_);
        assert(dc.iohgatp_ == 0); // XXX: only testing Bare right now; should test all modes
        for (int i = levels - 1; i > 0; i--) {
            // Translate through G-stage if needed
            if (iohgatp.bits_.mode_ != TT_IOMMU::IohgatpMode::Bare) {
                uint64_t spa = 0;
                if (!translateGPA(iohgatp, addr, spa)) {
                    std::cerr << "[TABLE] G-stage translation failed for addr 0x"
                              << std::hex << addr << '\n';
                    return 0;
                }
                addr = spa;
            }

            Pdte pdte;
            bool corrupted = false;
            uint64_t entry_addr = addr + (pdi.at(i) * uint64_t(8));

            if (!read_func_(entry_addr, 8, pdte.value_, corrupted)) {
                std::cerr << "[TABLE] Failed to read PDTE at 0x" << std::hex << entry_addr << '\n';
                return 0;
            }

            if (pdte.bits_.v_ == 0) {
                pdte.bits_.v_ = 1;

                if (iohgatp.bits_.mode_ != TT_IOMMU::IohgatpMode::Bare) {
                    // Allocate guest page and map it
                    pdte.bits_.ppn_ = mem_mgr_.getFreeGuestPages(1, iohgatp);

                    // Create G-stage mapping for the allocated page
                    gpte_t gpte;
                    gpte.V = 1;
                    gpte.R = 1;
                    gpte.W = 0;
                    gpte.X = 0;
                    gpte.U = 1;
                    gpte.G = 0;
                    gpte.A = 0;
                    gpte.D = 0;
                    gpte.PBMT = PMA;
                    gpte.PPN = mem_mgr_.getFreePhysicalPages(1);

                    if (!addGStagePageTableEntry(iohgatp, gxl, PAGESIZE * pdte.bits_.ppn_, gpte, 0)) {
                        std::cerr << "[TABLE] Failed to create G-stage mapping" << '\n';
                        return 0;
                    }
                } else {
                    pdte.bits_.ppn_ = mem_mgr_.getFreePhysicalPages(1);
                }

                if (!write_func_(entry_addr, 8, pdte.value_)) {
                    std::cerr << "[TABLE] Failed to write PDTE at 0x" << std::hex << entry_addr << '\n';
                    return 0;
                }

                std::cout << "[TABLE] Created PDT level " << i << " entry at 0x"
                          << std::hex << entry_addr << " -> PPN 0x" << pdte.bits_.ppn_ << std::dec << '\n';
            }

            addr = pdte.bits_.ppn_ * PAGESIZE;
        }

        // Translate final address if needed
        if (iohgatp.bits_.mode_ != TT_IOMMU::IohgatpMode::Bare) {
            uint64_t spa = 0;
            if (!translateGPA(iohgatp, addr, spa)) {
                std::cerr << "[TABLE] Final G-stage translation failed" << '\n';
                return 0;
            }
            addr = spa;
        }

        // Write process context at leaf level
        uint64_t pc_addr = addr + (pdi.at(0) * sizeof(pc));
        bool ok = true;
        ok = ok and write_func_(pc_addr + 0, 8, pc.ta());
        ok = ok and write_func_(pc_addr + 8, 8, pc.fsc());
        if (not ok) {
            std::cerr << "[TABLE] Failed to write process context at 0x" << std::hex << pc_addr << '\n';
            return 0;
        }

        std::cout << "[TABLE] Added process context for process_id 0x" << std::hex << process_id
                  << " at address 0x" << pc_addr << std::dec << '\n';

        return pc_addr;
    }

    // Add G-stage page table entry (adapted from add_g_stage_pte)
    bool addGStagePageTableEntry(const Iohgatp& iohgatp, bool gxl, uint64_t gpa,
                                const gpte_t& gpte, uint8_t add_level) {
        std::array<uint16_t, 5> vpn{};
        uint8_t levels = 0, pte_size = 8;

        // Determine levels and VPN extraction based on mode
        switch (iohgatp.bits_.mode_) {
            // NOTE: Sv32x4 == Sv39x4 == 8. Use fctl.GXL to differentiate.
            case TT_IOMMU::IohgatpMode::Sv39x4: {
                if (gxl) {
                    vpn.at(0) = get_bits(21, 12, gpa);
                    vpn.at(1) = get_bits(34, 22, gpa);
                    levels = 2;
                    pte_size = 4; // 32-bit PTEs
                } else {
                    vpn.at(0) = get_bits(20, 12, gpa);
                    vpn.at(1) = get_bits(29, 21, gpa);
                    vpn.at(2) = get_bits(40, 30, gpa);
                    levels = 3;
                }
                break;
            }
            case TT_IOMMU::IohgatpMode::Sv48x4:
                vpn.at(0) = get_bits(20, 12, gpa);
                vpn.at(1) = get_bits(29, 21, gpa);
                vpn.at(2) = get_bits(38, 30, gpa);
                vpn.at(3) = get_bits(49, 39, gpa);
                levels = 4;
                break;
            case TT_IOMMU::IohgatpMode::Sv57x4:
                vpn.at(0) = get_bits(20, 12, gpa);
                vpn.at(1) = get_bits(29, 21, gpa);
                vpn.at(2) = get_bits(38, 30, gpa);
                vpn.at(3) = get_bits(47, 39, gpa);
                vpn.at(4) = get_bits(58, 48, gpa);
                levels = 5;
                break;
            default:
                std::cerr << "[TABLE] Invalid IOHGATP mode: " << static_cast<int>(iohgatp.bits_.mode_) << '\n';
                return false;
        }

        uint64_t addr = iohgatp.bits_.ppn_ * PAGESIZE;

        // Walk down page table levels
        for (int i = levels - 1; i > add_level; i--) {
            gpte_t nl_gpte;
            bool corrupted = false;
            uint64_t entry_addr = addr | (vpn.at(i) * uint64_t(pte_size));

            if (!read_func_(entry_addr, pte_size, nl_gpte.raw, corrupted)) {
                std::cerr << "[TABLE] Failed to read G-stage PTE at 0x" << std::hex << entry_addr << '\n';
                return false;
            }

            if (nl_gpte.V == 0) {
                nl_gpte.V = 1;
                nl_gpte.PPN = mem_mgr_.getFreePhysicalPages(1);

                if (!write_func_(entry_addr, pte_size, nl_gpte.raw)) {
                    std::cerr << "[TABLE] Failed to write G-stage PTE at 0x" << std::hex << entry_addr << '\n';
                    return false;
                }

                std::cout << "[TABLE] Created G-stage PT level " << i << " entry at 0x"
                          << std::hex << entry_addr << " -> PPN 0x" << nl_gpte.PPN << std::dec << '\n';
            }

            addr = nl_gpte.PPN * PAGESIZE;
        }

        // Write leaf PTE
        uint64_t leaf_addr = addr | (vpn.at(add_level) * uint64_t(pte_size));
        if (!write_func_(leaf_addr, pte_size, gpte.raw)) {
            std::cerr << "[TABLE] Failed to write G-stage leaf PTE at 0x" << std::hex << leaf_addr << '\n';
            return false;
        }

        std::cout << "[TABLE] Added G-stage PTE for GPA 0x" << std::hex << gpa
                  << " at address 0x" << leaf_addr << std::dec << '\n';

        return true;
    }

        // Create MSI-enabled device context (extension of addDeviceContext)
    uint64_t addMsiDeviceContext(const ExtendedDeviceContext& dc, uint32_t device_id,
                                const Ddtp& ddtp, bool msi_flat,
                                uint64_t msi_addr_mask, uint64_t msi_addr_pattern,
                                uint64_t msiptp) {

        // First create the basic device context structure
        uint64_t dc_addr = addDeviceContext(dc, device_id, ddtp, msi_flat);

        if (dc_addr == 0) {
            std::cerr << "[TABLE] Failed to create basic device context for MSI device" << '\n';
            return 0;
        }

        // If using extended format, write additional MSI fields
        if (msi_flat) {
            // Write MSI-specific fields to extended device context
            // Extended format: tc, iohgatp, ta, fsc, msiptp, msi_addr_mask, msi_addr_pattern, reserved

            uint64_t msiptp_addr = dc_addr + 32; // After base format fields
            uint64_t msi_mask_addr = dc_addr + 40;
            uint64_t msi_pattern_addr = dc_addr + 48;

            if (!write_func_(msiptp_addr, 8, msiptp) ||
                !write_func_(msi_mask_addr, 8, msi_addr_mask) ||
                !write_func_(msi_pattern_addr, 8, msi_addr_pattern)) {
                std::cerr << "[TABLE] Failed to write MSI fields to device context" << '\n';
                return 0;
            }

            std::cout << "[TABLE] Added MSI fields to device context at 0x" << std::hex << dc_addr
                      << ": MSIPTP=0x" << msiptp << ", mask=0x" << msi_addr_mask
                      << ", pattern=0x" << msi_addr_pattern << std::dec << '\n';
        }

        return dc_addr;
    }

    // Setup MSI page table for Flat mode
    bool setupMsiPageTable(uint64_t msi_ppn, uint64_t target_ppn,
                          uint16_t num_entries = 16) {
        uint64_t pageSize = 4096;
        uint64_t msiTableAddr = msi_ppn * pageSize;

        std::cout << "[TABLE] Setting up MSI page table at PPN 0x" << std::hex << msi_ppn
                  << " with " << std::dec << num_entries << " entries" << '\n';

        // Create valid MSI PTEs for each entry
        for (uint16_t i = 0; i < num_entries; i++) {
            // Basic translate mode PTE (mode 3)
            // Format: V=1, M=3 (basic translate), PPN=target
            uint64_t pte = 0;
            pte |= 0x1;                    // V bit (valid)
            pte |= (0x3ULL << 1);          // M bits (mode 3 = basic translate)
            pte |= (target_ppn << 10);     // PPN field

            uint64_t pte_addr = msiTableAddr + (i * uint64_t(8));
            if (!write_func_(pte_addr, 8, pte)) {
                std::cerr << "[TABLE] Failed to write MSI PTE " << i
                          << " at address 0x" << std::hex << pte_addr << '\n';
                return false;
            }
        }

        std::cout << "[TABLE] MSI page table setup complete: " << num_entries
                  << " entries pointing to PPN 0x" << std::hex << target_ppn << std::dec << '\n';

        return true;
    }

    // Add S-stage page table entry (adapted from add_s_stage_pte)
    bool addSStagePageTableEntry(const Iosatp& satp, uint64_t va,
                                const pte_t& pte, uint8_t add_level, uint8_t sxl = 0) {
        std::array<uint16_t, 5> vpn{};
        uint8_t levels = 0, pte_size = 8;

        // Determine levels and VPN extraction based on mode
        switch (satp.bits_.mode_) {
            // NOTE: Sv32 == Sv39 == 8. Use DC.tc.SXL to differentiate.
            case TT_IOMMU::IosatpMode::Sv39:
                if (sxl == 1) {
                    vpn.at(0) = get_bits(21, 12, va);
                    vpn.at(1) = get_bits(31, 22, va);
                    levels = 2;
                    pte_size = 4; // 32-bit PTEs
                } else {
                    vpn.at(0) = get_bits(20, 12, va);
                    vpn.at(1) = get_bits(29, 21, va);
                    vpn.at(2) = get_bits(38, 30, va);
                    levels = 3;
                }
                break;
            case TT_IOMMU::IosatpMode::Sv48:
                vpn.at(0) = get_bits(20, 12, va);
                vpn.at(1) = get_bits(29, 21, va);
                vpn.at(2) = get_bits(38, 30, va);
                vpn.at(3) = get_bits(47, 39, va);
                levels = 4;
                break;
            case TT_IOMMU::IosatpMode::Sv57:
                vpn.at(0) = get_bits(20, 12, va);
                vpn.at(1) = get_bits(29, 21, va);
                vpn.at(2) = get_bits(38, 30, va);
                vpn.at(3) = get_bits(47, 39, va);
                vpn.at(4) = get_bits(56, 48, va);
                levels = 5;
                break;
            default:
                std::cerr << "[TABLE] Invalid IOSATP mode\n";
                return false;
        }

        uint64_t addr = satp.bits_.ppn_ * PAGESIZE;

        // Walk down page table levels
        for (int i = levels - 1; i > add_level; i--) {
            pte_t nl_pte;
            bool corrupted = false;
            uint64_t entry_addr = addr | (vpn.at(i) * uint64_t(pte_size));

            if (!read_func_(entry_addr, pte_size, nl_pte.raw, corrupted)) {
                std::cerr << "[TABLE] Failed to read S-stage PTE at 0x" << std::hex << entry_addr << '\n';
                return false;
            }

            if (nl_pte.V == 0) {
                nl_pte.V = 1;
                nl_pte.PPN = mem_mgr_.getFreePhysicalPages(1);

                if (!write_func_(entry_addr, pte_size, nl_pte.raw)) {
                    std::cerr << "[TABLE] Failed to write S-stage PTE at 0x" << std::hex << entry_addr << '\n';
                    return false;
                }

                std::cout << "[TABLE] Created S-stage PT level " << i << " entry at 0x"
                          << std::hex << entry_addr << " -> PPN 0x" << nl_pte.PPN << std::dec << '\n';
            }

            addr = nl_pte.PPN * PAGESIZE;
        }

        // Write leaf PTE
        uint64_t leaf_addr = addr | (vpn.at(add_level) * uint64_t(pte_size));
        if (!write_func_(leaf_addr, pte_size, pte.raw)) {
            std::cerr << "[TABLE] Failed to write S-stage leaf PTE at 0x" << std::hex << leaf_addr << '\n';
            return false;
        }

        std::cout << "[TABLE] Added S-stage PTE for VA 0x" << std::hex << va
                  << " at address 0x" << leaf_addr << std::dec << '\n';

        return true;
    }

    // Simplified GPA translation (basic version of translate_gpa)
    static bool translateGPA(const Iohgatp& iohgatp, uint64_t gpa, uint64_t& spa) {
        if (iohgatp.bits_.mode_ == TT_IOMMU::IohgatpMode::Bare) {
            spa = gpa;
            return true;
        }

        // For now, implement a basic translation - in real implementation
        // this would walk the G-stage page tables
        spa = gpa; // Placeholder - implement full translation as needed
        return true;
    }

    // Create fault-testing device context with specific flags
    uint64_t addFaultTestDevice(uint32_t device_id, const Ddtp& ddtp,
                               bool dtf_enabled = false, bool sbe_enabled = false,
                               bool pdtv_enabled = false, uint64_t pdtp_value = 0) {
        ExtendedDeviceContext dc = {};

        // Set basic fields in tc (translation control) field
        uint64_t tc_value = 0;
        tc_value |= 1ULL;  // V=1 (Valid)
        if (dtf_enabled) tc_value |= (1ULL << 6);  // DTF bit
        if (sbe_enabled) tc_value |= (1ULL << 8);  // SBE bit
        if (pdtv_enabled) tc_value |= (1ULL << 10); // PDTV bit
        dc.tc_ = tc_value;

        // Set process directory pointer if PDTV is enabled
        if (pdtv_enabled && pdtp_value != 0) {
            dc.fsc_ = pdtp_value;
        }

        return addDeviceContext(dc, device_id, ddtp);
    }

    // Create invalid device context (V=0) for fault testing
    uint64_t addInvalidDevice(uint32_t device_id, const Ddtp& ddtp) {
        ExtendedDeviceContext dc = {}; // All zeros - V=0 makes it invalid
        return addDeviceContext(dc, device_id, ddtp);
    }

private:
    MemoryManager& mem_mgr_;
    MemoryReadFunc read_func_;
    MemoryWriteFunc write_func_;
};

} // namespace IOMMU
