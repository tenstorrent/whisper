#include "Aplic.hpp"
#include "Domain.hpp"

using namespace TT_APLIC;

Domain::Domain(
    const Aplic *aplic,
    const std::shared_ptr<Domain>& parent,
    const DomainParams& params
):
    aplic_(aplic),
    parent_(parent),
    params_(params)
{
    assert(params.direct_mode_supported or params.msi_mode_supported);
    assert(params.le_supported or params.be_supported);
    unsigned num_harts = aplic->numHarts();
    xeip_bits_.resize(num_harts);
    idcs_.resize(num_harts);
    reset();
}

void Domain::reset()
{
    domaincfg_ = Domaincfg{};
    domaincfg_.legalize(params_);
    mmsiaddrcfg_ = params_.mmsiaddrcfg;
    mmsiaddrcfgh_ = Mmsiaddrcfgh{params_.mmsiaddrcfgh};
    smsiaddrcfg_ = params_.smsiaddrcfg;
    smsiaddrcfgh_ = Smsiaddrcfgh{params_.smsiaddrcfgh};
    sourcecfg_.fill(Sourcecfg{});
    target_.fill(Target{});
    setip_.fill(0);
    setie_.fill(0);

    unsigned num_harts = aplic_->numHarts();
    for (unsigned i = 0; i < num_harts; i++)
        xeip_bits_[i] = 0;

    for (unsigned i = 0; i < num_harts; i++)
        idcs_[i] = Idc{};

    for (const auto& child : children_)
        child->reset();
}

void Domain::updateTopi()
{
    if (dmIsMsi())
        return;
    unsigned num_sources = aplic_->numSources();
    for (auto hart_index : params_.hart_indices) {
        idcs_[hart_index].topi = Topi{};
    }
    for (unsigned i = 1; i <= num_sources; i++) {
        unsigned hart_index = target_.at(i).dm0.hart_index;
        if (not includesHart(hart_index))
            continue;
        unsigned priority = target_.at(i).dm0.iprio;
        unsigned ithreshold = idcs_[hart_index].ithreshold;
        auto& topi = idcs_[hart_index].topi;
        unsigned topi_prio = topi.fields.priority;
        bool under_threshold = ithreshold == 0 or priority < ithreshold;
        if (under_threshold and pending(i) and enabled(i) and (priority < topi_prio or topi_prio == 0)) {
            topi.fields.priority = priority;
            topi.fields.iid = i;
        }
        topi.legalize();
    }
}

void Domain::inferXeipBits()
{
    for (unsigned i : params_.hart_indices)
        xeip_bits_[i] = 0;
    if (domaincfg_.fields.ie) {
        for (unsigned hart_index : params_.hart_indices) {
            if (idcs_[hart_index].iforce and idcs_[hart_index].idelivery)
                xeip_bits_[hart_index] = 1;
        }
        unsigned num_sources = aplic_->numSources();
        for (unsigned i = 1; i <= num_sources; i++) {
            unsigned hart_index = target_.at(i).dm0.hart_index;
            if (not includesHart(hart_index))
                continue;
            unsigned priority = target_.at(i).dm0.iprio;
            unsigned idelivery = idcs_[hart_index].idelivery;
            unsigned ithreshold = idcs_[hart_index].ithreshold;
            bool under_threshold = ithreshold == 0 or priority < ithreshold;
            //std::cerr << "Error: idelivery: " << idelivery << ", under_threshold: " << under_threshold << ", pending: " << pending(i) << ", enabled: " << enabled(i) << ")\n";
            if (idelivery and under_threshold and pending(i) and enabled(i))
                xeip_bits_[hart_index] = 1;
        }
    }
}

void Domain::runCallbacksAsRequired()
{
    if (dmIsDirect()) {
        auto prev_xeip_bits = xeip_bits_;
        inferXeipBits();
        for (unsigned hart_index : params_.hart_indices) {
            auto xeip_bit = xeip_bits_[hart_index];
            if (prev_xeip_bits[hart_index] != xeip_bit and direct_callback_)
                direct_callback_(hart_index, params_.privilege, xeip_bit);
        }
    } else if (aplic_->autoForwardViaMsi) {
        unsigned num_sources = aplic_->numSources();
        for (unsigned i = 0; i <= num_sources; i++) {
            if (readyToForwardViaMsi(i))
                forwardViaMsi(i);
        }
    }
    for (const auto& child : children_)
        child->runCallbacksAsRequired();
}

uint64_t Domain::msiAddr(unsigned hart_index, unsigned guest_index) const
{
    uint64_t addr = 0;
    auto cfgh = root()->mmsiaddrcfgh_;
    uint64_t g = (hart_index >> cfgh.fields.lhxw) & ((1 << cfgh.fields.hhxw) - 1);
    uint64_t h = hart_index & ((1 << cfgh.fields.lhxw) - 1);
    uint64_t hhxs = cfgh.fields.hhxs;
    if (params_.privilege == Machine) {
        uint64_t low = root()->mmsiaddrcfg_;
        addr = (uint64_t(cfgh.fields.ppn) << 32) | low;
        addr = (addr | (g << (hhxs + 12)) | (h << cfgh.fields.lhxs)) << 12;
    } else {
        auto scfgh = root()->smsiaddrcfgh_;
        uint64_t low = root()->smsiaddrcfg_;
        addr = (uint64_t(scfgh.fields.ppn) << 32) | low;
        addr = (addr | (g << (hhxs + 12)) | (h << scfgh.fields.lhxs) | guest_index) << 12;
    }
    return addr;
}

bool Domain::rectifiedInputValue(unsigned i) const
{
    if (not sourceIsActive(i))
        return false;
    bool state = aplic_->getSourceState(i);
    switch (sourcecfg_.at(i).d0.sm) {
        case Detached:
            return false;
        case Edge1:
        case Level1:
            return state;
        case Edge0:
        case Level0:
            return not state;
        default: assert(false);
    }
    return false;
}

bool Domain::sourceIsImplemented(unsigned i) const
{
    if (i == 0 or i > aplic_->numSources())
        return false;
    if (parent() and not parent()->sourcecfg_.at(i).dx.d)
        return false;
    return true;
}

std::shared_ptr<Domain> Domain::root() const { return aplic_->root(); }
