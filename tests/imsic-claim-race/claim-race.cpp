// Check for the race between an MSI write to an IMSIC interrupt file and
// a claim (write to MTOPEI/STOPEI) by the hart that owns the file.
//
// In threaded mode each hart runs in its own thread. An MSI from hart A
// reaches Imsic::write via Hart::store -> deviceWrite -> imsicWrite_ on
// hart A's thread. A claim by hart B runs CsRegs::writeMtopei (or
// writeStopei) on hart B's thread, which finds the top id and clears its
// pending bit. Both do a read-modify-write of the same std::vector<bool>
// word and of topId_, which the IMSIC must serialise with its own lock.
//
// The machine file is the M-mode path. The supervisor file is the path
// Linux uses: IPIs and device MSIs are writes to another hart's S-file,
// and the IMSIC driver claims them with csr_swap(CSR_TOPEI, 0).
//
// The sender sends an id only once the previous instance of that id has
// been claimed, so with no race every send is claimed exactly once. A
// lost update shows up as an id that was sent but never claimed (the
// claimer's clear of a neighbouring id wiped it out) or as an id claimed
// twice (the sender's set restored a bit the claimer had just cleared).

#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include "Imsic.hpp"

using namespace TT_IMSIC;
using EIC = File::ExternalInterruptCsr;

static constexpr uint64_t MADDR = 0x1000000;
static constexpr uint64_t SADDR = 0x2000000;
static constexpr unsigned PAGE_SIZE = 4096;
static constexpr unsigned ID_COUNT = 64;  // All ids share one 64-bit word.
static constexpr uint64_t MAX_SENDS = 1'000'000;
static constexpr auto TIME_LIMIT = std::chrono::seconds(10);

/// The interrupt file under test and how its owning hart accesses it.
struct Mode
{
  const char* name;
  uint64_t addr;                          // MSI address of the file.
  void (*setup)(Imsic&);                  // Configure, enable delivery and all ids.
  unsigned (*claim)(Imsic&);              // Write to *TOPEI.
  uint64_t (*pending)(Imsic&);            // Read eip0.
};

static const Mode machineMode = {
  .name = "machine (MTOPEI)",
  .addr = MADDR,
  .setup = [] (Imsic& imsic) {
    imsic.configureMachine(MADDR, ID_COUNT, PAGE_SIZE, 0, false /*aplic*/);
    imsic.writeMireg<uint64_t>(EIC::DELIVERY, 1);
    imsic.writeMireg<uint64_t>(EIC::E0, ~uint64_t(0));
  },
  .claim = [] (Imsic& imsic) { return imsic.claimMachineTop(); },
  .pending = [] (Imsic& imsic) {
    uint64_t val = 0;
    imsic.readMireg<uint64_t>(EIC::P0, val);
    return val;
  },
};

static const Mode supervisorMode = {
  .name = "supervisor (STOPEI)",
  .addr = SADDR,
  .setup = [] (Imsic& imsic) {
    imsic.configureSupervisor(SADDR, ID_COUNT, PAGE_SIZE, 0, false /*aplic*/);
    imsic.writeSireg<uint64_t>(false /*virt*/, 0, EIC::DELIVERY, 1);
    imsic.writeSireg<uint64_t>(false /*virt*/, 0, EIC::E0, ~uint64_t(0));
  },
  .claim = [] (Imsic& imsic) { return imsic.claimSupervisorTop(); },
  .pending = [] (Imsic& imsic) {
    uint64_t val = 0;
    imsic.readSireg<uint64_t>(false /*virt*/, 0, EIC::P0, val);
    return val;
  },
};


/// Race MSI writes against claims on the given file. Return true if
/// every send was claimed exactly once.
static bool
runTest(const Mode& mode)
{
  Imsic imsic;
  mode.setup(imsic);

  std::array<std::atomic<bool>, ID_COUNT> outstanding{};
  std::atomic<uint64_t> sent = 0, claimed = 0, doubleClaims = 0;
  std::atomic<bool> stop = false;

  // Owning hart: same sequence as CsRegs::writeMtopei/writeStopei.
  auto claimer = [&] {
    while (not stop.load(std::memory_order_relaxed))
      {
        unsigned id = mode.claim(imsic);
        if (not id)
          continue;
        if (not outstanding.at(id).exchange(false))
          doubleClaims++;
        claimed++;
      }
  };

  // Remote hart: same call as Hart::deviceWrite for an IMSIC address.
  auto sender = [&] {
    auto deadline = std::chrono::steady_clock::now() + TIME_LIMIT;
    while (sent < MAX_SENDS and std::chrono::steady_clock::now() < deadline)
      for (unsigned id = 1; id < ID_COUNT and sent < MAX_SENDS; ++id)
        {
          if (outstanding.at(id).exchange(true))
            continue;
          sent++;
          imsic.write(mode.addr, 4, id);
        }
  };

  std::thread c(claimer);
  std::thread s(sender);
  s.join();

  // Let the claimer drain whatever is still pending, then stop it.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop = true;
  c.join();

  // An id still marked outstanding after the drain was sent but its
  // pending bit is gone.
  uint64_t pending = mode.pending(imsic);
  unsigned lost = 0;
  for (unsigned id = 1; id < ID_COUNT; ++id)
    if (outstanding.at(id) and not ((pending >> id) & 1))
      lost++;

  std::cout << mode.name << ": sent=" << sent << " claimed=" << claimed
            << " lost=" << lost << " double-claims=" << doubleClaims << '\n';

  if (lost or doubleClaims or sent != claimed)
    {
      std::cout << "FAIL: " << mode.name << ": IMSIC MSI write raced with claim\n";
      return false;
    }

  return true;
}


int
main()
{
  bool ok = runTest(machineMode);
  ok = runTest(supervisorMode) and ok;

  std::cout << (ok ? "PASS\n" : "FAIL\n");
  return ok ? 0 : 1;
}
