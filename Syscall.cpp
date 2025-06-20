// Copyright 2020 Western Digital Corporation or its affiliates.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iomanip>
#include <iostream>
#include <fstream>
#include <optional>
#include <span>
#include <sstream>
#include <cstring>
#include <ctime>
#include <sys/times.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/mman.h>

#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <sys/utsname.h>
#include <linux/futex.h>
#include <sched.h>

#include "Hart.hpp"
#include "Syscall.hpp"
#include "Filesystem.hpp"


using namespace WdRiscv;


#if defined(__APPLE__)
  #define off64_t off_t
  #define MREMAP_MAYMOVE 0
#endif


template <typename URV, typename T>
requires requires (T& t) { std::span<char>(std::begin(t), std::end(t)); }
static bool
copyRvString(Hart<URV>& hart, uint64_t rvAddr,
             T& dest)
{
  for (size_t i = 0; i < std::size(dest); ++i)
    {
      uint8_t byte = 0;
      if (not hart.peekMemory(rvAddr + i, byte, true))
        return false;
      dest[i] = byte;
      if (byte == 0)
        return true;
    }
  return false;
}


/// Read from the RISCV system memory at address readAddr up size
/// elements placing them in the dest array (which must be large enough
/// to accomodate size bytes). Return number of elements successfully
/// read. Reading of elements is done sequentially and stopped at the
/// first failure.
template <typename URV, typename T>
requires requires(T t) { std::span(t); }
static uint64_t
readHartMemory(Hart<URV>& hart, uint64_t readAddr, T&& dest, std::optional<std::size_t> size = std::nullopt)
{
  std::span<std::byte> destBytes = std::as_writable_bytes(std::span(std::forward<T>(dest)));

  for (uint64_t i = 0; i < size.value_or(destBytes.size()); ++i)
    {
      uint8_t byte = 0;
      if (not hart.peekMemory(readAddr + i, byte, true))
        return i;
      destBytes[i] = static_cast<std::byte>(byte);
    }

  return size.value_or(dest.size());
}

template <typename URV, typename T>
static inline uint64_t
readHartMemory(Hart<URV>& hart, uint64_t readAddr, T& dest)
{
  return readHartMemory(hart, readAddr, std::span<T, 1>(&dest, 1));
}



template <typename URV, typename T>
requires requires(T t) { std::span(t); }
static uint64_t
writeHartMemory(Hart<URV>& hart, T&& data, uint64_t writeAddr, std::optional<std::size_t> size = std::nullopt)
{
  std::span<const std::byte> dataBytes = std::as_bytes(std::span(std::forward<T>(data)));

  for (uint64_t i = 0; i < size.value_or(dataBytes.size()); ++i)
    {
      uint8_t byte = static_cast<uint8_t>(dataBytes[i]);
      if (not hart.pokeMemory(writeAddr + i, byte, true))
        return i;
    }
  return size.value_or(dataBytes.size());
}

template <typename URV, typename T>
static uint64_t
writeHartMemory(Hart<URV>& hart, const T& data, uint64_t writeAddr)
{
  return writeHartMemory(hart, std::span<const T, 1>(&data, 1), writeAddr);
}


// Copy x86 stat buffer to riscv kernel_stat buffer.
template <typename URV>
static size_t
copyStatBufferToRiscv(Hart<URV>& hart, const struct stat& buff,
                      uint64_t rvBuff, bool& writeOk)
{
  writeOk = false;
  uint64_t addr = rvBuff;

  if (not hart.pokeMemory(addr, uint64_t(buff.st_dev), true))
    return addr - rvBuff;
  addr += 8;

  if (not hart.pokeMemory(addr, uint64_t(buff.st_ino), true))
    return addr - rvBuff;
  addr += 8;

  if (not hart.pokeMemory(addr, uint32_t(buff.st_mode), true))
    return addr - rvBuff;
  addr += 4;

  if (not hart.pokeMemory(addr, uint32_t(buff.st_nlink), true))
    return addr - rvBuff;
  addr += 4;

  if (not hart.pokeMemory(addr, uint32_t(buff.st_uid), true))
    return addr - rvBuff;
  addr += 4;

  if (not hart.pokeMemory(addr, uint32_t(buff.st_gid), true))
    return addr - rvBuff;
  addr += 4;

  if (not hart.pokeMemory(addr, uint64_t(buff.st_rdev), true))
    return addr - rvBuff;
  addr += 8;

  addr += 8; // __pad1

  if (not hart.pokeMemory(addr, uint64_t(buff.st_size), true))
    return addr - rvBuff;
  addr += 8;

#ifdef __APPLE__
  // TODO: adapt code for Mac OS.
  addr += 40;
#else
  if (not hart.pokeMemory(addr, uint32_t(buff.st_blksize), true))
    return addr - rvBuff;
  addr += 4;

  addr += 4; // __pad2

  if (not hart.pokeMemory(addr, uint64_t(buff.st_blocks), true))
    return addr - rvBuff;
  addr += 8;

  if (not hart.pokeMemory(addr, uint64_t(buff.st_atim.tv_sec), true))
    return addr - rvBuff;
  addr += 8;

  if (not hart.pokeMemory(addr, uint64_t(buff.st_atim.tv_nsec), true))
    return addr - rvBuff;
  addr += 8;

  if (not hart.pokeMemory(addr, uint64_t(buff.st_mtim.tv_sec), true))
    return addr - rvBuff;
  addr += 8;

  if (not hart.pokeMemory(addr, uint64_t(buff.st_mtim.tv_nsec), true))
    return addr - rvBuff;
  addr += 8;

  if (not hart.pokeMemory(addr, uint64_t(buff.st_ctim.tv_sec), true))
    return addr - rvBuff;
  addr += 8;

  if (not hart.pokeMemory(addr, uint64_t(buff.st_ctim.tv_nsec), true))
    return addr - rvBuff;
  addr += 8;

#endif

  writeOk = true;
  return addr - rvBuff;
}


// Copy x86 tms struct (used by times) to riscv.
template <typename URV>
static size_t
copyTmsToRiscv(Hart<URV>& hart, const struct tms& buff, URV addr)
{
  URV addr0 = addr;
  if (not hart.pokeMemory(addr, URV(buff.tms_utime), true))
    return addr - addr0;
  addr += sizeof(URV);

  if (not hart.pokeMemory(addr, URV(buff.tms_stime), true))
    return addr - addr0;
  addr += sizeof(URV);

  if (not hart.pokeMemory(addr, URV(buff.tms_cutime), true))
    return addr - addr0;
  addr += sizeof(URV);

  if (not hart.pokeMemory(addr, URV(buff.tms_cstime), true))
    return addr -addr0;
  addr += sizeof(URV);

  return addr -addr0;
}


// Copy x86 timeval buffer to riscv timeval buffer (32-bit version).
template <typename URV>
static size_t
copyTimevalToRiscv32(Hart<URV>& hart, const struct timeval& tv, URV addr)
{
  size_t written = 0;
  if (not hart.pokeMemory(addr, uint32_t(tv.tv_sec), true))
    return written;
  written += sizeof(uint32_t);
  addr += sizeof(uint32_t);

  if (not hart.pokeMemory(addr, uint64_t(tv.tv_usec), true))
    return written;
  written += sizeof(uint64_t);

  return written;
}


// Copy x86 timeval buffer to riscv timeval buffer (32-bit version).
template <typename URV>
static size_t
copyTimevalToRiscv64(Hart<URV>& hart, const struct timeval& tv, URV addr)
{
  size_t written = 0;
  if (not hart.pokeMemory(addr, uint64_t(tv.tv_sec), true))
    return written;
  written += sizeof(uint64_t);
  addr += sizeof(uint64_t);

  if (not hart.pokeMemory(addr, uint64_t(tv.tv_usec), true))
    return written;
  written += sizeof(uint64_t);

  return written;
}


// Copy x86 timezone to riscv
template<typename URV>
static size_t
copyTimezoneToRiscv(Hart<URV>& hart, const struct timezone& tz, URV dest)
{
  size_t written = 0;
  if (not hart.pokeMemory(dest, URV(tz.tz_minuteswest), true))
    return written;
  written += sizeof(URV);
  dest += sizeof(URV);

  if (not hart.pokeMemory(dest, URV(tz.tz_dsttime), true))
    return written;
  written += sizeof(URV);

  return written;
}


template <typename URV>
bool
Syscall<URV>::redirectOutputDescriptor(int fd, const std::string& path)
{
  if (fdMap_.contains(fd))
    {
      std::cerr << "Hart::redirectOutputDecritpor: Error: File decriptor " << fd
                << " alrady used.\n";
      return false;
    }

  int newFd = open(path.c_str(), O_WRONLY | O_CREAT,
                   S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (newFd < 0)
    {
      std::cerr << "Error: Failed to open file " << path << " for output\n";
      return false;
    }
  fdMap_[fd] = newFd;
  fdIsRead_[fd] = false;
  fdPath_[fd] = path;

  auto absPath = Filesystem::absolute(path);
  writePaths_.insert(absPath.string());

  return true;
}


template <typename URV>
bool
Syscall<URV>::redirectInputDescriptor(int fd, const std::string& path)
{
  if (fdMap_.contains(fd))
    {
      std::cerr << "Hart::redirectOutputDecritpor: Error: File decriptor " << fd
                << " already used.\n";
      return false;
    }

  int newFd = open(path.c_str(), O_RDONLY);
  if (newFd < 0)
    {
      std::cerr << "Error: Failed to open file " << path << " for input\n";
      return false;
    }
  fdMap_[fd] = newFd;
  fdIsRead_[fd] = true;
  fdPath_[fd] = path;

  return true;
}


template <typename URV>
void
Syscall<URV>::reportOpenedFiles(std::ostream& out)
{
  if (not readPaths_.empty())
    {
      out << "Files opened for read:\n";
      for (const auto& path : readPaths_)
        out << "  " << path << '\n';
    }

  if (not writePaths_.empty())
    {
      out << "Files opened for write/read-write:\n";
      for (const auto& path : writePaths_)
        out << "  " << path << '\n';
    }
}


template <typename URV>
int
Syscall<URV>::registerLinuxFd(int linuxFd, const std::string& path, bool isRead)
{
  if (linuxFd < 0)
    return linuxFd;

  int riscvFd = linuxFd;
  int maxFd = linuxFd;
  bool used = false;

  for (auto kv : fdMap_)
    {
      int rfd = kv.first;
      if (riscvFd == rfd)
        used = true;
      maxFd = std::max(maxFd, rfd);
    }

  if (used)
    riscvFd = maxFd + 1;

  fdMap_[riscvFd] = linuxFd;
  fdIsRead_[riscvFd] = isRead;
  fdPath_[riscvFd] = path;

  auto absPath = Filesystem::absolute(path);
  if (isRead)
    readPaths_.insert(absPath.string());
  else
    writePaths_.insert(absPath.string());

  return riscvFd;
}


template <typename URV>
URV
Syscall<URV>::emulateSemihost(unsigned hartIx, URV a0, URV a1)
{
  enum Operation { Open = 1, Close = 2, Writec = 3, Write0 = 4, Write = 5, Read = 6,
                   Readc = 7, Iserror = 8, Istty = 9, Seek = 10, Flen = 12, Tmpnam = 13,
                   Remove = 14, Rename = 15, Clock = 16, Time = 17, System = 18,
                   Errno = 19, GetCmdline = 21, Heapinfo = 22, Exit = 24, ExitExtended = 32,
                   Elapsed = 48, Tickfreq = 49 };

  static std::unordered_map<Operation, std::string>  names =
    {
      { Open,         "open" },
      { Close,        "close" },
      { Writec,       "writec" },
      { Write0,       "write0" },
      { Write,        "write" },
      { Read,         "read" },
      { Readc,        "readc" },
      { Iserror,      "iserror" },
      { Istty,        "istty" },
      { Seek,         "seek" },
      { Flen,         "flen" },
      { Tmpnam,       "tmpnam" },
      { Remove,       "remove" },
      { Rename,       "rename" },
      { Clock,        "clock" },
      { Time,         "time" },
      { System,       "system" },
      { Errno,        "errno" },
      { GetCmdline,   "get_cmdline" },
      { Heapinfo,     "heapinfo" },
      { Exit,         "exit" },
      { ExitExtended, "exit_extended" },
      { Elapsed,      "elapsed" },
      { Tickfreq,     "tickfreq" }
    };

  std::lock_guard<std::mutex> lock(semihostMutex_);

  auto& hart = *harts_.at(hartIx);
  Operation op = Operation(a0);

  switch (op)
    {
    case Open:
      {
        URV addr = 0, mode = 0, len = 0;  // Addr: address of name string
        if (not hart.peekMemory(a1, addr, true)  or
            not hart.peekMemory(a1 + sizeof(URV), mode, true) or
            not hart.peekMemory(a1 + 2*sizeof(URV), len, true))
          return SRV(-1);

        std::array<char, 1024> path;
        if (not copyRvString(hart, addr, path))
          return SRV(-1);

        int flags = O_RDONLY;   // Linux flags corresponding to mode.
        switch (mode)
          {
          case 2:
          case 3:  flags = O_RDWR; break;

          case 4:
          case 5:  flags = O_WRONLY | O_CREAT; break;

          case 6:
          case 7:  flags = O_RDWR | O_CREAT; break;

          case 8:
          case 9:  flags = O_WRONLY | O_APPEND | O_CREAT; break;

          case 10:
          case 11:  flags = O_RDWR | O_APPEND | O_CREAT; break;
          }

	int handle = open(path.data(), flags, S_IRWXU);
        if (handle < 0)
          return SRV(-1);
        bool isRead = not (flags & (O_WRONLY | O_RDWR));
        int effHandle = registerLinuxFd(handle, path.data(), isRead);
        if (effHandle < 0)
          {
            close(handle);
            return SRV(-1);
          }
        return effHandle;
      }
      break;
 
    case Close:
      {
        URV handle = 0;
        if (not hart.peekMemory(a1, handle, true))
          return SRV(-1);
        SRV rc = emulate(hartIx, 57 /*close*/, handle, 0, 0, 0);
        return rc == 0? 0 : SRV(-1);
      }

    case Writec:
      {
        uint8_t c = 0;
        hart.peekMemory(a1, c, true);
        fputc(c, stderr);
        return c;
      }

    case Write0:
      {
        uint8_t c = 0;
        URV addr = a1;
        while (hart.peekMemory(addr++, c, true) and c != 0)
          fputc(c, stderr);
        return a1;
      }

    case Write:
      {
        URV handle = 0, addr = 0, size = 0;
        if (not hart.peekMemory(a1, handle, true)  or
            not hart.peekMemory(a1 + sizeof(URV), addr, true) or
            not hart.peekMemory(a1 + 2*sizeof(URV), size, true))
          return SRV(-1);

        SRV rc = emulate(hartIx, 64 /*write*/, handle, addr, size, 0);
        return rc >= 0? rc : SRV(-1);
      }

    case Read:
      {
        URV handle = 0, addr = 0, size = 0;
        if (not hart.peekMemory(a1, handle, true)  or
            not hart.peekMemory(a1 + sizeof(URV), addr, true) or
            not hart.peekMemory(a1 + 2*sizeof(URV), size, true))
          return SRV(-1);

        SRV rc = emulate(hartIx, 63 /*read*/, handle, addr, size, 0);
        return rc >= 0? rc : SRV(-1);
      }

    case Readc:
      break;

    case Iserror:
      {
        URV code = 0;
        if (not hart.peekMemory(a1, code, true))
          return SRV(-1);
        if (code == 0)
          return 0;
        return SRV(-1);
      }

    case Istty:
      {
        URV fd = 0;
        if (not hart.peekMemory(a1, fd, true))
          return SRV(-1);

        fd = effectiveFd(fd);
        if (isatty(fd))
          return 1;
        return 0;
      }

    case Seek:
      {
        URV fd = 0, position = 0;
        if (not hart.peekMemory(a1, fd, true) or
            not hart.peekMemory(a1 + sizeof(URV), position, true))
          return SRV(-1);

        fd = effectiveFd(fd);
        off_t offset = position;
        ssize_t rc = lseek(fd, offset, SEEK_SET);
        return rc < 0 ? SRV(-1) : SRV(0);
      }

    case Flen:
      {
        URV fd = 0;
        if (not hart.peekMemory(a1, fd, true))
          return SRV(-1);

        fd = effectiveFd(fd);
        struct stat buff;
	int rc = fstat(fd, &buff);
        if (rc < 0)
          return SRV(-1);
        return buff.st_size;
      }

    case Tmpnam:
      break;
#if 0
      // Spec is dangerous. Unimplementable.
      {
        URV addr = 0, id = 0, len = 0;
        if (not hart.peekMemory(a1, addr, true) or
            not hart.peekMemory(a1 + sizeof(URV), id, true) or
            not hart.peekMemory(a1 + 2*sizeof(URV), len, true))
          return SRV(-1);

        if (len > L_tmpnam)
          return SRV(-1);

        char name[L_tmpnam];
        if (not std::mkstmp(name))
          return SRV(-1);

        writeHartMemory(hart, name, len);
        return 0;
      }
#endif

    case Remove:
      {
        URV addr = 0, len = 0;
        if (not hart.peekMemory(a1, addr, true) or
            not hart.peekMemory(a1 + sizeof(URV), len, true))
          return SRV(-1);

        SRV rc = emulate(hartIx, 1026 /*unlink*/, addr, 0, 0, 0);
        return rc >= 0 ? rc : SRV(-1);
      }

    case Rename:
      {
        URV addr1 = 0, addr2 = 0;  // Old and new name addresses
        URV len1 = 0, len2 = 0;
        if (not hart.peekMemory(a1, addr1, true) or
            not hart.peekMemory(a1 + sizeof(URV), len1, true) or
            not hart.peekMemory(a1 + 2*sizeof(URV), addr2, true) or
            not hart.peekMemory(a1 + 3*sizeof(URV), len2, true))
          return SRV(-1);
        SRV rc = emulate(hartIx, 276 /*rename*/, len1, addr1, len2, addr2);
        return rc >= 0 ? rc : SRV(-1);
      }

    case Clock:
      break;

    case Time:
      break;

    case System:
      break;

    case Errno:
      break;

    case GetCmdline:
      break;

    case Heapinfo:
      break;

    case Exit:
      throw CoreException(CoreException::Exit, "", a1);
      break;

    case Elapsed:
      break;

    case Tickfreq:
      break;

    default:
      std::cerr << "Error: Unknown semi-hosting syscall number: " << a0 << '\n';
      return SRV(-1);
    }

  std::cerr << "Error: Unimplemented semi-hosting syscall \"" << names[op]
            << "\" number " << a0 << '\n';

  return SRV(-1);
}
      


template <typename URV>
URV
Syscall<URV>::emulate(unsigned hartIx, unsigned syscallIx, URV a0, URV a1, URV a2, URV a3)
{
  static std::unordered_map<unsigned, std::string> names =
    {
     {0,    "io_setup"},
     {1,    "io_destroy"},
     {2,    "io_submit"},
     {3,    "io_cancel"},
     {4,    "io_getevents"},
     {5,    "setxattr"},
     {6,    "lsetxattr"},
     {7,    "fsetxattr"},
     {8,    "getxattr"},
     {9,    "lgetxattr"},
     {10,   "fgetxattr"},
     {11,   "listxattr"},
     {12,   "llistxattr"},
     {13,   "flistxattr"},
     {14,   "removexattr"},
     {15,   "lremovexattr"},
     {16,   "fremovexattr"},
     {17,   "getcwd"},
     {18,   "lookup_dcookie"},
     {19,   "eventfd2"},
     {20,   "epoll_create1"},
     {21,   "epoll_ctl"},
     {22,   "epoll_pwait"},
     {23,   "dup"},
     {24,   "dup3"},
     {25,   "fcntl"},
     {26,   "inotify_init1"},
     {27,   "inotify_add_watch"},
     {28,   "inotify_rm_watch"},
     {29,   "ioctl"},
     {30,   "ioprio_get"},
     {31,   "ioprio_set"},
     {32,   "flock"},
     {33,   "mknodat"},
     {34,   "mkdirat"},
     {35,   "unlinkat"},
     {36,   "symlinkat"},
     {37,   "linkat"},
     {38,   "renameat"},
     {39,   "umount2"},
     {40,   "mount"},
     {41,   "pivot_root"},
     {42,   "nfsservctl"},
     {43,   "statfs"},
     {44,   "fstatfs"},
     {45,   "truncate"},
     {46,   "ftruncate"},
     {47,   "fallocate"},
     {48,   "faccessat"},
     {49,   "chdir"},
     {50,   "fchdir"},
     {51,   "chroot"},
     {52,   "fchmod"},
     {53,   "fchmodat"},
     {54,   "fchownat"},
     {55,   "fchown"},
     {56,   "openat"},
     {57,   "close"},
     {58,   "vhangup"},
     {59,   "pipe2"},
     {60,   "quotactl"},
     {61,   "getdents64"},
     {62,   "lseek"},
     {63,   "read"},
     {64,   "write"},
     {66,   "writev"},
     {67,   "pread64"},
     {68,   "pwrite64"},
     {69,   "preadv"},
     {70,   "pwritev"},
     {71,   "sendfile"},
     {72,   "pselect6"},
     {73,   "ppoll"},
     {74,   "signalfd64"},
     {75,   "vmsplice"},
     {76,   "splice"},
     {77,   "tee"},
     {78,   "readlinkat"},
     {79,   "fstatat"},
     {80,   "fstat"},
     {81,   "sync"},
     {82,   "fsync"},
     {83,   "fdatasync"},
     {84,   "sync_file_range2"},
     {85,   "timerfd_create"},
     {86,   "timerfd_settime"},
     {87,   "timerfd_gettime"},
     {88,   "utimensat"},
     {89,   "acct"},
     {90,   "capget"},
     {91,   "capset"},
     {92,   "personality"},
     {93,   "exit"},
     {94,   "exit_group"},
     {95,   "waitid"},
     {96,   "set_tid_address"},
     {97,   "unshare"},
     {98,   "futex"},
     {99,   "set_robust_list"},
     {100,  "get_robust_list"},
     {101,  "nanosleep"},
     {102,  "getitimer"},
     {103,  "setitimer"},
     {104,  "kexec_load"},
     {105,  "init_module"},
     {106,  "delete_module"},
     {107,  "timer_create"},
     {108,  "timer_gettime"},
     {109,  "timer_getoverrun"},
     {110,  "timer_settime"},
     {111,  "timer_delete"},
     {112,  "clock_settime"},
     {113,  "clock_gettime"},
     {114,  "clock_getres"},
     {115,  "clock_nanosleep"},
     {116,  "syslog"},
     {117,  "ptrace"},
     {118,  "sched_setparam"},
     {119,  "sched_setscheduler"},
     {120,  "sched_getscheduler"},
     {121,  "sched_getparam"},
     {122,  "sched_setaffinity"},
     {123,  "sched_getaffinity"},
     {124,  "sched_yield"},
     {125,  "sched_get_priority_max"},
     {126,  "sched_get_priority_min"},
     {127,  "scheD_rr_get_interval"},
     {128,  "restart_syscall"},
     {129,  "kill"},
     {130,  "tkill"},
     {131,  "tgkill"},
     {132,  "sigaltstack"},
     {133,  "rt_sigsuspend"},
     {134,  "rt_sigaction"},
     {135,  "rt_sigprocmask"},
     {136,  "rt_sigpending"},
     {137,  "rt_sigtimedwait"},
     {138,  "rt_sigqueueinfo"},
     {139,  "rt_sigreturn"},
     {140,  "setpriority"},
     {141,  "getpriority"},
     {142,  "reboot"},
     {143,  "setregid"},
     {144,  "setgid"},
     {145,  "setreuid"},
     {146,  "setuid"},
     {147,  "setresuid"},
     {148,  "getresuid"},
     {149,  "getresgid"},
     {150,  "getresgid"},
     {151,  "setfsuid"},
     {152,  "setfsgid"},
     {153,  "times"},
     {154,  "setpgid"},
     {155,  "getpgid"},
     {156,  "getsid"},
     {157,  "setsid"},
     {158,  "getgroups"},
     {159,  "setgroups"},
     {160,  "uname"},
     {161,  "sethostname"},
     {162,  "setdomainname"},
     {163,  "getrlimit"},
     {164,  "setrlimit"},
     {165,  "getrusage"},
     {166,  "umask"},
     {167,  "prctl"},
     {168,  "getcpu"},
     {169,  "gettimeofday"},
     {170,  "settimeofday"},
     {171,  "adjtimex"},
     {172,  "getpid"},
     {173,  "getppid"},
     {174,  "getuid" },
     {175,  "geteuid"},
     {176,  "getgid" },
     {177,  "getegid"},
     {178,  "gettid" },
     {179,  "sysinfo"},
     {180,  "mq_open"},
     {181,  "mq_unlink"},
     {182,  "mq_timedsend"},
     {183,  "mq_timedrecieve"},
     {184,  "mq_notify"},
     {185,  "mq_getsetattr"},
     {186,  "msgget"},
     {187,  "msgctl"},
     {188,  "msgrcv"},
     {189,  "msgsnd"},
     {190,  "semget"},
     {191,  "semctl"},
     {192,  "semtimedop"},
     {193,  "semop"},
     {194,  "shmget"},
     {195,  "shmctl"},
     {196,  "shmat"},
     {197,  "shmdt"},
     {198,  "socket"},
     {199,  "socketpair"},
     {200,  "bind"},
     {201,  "listen"},
     {202,  "accept"},
     {203,  "connect"},
     {204,  "getsockname"},
     {205,  "getpeername"},
     {206,  "sendo"},
     {207,  "recvfrom"},
     {208,  "setsockopt"},
     {209,  "getsockopt"},
     {210,  "shutdown"},
     {211,  "sendmsg"},
     {212,  "recvmsg"},
     {213,  "readahead"},
     {214,  "brk"},
     {215,  "munmap"},
     {216,  "mremap"},
     {217,  "add_key"},
     {218,  "request_key"},
     {219,  "keyctl"},
     {220,  "clone"},
     {221,  "execve"},
     {222,  "mmap"},
     {223,  "fadvise64"},
     {224,  "swapon"},
     {225,  "swapoff"},
     {226,  "mprotect"},
     {227,  "msync"},
     {228,  "mlock"},
     {229,  "munlock"},
     {230,  "mlockall"},
     {231,  "munlockall"},
     {232,  "mincore"},
     {233,  "madvise"},
     {234,  "remap_file_pages"},
     {235,  "mbind"},
     {236,  "get_mempolicy"},
     {237,  "set_mempolicy"},
     {238,  "migrate_pages"},
     {239,  "move_pages"},
     {240,  "tgsigqueueinfo"},
     {241,  "perf_event_open"},
     {242,  "accept4"},
     {243,  "recvmmsg"},
     {260,  "wait4"},
     {261,  "prlimit64"},
     {262,  "fanotify_init"},
     {263,  "fanotify_mark"},
     {264,  "name_to_handle_at"},
     {265,  "open_by_handle_at"},
     {266,  "clock_adjtime"},
     {267,  "syncfs"},
     {268,  "setns"},
     {269,  "sendmmsg"},
     {270,  "process_vm_ready"},
     {271,  "process_vm_writev"},
     {272,  "kcmp"},
     {273,  "finit_module"},
     {274,  "sched_setattr"},
     {275,  "sched_getattr"},
     {276,  "renameat2"},
     {277,  "seccomp"},
     {278,  "getrandom"},
     {279,  "memfd_create"},
     {280,  "bpf"},
     {281,  "execveat"},
     {282,  "userfaultid"},
     {283,  "membarrier"},
     {284,  "mlock2"},
     {285,  "copy_file_range"},
     {286,  "preadv2"},
     {287,  "pwritev2"},
     {435,  "clone3"},
     {1024, "open"},
     {1025, "link"},
     {1026, "unlink"},
     {1027, "mknod"},
     {1028, "chmod"},
     {1029, "chown"},
     {1030, "mkdir"},
     {1031, "rmdir"},
     {1032, "lchown"},
     {1033, "access"},
     {1034, "rename"},
     {1035, "readlink"},
     {1036, "symlink"},
     {1037, "utimes"},
     {1038, "stat"},
     {1039, "lstat"},
     {1040, "pipe"},
     {1041, "dup2"},
     {1042, "epoll_create"},
     {1043, "inotifiy_init"},
     {1044, "eventfd"},
     {1045, "signalfd"},
     {1046, "sendfile"},
     {1047, "ftruncate"},
     {1048, "truncate"},
     {1049, "stat"},
     {1050, "lstat"},
     {1051, "fstat"},
     {1052, "fcntl" },
     {1053, "fadvise64"},
     {1054, "newfstatat"},
     {1055, "fstatfs"},
     {1056, "statfs"},
     {1057, "lseek"},
     {1058, "mmap"},
     {1059, "alarm"},
     {1060, "getpgrp"},
     {1061, "pause"},
     {1062, "time"},
     {1063, "utime"},
     {1064, "creat"},
     {1065, "getdents"},
     {1066, "futimesat"},
     {1067, "select"},
     {1068, "poll"},
     {1069, "epoll_wait"},
     {1070, "ustat"},
     {1071, "vfork"},
     {1072, "oldwait4"},
     {1073, "recv"},
     {1074, "send"},
     {1075, "bdflush"},
     {1076, "umount"},
     {1077, "uselib"},
     {1078, "sysctl"},
     {1079, "fork"},
     {2011, "getmainvars"}

    };
  // Preliminary. Need to avoid using syscall numbers.

  // On success syscall returns a non-negtive integer.
  // On failure it returns the negative of the error number.
  std::lock_guard<std::mutex> lock(emulateMutex_);

  auto& hart = *harts_.at(hartIx);

  switch (syscallIx)
    {
    case 17:       // getcwd
      {
	size_t size = a1;
        size_t rvBuff = a0;

	errno = 0;
        std::array<char, 1024> buffer;
        
        if (not getcwd(buffer.data(), buffer.size()))
	  return SRV(-errno);

        size_t len = strlen(buffer.data()) + 1;
        if (len > size)
          return SRV(-EINVAL);

        for (size_t i = 0; i < len; ++i)
          if (not hart.pokeMemory(rvBuff+i, uint8_t(buffer[i]), true))
            return SRV(-EINVAL);

        return len;
      }

    case 25:       // fcntl
      {
	int fd = effectiveFd(SRV(a0));
	int cmd = SRV(a1);
	void* arg = std::bit_cast<void*>(static_cast<uintptr_t>(a2));
        int rc = 0;
	switch (cmd)
	  {
	  case F_GETLK:
	  case F_SETLK:
	  case F_SETLKW:
	    {
              // Assume linux and riscv have same flock structure.
              // Copy riscv flock struct into fl.
              struct flock fl;

              if (readHartMemory(hart, a2, fl) != sizeof(fl))
                return SRV(-EINVAL);

              rc = fcntl(fd, cmd, &fl);
              if (rc < 0)
                return rc;

              uint64_t written = writeHartMemory(hart, fl, a2);
              return written == sizeof(fl)? rc : SRV(-EINVAL);
	    }

          default:
            rc = fcntl(fd, cmd, arg);
	  }
	return rc;
      }

    case 29:       // ioctl
      {
	int fd = effectiveFd(SRV(a0));
	int req = SRV(a1);

        std::vector<char> tmp;
        char* arg = nullptr;

#ifndef __APPLE__
        URV rvArg = a2;
	if (rvArg != 0)
          {
            size_t size = _IOC_SIZE(req);
            tmp.resize(size);
            if (readHartMemory(hart, rvArg, tmp, size) != size)
              return SRV(-EINVAL);
            arg = tmp.data();
          }
#endif

	errno = 0;
	int rc = ioctl(fd, req, arg);
	return rc < 0 ? SRV(-errno) : rc;
      }

    case 34:       // mkdirat
      {
	int fd = effectiveFd(SRV(a0));
	uint64_t rvPath = a1;
	mode_t mode = a2;
	std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
          return SRV(-EINVAL);

	errno = 0;
	int rc = mkdirat(fd, path.data(), mode);
	return rc < 0 ? SRV(-errno) : rc;
      }

    case 35:       // unlinkat
      {
	int fd = effectiveFd(SRV(a0));

        uint64_t rvPath = a1;
        std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
          return SRV(-EINVAL);

	int flags = SRV(a2);

	errno = 0;
	int rc = unlinkat(fd, path.data(), flags);
	return rc < 0 ? SRV(-errno) : rc;
      }

    case 46:       // ftruncate
      {
        errno = 0;
        SRV rc =  ftruncate(a0, a1);
        return rc < 0 ? SRV(-errno) : rc;
      }

    case 48:       // faccessat
      {
        int dirfd = effectiveFd(SRV(a0));

        uint64_t rvPath = a1;
        std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
          return SRV(-EINVAL);

        int mode = a2;
        int flags = 0; // Should be a3
        int rc = faccessat(dirfd, path.data(), mode, flags);
        return rc < 0 ? SRV(-errno) : rc;
      }

    case 49:       // chdir
      {
        uint64_t rvPath = a0;
        std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
          return SRV(-EINVAL);

	errno = 0;
	int rc = chdir(path.data());
	return rc < 0 ? SRV(-errno) : rc;
      }

    case 53:       // fchmodat
      {
        int dirfd = effectiveFd(SRV(a0));

        uint64_t rvPath = a1;
        std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
          return SRV(-EINVAL);

        mode_t mode = a2;
        int flags = 0; // Should be a3 -- non-zero not working on rhat6
        int rc = fchmodat(dirfd, path.data(), mode, flags);
        return rc < 0 ? SRV(-errno) : rc;
      }

    case 56:       // openat
      {
	int dirfd = effectiveFd(SRV(a0));

	uint64_t rvPath = a1;
        std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
          return SRV(-EINVAL);

	int flags = a2;
	int x86Flags = 0;
	if (linux_)
	  x86Flags = flags;
	else
	  {
	    // Newlib constants differ from Linux: compensate.
	    if (flags & 1)     x86Flags |= O_WRONLY;
	    if (flags & 0x2)   x86Flags |= O_RDWR;
	    if (flags & 0x200) x86Flags |= O_CREAT;
	  }

	mode_t mode = a3;

	errno  = 0;
	int rc = openat(dirfd, path.data(), x86Flags, mode);
        if (rc >= 0)
          {
            bool isRead = not (x86Flags & (O_WRONLY | O_RDWR));
            rc = registerLinuxFd(rc, path.data(), isRead);
            if (rc < 0)
              return SRV(-EINVAL);
          }
	return rc < 0 ? SRV(-errno) : rc;
      }

    case 61:       // getdents64  -- get directory entries
      {
#if defined(__APPLE__)
	return SRV(-1);
#else
	// TBD: double check that struct linux_dirent is same
	// in x86 and RISCV 32/64.
	int fd = effectiveFd(SRV(a0));
        uint64_t rvBuff = a1;
	size_t count = a2;
	off64_t base = 0;

        std::vector<char> buff(count);

	errno = 0;
	ssize_t rc = getdirentries64(fd, buff.data(), count, &base);
        if (rc >= 0)
          {
            ssize_t written = writeHartMemory(hart, buff, rvBuff, rc);
            return written == rc? rc : SRV(-EINVAL);
          }
	return SRV(-errno);
#endif
      }

    case 62:       // lseek
      {
	int fd = effectiveFd(a0);
	off_t offset = a1;
	int whence = a2;

	errno = 0;
	ssize_t rc = lseek(fd, offset, whence);
	return rc < 0 ? SRV(-errno) : rc;
      }

    case 66:       // writev
      {
	int fd = effectiveFd(SRV(a0));
	URV rvIov = a1;
	SRV count = a2;

        std::vector< std::vector<char> > buffers(count);
        std::vector<struct iovec> iov(count);

        for (SRV i = 0; i < count; ++i)
          {
            URV base = 0, len = 0;
            if (not hart.peekMemory(rvIov, base, true))
              return SRV(-EINVAL);
            rvIov += sizeof(base);

            if (not hart.peekMemory(rvIov, len, true))
              return SRV(-EINVAL);
            rvIov += sizeof(len);

            auto& buffer = buffers.at(i);
            buffer.resize(len);
            if (readHartMemory(hart, base, buffer, len) != len)
              return SRV(-EINVAL);

            iov.at(i).iov_base = buffer.data();
            iov.at(i).iov_len = len;
          }

        errno = 0;
        ssize_t rc = writev(fd, iov.data(), count);
        return rc < 0 ? SRV(-errno) : rc;
      }

    case 67:       // pread64
      {
	int fd = effectiveFd(SRV(a0));
	uint64_t buffAddr = a1;
	size_t count = a2;
	off_t offset = a3;

	std::vector<uint8_t> temp(count);

	errno = 0;
	ssize_t rc = pread(fd, temp.data(), count, offset);
        if (rc < 0)
          return SRV(-errno);

        ssize_t written = writeHartMemory(hart, temp, buffAddr, rc);
	return written == rc? written : SRV(-EINVAL);
      }

    case 68:       // pwrite64
      {
	int fd = effectiveFd(SRV(a0));

	uint64_t buffAddr = a1;
	size_t count = a2;
	off_t offset = a3;

        std::vector<uint8_t> temp(count);
        if (readHartMemory(hart, buffAddr, temp, count) != count)
          return SRV(-EINVAL);

	errno = 0;
	auto rc = pwrite(fd, temp.data(), count, offset);
	return rc < 0 ? SRV(-errno) : rc;
      }

    case 78:       // readlinat
      {
	int dirfd = effectiveFd(SRV(a0));
	URV rvPath = a1, rvBuff = a2, buffSize = a3;

        std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
          return SRV(-EINVAL);

        std::vector<char> buff(buffSize);

	errno = 0;
	ssize_t rc = readlinkat(dirfd, path.data(), buff.data(), buffSize);
        if (rc < 0)
          return SRV(-errno);

        ssize_t written = writeHartMemory(hart, buff, rvBuff, rc);
	return  written == rc ? written : SRV(-EINVAL);
      }

    case 79:       // fstatat
      {
	int dirFd = effectiveFd(SRV(a0));

        // Copy rv path into path.
        uint64_t rvPath = a1;
        std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
          return SRV(-EINVAL);

	uint64_t rvBuff = a2;
	int flags = a3;

        int rc = 0;
	struct stat buff;

	errno = 0;

        // Host OS may not support AT_EMPTY_PATH (0x1000) of fstatat: compensate.
        if ((flags & 0x1000) != 0 and path[0] == 0)
          rc = fstat(dirFd, &buff);
        else
          rc = fstatat(dirFd, path.data(), &buff, flags);

	if (rc < 0)
          {
            // perror("fstatat error: ");
            return SRV(-errno);
          }

        bool copyOk = true;
        copyStatBufferToRiscv(hart, buff, rvBuff, copyOk);
	return copyOk? rc : SRV(-1);
      }

    case 80:       // fstat
      {
	int fd = effectiveFd(SRV(a0));
	uint64_t rvBuff = a1;

	errno = 0;
	struct stat buff;
	int rc = fstat(fd, &buff);
	if (rc < 0)
	  return SRV(-errno);

        bool copyOk  = true;
        copyStatBufferToRiscv(hart, buff, rvBuff, copyOk);
	return copyOk? rc : SRV(-1);
      }

#if 0
    case 98:  // futex
      {
        if (a1 == FUTEX_WAIT_PRIVATE)
          {
            uint32_t value;
            hart.peekMemory(a0, value, true);

            // assume CLOCK_MONOTONIC
            uint64_t timeout = 0;
            if (a3 != 0)
              {
                uint64_t sec, nsec;
                hart.peekMemory(a3, sec, true);
                hart.peekMemory(a3 + 8, nsec, true);

                timeout += sec*1000000000;
                timeout += nsec%1000000000;
              }

            if (a2 == value)
              {
                hart.setSuspendState(true, timeout);
                futexMap_[a0].insert(hart.sysHartIndex());
              }
            else
              {
                hart.setSuspendState(false, 0);
                if (futexMap_[a0].contains(hart.sysHartIndex()))
                  futexMap_[a0].erase(hart.sysHartIndex());
              }
            return 0;
          }

        if (a1 == FUTEX_WAKE_PRIVATE)
          {
            unsigned count = std::min(size_t(a2), futexMap_[a0].size());
            for (unsigned i = 0; i < count; ++i)
              {
                auto it = futexMap_[a0].begin();
                futexMap_[a0].erase(it);
                auto resumeHart = harts_.at(*it);
                resumeHart->setSuspendState(false, 0);
              }
            return count;
          }

        std::cerr << "Error: unimplemented futex operation " << a1 << std::endl;
        return 0;
      }
#endif

    case 214: // brk
       {
     	  URV newBrk = a0;
     	  URV rc = newBrk;
          if (newBrk == 0)
            rc = progBreak_;
          else
            {
              for (URV addr = progBreak_; addr < newBrk; addr++)
                hart.pokeMemory(addr, uint8_t(0), true /*usePma*/);
              rc = progBreak_ = newBrk;
            }
     	  return rc;
       }

    case 226: //  mprotect
      return 0;

    case 57: // close
      {
	int fd = effectiveFd(SRV(a0));
	int rc = 0;
	if (fd > 2)
	  {
	    errno = 0;
	    rc = close(fd);
	    rc = rc < 0? -errno : rc;
            fdMap_.erase(a0);
            fdIsRead_.erase(a0);
            fdPath_.erase(a0);
	  }
	return SRV(rc);
      }

    case 63: // read
      {
	int fd = effectiveFd(SRV(a0));

	uint64_t buffAddr = a1;
	size_t count = a2;

        std::vector<uint8_t> temp(count);

	errno = 0;
	ssize_t rc = read(fd, temp.data(), count);
        if (rc < 0)
          return SRV(-errno);

        ssize_t written = writeHartMemory(hart, temp, buffAddr, rc);
	return written == rc? written : SRV(-EINVAL);
      }

    case 64: // write
      {
	int fd = effectiveFd(SRV(a0));

	uint64_t buffAddr = a1;
	size_t count = a2;

        std::vector<uint8_t> temp(count);
        if (readHartMemory(hart, buffAddr, temp, count) != count)
          return SRV(-EINVAL);

	errno = 0;
	auto rc = write(fd, temp.data(), count);
	return rc < 0 ? SRV(-errno) : rc;
      }

    case 88:  // utimensat
      {
        int dirfd = effectiveFd(SRV(a0));

	uint64_t rvPath = a1;
        std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
	  return SRV(-EINVAL);

        uint64_t rvTimeAddr = a2;
        std::array<timespec, 2> spec;
        if (readHartMemory(hart, rvTimeAddr, spec) != sizeof(spec))
          return SRV(-EINVAL);

        int flags = a3;
        int rc = utimensat(dirfd, path.data(), spec.data(), flags);
        return rc < 0 ? SRV(-errno) : rc;
      }

    case 93:  // exit
    case 94:  // exit_group
      {
	throw CoreException(CoreException::Exit, "", a0);
	return 0;
      }

    case 113:  // clock_gettime
      {
	clockid_t clk_id = a0;
	uint64_t rvBuff = a1;

	struct timespec tp;
	if (clk_id == CLOCK_MONOTONIC or clk_id == CLOCK_MONOTONIC_COARSE or
	    clk_id == CLOCK_MONOTONIC_RAW)
	  {
	    // For repeatabilty. Pretend hart is running at 1 GHZ. Use instruction count.
	    tp.tv_sec = hart.getTime() / 1000000000;
	    tp.tv_nsec = hart.getTime() % 1000000000;
	  }
	else if (clock_gettime(clk_id, &tp) != 0)
	  return SRV(-errno);
	if (not hart.pokeMemory(rvBuff, uint64_t(tp.tv_sec), true))
	  return SRV(-1);
	if (not hart.pokeMemory(rvBuff + 8, uint64_t(tp.tv_nsec), true))
	  return SRV(-1);
	return 0;
      }

    case 153: // times
      {
	URV rvBuff = a0;

	errno = 0;
	struct tms tms0;
	auto ticks = times(&tms0);
	if (ticks < 0)
	  return SRV(-errno);

        size_t len = copyTmsToRiscv(hart, tms0, rvBuff);
        size_t expected = 4*sizeof(URV);

	return (len == expected)? ticks : SRV(-EINVAL);
      }

    case 160: // uname
      {
	// Assumes that x86 and rv Linux have same layout for struct utsname.
        URV rvBuff = a0;

	errno = 0;
	struct utsname uts;

	int rc = uname(&uts);
        if (rc >= 0)
          {
            strcpy(uts.release, "5.16.0");
            size_t len = writeHartMemory(hart, uts, rvBuff);
            return len == sizeof(uts)? rc : SRV(-EINVAL);
          }
	return SRV(-errno);
      }

    case 169: // gettimeofday
      {
	URV tvAddr = a0, tzAddr = 0;

	struct timeval tv0;
	struct timeval* tv0Ptr = &tv0;

	struct timezone tz0;
	struct timezone* tz0Ptr = &tz0;
	
	if (tvAddr == 0) tv0Ptr = nullptr;
	if (tzAddr == 0) tz0Ptr = nullptr;

	errno = 0;
	int rc = gettimeofday(tv0Ptr, tz0Ptr);
	if (rc < 0)
	  return SRV(-errno);

	if (tvAddr)
	  {
            size_t len = 0;
            size_t expected = 12; // uint64_t & uint32_t
	    if (sizeof(URV) == 4)
	      len = copyTimevalToRiscv32(hart, tv0, tvAddr);
	    else
              {
                len = copyTimevalToRiscv64(hart, tv0, tvAddr);
                expected = 16; // uint64_t & unit64_t
              }
            if (len != expected)
              return SRV(-EINVAL);
	  }

	if (tzAddr)
          {
            size_t len = copyTimezoneToRiscv(hart, tz0, tzAddr);
            if (len != 2*sizeof(URV))
              return SRV(-EINVAL);
          }

	return rc;
      }

    case 174: // getuid
      {
	SRV rc = getuid();
	return rc;
      }

    case 175: // geteuid
      {
	SRV rv = geteuid();
	return rv;
      }

    case 176: // getgid
      {
	SRV rv = getgid();
	return rv;
      }

    case 177: // getegid
      {
	SRV rv = getegid();
	return rv;
      }

    case 215: // unmap
      {
    	URV addr = a0;
    	URV size = a1;
    	return mmap_dealloc(hart, addr, size);
      }

    case 216: // mremap
      {
    	URV addr = a0;
    	URV old_size = a1;
    	URV new_size = ((a2+(1<<12)-1)>>12)<<12;
    	bool maymove = a3 & MREMAP_MAYMOVE;
    	return mmap_remap(hart, addr,old_size,new_size, maymove);
      }

    case 222: // mmap2
      {
        URV start = a0;
        URV length = a1;
        URV prot = a2;
        URV tgt_flags = a3;

        if ((start & (((1<<12)-1) - 1)) ||
            ((tgt_flags & MAP_PRIVATE) == (tgt_flags & MAP_SHARED))  ||
            ((prot & PROT_WRITE) && (tgt_flags & MAP_SHARED)) ||
            !(tgt_flags & MAP_ANONYMOUS) or (tgt_flags & MAP_FIXED) ||
            !length) {
          return -1;
        }

        length = ((length+(1<<12)-1)>>12)<<12;

        return mmap_alloc(length);
      }

    case 276:  // rename
      {
        size_t pathAddr = a1;
        std::array<char, 1024> oldName;
        if (not copyRvString(hart, pathAddr, oldName))
          return SRV(-EINVAL);

        size_t newPathAddr = a3;
        std::array<char, 1024> newName;
        if (not copyRvString(hart, newPathAddr, newName))
          return SRV(-EINVAL);

        errno = 0;
        int result = rename(oldName.data(), newName.data());
        return (result == -1) ? -errno : result;
      }

#if 0
    case 435:  // clone3
      {
        auto nextAvail = nextAvailHart();
        if (not nextAvail)
          return SRV(-EAGAIN);

        std::array<uint64_t, 11> clone_args;
        if (not readHartMemory(hart, a0, clone_args))
          return SRV(-EINVAL);

        nextAvail->pokePc(hart.peekPc());
        for (int i = 0; i < 32; i++)
          {
            URV val;
            hart.peekIntReg(i, val);
            nextAvail->pokeIntReg(i, val);
          }
        nextAvail->pokeIntReg(RegSp, clone_args[5] + clone_args[6]);
        if (clone_args[0] & CLONE_SETTLS)
          nextAvail->pokeIntReg(RegTp, clone_args[7]);
        nextAvail->pokeIntReg(RegA0, 0);
        nextAvail->setSuspendState(false, 0);

        assert(clone_args[2] == clone_args[3]);
        if (clone_args[0] & CLONE_CHILD_SETTID)
          hart.pokeMemory(clone_args[2], nextAvail->sysHartIndex(), true);

        return nextAvail->sysHartIndex();
      }
#endif

    case 1024: // open
      {
	uint64_t rvPath = a0;
	int flags = a1;
	int x86Flags = 0;
	if (linux_)
          x86Flags = flags;
	else
	  {
	    // Newlib constants differ from Linux: compensate.
	    if (flags & 1)     x86Flags |= O_WRONLY;
	    if (flags & 0x2)   x86Flags |= O_RDWR;
	    if (flags & 0x200) x86Flags |= O_CREAT;
	  }
	int mode = a2;

        std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
          return SRV(-EINVAL);

	errno = 0;
	int rc = open(path.data(), x86Flags, mode);
        if (rc >= 0)
          {
            bool isRead = not (x86Flags & (O_WRONLY | O_RDWR));
            rc = registerLinuxFd(rc, path.data(), isRead);
            if (rc < 0)
              return SRV(-EINVAL);
          }
	return rc < 0 ? SRV(-errno) : rc;
      }

    case 1026: // unlink
      {
        uint64_t rvPath = a0;

        std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
          return SRV(-EINVAL);

	errno = 0;
	int rc = unlink(path.data());
	return rc < 0 ? SRV(-errno) : rc;
      }

    case 1038: // stat
      {
        uint64_t rvPath = a0;

        // Copy rv path into path.
        std::array<char, 1024> path;
        if (not copyRvString(hart, rvPath, path))
          return SRV(-EINVAL);

	struct stat buff;
	errno = 0;
	SRV rc = stat(path.data(), &buff);
	if (rc < 0)
	  return SRV(-errno);

	uint64_t rvBuff = a1;

        bool copyOk  = true;
        copyStatBufferToRiscv(hart, buff, rvBuff, copyOk);
	return copyOk? rc : SRV(-1);
      }
    }

  /// Syscall numbers about which we have already complained.
  static std::array<bool, 4096> reportedCalls;

  // using urv_ll = long long;
  //printf("syscall %s (0x%llx, 0x%llx, 0x%llx, 0x%llx) = 0x%llx\n",names[num].c_str(),urv_ll(a0), urv_ll(a1),urv_ll(a2), urv_ll(a3), urv_ll(retVal));
  //printf("syscall %s (0x%llx, 0x%llx, 0x%llx, 0x%llx) = unimplemented\n",names[num].c_str(),urv_ll(a0), urv_ll(a1),urv_ll(a2), urv_ll(a3));
  if (syscallIx < reportedCalls.size() and reportedCalls.at(syscallIx))
    return -1;

  std::cerr << "Error: Unimplemented syscall \"" << names[syscallIx] << "\" number "
            << syscallIx << "\n";

   if (syscallIx < reportedCalls.size())
     reportedCalls.at(syscallIx) = true;
   return -1;
}


template <typename URV>
bool
Syscall<URV>::saveFileDescriptors(const std::string& path)
{
  std::ofstream ofs(path, std::ios::trunc);
  if (not ofs)
    {
      std::cerr << "Error: Syscall::saveFileDescriptors: Failed to open " << path << " for write\n";
      return false;
    }

  for (auto kv : fdMap_)
    {
      int fd = kv.first;
      int remapped = kv.second;
      std::string path = fdPath_[fd];
      bool isRead = fdIsRead_[fd];
      off_t position = lseek(remapped, 0, SEEK_CUR);
      ofs << path << ' ' << fd << ' ' << position << ' ' << isRead << '\n';
    }

  return true;
}


template <typename URV>
bool
Syscall<URV>::loadFileDescriptors(const std::string& path)
{
  std::ifstream ifs(path);
  if (not ifs)
    {
      std::cerr << "Error: Syscall::loadFileDescriptors: Failed to open "
                << path << " for read\n";
      return false;
    }

  unsigned errors = 0;

  std::string line;
  unsigned lineNum = 0;
  while (std::getline(ifs, line))
    {
      lineNum++;
      std::istringstream iss(line);
      std::string fdPath;
      int fd = 0;
      off_t position = 0;
      bool isRead = false;
      if (not (iss >> fdPath >> fd >> position >> isRead))
        {
          std::cerr << "Error: File " << path << ", Line " << lineNum << ": "
                    << "Failed to parse line\n";
          return false;
        }

      if (isRead)
        {
          int newFd = open(fdPath.c_str(), O_RDONLY);
          if (newFd < 0)
            {
              std::cerr << "Error: Hart::loadFileDecriptors: Failed to open file "
                        << fdPath << " for read\n";
              errors++;
              continue;
            }
          if (lseek(newFd, position, SEEK_SET) == off_t(-1))
            {
              std::cerr << "Error: Hart::loadFileDecriptors: Failed to seek on file "
                        << fdPath << '\n';
              errors++;
              continue;
            }
          fdMap_[fd] = newFd;
	  fdPath_[fd] = fdPath;
          fdIsRead_[fd] = true;
          readPaths_.insert(fdPath);
        }
      else
        {
          int newFd = -1;
          if (Filesystem::is_regular_file(fdPath))
            {
              newFd = open(fdPath.c_str(), O_RDWR);
              if (lseek(newFd, position, SEEK_SET) == off_t(-1))
                {
                  std::cerr << "Error: Hart::loadFileDecriptors: Failed to seek on file "
                            << fdPath << '\n';
                  errors++;
                  continue;
                }
            }
          else
            newFd = open(fdPath.c_str(), O_WRONLY | O_CREAT, S_IRUSR | S_IWUSR);

          if (newFd < 0)
            {
              std::cerr << "Error: Hart::loadFileDecriptors: Failed to open file "
                        << fdPath << " for write\n";
              errors++;
              continue;
            }
          fdMap_[fd] = newFd;
	  fdPath_[fd] = fdPath;
          fdIsRead_[fd] = false;
          writePaths_.insert(fdPath);
        }
    }

  return errors == 0;
}


template <typename URV>
uint64_t
Syscall<URV>::mmap_alloc(uint64_t size)
{
  auto it = mmap_blocks_.begin();
  for (; it!=mmap_blocks_.end(); ++it)
    if(it->second.free and it->second.length>=size)
      break;

  if (it != mmap_blocks_.end())
    {
      auto orig_size = it->second.length;
      auto addr = it->first;
      it->second.free = false;
      if(orig_size > size)
        {
          mmap_blocks_.insert(std::make_pair(addr+size, blk_t(orig_size-size, true)));
          it->second.length =  size;
        }
      return addr;
    }

  std::cerr << "Error: Whisper: Target program failed in mmap: size=" << size << '\n';
  return uint64_t(-1);
}


template <typename URV>
int
Syscall<URV>::mmap_dealloc(Hart<URV>& hart, uint64_t addr, uint64_t size)
{
  if (mmap_blocks_.empty())
    return -1;

  // Find block containing address.
  auto curr = mmap_blocks_.lower_bound(addr);  // Smallest item that is >= addr
  if (curr == mmap_blocks_.end() or curr->first > addr)
    --curr;

  // Check that requested unmap falls within a prevously mapped block.
  auto curr_size = curr->second.length;
  if (addr < curr->first or addr > (curr->first + curr->second.length) or
      curr->second.free or (addr + size) > (curr->first + curr_size))
    return -1;

  if (addr > curr->first)
    {
      // Deallocating tail part of block.
      auto next = curr; ++next;  // Block following current
      curr->second.length -= addr - curr->first; // Trim current block
      // Create a new free block
      auto latest = mmap_blocks_.insert(std::make_pair(addr, blk_t(size, true))).first;
      // Merge new block with block following it if that is free.
      if (next != mmap_blocks_.end() and next->second.free and addr + size == next->first)
	{
	  latest->second.length += next->second.length;
	  mmap_blocks_.erase(next);
	}
      return 0;
    }

  assert(not curr->second.free and size <= curr_size);
  curr->second.free = true;

  // Clear deallocated space
  auto mem_addr = curr->first;
  auto mem_end_addr = mem_addr + size;
  for (; mem_addr<mem_end_addr; mem_addr+=uint64_t(sizeof(uint64_t)))
    hart.pokeMemory(mem_addr,uint64_t(0), true /*usePma*/);

  if (size < curr_size)
    {
      // Deallocating leading part of block. Put back as used tail end
      // of original block
      mmap_blocks_.insert(std::make_pair(mem_end_addr, blk_t(curr_size - size, false)));
      curr->second.length = size;
    }
  else
    {
      // Merge block with subsequent block if they are adjacent in memory.
      auto next = curr; ++next;
      if (next != mmap_blocks_.end() and next->second.free and mem_end_addr == next->first)
	{
	  curr->second.length += next->second.length;
	  mmap_blocks_.erase(next);
	}
    }

  // Merge block with preceeding block if the are adjacent in memory.
  if(curr != mmap_blocks_.begin())
    {
      auto prev = curr; --prev;
      if (prev->second.free and (prev->first + prev->second.length) == mem_addr)
        {
          prev->second.length += curr->second.length;
          mmap_blocks_.erase(curr);
        }
    }
  //print_mmap("dealloc");
  return 0;
}


template <typename URV>
uint64_t
Syscall<URV>::mmap_remap(Hart<URV>& hart, uint64_t addr, uint64_t old_size, uint64_t new_size,
                         bool maymove)
{
  if (old_size == new_size) return addr;
  auto curr = mmap_blocks_.find(addr);

  if (old_size>new_size)
    {
      assert(curr != mmap_blocks_.end() and curr->second.length == old_size and not curr->second.free);
      curr->second.length = new_size;
      mmap_blocks_.insert(std::make_pair(addr+new_size, blk_t(old_size-new_size, false)));
      mmap_dealloc(hart,addr+new_size,old_size-new_size);
      //print_mmap("remap1");
      return addr;
    }
  auto next = curr;
  auto diff = new_size - old_size;
  if ((++next) != mmap_blocks_.end() and next->second.free and next->second.length >= diff)
    {
      curr->second.length = new_size;
      if(auto rest = next->second.length - diff)
        mmap_blocks_.insert(std::make_pair(next->first+diff, blk_t(rest, true)));
      mmap_blocks_.erase(next);
      //print_mmap("remap2");
      return addr;
    }
  if (maymove)
    {
      auto new_addr = mmap_alloc(new_size);
      for (uint64_t index=0; index<old_size; index+=uint64_t(sizeof(uint64_t)))
        {
          uint64_t data;
          bool usePma = true;
          hart.peekMemory(addr+index, data, usePma);
          hart.pokeMemory(new_addr+index, data, usePma);
        }
      mmap_dealloc(hart, addr, old_size);
      //print_mmap("remap3");
      return new_addr;
    }
  return -1;
}


// TBD FIX: Needs improvement.
template<typename URV>
void
Syscall<URV>::getUsedMemBlocks(uint64_t sp, std::vector<AddrLen>& usedBlocks)
{
  usedBlocks.clear();

  // Up to 16 GB, snapshot the whole memory.
  uint64_t memSize = harts_.at(0)->getMemorySize();
  if (memSize <= 0x400000000L)
    {
      usedBlocks.emplace_back(0, memSize);
      return;
    }

  // This does not work for raw mode.
  usedBlocks.emplace_back(0, progBreak_);
  for (auto& it:mmap_blocks_)
    if (not it.second.free)
      usedBlocks.emplace_back(it.first, it.second.length);

  const uint64_t maxStackSize = UINT64_C(1024)*1024*256;
  uint64_t stackSize = memSize - sp + 4096;;
  if (stackSize > maxStackSize)
    std::cerr << "Error: Info: detUsedMemBlocks: Stack size too large: " << stackSize << "\n";
  
  usedBlocks.emplace_back(memSize - stackSize, stackSize);
}


template <typename URV>
bool
Syscall<URV>::saveMmap(const std::string & filename)
{
  // open file for write, check success
  std::ofstream ofs(filename, std::ios::trunc);
  if (not ofs)
    {
      std::cerr << "Error: Syscall::saveMmap failed - cannot open " << filename
                << " for write\n";
      return false;
    }

  for (auto& it: mmap_blocks_)
    ofs << it.first << " " << it.second.length << " " << it.second.free <<"\n";

  return true;
}


template <typename URV>
bool
Syscall<URV>::loadMmap(const std::string & filename)
{
  // open file for read, check success
  std::ifstream ifs(filename);
  if (not ifs)
    {
      std::cerr << "Error: Syscall::loadMmap failed - cannot open " << filename
                << " for read\n";
      return false;
    }
  std::string line;
  mmap_blocks_.clear();
  while(std::getline(ifs, line))
    {
      std::istringstream iss(line);
      uint64_t addr, length;
      bool valid;
      iss >> addr;
      iss >> length;
      iss >> valid;
      mmap_blocks_.insert(std::make_pair(addr, blk_t(length, valid)));
    }

  return true;
}

template class WdRiscv::Syscall<uint32_t>;
template class WdRiscv::Syscall<uint64_t>;
