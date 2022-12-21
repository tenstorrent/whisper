#pragma once

namespace WdRiscv
{
  /// Structure used to unpack/pack the fields of the machine status
  /// register.
  template <typename URV>
  union MstatusFields;

  /// 32-bit version.
  template <>
  union MstatusFields<uint32_t>
  {
    MstatusFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_;   // Machine status register value.
    struct
    {
      unsigned UIE      : 1;
      unsigned SIE      : 1;
      unsigned res2     : 1;
      unsigned MIE      : 1;
      unsigned UPIE     : 1;
      unsigned SPIE     : 1;
      unsigned UBE      : 1;
      unsigned MPIE     : 1;
      unsigned SPP      : 1;
      unsigned VS       : 2;
      unsigned MPP      : 2;
      unsigned FS       : 2;
      unsigned XS       : 2;
      unsigned MPRV     : 1;
      unsigned SUM      : 1;
      unsigned MXR      : 1;
      unsigned TVM      : 1;
      unsigned TW       : 1;
      unsigned TSR      : 1;
      unsigned res0     : 8;  // Reserved
      unsigned SD       : 1;
    } bits_;
  };

  /// 64-bit version.
  template <>
  union MstatusFields<uint64_t>
  {
    MstatusFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;   // Machine status register value.
    struct
    {
      unsigned UIE      : 1;
      unsigned SIE      : 1;
      unsigned res0     : 1;
      unsigned MIE      : 1;
      unsigned UPIE     : 1;
      unsigned SPIE     : 1;
      unsigned UBE      : 1;
      unsigned MPIE     : 1;
      unsigned SPP      : 1;
      unsigned VS       : 2;
      unsigned MPP      : 2;
      unsigned FS       : 2;
      unsigned XS       : 2;
      unsigned MPRV     : 1;
      unsigned SUM      : 1;
      unsigned MXR      : 1;
      unsigned TVM      : 1;
      unsigned TW       : 1;
      unsigned TSR      : 1;
      unsigned res1     : 9;
      unsigned UXL      : 2;
      unsigned SXL      : 2;
      unsigned SBE      : 1;
      unsigned MBE      : 1;
      unsigned GVA      : 1;
      unsigned MPV      : 1;
      unsigned res2     : 23;  // Reserved
      unsigned SD       : 1;
    } bits_;
  };


  /// Structure used to unpack/pack the fields of the hypervisor
  /// status register.
  template <typename URV>
  union HstatusFields;

  /// 32-bit version.
  template <>
  union HstatusFields<uint32_t>
  {
    HstatusFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_;   // Hypervisor status register value.
    struct
    {
      unsigned res0     : 5;
      unsigned VSBE     : 1;   // Virt supervisor big endian
      unsigned GVA      : 1;   // Guest virtual address
      unsigned SPV      : 1;   // Supervisor previous virtual mode
      unsigned SPVP     : 1;   // Supervisor previous virtual privilege (nominal priv)
      unsigned HU       : 1;   // Hypervisor instructions available in user mode
      unsigned res1     : 2;
      unsigned VGEIN    : 6;   // Virtual guest external interrupt number
      unsigned res2     : 2;
      unsigned VTVM     : 1;   // Trap on access to vsatp or SFENCE.VMA or SINVAL.VMA
      unsigned VTW      : 1;
      unsigned VTSR     : 1;   // Trap on sret
      unsigned res3     : 9;
    } bits_;
  };

  /// 64-bit version.
  template <>
  union HstatusFields<uint64_t>
  {
    HstatusFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;   // Machine status register value.
    struct
    {
      unsigned res0     : 5;
      unsigned VSBE     : 1;   // Virt supervisor big endian
      unsigned GVA      : 1;   // Guest virtual address
      unsigned SPV      : 1;   // Supervisor previous virtual mode
      unsigned SPVP     : 1;   // Supervisor previous virtual privilege (nominal priv)
      unsigned HU       : 1;   // Hypervisor instructions available in user mode
      unsigned res1     : 2;
      unsigned VGEIN    : 6;   // Virtual guest external interrupt number
      unsigned res2     : 2;
      unsigned VTVM     : 1;   // Trap on access to vsatp or SFENCE.VMA or SINVAL.VMA
      unsigned VTW      : 1;
      unsigned VTSR     : 1;   // Trap on sret
      unsigned res3     : 9;
      unsigned VSXL     : 2;
      unsigned res4     : 29;
    } bits_;
  };


  /// Structure used to unpack/pack the fields of the SATP register
  template <typename URV>
  union SatpFields;

  template <>
  union SatpFields<uint32_t>
  {
    SatpFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_;  // SATP register value
    struct
    {
      unsigned PPN : 22;
      unsigned ASID : 9;
      unsigned MODE : 1;
    } bits_;
  };

  template <>
  union SatpFields<uint64_t>
  {
    SatpFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // SATP register value
    struct
    {
      uint64_t PPN : 44;
      unsigned ASID : 16;
      unsigned MODE : 4;
    } bits_;
  };

  /// Structure used to unpack/pack the fields of the MENVCFG register
  template <typename URV>
  union MenvcfgFields;

  template <>
  union MenvcfgFields<uint32_t>
  {
    MenvcfgFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // MENVCFG register value
    struct
    {
      unsigned FIOM : 1;
      unsigned reserved0: 3;
      unsigned CBIE : 2;
      unsigned CBCFE : 1;
      unsigned CBZE : 1;
      unsigned reserved1 : 24;
    } bits_;
  };

  template <>
  union MenvcfgFields<uint64_t>
  {
    MenvcfgFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // MENVCFG register value
    struct
    {
      unsigned FIOM : 1;
      unsigned reserved0: 3;
      unsigned CBIE : 2;
      unsigned CBCFE : 1;
      unsigned CBZE : 1;
      uint64_t reserved1 : 54;
      unsigned PBMTE : 1;
      unsigned STCE : 1;
    } bits_;
  };

  /// Structure used to unpack/pack the fields of the MENVCFGH register (rv32 only)
  template <typename URV>
  union MenvcfghFields;

  template <>
  union MenvcfghFields<uint32_t>
  {
    MenvcfghFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // MENVCFGH register value
    struct
    {
      unsigned reserved0 : 30;
      unsigned PBMTE : 1;
      unsigned STCE : 1;
    } bits_;
  };

  /// Structure used to unpack/pack the fields of the SENVCFG register
  template <typename URV>
  union SenvcfgFields;

  template <>
  union SenvcfgFields<uint32_t>
  {
    SenvcfgFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // SENVCFG register value
    struct
    {
      unsigned FIOM : 1;
      unsigned reserved0 : 3;
      unsigned CBIE : 2;
      unsigned CBCFE : 1;
      unsigned CBZE : 1;
      unsigned reserved1 : 24;
    } bits_;
  };

  template <>
  union SenvcfgFields<uint64_t>
  {
    SenvcfgFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // SENVCFG register value
    struct
    {
      unsigned FIOM : 1;
      unsigned reserved0 : 3;
      unsigned CBIE : 2;
      unsigned CBCFE : 1;
      unsigned CBZE : 1;
      uint64_t reserved1 : 56;
    } bits_;
  };

  /// Structure used to unpack/pack the fields of the DCSR register
  template <typename URV>
  union DcsrFields;

  template <>
  union DcsrFields<uint32_t>
  {
    DcsrFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // DCSR register value
    struct
    {
      unsigned PRV : 2;
      unsigned STEP : 1;
      unsigned NMIP : 1;
      unsigned MPRVEN : 1;
      unsigned reserved0 : 1;
      unsigned CAUSE : 3;
      unsigned STOPTIME : 1;
      unsigned STOPCOUNT : 1;
      unsigned STEPIE : 1;
      unsigned EBREAKU : 1;
      unsigned EBREAKS : 1;
      unsigned reserved1 : 1;
      unsigned EBREAKM : 1;
      unsigned reserved2 : 12;
      unsigned DEBUGVER : 4;
    } bits_;
  };

  template <>
  union DcsrFields<uint64_t>
  {
    DcsrFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // DCSR register value
    struct
    {
      unsigned PRV : 2;
      unsigned STEP : 1;
      unsigned NMIP : 1;
      unsigned MPRVEN : 1;
      unsigned reserved0 : 1;
      unsigned CAUSE : 3;
      unsigned STOPTIME : 1;
      unsigned STOPCOUNT : 1;
      unsigned STEPIE : 1;
      unsigned EBREAKU : 1;
      unsigned EBREAKS : 1;
      unsigned reserved1 : 1;
      unsigned EBREAKM : 1;
      unsigned reserved2 : 12;
      unsigned DEBUGVER : 4;
    } bits_;
  };

  /// Structure used to unpack/pack the fields of the VTYPE register
  template<typename URV>
  union VtypeFields;

  template <>
  union VtypeFields<uint32_t>
  {
    VtypeFields(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_; // VTYPE register value
    struct
    {
      unsigned LMUL : 3;
      unsigned SEW : 3;
      unsigned VTA : 1;
      unsigned VMA : 1;
      unsigned reserved0 : 23;
      unsigned VILL : 1;
    } bits_;
  };

  template <>
  union VtypeFields<uint64_t>
  {
    VtypeFields(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_; // VTYPE register value
    struct
    {
      unsigned LMUL : 3;
      unsigned SEW : 3;
      unsigned VTA : 1;
      unsigned VMA : 1;
      uint64_t reserved0 : 55;
      unsigned VILL : 1;
    } bits_;
  };
}