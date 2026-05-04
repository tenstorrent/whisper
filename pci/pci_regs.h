#pragma once

#define PCI_BASE_ADDRESS_SPACE          0x01
#define PCI_BASE_ADDRESS_SPACE_IO       0x01
#define PCI_BASE_ADDRESS_SPACE_MEMORY   0x00
#define PCI_BASE_ADDRESS_0              0x10
#define PCI_BASE_ADDRESS_5              0x24
#define PCI_BASE_ADDRESS_IO_MASK        (~0x03UL)
#define PCI_BASE_ADDRESS_MEM_MASK       (~0x0fUL)
#define PCI_CAP_ID_MSIX                 0x11
#define PCI_CAP_ID_VNDR                 0x09
#define PCI_COMMAND_IO                  0x1
#define PCI_COMMAND_MEMORY              0x2
#define PCI_HEADER_TYPE_NORMAL          0
#define PCI_MSIX_ENTRY_CTRL_MASKBIT     1
#define PCI_MSIX_FLAGS_ENABLE           0x8000
#define PCI_MSIX_FLAGS_MASKALL          0x4000
#define PCI_ROM_ADDRESS                 0x30
#define PCI_STATUS_CAP_LIST             0x10
