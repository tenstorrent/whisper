#pragma once

#include <stdint.h>

struct virtio_blk_config {

 uint64_t capacity;

 uint32_t size_max;

 uint32_t seg_max;

 struct virtio_blk_geometry {
  uint16_t cylinders;
  uint8_t heads;
  uint8_t sectors;
 } geometry;


 uint32_t blk_size;



 uint8_t physical_block_exp;

 uint8_t alignment_offset;

 uint16_t min_io_size;

 uint32_t opt_io_size;


 uint8_t wce;
 uint8_t unused;


 uint16_t num_queues;
} __attribute__((packed));

struct virtio_blk_outhdr {
	/* VIRTIO_BLK_T* */
	uint32_t type;
	/* io priority. */
	uint32_t ioprio;
	/* Sector (ie. 512 byte offset) */
	uint64_t sector;
};

#define VIRTIO_BLK_F_RO             5
#define VIRTIO_BLK_S_IOERR          1
#define VIRTIO_BLK_S_OK             0
#define VIRTIO_BLK_T_GET_ID         8
#define VIRTIO_BLK_T_IN             0
#define VIRTIO_BLK_T_OUT            1
#define VIRTIO_CONFIG_S_FAILED      0x80
#define VIRTIO_F_VERSION_1          32
#define VIRTIO_ID_BLOCK             2
#define VIRTIO_MSI_NO_VECTOR        0xffff
#define VIRTIO_PCI_CAP_COMMON_CFG   1
#define VIRTIO_PCI_CAP_DEVICE_CFG   4
#define VIRTIO_PCI_CAP_ISR_CFG      3
#define VIRTIO_PCI_CAP_NOTIFY_CFG   2
#define VIRTIO_PCI_CAP_PCI_CFG      5
#define VIRTIO_PCI_COMMON_DF        4
#define VIRTIO_PCI_COMMON_DFSELECT  0
#define VIRTIO_PCI_COMMON_MSIX      16
#define VIRTIO_PCI_COMMON_Q_AVAILHI 44
#define VIRTIO_PCI_COMMON_Q_AVAILLO 40
#define VIRTIO_PCI_COMMON_Q_DESCHI  36
#define VIRTIO_PCI_COMMON_Q_DESCLO  32
#define VIRTIO_PCI_COMMON_Q_ENABLE  28
#define VIRTIO_PCI_COMMON_Q_MSIX    26
#define VIRTIO_PCI_COMMON_Q_SELECT  22
#define VIRTIO_PCI_COMMON_Q_SIZE    24
#define VIRTIO_PCI_COMMON_Q_USEDHI  52
#define VIRTIO_PCI_COMMON_Q_USEDLO  48
#define VIRTIO_PCI_COMMON_STATUS    20
