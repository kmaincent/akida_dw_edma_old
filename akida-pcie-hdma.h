#ifndef _AKIDA_PCIE_HDMA_H
#define _AKIDA_PCIE_HDMA_H

#define HDMA_CHAN_I_OFF	0x200

#define HDMA_OFF_WRCH		0x0
#define HDMA_OFF_RDCH		0x100

#define HDMA_EN			0x0
#define HDMA_DOORBELL		0x4
#define HDMA_ELEM_PF		0x8
#define HDMA_HANDSHAKE		0xc
#define HDMA_LLP_LOW		0x10
#define HDMA_LLP_HIGH		0x14
#define HDMA_CYCLE		0x18
#define HDMA_XFERSIZE		0x1c
#define HDMA_SAR_LOW		0x20
#define HDMA_SAR_HIGH		0x24
#define HDMA_DAR_LOW		0x28
#define HDMA_DAR_HIGH		0x2c
#define HDMA_WATERMARK_EN	0x30
#define HDMA_CONTROL1		0x34
#define HDMA_FUNC_NUM		0x38
#define HDMA_QOS		0x3c
#define HDMA_STATUS		0x80
#define HDMA_INT_STATUS		0x84
#define HDMA_INT_SETUP		0x88
#define HDMA_INT_CLEAR		0x8c
#define HDMA_MSI_STOP_LOW	0x90
#define HDMA_MSI_STOP_HIGH	0x94
#define HDMA_MSI_ABORT_LOW	0x98
#define HDMA_MSI_ABORT_HIGH	0x9c
#define HDMA_MSI_WATERMARK_LOW	0xa0
#define HDMA_MSI_WATERMARK_HIGH	0xa4
#define HDMA_MSI_MSGD		0xa8


/* Maximum DMA transfer chunk size */
#define AKIDA_DMA_XFER_MAX_SIZE  1024

#include <linux/miscdevice.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include "dw-edma-core.h"
#include "akida-edma.h"
#include "akida-pcie-hdma.h"

enum dw_hdma_status {
	HDMA_ST_UNSPEC = 0,
	HDMA_ST_RUNNING,
	HDMA_ST_ABORTED,
	HDMA_ST_STOPPED
};

struct akida_dma_chan {
	struct dma_chan *chan;
	struct completion dma_complete;
	enum dma_transfer_direction dma_xfer_dir;
	enum dma_data_direction dma_data_dir;
	dma_addr_t dma_buf;
	size_t dma_len;
	bool is_used;
	spinlock_t lock;
	bool abort;
	void __iomem *mmio_chan;
};

struct akida_dev {
	struct pci_dev *pdev;
	int devno;
	struct miscdevice miscdev;
	struct dw_edma_chip edma_chip;
	struct dw_edma dw;
	struct akida_dma_chan rxchan[2];
	struct akida_dma_chan txchan[2];
	wait_queue_head_t wq_rxchan;
	wait_queue_head_t wq_txchan;
	void __iomem *mmio_bar0;
};

void akida_hdma_init(struct akida_dev *akida);
int akida_hdma_irq_init(struct akida_dev *akida);
void akida_hdma_irq_off(struct akida_dev *akida);

ssize_t akida_1500_skel_read(struct file *, char __user *, size_t,
			     loff_t *);

ssize_t akida_1500_skel_write(struct file *, const char __user *, size_t,
			      loff_t *);

inline struct akida_dma_chan *akida_acquire_rxchan(struct akida_dev *);
inline void akida_release_rxchan(struct akida_dev *akida, struct akida_dma_chan *rxchan);

inline struct akida_dma_chan *akida_acquire_txchan(struct akida_dev *);
inline void akida_release_txchan(struct akida_dev *akida, struct akida_dma_chan *txchan);
#endif /* _AKIDA_PCIE_HDMA_H */
