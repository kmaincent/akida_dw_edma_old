// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 Brainchip.
 * Akida PCIe driver HDMA IP
 *
 * Author: Köry Maincent <kory.maincent@bootlin.com>
 */
#include "akida-pcie-hdma.h"

/*
 * 0x00 00000001           // Enable Wr
 * 0x1c 00000020           // 32:0 transfer size
 * 0x20 20000100           // 32b SAR source addr
 * 0x24 00000000           // 32b High SAR source addr
 * 0x28 FCC00000           // 32b DAR source addr
 * 0x2c 00000000           // 32b High DAR source addr
 * 0x34 00000000           // wr control
 * 0x38 00000000           // wr ch PCIe function
 * 0x04 00000001           // 1: stop 0: start
 * 0x80 00000000           // DMA status. 3: done 2: error 1: running
 */
static void akida_1500_dma_check_status(struct akida_dma_chan *dma_chan)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&dma_chan->lock, flags);
	if (completion_done(&dma_chan->dma_complete))
		goto out;

	val = readl(dma_chan->mmio_chan + HDMA_STATUS);
	if ((val & 0x3) == HDMA_ST_STOPPED) {
		val = readl(dma_chan->mmio_chan + HDMA_XFERSIZE);
		if (val)
			goto out;
		complete(&dma_chan->dma_complete);
	} else if ((val & 0x3) == HDMA_ST_ABORTED) {
		dma_chan->abort = true;
		complete(&dma_chan->dma_complete);
	}

out:
	spin_unlock_irqrestore(&dma_chan->lock, flags);
}

static irqreturn_t akida_1500_dma_callback(int irq, void *dev_id)
{
	struct akida_dev *akida = dev_id;
	int i;

	for (i = 0; i < ARRAY_SIZE(akida->rxchan); i++) {
		akida_1500_dma_check_status(&akida->rxchan[i]);
	}

	for (i = 0; i < ARRAY_SIZE(akida->txchan); i++) {
		akida_1500_dma_check_status(&akida->txchan[i]);
	}

	return IRQ_HANDLED;
}

void akida_hdma_init(struct akida_dev *akida)
{
	int i;

	/* Init rx channels */
	for (i = 0; i < ARRAY_SIZE(akida->rxchan); i++) {
		init_completion(&akida->rxchan[i].dma_complete);
		akida->rxchan[i].mmio_chan = akida->mmio_bar0 + i * 0x200 + HDMA_OFF_WRCH;
		spin_lock_init(&akida->rxchan[i].lock);
	}

	/* Init tx channels */
	for (i = 0; i < ARRAY_SIZE(akida->txchan); i++) {
		init_completion(&akida->txchan[i].dma_complete);
		akida->txchan[i].mmio_chan = akida->mmio_bar0 + i * 0x200 + HDMA_OFF_RDCH;
		spin_lock_init(&akida->txchan[i].lock);
	}
}

int akida_hdma_irq_init(struct akida_dev *akida)
{
	struct pci_dev *pdev = akida->pdev;
	struct msi_msg msi;
	int ret, i, irq;

	irq = pci_irq_vector(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq,
			       akida_1500_dma_callback, 0,
			       pdev->driver->name, akida);
	if (ret) {
		pci_err(pdev, "request interrupt failed (%d)\n", ret);
		pci_free_irq_vectors(pdev);
		return ret;
	}

	if (irq_get_msi_desc(irq))
		get_cached_msi_msg(irq, &msi);
	else {
		pci_err(pdev, "msi descriptor is NULL\n");
		free_irq(pci_irq_vector(pdev, 0), akida);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(akida->rxchan); i++) {
		writel(0x28, akida->rxchan[i].mmio_chan + HDMA_INT_SETUP);
		writel(msi.address_lo, akida->rxchan[i].mmio_chan + HDMA_MSI_STOP_LOW);
		writel(msi.address_hi, akida->rxchan[i].mmio_chan + HDMA_MSI_STOP_HIGH);
		writel(msi.address_lo, akida->rxchan[i].mmio_chan + HDMA_MSI_ABORT_LOW);
		writel(msi.address_hi, akida->rxchan[i].mmio_chan + HDMA_MSI_ABORT_HIGH);
		writel(msi.data, akida->rxchan[i].mmio_chan + HDMA_MSI_MSGD);
	}

	for (i = 0; i < ARRAY_SIZE(akida->txchan); i++) {
		writel(0x28, akida->txchan[i].mmio_chan + HDMA_INT_SETUP);
		writel(msi.address_lo, akida->txchan[i].mmio_chan + HDMA_MSI_STOP_LOW);
		writel(msi.address_hi, akida->txchan[i].mmio_chan + HDMA_MSI_STOP_HIGH);
		writel(msi.address_lo, akida->txchan[i].mmio_chan + HDMA_MSI_ABORT_LOW);
		writel(msi.address_hi, akida->txchan[i].mmio_chan + HDMA_MSI_ABORT_HIGH);
		writel(msi.data, akida->txchan[i].mmio_chan + HDMA_MSI_MSGD);
	}

	return ret;
}

void akida_hdma_irq_off(struct akida_dev *akida)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(akida->rxchan); i++)
		writel(0x0, akida->rxchan[i].mmio_chan + HDMA_INT_SETUP);

	for (i = 0; i < ARRAY_SIZE(akida->txchan); i++)
		writel(0x0, akida->txchan[i].mmio_chan + HDMA_INT_SETUP);
	free_irq(pci_irq_vector(akida->pdev, 0), akida);
}

static int akida_1500_dma_transfer(struct akida_dma_chan *dma_chan,
				    u64 src_addr,
				    u64 dst_addr, size_t size)
{
	void __iomem * mmio = dma_chan->mmio_chan;
	unsigned long flags;
	int ret;

	/* Prepare transactions */
	writel(0x1, mmio + HDMA_EN);
	writel(size, mmio + HDMA_XFERSIZE);
	writel((u32)(src_addr & U32_MAX), mmio + HDMA_SAR_LOW);
	writel((u32)(src_addr >> 32), mmio + HDMA_SAR_HIGH);
	writel((u32)(dst_addr & U32_MAX), mmio + HDMA_DAR_LOW);
	writel((u32)(dst_addr >> 32), mmio + HDMA_DAR_HIGH);
	writel(0x0, mmio + HDMA_CONTROL1);
	writel(0x0, mmio + HDMA_FUNC_NUM);

	spin_lock_irqsave(&dma_chan->lock, flags);
	/* Clear completion */
	reinit_completion(&dma_chan->dma_complete);

	/* Start transactions */
	writel(0x1, mmio + HDMA_DOORBELL);

	spin_unlock_irqrestore(&dma_chan->lock, flags);

	/* Wait for completion */
	ret = wait_for_completion_timeout(&dma_chan->dma_complete,
					  msecs_to_jiffies(2000));
	if (!ret) {
		pr_err("Akida: DMA wait completion timed out\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int akida_1500_dma_transfer_read(struct akida_dev *akida,
				struct akida_dma_chan *dma_chan,
			       phys_addr_t dev_addr,
			       void * tmp,
			       size_t size)
{
	struct device * dev = &akida->pdev->dev;
	dma_addr_t cpu_addr;
	int ret;
	dma_chan->abort = false;

	cpu_addr = dma_map_single(dev, tmp, size, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, cpu_addr)) {
		pci_err(akida->pdev, "DMA mapping failed\n");
		return -EINVAL;
	}

	ret = akida_1500_dma_transfer(dma_chan,
					       dev_addr,
					       cpu_addr,
					       size);

	if (dma_chan->abort) {
		pci_err(akida->pdev, "DMA transfert aborted\n");
		ret = -EFAULT;
	}

	dma_unmap_single(dev, cpu_addr, size, DMA_FROM_DEVICE);
	return ret;
}

static int akida_1500_dma_transfer_write(struct akida_dev *akida,
				struct akida_dma_chan *dma_chan,
			       phys_addr_t dev_addr,
			       void * tmp,
			       size_t size)
{
	struct device * dev = &akida->pdev->dev;
	dma_addr_t cpu_addr;
	int ret;
	dma_chan->abort = false;

	cpu_addr = dma_map_single(dev, tmp, size, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, cpu_addr)) {
		pci_err(akida->pdev, "DMA mapping failed\n");
		return -EINVAL;
	}

	ret = akida_1500_dma_transfer(dma_chan,
					       cpu_addr,
					       dev_addr,
					       size);

	if (dma_chan->abort) {
		pci_err(akida->pdev, "DMA transfert aborted\n");
		ret = -EFAULT;
	}

	dma_unmap_single(dev, cpu_addr, size, DMA_TO_DEVICE);
	return ret;
}

ssize_t akida_1500_skel_read(struct file *file, char __user *buf,
			     size_t sz, loff_t *ppos)
{
	struct akida_dev *akida =
		container_of(file->private_data, struct akida_dev, miscdev);
	struct akida_dma_chan *rxchan;
	char __user *usr_buf;
	size_t left, size, ret;
	void *tmp;

	tmp = kmalloc(AKIDA_DMA_XFER_MAX_SIZE, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	rxchan = akida_acquire_rxchan(akida);
	if (IS_ERR(rxchan)) {
		kfree(tmp);
		return PTR_ERR(rxchan);
	}

	left = sz;
	usr_buf = buf;
	while (left) {
		/* Limit transfer chunk ... */
		size = left > AKIDA_DMA_XFER_MAX_SIZE ?
			AKIDA_DMA_XFER_MAX_SIZE : left;

		/* ... do transfer ... */
		ret = akida_1500_dma_transfer_read(akida, rxchan, *ppos, tmp, size);
		if (ret < 0)
			goto end;

		/* ... copy transfered chunk to the user buffer */
		if (copy_to_user(usr_buf, tmp, size)) {
			ret = -EFAULT;
			goto end;
		}

		*ppos += size;
		usr_buf += size;
		left -= size;
	}

	ret = sz;
end:
	akida_release_rxchan(akida, rxchan);
	kfree(tmp);
	return ret;
}

ssize_t akida_1500_skel_write(struct file *file,
				     const char __user *buf, size_t sz,
				     loff_t *ppos)
{
	struct akida_dev *akida =
		container_of(file->private_data, struct akida_dev, miscdev);
	struct akida_dma_chan *txchan;
	const char __user *usr_buf;
	size_t left, size, ret;
	void *tmp;

	tmp = kmalloc(AKIDA_DMA_XFER_MAX_SIZE, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	txchan = akida_acquire_txchan(akida);
	if (IS_ERR(txchan)) {
		kfree(tmp);
		return PTR_ERR(txchan);
	}

	left = sz;
	usr_buf = buf;
	while (left) {
		/* Limit transfer chunk ... */
		size = left > AKIDA_DMA_XFER_MAX_SIZE ?
			AKIDA_DMA_XFER_MAX_SIZE : left;

		/* ... copy chunk from the user buffer ... */
		if (copy_from_user(tmp, usr_buf, size)) {
			ret = -EFAULT;
			goto end;
		}

		/* ... do transfer ... */
		ret = akida_1500_dma_transfer_write(akida, txchan, *ppos, tmp, size);
		if (ret < 0)
			goto end;

		*ppos += size;
		usr_buf += size;
		left -= size;
	}

	ret = sz;

end:
	akida_release_txchan(akida, txchan);
	kfree(tmp);
	return ret;
}
