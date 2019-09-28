/*
 * SwitchPi Thunder module Driver for DAHDI Telephony interface.
 * This driver is developed to support SwitchPi Thunder 4 ports board only,
 * you can use it by freely, but there is no warranty as it is.
 * Written by Xin Li <xin.li@switchpi.com>
 *
 * Copyright (C) 2017-2018, SwitchPi, Inc.
 *
 * All rights reserved.
 *
 */
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_data/dma-bcm2708.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/seq_file.h>

#define BCM2835_DMA_MAX_DMA_CHAN_SUPPORTED 14
#define BCM2835_DMA_CHAN_NAME_SIZE 8
#define BCM2835_DMA_BULK_MASK  BIT(0)

#define BCM2835_DMA_CS		0x00
#define BCM2835_DMA_ADDR	0x04
#define BCM2835_DMA_TI		0x08
#define BCM2835_DMA_SOURCE_AD	0x0c
#define BCM2835_DMA_DEST_AD	0x10
#define BCM2835_DMA_LEN		0x14
#define BCM2835_DMA_STRIDE	0x18
#define BCM2835_DMA_NEXTCB	0x1c
#define BCM2835_DMA_DEBUG	0x20

/* DMA CS Control and Status bits */
#define BCM2835_DMA_ACTIVE	BIT(0)  /* activate the DMA */
#define BCM2835_DMA_END		BIT(1)  /* current CB has ended */
#define BCM2835_DMA_INT		BIT(2)  /* interrupt status */
#define BCM2835_DMA_DREQ	BIT(3)  /* DREQ state */
#define BCM2835_DMA_ISPAUSED	BIT(4)  /* Pause requested or not active */
#define BCM2835_DMA_ISHELD	BIT(5)  /* Is held by DREQ flow control */
#define BCM2835_DMA_WAITING_FOR_WRITES BIT(6) /* waiting for last
					       * AXI-write to ack
					       */
#define BCM2835_DMA_ERR		BIT(8)
#define BCM2835_DMA_PRIORITY(x) ((x & 15) << 16) /* AXI priority */
#define BCM2835_DMA_PANIC_PRIORITY(x) ((x & 15) << 20) /* panic priority */
/* current value of TI.BCM2835_DMA_WAIT_RESP */
#define BCM2835_DMA_WAIT_FOR_WRITES BIT(28)
#define BCM2835_DMA_DIS_DEBUG	BIT(29) /* disable debug pause signal */
#define BCM2835_DMA_ABORT	BIT(30) /* Stop current CB, go to next, WO */
#define BCM2835_DMA_RESET	BIT(31) /* WO, self clearing */

/* Transfer information bits - also bcm2835_cb.info field */
#define BCM2835_DMA_INT_EN	BIT(0)
#define BCM2835_DMA_TDMODE	BIT(1) /* 2D-Mode */
#define BCM2835_DMA_WAIT_RESP	BIT(3) /* wait for AXI-write to be acked */
#define BCM2835_DMA_D_INC	BIT(4)
#define BCM2835_DMA_D_WIDTH	BIT(5) /* 128bit writes if set */
#define BCM2835_DMA_D_DREQ	BIT(6) /* enable DREQ for destination */
#define BCM2835_DMA_D_IGNORE	BIT(7) /* ignore destination writes */
#define BCM2835_DMA_S_INC	BIT(8)
#define BCM2835_DMA_S_WIDTH	BIT(9) /* 128bit writes if set */
#define BCM2835_DMA_S_DREQ	BIT(10) /* enable SREQ for source */
#define BCM2835_DMA_S_IGNORE	BIT(11) /* ignore source reads - read 0 */
#define BCM2835_DMA_BURST_LENGTH(x) ((x & 15) << 12)
#define BCM2835_DMA_PER_MAP(x)	((x & 31) << 16) /* REQ source */
#define BCM2835_DMA_WAIT(x)	((x & 31) << 21) /* add DMA-wait cycles */
#define BCM2835_DMA_NO_WIDE_BURSTS BIT(26) /* no 2 beat write bursts */

/* debug register bits */
#define BCM2835_DMA_DEBUG_LAST_NOT_SET_ERR	BIT(0)
#define BCM2835_DMA_DEBUG_FIFO_ERR		BIT(1)
#define BCM2835_DMA_DEBUG_READ_ERR		BIT(2)
#define BCM2835_DMA_DEBUG_OUTSTANDING_WRITES_SHIFT 4
#define BCM2835_DMA_DEBUG_OUTSTANDING_WRITES_BITS 4
#define BCM2835_DMA_DEBUG_ID_SHIFT		16
#define BCM2835_DMA_DEBUG_ID_BITS		9
#define BCM2835_DMA_DEBUG_STATE_SHIFT		16
#define BCM2835_DMA_DEBUG_STATE_BITS		9
#define BCM2835_DMA_DEBUG_VERSION_SHIFT		25
#define BCM2835_DMA_DEBUG_VERSION_BITS		3
#define BCM2835_DMA_DEBUG_LITE			BIT(28)

/* shared registers for all dma channels */
#define BCM2835_DMA_INT_STATUS         0xfe0
#define BCM2835_DMA_ENABLE             0xff0

#define BCM2835_DMA_DATA_TYPE_S8	1
#define BCM2835_DMA_DATA_TYPE_S16	2
#define BCM2835_DMA_DATA_TYPE_S32	4
#define BCM2835_DMA_DATA_TYPE_S128	16

#define CS_A     0
#define FIFO_A   1
#define MODE_A   2
#define RXC_A    3
#define TXC_A    4
#define DREQ_A   5
#define INTEN_A  6
#define INTSTC_A 7
#define GRAY     8

/* Valid only for channels 0 - 14, 15 has its own base address */
#define BCM2835_DMA_CHAN(n)	((n) << 8) /* Base address */
#define BCM2835_DMA_CHANIO(base, n) ((base) + BCM2835_DMA_CHAN(n))

/* the max dma length for different channels */
#define MAX_DMA_LEN SZ_1G
#define MAX_LITE_DMA_LEN (SZ_64K - 4)
#define BCM2708_PERI_BASE        0x3f000000
#define DMA_BASE                 (BCM2708_PERI_BASE + 0x007000)
#define I2S_BASE                 (BCM2708_PERI_BASE + 0x203000)
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000)
#define SPI0_BASE                (BCM2708_PERI_BASE + 0x204000)

#define CORE_CLOCK 250000000
#define CODEC_RESET 17
#define CODEC_CS0 16//22//16//cs0
#define CODEC_CS1 27 //cs1
#define CODEC_CS2 12//cs2
//#define CODEC_CS3 12 //cs3
#define OAKDSP_EN 23 //gpio12
#define MODULE_LED0 24
#define MODULE_LED1 3
#define MODULE_LED2 2
#define MODULE_LED3 4

#define CHANNEL_TX				9
#define CHANNEL_RX				10
#define PCM_TX					2
#define PCM_RX					3
#define DRV_NAME	"pidma"
#define HARDWARE_BUFFER_BYTES 64 //* 2

#define DMA_CB_TXFR_LEN_YLENGTH(y) (((y-1)&0x4fff) << 16)
#define DMA_CB_TXFR_LEN_XLENGTH(x) ((x)&0xffff)
#define DMA_CB_STRIDE_D_STRIDE(x)  (((x)&0xffff) << 16)
#define DMA_CB_STRIDE_S_STRIDE(x)  ((x)&0xffff)

static int oakdsp = 1;

struct dma_channel_header {
	  uint32_t cs;        // control and status.
	  uint32_t cblock;    // control block address.
	  void __iomem *chan_base;
	  int irq_number;
	  unsigned int irq_flags;
};

struct dma_cb {    // 32 bytes.
	  uint32_t info;   // transfer information.
	  uint32_t src;    // physical source address.
	  uint32_t dst;    // physical destination address.
	  uint32_t length; // transfer length.
	  uint32_t stride; // stride mode.
	  uint32_t next;   // next control block; Physical address. 32 byte aligned.
	  uint32_t pad[2];
};

struct pi_dma {
	spinlock_t lock;
	//TX Stuff
	struct dma_channel_header *tx_dma_chan;
	struct dma_cb *tx_dma_cb_ping;
	struct dma_cb *tx_dma_cb_pong;
	dma_addr_t tx_dma_addr;
	dma_addr_t tx_cb_addr_ping; //Physical address
	dma_addr_t tx_cb_addr_pong; //Physical address
	u32 *tx_buffer;

	//RX Stuff
	struct dma_channel_header *rx_dma_chan;
	struct dma_cb *rx_dma_cb_ping;
	struct dma_cb *rx_dma_cb_pong;
	dma_addr_t rx_dma_addr;
	dma_addr_t rx_cb_addr_ping; //Physical address
	dma_addr_t rx_cb_addr_pong; //Physical address
	u32 *rx_buffer;
};

int dmacnt, txmidcnt, txstacnt, rxmidcnt, rxstacnt;
int txdsrc, rxdsrc;
unsigned int *spi_base = NULL;
unsigned int *spi_fifo = NULL;
unsigned int *i2s_registers  = NULL;
void __iomem *base  = NULL;
unsigned int *gpio  = NULL;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_SETB(g) GPIO_SET |= 1 <<(g)
#define GPIO_CLRB(g) GPIO_CLR |= 1 <<(g)
#define GPIO_READ(g)  (*(gpio + 13) &= (1<<(g)))

static void (*thunder_isr_callback)(u32 *read_samples, u32 *write_samples) = NULL;
static int __readchunk ( void (*isr_callback)(u32 *read_samples, u32 *write_samples)) {
	thunder_isr_callback = isr_callback;
	return 0;
}

void init_gpio_cs(void)
{
    INP_GPIO(CODEC_RESET);
    OUT_GPIO(CODEC_RESET);
    INP_GPIO(CODEC_CS0);
    OUT_GPIO(CODEC_CS0);
    INP_GPIO(CODEC_CS1);
    OUT_GPIO(CODEC_CS1);
    INP_GPIO(CODEC_CS2);
    OUT_GPIO(CODEC_CS2);
    INP_GPIO(OAKDSP_EN);
    OUT_GPIO(OAKDSP_EN);
    //initial H
	GPIO_SETB(CODEC_RESET);
	GPIO_SETB(CODEC_CS0);
	GPIO_SETB(CODEC_CS1);
	GPIO_SETB(CODEC_CS2);
	GPIO_SETB(OAKDSP_EN);


    INP_GPIO(MODULE_LED3);
    OUT_GPIO(MODULE_LED3);
    //initial L
	GPIO_CLRB(MODULE_LED3);
}

static void led_on(void) {
	GPIO_CLRB(MODULE_LED3);
	GPIO_SETB(MODULE_LED3);
}

static void led_off(void) {
	GPIO_SETB(MODULE_LED3);
	GPIO_CLRB(MODULE_LED3);
}

static void oakdsp_on(void) {
	GPIO_SETB(OAKDSP_EN);
}

static void oakdsp_off(void) {
	GPIO_CLRB(OAKDSP_EN);
}

static void codec_reset(void)
{
	GPIO_SETB(CODEC_RESET);
	GPIO_CLRB(CODEC_RESET);
	udelay(1000);
	GPIO_SETB(CODEC_RESET);
	udelay(2000);
}

void set_codec_cs(int cs)
{
	switch (cs){
	case 0:
		GPIO_SETB(CODEC_CS0);
		break;
	case 1:
		GPIO_SETB(CODEC_CS1);
		break;
	case 2:
		GPIO_SETB(CODEC_CS2);
		break;
	case 3:
		//GPIO_SETB(CODEC_CS2);
		break;
	default:
		break;
	}
}

void clr_codec_cs(int cs)
{
	switch (cs){
	case 0:
		GPIO_CLRB(CODEC_CS0);
		break;
	case 1:
		GPIO_CLRB(CODEC_CS1);
		break;
	case 2:
		GPIO_CLRB(CODEC_CS2);
		break;
	case 3:
		//GPIO_SETB(CODEC_CS2);
		break;
	default:
		break;
	}
}

static int __init_spi_bus(void){
	int div;
	spi_base = ioremap(SPI0_BASE, SZ_4K);
	if (!spi_base) {
		printk("could not remap memory\n");
		return 1;
	}
	div = CORE_CLOCK / 10000000;//10M

	*spi_base             = 0x00;
	*spi_base            |= ((1<<4) | (1<<5)); // clear tx,rx fifo
	*spi_base            |= ((1<<2) | (1<<3)); // setup SPI mode 0 : CPOL and CPHA = 1
	*(spi_base + 2 ) = div;  // setup CLK_DIV as div
	*(spi_base)          &=~((1<<0) | (1<<1));// chip select :CS0
	*(spi_base)          &=~( 1<<21) ; // CSPOL0
	spi_fifo = spi_base + 1;
	return 0;
}

static int __stop_spi_bus(void){
	*spi_base &=~(1<<7); // clear TA
	*spi_base             = 0x00;
	*(spi_base + 2 ) = 0;
	return 0;
}

static void __spi_write(unsigned char cmd, int cs) {
	char temp_rx = 0x0;
	unsigned int poll_time_tx = 0xffff;
	//*spi_base &=~(1<<7); // clear TA
	clr_codec_cs(cs);
	*spi_base            |= (1<<5); // clear tx,rx fifo
	*spi_base            |= (1<<7); // set TA
	while( !(readl(spi_base) & (1<<18)) & (--poll_time_tx > 0));//TXD
	writel(cmd, spi_fifo);
	while(!(readl(spi_base) & (1<<17)) & (--poll_time_tx > 0));//RXD
	while(!(readl(spi_base) & (1<<16)) & (--poll_time_tx > 0));//TX DONE
	temp_rx = readl(spi_fifo);
	*spi_base &=~(1<<7); // clear TA
	set_codec_cs(cs);
}

static unsigned char __spi_read(int cs) {
	char temp_rx = 0x0;
	unsigned int poll_time_tx = 0xfff;
	clr_codec_cs(cs);
	*spi_base            |= (1<<7); // set TA
	while( !(readl(spi_base) & (1<<18))  & (--poll_time_tx > 0));
	writel(0xff, spi_fifo);
	while(!(readl(spi_base) & (1<<16)) & (--poll_time_tx > 0));
	while(!(readl(spi_base) & (1<<17)) & (--poll_time_tx > 0));
	temp_rx = readl(spi_fifo);
	*spi_base &=~(1<<7); // clear TA
	set_codec_cs(cs);
	return temp_rx;
}

static struct proc_dir_entry *proc_frame;
static int frame_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "-----DMA LEN   CNT %d\n", dmacnt);
	seq_printf(m, "-----DMA TXMID CNT %d\n", txmidcnt);
	seq_printf(m, "-----DMA TXSTA CNT %d\n", txstacnt);
	seq_printf(m, "-----DMA TXSRC ADR %x\n", txdsrc);
	seq_printf(m, "-----DMA RXMID CNT %d\n", rxmidcnt);
	seq_printf(m, "-----DMA RXSTA CNT %d\n", rxstacnt);
	seq_printf(m, "-----DMA RXSRC ADR %x\n", rxdsrc);
	return 0;
}

static int frame_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, frame_proc_show, NULL);
}

static const struct file_operations frame_proc_ops = {
	.open		= frame_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void dma_proc_create(void)
{
	proc_frame = proc_create("pidmainfo", 0, NULL, &frame_proc_ops);
}

static irqreturn_t pi_dma_callback(int irq, void *data)
{
	struct pi_dma *pi_dma = data;
	unsigned long flags;
	u32 *read_samples;
	u32 *write_samples;

	spin_lock_irqsave(&pi_dma->lock, flags);
	dmacnt ++;
	//first of all, initial the buffers
	write_samples = pi_dma->rx_buffer;
	read_samples = pi_dma->rx_buffer;

	if (irq == pi_dma->tx_dma_chan->irq_number) {
		/* Acknowledge interrupt */
		writel(BCM2835_DMA_INT, pi_dma->tx_dma_chan->chan_base + BCM2835_DMA_CS);
		// do something
		//keep it running
		writel(BCM2835_DMA_ACTIVE,
				pi_dma->tx_dma_chan->chan_base + BCM2835_DMA_CS);
	}else if (irq == pi_dma->rx_dma_chan->irq_number) {
		/* Acknowledge interrupt */
		writel(BCM2835_DMA_INT, pi_dma->rx_dma_chan->chan_base + BCM2835_DMA_CS);
		// do something
		rxdsrc = readl(pi_dma->rx_dma_chan->chan_base + BCM2835_DMA_DEST_AD);
		if (rxdsrc == pi_dma->rx_dma_addr + 0x20) {
			rxmidcnt ++;
			read_samples = pi_dma->rx_buffer;
		}else {
			rxstacnt ++;
			read_samples = pi_dma->rx_buffer + 8;
		}

		txdsrc = readl(pi_dma->tx_dma_chan->chan_base + BCM2835_DMA_SOURCE_AD);
		if (txdsrc <= pi_dma->tx_dma_addr + 0x20) {
			txmidcnt ++;
			write_samples = pi_dma->tx_buffer;
		}else {
			txstacnt ++;
			write_samples = pi_dma->tx_buffer + 8;
		}

		// done
		//keep it running
		writel(BCM2835_DMA_ACTIVE,
				pi_dma->rx_dma_chan->chan_base + BCM2835_DMA_CS);

		if (thunder_isr_callback != NULL) {
			thunder_isr_callback(read_samples, write_samples);
		}

	}else {
		return IRQ_NONE;
	}

	spin_unlock_irqrestore(&pi_dma->lock, flags);
	return IRQ_HANDLED;
}

static void setup_gpio(void)
{

	int pin;
	gpio = ioremap(GPIO_BASE, SZ_16K);

	/* SPI is on GPIO 7..11 */
	for (pin = 7; pin <= 11; pin++) {
		INP_GPIO(pin);		/* set mode to GPIO input first */
		SET_GPIO_ALT(pin, 0);	/* set mode to ALT 0 */
	}

	for (pin = 18; pin <= 21; pin++) {
		INP_GPIO(pin);		/* set mode to GPIO input first */
		SET_GPIO_ALT(pin, 0);	/* set mode to ALT 0 */
	}
}

static void start_i2s_tx(void)
{
    *(i2s_registers+CS_A) |= 1<<2 | 1<<1;
    printk(KERN_INFO "ENABLED I2S\n");
}

void setup_i2s(void)
{
    int i=0;
    i2s_registers = ioremap(I2S_BASE, 32);

    *(i2s_registers+CS_A) &= ~(1<<24); //just for completeness
    *(i2s_registers+CS_A) = 0;
    *(i2s_registers+MODE_A) = 0;
    *(i2s_registers+TXC_A) = 0;
    *(i2s_registers+RXC_A) = 0;
    *(i2s_registers+GRAY) = 0;
    *(i2s_registers+DREQ_A) = 0;
    udelay(100);
    *(i2s_registers+TXC_A) = 1<<31 | 1<<30  | 8<<16 | 1<<20;
    *(i2s_registers+RXC_A) = 1<<31 | 1<<30  | 8<<16 | 1<<20;
    *(i2s_registers+MODE_A) = 1<< 23 | 511 <<10 | 1 ;
    *(i2s_registers+CS_A) |= 1<<25;
    udelay(50);
    *(i2s_registers+CS_A) |= 1<<3 |1<<4 ; 
    *(i2s_registers+CS_A) |= 0x01;

    for(i = 0; i < 32; i++)
       (*(i2s_registers+FIFO_A)) = 0;
    *(i2s_registers+CS_A) |= 1<<24;

    udelay(100);
    *(i2s_registers+CS_A) |=  1<<9 ; 
    *(i2s_registers+DREQ_A) |=  1 <<24 | 16<<16 | 16<<8 | 8;  
    *(i2s_registers+CS_A) |= 1<<3  |1<<4 ; 
    printk(KERN_INFO "I2S SETUP COMPLETE\n");

    return;
}

static void stop_i2s(void)
{
    *(i2s_registers+INTSTC_A) = 0x000f;// clear status bits
    *(i2s_registers+INTEN_A) = 0x00;
    udelay(100);
    *(i2s_registers+CS_A) = 0;
    *(i2s_registers+MODE_A) = 0;
    *(i2s_registers+TXC_A) = 0;
    *(i2s_registers+RXC_A) = 0;
    *(i2s_registers+GRAY) = 0;
    *(i2s_registers+DREQ_A) = 0;
}

static struct device * gdevice;
static struct device * get_thunder_device(void) {
	if (gdevice == NULL) return NULL;
	return gdevice;
}

static int pi_dma_probe(struct platform_device *pdev)
{
	struct pi_dma *pi_dma;
	int txirq, rxirq;
	int i;

	base = ioremap(DMA_BASE, SZ_16K);
	if (!base) {
		printk("could not get dma hardware register address\n");
		return -1;
	}

	pi_dma = devm_kzalloc(&pdev->dev, sizeof(*pi_dma), GFP_KERNEL);
	if (!pi_dma) {
		printk("dev alloc error\n");
		return -ENOMEM;
	}

	pi_dma->tx_dma_chan = kmalloc(sizeof(struct dma_channel_header), GFP_KERNEL);
	if (!pi_dma->tx_dma_chan) {
		printk("tx_dma_chan alloc error\n");
		return -ENOMEM;
	}

	pi_dma->rx_dma_chan = kmalloc(sizeof(struct dma_channel_header), GFP_KERNEL);
	if (!pi_dma->rx_dma_chan) {
		printk("rx_dma_chan alloc error\n");
		return -ENOMEM;
	}

	txirq = platform_get_irq_byname(pdev, "dma9");
	if (!txirq) {
		printk("Can't retrieve our tx interrupt from platform\n");
		return 0;
	}

	rxirq = platform_get_irq_byname(pdev, "dma10");
	if (!rxirq) {
		printk("Can't retrieve our rx interrupt from platform\n");
		return 0;
	}

	/////////////////////////////////////////////////////////////////////////////////
	//TX stuff initial
	pi_dma->tx_dma_chan->chan_base = BCM2835_DMA_CHANIO(base, CHANNEL_TX);
	pi_dma->tx_dma_chan->irq_number = txirq;
	pi_dma->tx_dma_chan->irq_flags = IRQF_SHARED;

	pi_dma->tx_buffer = dma_alloc_coherent(&pdev->dev, HARDWARE_BUFFER_BYTES, &pi_dma->tx_dma_addr, GFP_KERNEL);
	if (!pi_dma->tx_buffer) {
		printk("could not allocate pi_dma->tx_buffer address\n");
		return 0;
	}
	memset(pi_dma->tx_buffer, 0, HARDWARE_BUFFER_BYTES);

	for (i = 0; i < HARDWARE_BUFFER_BYTES; i++){
		pi_dma->tx_buffer[i] = i;
	}

	pi_dma->tx_dma_cb_ping = dma_alloc_coherent(&pdev->dev, sizeof(struct dma_cb), &pi_dma->tx_cb_addr_ping, GFP_KERNEL);
	if (!pi_dma->tx_dma_cb_ping) {
		printk("could not allocate pi_dma->tx_dma_cb_ping address\n");
		return 0;
	}
	memset(pi_dma->tx_dma_cb_ping, 0, sizeof(struct dma_cb));

	pi_dma->tx_dma_cb_pong = dma_alloc_coherent(&pdev->dev, sizeof(struct dma_cb), &pi_dma->tx_cb_addr_pong, GFP_KERNEL);
	if (!pi_dma->tx_dma_cb_pong) {
		printk("could not allocate pi_dma->tx_dma_cb_pong address\n");
		return 0;
	}
	memset(pi_dma->tx_dma_cb_pong, 0, sizeof(struct dma_cb));

	pi_dma->tx_dma_cb_ping->dst = 0x7e203000 + 0x4;
	pi_dma->tx_dma_cb_ping->src = pi_dma->tx_dma_addr;
	pi_dma->tx_dma_cb_ping->info = BCM2835_DMA_D_DREQ | BCM2835_DMA_S_INC | BCM2835_DMA_NO_WIDE_BURSTS |
			 BCM2835_DMA_WAIT_RESP | BCM2835_DMA_INT_EN | BCM2835_DMA_PER_MAP(PCM_TX);
	pi_dma->tx_dma_cb_ping->length = DMA_CB_TXFR_LEN_XLENGTH(HARDWARE_BUFFER_BYTES/2);
	pi_dma->tx_dma_cb_ping->stride = 0;
	pi_dma->tx_dma_cb_ping->next = pi_dma->tx_cb_addr_pong;

	pi_dma->tx_dma_cb_pong->dst = 0x7e203000 + 0x4;
	pi_dma->tx_dma_cb_pong->src = pi_dma->tx_dma_addr + HARDWARE_BUFFER_BYTES/2;
	pi_dma->tx_dma_cb_pong->info = BCM2835_DMA_D_DREQ | BCM2835_DMA_S_INC | BCM2835_DMA_NO_WIDE_BURSTS |
			 BCM2835_DMA_WAIT_RESP | BCM2835_DMA_INT_EN | BCM2835_DMA_PER_MAP(PCM_TX);
	pi_dma->tx_dma_cb_pong->length = DMA_CB_TXFR_LEN_XLENGTH(HARDWARE_BUFFER_BYTES/2);
	pi_dma->tx_dma_cb_pong->stride = 0;//DMA_CB_STRIDE_D_STRIDE(1) | DMA_CB_STRIDE_S_STRIDE(1);
	pi_dma->tx_dma_cb_pong->next = pi_dma->tx_cb_addr_ping;

	txirq = request_irq(pi_dma->tx_dma_chan->irq_number, pi_dma_callback,
			pi_dma->tx_dma_chan->irq_flags, "PI DMA TX IRQ", pi_dma);
	if (txirq < 0) {
		printk("Can't request our interrupt\n");
		//return 0;
	}
	//TX Stuff initial done
	/////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////
	//RX stuff initial
	pi_dma->rx_dma_chan->chan_base = BCM2835_DMA_CHANIO(base, CHANNEL_RX);
	pi_dma->rx_dma_chan->irq_number = rxirq;
	pi_dma->rx_dma_chan->irq_flags = IRQF_SHARED;

	pi_dma->rx_buffer = dma_alloc_coherent(&pdev->dev, HARDWARE_BUFFER_BYTES, &pi_dma->rx_dma_addr, GFP_KERNEL);
	if (!pi_dma->rx_buffer) {
		printk("could not allocate pi_dma->rx_buffer address\n");
		return 0;
	}
	memset(pi_dma->rx_buffer, 0, HARDWARE_BUFFER_BYTES);

	for (i = 0; i < HARDWARE_BUFFER_BYTES; i++){
		pi_dma->rx_buffer[i] = i;
	}

	pi_dma->rx_dma_cb_ping = dma_alloc_coherent(&pdev->dev, sizeof(struct dma_cb), &pi_dma->rx_cb_addr_ping, GFP_KERNEL);
	if (!pi_dma->rx_dma_cb_ping) {
		printk("could not allocate pi_dma->rx_dma_cb_ping address\n");
		return 0;
	}
	memset(pi_dma->rx_dma_cb_ping, 0, sizeof(struct dma_cb));

	pi_dma->rx_dma_cb_pong = dma_alloc_coherent(&pdev->dev, sizeof(struct dma_cb), &pi_dma->rx_cb_addr_pong, GFP_KERNEL);
	if (!pi_dma->rx_dma_cb_pong) {
		printk("could not allocate pi_dma->rx_dma_cb_pong address\n");
		return 0;
	}
	memset(pi_dma->rx_dma_cb_pong, 0, sizeof(struct dma_cb));

	pi_dma->rx_dma_cb_ping->dst = pi_dma->rx_dma_addr;
	pi_dma->rx_dma_cb_ping->src = 0x7e203000 + 0x4;
	pi_dma->rx_dma_cb_ping->info = BCM2835_DMA_S_DREQ | BCM2835_DMA_D_INC | BCM2835_DMA_NO_WIDE_BURSTS |
			 BCM2835_DMA_WAIT_RESP | BCM2835_DMA_INT_EN | BCM2835_DMA_PER_MAP(PCM_RX);
	pi_dma->rx_dma_cb_ping->length = DMA_CB_TXFR_LEN_XLENGTH(HARDWARE_BUFFER_BYTES/2);
	pi_dma->rx_dma_cb_ping->stride = 0;
	pi_dma->rx_dma_cb_ping->next = pi_dma->rx_cb_addr_pong;

	pi_dma->rx_dma_cb_pong->dst = pi_dma->rx_dma_addr + HARDWARE_BUFFER_BYTES/2;
	pi_dma->rx_dma_cb_pong->src = 0x7e203000 + 0x4;
	pi_dma->rx_dma_cb_pong->info = BCM2835_DMA_S_DREQ | BCM2835_DMA_D_INC | BCM2835_DMA_NO_WIDE_BURSTS |
			 BCM2835_DMA_WAIT_RESP | BCM2835_DMA_INT_EN | BCM2835_DMA_PER_MAP(PCM_RX);
	pi_dma->rx_dma_cb_pong->length = DMA_CB_TXFR_LEN_XLENGTH(HARDWARE_BUFFER_BYTES/2);
	pi_dma->rx_dma_cb_pong->stride = 0;
	pi_dma->rx_dma_cb_pong->next = pi_dma->rx_cb_addr_ping;

	rxirq = request_irq(pi_dma->rx_dma_chan->irq_number, pi_dma_callback,
			pi_dma->rx_dma_chan->irq_flags, "PI DMA RX IRQ", pi_dma);
	if (rxirq < 0) {
		printk("Can't request our rx interrupt\n");
	}
	//RX Stuff initial done
	/////////////////////////////////////////////////////////////////////////////////


	platform_set_drvdata(pdev, pi_dma);
	gdevice = &pdev->dev;

	dma_proc_create();
	setup_gpio();
	init_gpio_cs();
	__init_spi_bus();
	setup_i2s();
	udelay(500);
	
	writel(pi_dma->tx_cb_addr_ping, pi_dma->tx_dma_chan->chan_base + BCM2835_DMA_ADDR);
	writel(BCM2835_DMA_ACTIVE, pi_dma->tx_dma_chan->chan_base + BCM2835_DMA_CS);
	
	udelay(200);

	writel(pi_dma->rx_cb_addr_ping, pi_dma->rx_dma_chan->chan_base + BCM2835_DMA_ADDR);
	writel(BCM2835_DMA_ACTIVE, pi_dma->rx_dma_chan->chan_base + BCM2835_DMA_CS);

	udelay(200);

    if (oakdsp) {
    	oakdsp_on();
    	printk("Turn on OAKDSP\n");    	
    }else {
    	oakdsp_off();
    	printk("Turn off OAKDSP\n");    	    	
    }


	return 0;
}

static void stop_dma(void __iomem *chan_base) {
	writel(0, chan_base + BCM2835_DMA_NEXTCB);

	writel(BCM2835_DMA_ABORT | BCM2835_DMA_ACTIVE,
	       chan_base + BCM2835_DMA_CS);
}

static int pi_dma_remove(struct platform_device *pdev)
{
	struct pi_dma *pi_dma;

	pi_dma = platform_get_drvdata(pdev);

	/////////////////////////////////////////////////////////////////////////////////
	//TX stuff remove
	if (pi_dma->tx_dma_addr) {
		dma_free_coherent(&pdev->dev,
			HARDWARE_BUFFER_BYTES,
			pi_dma->tx_buffer,
			pi_dma->tx_dma_addr);
	}

	if (pi_dma->tx_cb_addr_ping) {
		dma_free_coherent(&pdev->dev,
				sizeof(*pi_dma->tx_dma_cb_ping),
			pi_dma->tx_dma_cb_ping,
			pi_dma->tx_cb_addr_ping);
	}

	if (pi_dma->tx_cb_addr_pong) {
		dma_free_coherent(&pdev->dev,
				sizeof(*pi_dma->tx_dma_cb_pong),
			pi_dma->tx_dma_cb_pong,
			pi_dma->tx_cb_addr_pong);
	}
	//TX Stuff remove done
	/////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////
	//RX stuff remove
	if (pi_dma->rx_dma_addr) {
		dma_free_coherent(&pdev->dev,
			HARDWARE_BUFFER_BYTES,
			pi_dma->rx_buffer,
			pi_dma->rx_dma_addr);
	}

	if (pi_dma->rx_cb_addr_ping) {
		dma_free_coherent(&pdev->dev,
				sizeof(*pi_dma->rx_dma_cb_ping),
			pi_dma->rx_dma_cb_ping,
			pi_dma->rx_cb_addr_ping);
	}

	if (pi_dma->rx_cb_addr_pong) {
		dma_free_coherent(&pdev->dev,
				sizeof(*pi_dma->rx_dma_cb_pong),
			pi_dma->rx_dma_cb_pong,
			pi_dma->rx_cb_addr_pong);
	}
	//RX Stuff remove done
	/////////////////////////////////////////////////////////////////////////////////

	stop_i2s();
	__stop_spi_bus();

	if(i2s_registers != NULL) {
		iounmap(i2s_registers);
	}

	if(gpio != NULL) {
		iounmap(gpio);
	}

	if (base != NULL) {
		stop_dma(pi_dma->tx_dma_chan->chan_base);
		stop_dma(pi_dma->rx_dma_chan->chan_base);
		iounmap(base);
	}

	if (pi_dma != NULL) {
		free_irq(pi_dma->tx_dma_chan->irq_number, pi_dma);
		free_irq(pi_dma->rx_dma_chan->irq_number, pi_dma);
		pi_dma->tx_dma_chan->chan_base = NULL;
		pi_dma->tx_dma_cb_ping = NULL;
		pi_dma->tx_dma_cb_pong = NULL;
		pi_dma->tx_dma_chan = NULL;
		pi_dma->rx_dma_chan->chan_base = NULL;
		pi_dma->rx_dma_cb_ping = NULL;
		pi_dma->rx_dma_cb_pong = NULL;
		pi_dma->rx_dma_chan = NULL;
		pi_dma = NULL;
	}
	if (proc_frame)
		remove_proc_entry("pidmainfo", NULL);
	return 0;
}

static const struct of_device_id pi_dma_match[] = {
	{ .compatible = "brcm,pi-tdm", },
	{}
};
MODULE_DEVICE_TABLE(of, pi_dma_match);

static struct platform_driver pi_dma_driver = {
	.driver		= {
		.name		= DRV_NAME,
		.of_match_table	= pi_dma_match,
	},
	.probe		= pi_dma_probe,
	.remove		= pi_dma_remove,
};
module_platform_driver(pi_dma_driver);

EXPORT_SYMBOL(get_thunder_device);
EXPORT_SYMBOL(stop_i2s);
EXPORT_SYMBOL(start_i2s_tx);
EXPORT_SYMBOL(__spi_read);
EXPORT_SYMBOL(__spi_write);
EXPORT_SYMBOL(codec_reset);
EXPORT_SYMBOL(__readchunk);
EXPORT_SYMBOL(led_on);
EXPORT_SYMBOL(led_off);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xin Li <xin.li@switchpi.com>");
MODULE_DESCRIPTION("WC card driver");

module_param(oakdsp, int, 0600);


