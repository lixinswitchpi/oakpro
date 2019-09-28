#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/moduleparam.h>

#include <dahdi/kernel.h>

static int echo_can_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
			   struct dahdi_echocanparam *p, struct dahdi_echocan_state **ec);
static void echo_can_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec);
static void echo_can_process(struct dahdi_echocan_state *ec, short *isig, const short *iref, u32 size);
static int echo_can_traintap(struct dahdi_echocan_state *ec, int pos, short val);
static const char *name = "OAKDSP";
static const char *ec_name(const struct dahdi_chan *chan) { return name; }

static const struct dahdi_echocan_factory my_factory = {
	.get_name = ec_name,
	.owner = THIS_MODULE,
	.echocan_create = echo_can_create,
};

static const struct dahdi_echocan_ops my_ops = {
	.echocan_free = echo_can_free,
	.echocan_process = echo_can_process,
	.echocan_traintap = echo_can_traintap,
};

struct ec_pvt {
	//struct oslec_state *oslec;
	struct dahdi_echocan_state dahdi;
};

#define dahdi_to_pvt(a) container_of(a, struct ec_pvt, dahdi)

static void echo_can_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec)
{
	struct ec_pvt *pvt = dahdi_to_pvt(ec);
	printk(KERN_WARNING "OAKDSP freed \n");
	//do something to free echo
	kfree(pvt);
}

static void echo_can_process(struct dahdi_echocan_state *ec, short *isig, const short *iref, u32 size)
{
	struct ec_pvt *pvt = dahdi_to_pvt(ec);
	//do something let the OAKDSP runs
	//kfree(pvt);
}

static int echo_can_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
			   struct dahdi_echocanparam *p, struct dahdi_echocan_state **ec)
{
	struct ec_pvt *pvt;

	if (ecp->param_count > 0) {
		printk(KERN_WARNING "OAKDSP does not support parameters; failing request\n");
		return -EINVAL;
	}

	pvt = kzalloc(sizeof(*pvt), GFP_KERNEL);
	if (!pvt)
		return -ENOMEM;

	pvt->dahdi.ops = &my_ops;

	*ec = &pvt->dahdi;
	//kfree(pvt);

	printk(KERN_WARNING "OAKDSP called in echo_can_create\n");
	
	return 0;

}

static int echo_can_traintap(struct dahdi_echocan_state *ec, int pos, short val)
{
	return 1;
}

static int __init mod_init(void)
{
	if (dahdi_register_echocan_factory(&my_factory)) {
		module_printk(KERN_ERR, "could not register with DAHDI core\n");

		return -EPERM;
	}

	module_printk(KERN_INFO, "Registered echo canceler '%s'\n",
		my_factory.get_name(NULL));

	return 0;
}

static void __exit mod_exit(void)
{
	dahdi_unregister_echocan_factory(&my_factory);
}

MODULE_DESCRIPTION("DAHDI OAKDSP wrapper");
MODULE_AUTHOR("Xin Li <xin.li@switchpi.com>");
MODULE_LICENSE("GPL");

module_init(mod_init);
module_exit(mod_exit);

