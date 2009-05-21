#include <stdarg.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/threads.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/stringify.h>
#include <linux/delay.h>
#include <linux/initrd.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/bootmem.h>
#include <linux/kexec.h>
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <asm/prom.h>
#include <asm/setup.h>
#include <asm/memory.h>
#define DEBUG
#ifdef DEBUG
#define DBG(fmt...) printk(KERN_ERR fmt)
#else
#define DBG(fmt...)
#endif

struct boot_param_header *initial_boot_params;

extern struct device_node *allnodes;	/* temporary while merging */

extern rwlock_t devtree_lock;	/* temporary while merging */

/* export that to outside world */
struct device_node *of_chosen;

static inline char *find_flat_dt_string(u32 offset)
{
	return ((char *)initial_boot_params) +
		initial_boot_params->off_dt_strings + offset;
}

static void *__init unflatten_dt_alloc(unsigned long *mem, unsigned long size,
				       unsigned long align)
{
	void *res;

	*mem = _ALIGN(*mem, align);
	res = (void *)*mem;
	*mem += size;

	return res;
}

static unsigned long __init unflatten_dt_node(unsigned long mem,
					      unsigned long *p,
					      struct device_node *dad,
					      struct device_node ***allnextpp,
					      unsigned long fpsize)
{
	struct device_node *np;
	struct property *pp, **prev_pp = NULL;
	char *pathp;
	u32 tag;
	unsigned int l, allocl;
	int has_name = 0;
	int new_format = 0;

	tag = *((u32 *)(*p));
	if (tag != OF_DT_BEGIN_NODE) {
		printk("Weird tag at start of node: %x\n", tag);
		return mem;
	}
	*p += 4;
	pathp = (char *)*p;
	l = allocl = strlen(pathp) + 1;
	*p = _ALIGN(*p + l, 4);

	/* version 0x10 has a more compact unit name here instead of the full
	 * path. we accumulate the full path size using "fpsize", we'll rebuild
	 * it later. We detect this because the first character of the name is
	 * not '/'.
	 */
	if ((*pathp) != '/') {
		new_format = 1;
		if (fpsize == 0) {
			/* root node: special case. fpsize accounts for path
			 * plus terminating zero. root node only has '/', so
			 * fpsize should be 2, but we want to avoid the first
			 * level nodes to have two '/' so we use fpsize 1 here
			 */
			fpsize = 1;
			allocl = 2;
		} else {
			/* account for '/' and path size minus terminal 0
			 * already in 'l'
			 */
			fpsize += l;
			allocl = fpsize;
		}
	}


	np = unflatten_dt_alloc(&mem, sizeof(struct device_node) + allocl,
				__alignof__(struct device_node));
	if (allnextpp) {
		memset(np, 0, sizeof(*np));
		np->full_name = ((char*)np) + sizeof(struct device_node);
		if (new_format) {
			char *p = np->full_name;
			/* rebuild full path for new format */
			if (dad && dad->parent) {
				strcpy(p, dad->full_name);
#ifdef DEBUG
				if ((strlen(p) + l + 1) != allocl) {
					DBG("%s: p: %d, l: %d, a: %d\n",
					    pathp, (int)strlen(p), l, allocl);
				}
#endif
				p += strlen(p);
			}
			*(p++) = '/';
			memcpy(p, pathp, l);
		} else
			memcpy(np->full_name, pathp, l);
		prev_pp = &np->properties;
		**allnextpp = np;
		*allnextpp = &np->allnext;
		if (dad != NULL) {
			np->parent = dad;
			/* we temporarily use the next field as `last_child'*/
			if (dad->next == 0)
				dad->child = np;
			else
				dad->next->sibling = np;
			dad->next = np;
		}
		kref_init(&np->kref);
	}
	while(1) {
		u32 sz, noff;
		char *pname;

		tag = *((u32 *)(*p));
		if (tag == OF_DT_NOP) {
			*p += 4;
			continue;
		}
		if (tag != OF_DT_PROP)
			break;
		*p += 4;
		sz = *((u32 *)(*p));
		noff = *((u32 *)((*p) + 4));
		*p += 8;
		if (initial_boot_params->version < 0x10)
			*p = _ALIGN(*p, sz >= 8 ? 8 : 4);

		pname = find_flat_dt_string(noff);
		if (pname == NULL) {
			printk("Can't find property name in list !\n");
			break;
		}
		if (strcmp(pname, "name") == 0)
			has_name = 1;
		l = strlen(pname) + 1;
		pp = unflatten_dt_alloc(&mem, sizeof(struct property),
					__alignof__(struct property));
		if (allnextpp) {
			if (strcmp(pname, "linux,phandle") == 0) {
				np->node = *((u32 *)*p);
				if (np->linux_phandle == 0)
					np->linux_phandle = np->node;
			}
			if (strcmp(pname, "ibm,phandle") == 0)
				np->linux_phandle = *((u32 *)*p);
			pp->name = pname;
			pp->length = sz;
			pp->value = (void *)*p;
			*prev_pp = pp;
			prev_pp = &pp->next;
		}
		*p = _ALIGN((*p) + sz, 4);
	}
	/* with version 0x10 we may not have the name property, recreate
	 * it here from the unit name if absent
	 */
	if (!has_name) {
		char *p = pathp, *ps = pathp, *pa = NULL;
		int sz;

		while (*p) {
			if ((*p) == '@')
				pa = p;
			if ((*p) == '/')
				ps = p + 1;
			p++;
		}
		if (pa < ps)
			pa = p;
		sz = (pa - ps) + 1;
		pp = unflatten_dt_alloc(&mem, sizeof(struct property) + sz,
					__alignof__(struct property));
		if (allnextpp) {
			pp->name = "name";
			pp->length = sz;
			pp->value = pp + 1;
			*prev_pp = pp;
			prev_pp = &pp->next;
			memcpy(pp->value, ps, sz - 1);
			((char *)pp->value)[sz - 1] = 0;
			DBG("fixed up name for %s -> %s\n", pathp,
				(char *)pp->value);
		}
	}
	if (allnextpp) {
		*prev_pp = NULL;
		np->name = of_get_property(np, "name", NULL);
		np->type = of_get_property(np, "device_type", NULL);

		if (!np->name)
			np->name = "<NULL>";
		if (!np->type)
			np->type = "<NULL>";
	}
	while (tag == OF_DT_BEGIN_NODE) {
		mem = unflatten_dt_node(mem, p, np, allnextpp, fpsize);
		tag = *((u32 *)(*p));
	}
	if (tag != OF_DT_END_NODE) {
		printk("Weird tag at end of node: %x\n", tag);
		return mem;
	}
	*p += 4;
	return mem;
}

/**
 * unflattens the device-tree passed by the firmware, creating the
 * tree of struct device_node. It also fills the "name" and "type"
 * pointers of the nodes so the normal device-tree walking functions
 * can be used (this used to be done by finish_device_tree)
 */
void __init unflatten_device_tree(void)
{
	unsigned long start, mem, size;
	struct device_node **allnextp = &allnodes;

	DBG(" -> unflatten_device_tree()\n");
	if (!initial_boot_params)
		return;
	/* First pass, scan for size */
	start = ((unsigned long)initial_boot_params) +
		initial_boot_params->off_dt_struct;
	size = unflatten_dt_node(0, &start, NULL, NULL, 0);
	size = (size | 3) + 1;

	DBG("  size is %lx, allocating...\n", size);

	/* Allocate memory for the expanded device tree */
	mem = (unsigned long) __alloc_bootmem(size + 4, __alignof__(struct device_node), 0);
	//mem = (unsigned long) __va(mem);

	((u32 *)mem)[size / 4] = 0xdeadbeef;

	DBG("  unflattening %lx...\n", mem);

	/* Second pass, do actual unflattening */
	start = ((unsigned long)initial_boot_params) +
		initial_boot_params->off_dt_struct;
	unflatten_dt_node(mem, &start, NULL, &allnextp, 0);
	if (*((u32 *)start) != OF_DT_END)
		printk(KERN_WARNING "Weird tag at end of tree: %08x\n", *((u32 *)start));
	if (((u32 *)mem)[size / 4] != 0xdeadbeef)
		printk(KERN_WARNING "End of tree marker overwritten: %08x\n",
		       ((u32 *)mem)[size / 4] );
	*allnextp = NULL;

	/* Get pointer to OF "/chosen" node for use everywhere */
	of_chosen = of_find_node_by_path("/chosen");
	if (of_chosen == NULL)
		of_chosen = of_find_node_by_path("/chosen@0");

	DBG(" <- unflatten_device_tree()\n");
}

/**
 *	of_find_node_by_phandle - Find a node given a phandle
 *	@handle:	phandle of the node to find
 *
 *	Returns a node pointer with refcount incremented, use
 *	of_node_put() on it when done.
 */
struct device_node *of_find_node_by_phandle(phandle handle)
{
	struct device_node *np;

	read_lock(&devtree_lock);
	for (np = allnodes; np != 0; np = np->allnext)
		if (np->linux_phandle == handle)
			break;
	of_node_get(np);
	read_unlock(&devtree_lock);
	return np;
}
EXPORT_SYMBOL(of_find_node_by_phandle);

struct device_node *of_node_get(struct device_node *node)
{
	if (node)
		kref_get(&node->kref);
	return node;
}
EXPORT_SYMBOL(of_node_get);
static inline struct device_node * kref_to_device_node(struct kref *kref)
{
	return container_of(kref, struct device_node, kref);
}

/**
 *	of_node_release - release a dynamically allocated node
 *	@kref:  kref element of the node to be released
 *
 *	In of_node_put() this function is passed to kref_put()
 *	as the destructor.
 */
static void of_node_release(struct kref *kref)
{
	struct device_node *node = kref_to_device_node(kref);
	struct property *prop = node->properties;

	/* We should never be releasing nodes that haven't been detached. */
	if (!of_node_check_flag(node, OF_DETACHED)) {
		printk("WARNING: Bad of_node_put() on %s\n", node->full_name);
		dump_stack();
		kref_init(&node->kref);
		return;
	}

	if (!of_node_check_flag(node, OF_DYNAMIC))
		return;

	while (prop) {
		struct property *next = prop->next;
		kfree(prop->name);
		kfree(prop->value);
		kfree(prop);
		prop = next;

		if (!prop) {
			prop = node->deadprops;
			node->deadprops = NULL;
		}
	}
	kfree(node->full_name);
	kfree(node->data);
	kfree(node);
}

/**
 *	of_node_put - Decrement refcount of a node
 *	@node:	Node to dec refcount, NULL is supported to
 *		simplify writing of callers
 *
 */
void of_node_put(struct device_node *node)
{
	if (node)
		kref_put(&node->kref, of_node_release);
}
EXPORT_SYMBOL(of_node_put);

int have_of;
u32 phys_flat_dev_tree_address __initdata = 0;
u32 phys_flat_dev_tree_size __initdata = 0;

/* process flat device tree for hardware configuration */
static int __init parse_tag_flat_dev_tree_address(const struct tag *tag)
{
    phys_flat_dev_tree_address =
                            tag->u.flat_dev_tree_address.flat_dev_tree_address;
    phys_flat_dev_tree_size = tag->u.flat_dev_tree_address.flat_dev_tree_size;

    have_of = 1;
    if (phys_flat_dev_tree_size)
    {
	initial_boot_params = phys_to_virt(phys_flat_dev_tree_address);	
    }
    printk(KERN_INFO
		    "%s: flat_dev_tree_address=0x%08x, flat_dev_tree_size == 0x%08X\n",
		    __FUNCTION__, phys_flat_dev_tree_address, phys_flat_dev_tree_size);

    return 0;
}

__tagtable(ATAG_FLAT_DEV_TREE_ADDRESS, parse_tag_flat_dev_tree_address);

