/* 
 * search: looks up also for unexported symbols in the kernel
 * exports function:
 *
 * Copyright (C) 2010 Skrilax_CZ
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include "kallsyms.h"
#include "symsearch.h"

MODULE_AUTHOR("Skrilax_CZ");
MODULE_DESCRIPTION("Symbol search module");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

SYMSEARCH_INIT_FUNCTION(lookup_symbol_address);
EXPORT_SYMBOL(lookup_symbol_address);

static void find_kallsyms_lookup_name(void)
{
	struct file *kallsyms;
	struct seq_file *seq;
	struct kallsym_iter *iter;
	loff_t pos = 0;

	kallsyms = filp_open("/proc/kallsyms", O_RDONLY, 0);
	if (!kallsyms)
	{
		printk(KERN_ERR "symsearch: failed to open /proc/kallsyms\n");
		return;
	}

	seq = (struct seq_file *)kallsyms->private_data;
	if (!seq)
	{
		printk(KERN_ERR "symsearch: failed to fetch sequential file for /proc/kallsyms\n");
		goto err_close;
	}

	for (iter = seq->op->start(seq, &pos);
		iter != NULL;
		iter = seq->op->next(seq, iter, &pos))
	{
		if (!strcmp(iter->name, "kallsyms_lookup_name"))
		{
			printk(KERN_INFO "symsearch: found kallsyms_lookup_name on 0x%lx.\n", iter->value);
			lookup_symbol_address = (lookup_symbol_address_fp)iter->value;
			break;
		}
	}

err_close:
	filp_close(kallsyms, NULL);
}

static int __init symsearch_init(void)
{
	find_kallsyms_lookup_name();
	if(!lookup_symbol_address) 
	{
		printk(KERN_ERR "symsearch: could not find kallsyms_lookup_name.\n");
		return -EBUSY;
	}
	
	return 0;
}

module_init(symsearch_init);
