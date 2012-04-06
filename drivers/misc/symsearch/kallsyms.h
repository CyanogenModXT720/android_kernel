/****************************************************/
/* The following must agree with: kernel/kallsyms.h */
/*                                kernel/kallsyms.c */
/****************************************************/

#define KSYM_NAME_LEN 128

struct kallsym_iter
{
	loff_t pos;
	unsigned long value;
	unsigned int nameoff; /* If iterating in core kernel symbols */
	char type;
	char name[KSYM_NAME_LEN];
	char module_name[MODULE_NAME_LEN];
	int exported;
};
