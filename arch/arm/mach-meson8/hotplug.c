#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <plat/io.h>
#include <mach/io.h>
#include <mach/cpu.h>
#include <asm/smp_scu.h>
#include <asm/hardware/gic.h>
#include <asm/smp_plat.h>
#include <asm/smp_scu.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>
#include <asm/cp15.h>

int meson_cpu_kill(unsigned int cpu)
{
	//meson_set_cpu_ctrl_reg(cpu, 0);
	//meson_set_cpu_power_ctrl(cpu, 0);

	return 1;
}



void meson_cpu_die(unsigned int cpu)
{
	meson_set_cpu_ctrl_reg(cpu, 0);
	flush_cache_all();
	dsb();
	dmb();	
	for (;;) {
	    
	    __asm__ __volatile__ ("wfi" : : : "memory");
	
		if (smp_processor_id() == cpu) {	
			if(((aml_read_reg32(MESON_CPU_CONTROL_REG)&0xf) & ((1 << cpu) | 1)) == ((1<<cpu) | 1))
			{
				break;
			}
		}	
	}

}

int meson_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}

