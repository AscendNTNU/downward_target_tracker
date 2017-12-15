# Notes on increasing performance on the Jetson TX2

The Jetson TX2 uses the [Nvidia Tegra X2](https://en.wikipedia.org/wiki/Tegra#Tegra_X2), whose CPU cluster consists of one Nvidia Denver2 ARMv8 (64-bit) dual-core, and one ARMv8 ARM Cortex-A57 quad-core (64-bit).

Making a program use the Denver 2 CPUs
--------------------------------------
According to someone on the nvidia forums, the Denver2 CPU is better than the ARMv8 CPU for single-thread performance. Since the grid detector and the target tracker are both single-threaded (each runs on its own thread when the downward target tracker application runs), it seems like a good idea assign each to its own Denver2 core.

This can apparently (see [this](https://devtalk.nvidia.com/default/topic/796850/only-2-of-4-cpus-on-jetson-/) and [that](https://devtalk.nvidia.com/default/topic/1000345/jetson-tx2/two-cores-disabled-/)) be done with the [taskset](http://www.tutorialspoint.com/unix_commands/taskset.htm) application, or programmatically using ```[pthread_setaffinity](http://man7.org/linux/man-pages/man3/pthread_setaffinity_np.3.html)``` or ```[sched_setaffinity](http://man7.org/linux/man-pages/man2/sched_setaffinity.2.html)```.

The Jetson also has multiple power mode configurations that can be set with [nvpmodel](http://www.jetsonhacks.com/2017/03/25/nvpmodel-nvidia-jetson-tx2-development-kit/). For example:
```
$ sudo nvpmodel -m 4
```
will set the mode to the so-called "Max-P Denver", where both Denver cores are online at 2.0 GHz, all ARM A57 cores are offline, and the GPU is at 1.12 GHz. Alternatively, you could go all out and use Max-N (see the link above for a description of the various modes).

To verify what mode you got and see which cores are active, you can run the command:
```
$ sudo tegrastats
```

We can also disable CPU scaling (so the cores run at a constant frequency) and force the two Denver cores to always be active (see [Maximizing CPU Performance](http://elinux.org/Jetson/Performance#Maximizing_CPU_performance)) by running these commands:
```
$ echo 1 > /sys/devices/system/cpu/cpu1/online
$ echo 1 > /sys/devices/system/cpu/cpu2/online
```

And [maximize clock frequencies](https://devtalk.nvidia.com/default/topic/1000345/jetson-tx2/two-cores-disabled-/):
```
$ sudo ~/jetson_clocks.sh
```

Finally, we can launch a program and assign it to the Denver cores:
```
$ taskset -c 1,2 program
```

If the program has two threads I think the scheduler is smart enough to allocate them to their own cores (but I'm not sure how to guarantee this).

SIMD instructions on Denver?
----------------------------
The grid detector uses SIMD (Single Instruction Multiple Data) to speed up the sobel filter (edge extraction). I originally wrote it to work on intel processors with SSE, and it lets me process 16 pixels in parallel (instead of going one by one). It's not quite a 16x speedup, because there's a serial step at the end of each batch of 16 pixels.

That was in 2016, and I had not envisioned that we would be strapping four Nvidia Jetson TX2's on our drone any time soon. That's a problem, because the Jetson has ARM processors, which do not support SSE instructions.

The [Cortex-A57](https://developer.arm.com/products/processors/cortex-a/cortex-a57) has [ARM NEON](https://developer.arm.com/technologies/neon) as its SIMD instruction set. The wiki page explains how to use auto vectorization and [ARM's library for computer vision](http://projectne10.github.io/Ne10/), and a reference for SIMD instructions.

The [Denver2](https://en.wikipedia.org/wiki/Project_Denver) supposedly implements the [ARMv8-A 64/32-bit instruction sets](https://www.arm.com/products/processors/armv8-architecture.php), so I would assume it supports the same NEON SIMD instructions.

If you want to write an ARM NEON version of the sobel filter, you could take a look at the above wiki pages, and I would assume that the program would work on both CPU types (Denver and the A57).

References
----------
* https://devtalk.nvidia.com/default/board/188/
* http://www.jetsonhacks.com/2017/03/25/nvpmodel-nvidia-jetson-tx2-development-kit/
* https://devtalk.nvidia.com/default/topic/1000345/jetson-tx2/two-cores-disabled-/
* http://elinux.org/Jetson/Performance#Maximizing_CPU_performance
* http://elinux.org/Jetson_TX2#Guides_and_Tutorials
* https://devtalk.nvidia.com/default/topic/1014505/denver2-cores/
