../xtensa-lx106-elf/bin/xtensa-lx106-elf-readelf -s build/obj/oc.elf | grep OBJECT | grep ': 3' | sort -k3 -n | tail -n 20
