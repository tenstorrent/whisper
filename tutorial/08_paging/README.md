# Paging

This example sets up a very minimal paging scenario.

The data section contains some hard-coded page table entries which map the
`.text` and `.data` sections into the virtual address space. The hart then sets
the `satp` CSR to enable paging (which applies at supervisor and user modes)
and executes an `mret` to switch to supervisor mode.

For simplicity, in this example, the pages all happen to be identity mapped
(i.e. the virtual addresses match the physical addresses they map to), but this
doesn't have to be the case.
