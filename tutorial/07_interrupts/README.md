# Interrupts

This example demonstrates how you would enable timer interrupts in a test.

It first enables interrupts globally by setting `mstatus.MIE` to 1. Then it
enables machine-mode timer interrupts specifically. It sets the trap handler
address to the end-of-test code so that the test finishes when the timer
interrupt happens. And it then stores a value of 100 to the memory-mapped
register `mtimecmp`. It then enters an infinite loop waiting for the interrupt.

Once the `mtime` register reaches 100, it triggers a machine-mode timer
interrupt, which causes the hart to jump to the handler.
