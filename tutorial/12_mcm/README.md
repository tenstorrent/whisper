# Memory Consistency Model Checking

This example shows a simple example of a PPO rule violation.

Rule 2 states:

> *a* and *b* are loads, *x* is a byte read by both *a* and *b*, there is no
> store to *x* between *a* and *b* in program order, and *a* and *b* return
> values for *x* written by different memory operations.

In this example, hart 0 executes two loads from the same address out of order
while a store from hart 1 writes to the address between the two loads, thus
violating rule 2. 
