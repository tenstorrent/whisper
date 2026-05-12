#include <stdio.h>

void hanoi(int n, int from, int to)
{
    if (n == 1) {
        printf("%d -> %d\n", from, to);
        return;
    }
    int spare = 3-from-to;
    hanoi(n-1, from, spare);
    hanoi(1, from, to);
    hanoi(n-1, spare, to);
}

int main()
{
    hanoi(5, 0, 1);
}
