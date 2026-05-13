# seed: 2310631079
hart=0 time=0 perf_model_fetch 1 0x80000000 # auipc    x10, 0x1
hart=0 time=1 perf_model_fetch 2 0x80000004 # ld       x10, 0x0(x10) [0x80001000]
hart=0 time=2 perf_model_decode 1
hart=0 time=3 perf_model_decode 2
hart=0 time=4 perf_model_execute 1
hart=0 time=5 perf_model_execute 2
hart=0 time=6 perf_model_retire 1
hart=0 time=7 perf_model_fetch 3 0x80000008 # auipc    x11, 0x1
hart=0 time=8 perf_model_retire 2
hart=0 time=9 perf_model_decode 3
hart=0 time=10 perf_model_fetch 4 0x8000000c # ld       x11, 0x0(x11) [0x80001008]
hart=0 time=11 perf_model_decode 4
hart=0 time=12 perf_model_execute 3
hart=0 time=13 perf_model_retire 3
hart=0 time=14 perf_model_execute 4
hart=0 time=15 perf_model_fetch 5 0x80000010 # jal      x1, . + 0x8
hart=0 time=16 perf_model_fetch 6 0x80000018 # beq      x10, x11, . + 0x10
hart=0 time=17 perf_model_decode 5
hart=0 time=18 perf_model_fetch 7 0x8000001c # blt      x11, x10, . + 0x8
hart=0 time=19 perf_model_decode 6
hart=0 time=20 perf_model_execute 6
hart=0 time=21 perf_model_retire 4
hart=0 time=22 perf_model_fetch 8 0x80000024 # c.sub    x10, x11
hart=0 time=23 perf_model_execute 5
hart=0 time=24 perf_model_decode 7
hart=0 time=25 perf_model_retire 5
hart=0 time=26 perf_model_execute 7
hart=0 time=27 perf_model_retire 6
hart=0 time=28 perf_model_retire 7
hart=0 time=29 perf_model_decode 8
hart=0 time=30 perf_model_fetch 9 0x80000026 # c.j      . - 0xe
hart=0 time=31 perf_model_execute 8
hart=0 time=32 perf_model_retire 8
hart=0 time=33 perf_model_fetch 10 0x80000018 # beq      x10, x11, . + 0x10
hart=0 time=34 perf_model_fetch 11 0x8000001c # blt      x11, x10, . + 0x8
hart=0 time=35 perf_model_decode 9
hart=0 time=36 perf_model_execute 9
hart=0 time=37 perf_model_decode 10
hart=0 time=38 perf_model_retire 9
hart=0 time=39 perf_model_fetch 12 0x80000020 # c.sub    x11, x10
hart=0 time=40 perf_model_decode 11
hart=0 time=41 perf_model_execute 10
hart=0 time=42 perf_model_execute 11
hart=0 time=43 perf_model_decode 12
hart=0 time=44 perf_model_retire 10
hart=0 time=45 perf_model_fetch 13 0x80000022 # c.j      . - 0xa
hart=0 time=46 perf_model_execute 12
hart=0 time=47 perf_model_retire 11
hart=0 time=48 perf_model_retire 12
hart=0 time=49 perf_model_decode 13
hart=0 time=50 perf_model_execute 13
hart=0 time=51 perf_model_retire 13
hart=0 time=52 perf_model_fetch 14 0x80000018 # beq      x10, x11, . + 0x10
hart=0 time=53 perf_model_decode 14
hart=0 time=54 perf_model_execute 14
hart=0 time=55 perf_model_fetch 15 0x8000001c # blt      x11, x10, . + 0x8
hart=0 time=56 perf_model_decode 15
hart=0 time=57 perf_model_retire 14
hart=0 time=58 perf_model_fetch 16 0x80000020 # c.sub    x11, x10
hart=0 time=59 perf_model_execute 15
hart=0 time=60 perf_model_fetch 17 0x80000022 # c.j      . - 0xa
hart=0 time=61 perf_model_retire 15
hart=0 time=62 perf_model_decode 16
hart=0 time=63 perf_model_execute 16
hart=0 time=64 perf_model_decode 17
hart=0 time=65 perf_model_fetch 18 0x80000018 # beq      x10, x11, . + 0x10
hart=0 time=66 perf_model_retire 16
hart=0 time=67 perf_model_fetch 19 0x8000001c # blt      x11, x10, . + 0x8
hart=0 time=68 perf_model_decode 18
hart=0 time=69 perf_model_execute 18
hart=0 time=70 perf_model_decode 19
hart=0 time=71 perf_model_execute 17
hart=0 time=72 perf_model_retire 17
hart=0 time=73 perf_model_fetch 20 0x80000024 # c.sub    x10, x11
hart=0 time=74 perf_model_decode 20
hart=0 time=75 perf_model_fetch 21 0x80000026 # c.j      . - 0xe
hart=0 time=76 perf_model_execute 19
hart=0 time=77 perf_model_retire 18
hart=0 time=78 perf_model_retire 19
hart=0 time=79 perf_model_execute 20
hart=0 time=80 perf_model_decode 21
hart=0 time=81 perf_model_fetch 22 0x80000018 # beq      x10, x11, . + 0x10
hart=0 time=82 perf_model_decode 22
hart=0 time=83 perf_model_execute 22
hart=0 time=84 perf_model_retire 20
hart=0 time=85 perf_model_execute 21
hart=0 time=86 perf_model_retire 21
hart=0 time=87 perf_model_fetch 23 0x80000028 # c.jr     x1
hart=0 time=88 perf_model_decode 23
hart=0 time=89 perf_model_execute 23
hart=0 time=90 perf_model_fetch 24 0x80000014 # c.mv     x10, x10
hart=0 time=91 perf_model_fetch 25 0x80000016 # c.j      . + 0x14
hart=0 time=92 perf_model_decode 24
hart=0 time=93 perf_model_execute 24
hart=0 time=94 perf_model_retire 22
hart=0 time=95 perf_model_retire 23
hart=0 time=96 perf_model_retire 24
hart=0 time=97 perf_model_decode 25
hart=0 time=98 perf_model_fetch 26 0x8000002a # c.li     x5, 0x1
hart=0 time=99 perf_model_execute 25
hart=0 time=100 perf_model_retire 25
hart=0 time=101 perf_model_decode 26
hart=0 time=102 perf_model_fetch 27 0x8000002c # auipc    x6, 0xf0000
hart=0 time=103 perf_model_execute 26
hart=0 time=104 perf_model_retire 26
hart=0 time=105 perf_model_decode 27
hart=0 time=106 perf_model_execute 27
hart=0 time=107 perf_model_fetch 28 0x80000030 # sd       x5, -0x2c(x6) [0x70000000]
hart=0 time=108 perf_model_decode 28
hart=0 time=109 perf_model_execute 28
hart=0 time=110 perf_model_retire 27
hart=0 time=111 perf_model_retire 28
hart=0 time=112 perf_model_drain_store 28
