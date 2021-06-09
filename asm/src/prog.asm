        movi r1, 1
        inci r1, 10
        deci r1, 1
        movi r2, 128
        addi r1, r2, 4
        subi r1, r2, 4
        nop
        movi r1, 5
        shli r1, r1, 5
        asri r1, r1, 3
        lsri r1, r1, 2
        rori r1, r1, 4
        halt
