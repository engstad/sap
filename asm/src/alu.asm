start:  movi r1, 5
        out  r1
        inci r1, 10
        out  r1
        deci r1, 1
        out  r1
        movi r2, 128
        out  r1
        addi r1, r2, 4
        out  r1
        subi r1, r2, 4
        out  r1
        nop
        out  r1
        movi r1, 5
        out  r1
        shli r1, r1, 5
        out  r1
        asri r1, r1, 3
        out  r1
        lsri r1, r1, 2
        out  r1
        rori r1, r1, 4
        out  r1
        movi r1, 85
        movi r2, 204
        and  r0, r1, r2
        out  r0
        xor  r0, r1, r2
        out  r0
        or   r0, r1, r2
        out  r0
        bic  r0, r1, r2
        out  r0
        mvn  r0, r1, r2
        out  r0
        mul  r0, r1, r2
        out  r0
        rsb  r0, r1, r2
        bra  start
        out  r0
