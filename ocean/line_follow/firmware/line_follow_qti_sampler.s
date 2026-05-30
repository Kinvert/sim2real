#
# PASM QTI sampler cog.
#

        .section .coguser0, "ax"
        .cog_ram

        mov     mailbox, par

        mov     control_addr, mailbox
        mov     seq_addr, mailbox
        add     seq_addr, #4
        mov     raw0_addr, mailbox
        add     raw0_addr, #8
        mov     raw1_addr, mailbox
        add     raw1_addr, #12
        mov     raw2_addr, mailbox
        add     raw2_addr, #16
        mov     sample_addr, mailbox
        add     sample_addr, #20
        mov     period_addr, mailbox
        add     period_addr, #24

        mov     t1, mailbox
        add     t1, #28
        rdlong  pin0, t1
        mov     t1, mailbox
        add     t1, #32
        rdlong  pin1, t1
        mov     t1, mailbox
        add     t1, #36
        rdlong  pin2, t1
        mov     t1, mailbox
        add     t1, #40
        rdlong  charge_ticks, t1
        mov     t1, mailbox
        add     t1, #44
        rdlong  timeout_ticks, t1
        mov     t1, mailbox
        add     t1, #48
        rdlong  sample_period_ticks, t1

        mov     mask0, #1
        shl     mask0, pin0
        mov     mask1, #1
        shl     mask1, pin1
        mov     mask2, #1
        shl     mask2, pin2
        mov     qti_mask, mask0
        or      qti_mask, mask1
        or      qti_mask, mask2

        mov     seq, #0
        mov     prev_start, #0

.loop
        rdlong  control, control_addr wz
        if_z    jmp     #.done

        mov     sample_start, cnt
        mov     raw0, timeout_ticks
        mov     raw1, timeout_ticks
        mov     raw2, timeout_ticks
        mov     done, #0

        or      outa, qti_mask
        or      dira, qti_mask
        mov     target, sample_start
        add     target, charge_ticks
        waitcnt target, zero
        andn    dira, qti_mask

        mov     decay_start, cnt
.poll
        mov     now, cnt
        mov     elapsed, now
        sub     elapsed, decay_start
        cmp     elapsed, timeout_ticks wc
        if_nc   jmp     #.finish_sample

        mov     pins_in, ina

        test    done, #1 wz
        if_nz   jmp     #.check1
        test    pins_in, mask0 wz
        if_nz   jmp     #.check1
        mov     raw0, elapsed
        or      done, #1

.check1
        test    done, #2 wz
        if_nz   jmp     #.check2
        test    pins_in, mask1 wz
        if_nz   jmp     #.check2
        mov     raw1, elapsed
        or      done, #2

.check2
        test    done, #4 wz
        if_nz   jmp     #.check_done
        test    pins_in, mask2 wz
        if_nz   jmp     #.check_done
        mov     raw2, elapsed
        or      done, #4

.check_done
        cmp     done, #7 wz
        if_nz   jmp     #.poll

.finish_sample
        cmp     sample_period_ticks, #0 wz
        if_z    jmp     #.publish
        mov     target, sample_start
        add     target, sample_period_ticks
.period_wait
        mov     now, cnt
        mov     elapsed, now
        sub     elapsed, sample_start
        cmp     elapsed, sample_period_ticks wc
        if_c    jmp     #.period_wait

.publish
        mov     period_ticks, #0
        cmp     prev_start, #0 wz
        if_z    jmp     #.publish_values
        mov     period_ticks, sample_start
        sub     period_ticks, prev_start

.publish_values
        mov     prev_start, sample_start
        add     seq, #2
        mov     t1, seq
        or      t1, #1
        wrlong  t1, seq_addr

        wrlong  raw0, raw0_addr
        wrlong  raw1, raw1_addr
        wrlong  raw2, raw2_addr
        mov     sample_ticks, cnt
        sub     sample_ticks, sample_start
        wrlong  sample_ticks, sample_addr
        wrlong  period_ticks, period_addr
        wrlong  seq, seq_addr
        jmp     #.loop

.done
        andn    dira, qti_mask
        mov     control, neg_one
        wrlong  control, control_addr
.halt
        jmp     #.halt

zero                    long    0
neg_one                 long    -1

mailbox                 res     1
control_addr            res     1
seq_addr                res     1
raw0_addr               res     1
raw1_addr               res     1
raw2_addr               res     1
sample_addr             res     1
period_addr             res     1
pin0                    res     1
pin1                    res     1
pin2                    res     1
charge_ticks            res     1
timeout_ticks           res     1
sample_period_ticks     res     1
mask0                   res     1
mask1                   res     1
mask2                   res     1
qti_mask                res     1
seq                     res     1
prev_start              res     1
control                 res     1
sample_start            res     1
decay_start             res     1
now                     res     1
elapsed                 res     1
raw0                    res     1
raw1                    res     1
raw2                    res     1
done                    res     1
target                  res     1
pins_in                 res     1
period_ticks            res     1
sample_ticks            res     1
t1                      res     1
