        nop               #    0x00                            nop
main:   jal  tm           #    0x01    Main:                   jal     TestMultiply             
        jal  td           #    0x02                            jal     TestDivide
        j    trap         #    0x03                            j       Trap
tm:     addi $8, $0, 0x1  #    0x04    TestMultiply:           addi    r8, r0, 0x0001              # set r8 = 1
        addi $9, $0, 0x2  #    0x05                            addi    r9, r0, 0x0002              # set r9 = 2
        addi $10, $0, 0x9 #    0x06                            addi    r10, r0, 0x0009              # set r10 = 9
        add  $12, $0, $0  #    0x07                            add     r12, r0, r0                  # set r12 = 0
lm:     beq  $9, $10, em  #    0x08    LoopMultiply:           beq     r9, r10, EndMultiply         # while(r9!=r10) do
        mult $8, $9       #    0x09                            mult    r8, r9                      
        mflo $11          #    0x0a                            mflo    r11                          #   r11 = r8 * r9
        addi $9, $9, 0x1  #    0x0b                            addi    r9, r9, 0x0001              #   r9 += 1
        add  $8, $0, $11  #    0x0c                            add     r8, r0, r11                  #   r8 = r11
        sw   $8, 0($12)   #    0x0d                            sw      r8, r12, 0x0000              #   output r8 to memory
        addi $12, $12, 4  #    0x0e                            addi    r12, r12, 0x0004              #   increment memory address
        j    lm           #    0x0f                            j       LoopMultiply                # end while
em:     jr   $31          #    0x10    EndMultiply:            jr      r31
td:     addi $9, $0, 0x2  #    0x11    TestDivide:             addi    r9, r0, 0x0002              # set r9 =  2
        addi $10, $0, 0x5 #    0x12                            addi    r10, r0, 0x0005              # set r10 =  5
        addi $11, $0, 0xb #    0x13                            addi    r11, r0, 0x000b              # set r11 = 11
        div  $8, $9       #    0x14                            div     r8, r9                      
        mflo $13          #    0x15                            mflo    r13                          # r13 = r8/r9
        mfhi $14          #    0x16                            mfhi    r14                          # r14 = r8%r9
        sw   $13, 0($12)  #    0x17                            sw      r13, r12, 0x0000              # output r13 to memory
        sw   $14, 4($12)  #    0x18                            sw      r14, r12, 0x0004              # output r14 to memory
        addi $12, $12, 8  #    0x19                            addi    r12, r12, 0x0008              # increment memory address
        div  $8, $10      #    0x1a                            div     r8, r10
        mflo $13          #    0x1b                            mflo    r13                          # r13 = r8/r10
        mfhi $14          #    0x1c                            mfhi    r14                          # r14 = r8%r10
        sw   $13, 0($12)  #    0x1d                            sw      r13, r12, 0x0000              # output r13 to memory
        sw   $14, 4($12)  #    0x1e                            sw      r14, r12, 0x0004              # output r14 to memory
        addi $12, $12, 8  #    0x1f                            addi    r12, r12, 0x0008              # increment memory address
        div  $8, $11      #    0x20                            div     r8, r11
        mflo $13          #    0x21                            mflo    r13                          # r13 = r8/r11
        mfhi $14          #    0x22                            mfhi    r14                          # r14 = r8%r11
        sw   $13, 0($12)  #    0x23                            sw      r13, r12, 0x0000              # output r13 to memory
        sw   $14, 4($12)  #    0x24                            sw      r14, r12, 0x0004              # output r14 to memory
        jr   $31          #    0x25                            jr      r31
trap:   j    trap         #    0x26    Trap:                   j       Trap                        # program end
        nop               #    0x27                            nop  