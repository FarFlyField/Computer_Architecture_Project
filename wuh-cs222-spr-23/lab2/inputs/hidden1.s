.text
mov x0, 1
mov x1, 2
mov x2, 40
mov x3, 20
lsl X4, X2, 1
lsr X5, X3, 1

mov X10, 0x1000
lsl X10, X10, 16
mov X20, 0X4321
sturb W20, [X10, 0x0]
adds X20, X20, X0
ldur X13, [X10, 0x0]
mul X14, X13, X1

mov X1, 0x1000
lsl X1, X1, 16
mov X10, 0xa234
stur X10, [X1, 0x0]
sturb W10, [X1, 0x6]
ldur X13, [X1, 0x0]
ldurb W14, [X1, 0x6]

cbz X12, target
add X1, X2, X3
target:

cbnz X12, target2
add X1, X2, X4
target2:

HLT 0

