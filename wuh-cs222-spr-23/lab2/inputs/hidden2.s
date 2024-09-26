.text

mov X0, 1
mov X1, 2
mov X2, 3
mov X3, 4
cmp X3, X3
beq tar1
sub X0, X0, X1
tar1:
cmp X3, X1
beq tar2
sub X0, X0, X2
tar2:

cmp X3, X1
bgt tar3
add X0, X1, X2
tar3:

cmp X3, X1
blt tar4
adds X1, X0, X2
mul X10, X1, X3
tar4:

cmp X1, X3
ble tar5
add X5, X0, X2
mul X20, X1, X3
tar5:
add X5, X5, X2

HLT 0

