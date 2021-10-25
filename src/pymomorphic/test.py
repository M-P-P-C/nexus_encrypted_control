#!/usr/bin/env python

import pymorph

#pymorph.main()

pymorph.variables_define()

var = pymorph.variables_import()

pymorph.key_generate(var.q, var.L, var.N,1)

print  var.q

sk = pymorph.key_import(1)

c = pymorph.enc_1(var.p, var.L, var.q, var.r, var.N, sk, [80])

print pymorph.dec_hom(var.p, var.L, sk, c[0])

c2 = pymorph.enc_2(var.p, var.L, var.q, var.r, var.N, sk, [2])

c3 = pymorph.hom_mul(var.q, c, c2)

#c3 = [int(i*2) for i in c3] #values can also be multiplied by constants

print pymorph.dec_hom(var.p, var.L, sk, c3)





