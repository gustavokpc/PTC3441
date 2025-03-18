urela = tinvert(arelu);
crelu = tinvert(urelc);

crela = tmult(urela,crelu);

crelb = tmult(arelb*crela)
