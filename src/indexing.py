import numpy as np
from fractions import Fraction

arr = np.array([[0.1, 0.2, 0.3],
                [1.1, 1.2, 1.3],
                [2.1, 2.2, 2.3],
                [3.1, 3.2, 3.3]
                ])
print(f'original array = \n{arr}')

print(f'Fractions:')

# Compute the fraction of each element and distinguish the numerator and denominator
for i in range(arr.shape[0]):
    for j in range(arr.shape[1]):
        frac = Fraction(arr[i,j]).limit_denominator()
        num = frac.numerator
        den = frac.denominator
