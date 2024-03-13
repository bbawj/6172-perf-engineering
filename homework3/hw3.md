Write-up 1: Look at the assembly code above. The compiler has translated the code to set
the start index at −2^16 and adds to it for each memory access. Why doesn’t it set the start
index to 0 and use small positive offsets?

This is so that the compiler can optimize away an extra CMP instruction per iteration of the loop. The `addq $4, %rax` instruction will automatically set the Z flag once rax becomes 0.

Write-up 3: Provide a theory for why the compiler is generating dramatically different
assembly

The ternary operator simplifies the loop such that branching code is removed.

Write-up 4: Inspect the assembly and determine why the assembly does not include
instructions with vector registers. Do you think it would be faster if it did vectorize?
Explain.

Calls to memcpy instead of vectorizing. Syscalls take around 100 cycles or so.
Vectorizing will take more cycles than that:
Each loop, we can handle 128 bits of data
2^16 / 2^7 = 2^9 > 100.

Write-up 6: What speedup does the vectorized code achieve over the unvectorized code?
What additional speedup does using -mavx2 give? You may wish to run this experiment
several times and take median elapsed times; you can report answers to the nearest 100%
(e.g., 2×, 3×, etc). What can you infer about the bit width of the default vector registers on
the awsrun machines? What about the bit width of the AVX2 vector registers? Hint: aside
from speedup and the vectorization report, the most relevant information is that the data
type for each array is uint32_t.

Elapsed execution time: 0.147595 sec; N: 1024, I: 100000, __OP__: +, __TYPE__: uint32_t
Elapsed execution time: 0.035450 sec; N: 1024, I: 100000, __OP__: +, __TYPE__: uint32_t
Elapsed execution time: 0.017748 sec; N: 1024, I: 100000, __OP__: +, __TYPE__: uint32_t

Vectorized 4x speedup
AVX2 2x speedup

Infer that the vector registers can do 4x more operations = 4 x 32 = 128 bit width
Infer that avx2 registers can do 2x more = 2 * 128 = 256 bit width

Write-up 7: Compare the contents of loop.s when the VECTORIZE flag is set/not set. Which
instruction (copy its text here) is responsible for the vector add operation? Which
instruction (copy its text here) is responsible for the vector add operation when you
additionally pass AVX2=1? You can find an x86 instruction manual on LMOD. Look for
MMX and SSE2 instructions, which are vector operations. To make the assembly code more
readable it may be a good idea to remove debug symbols from release builds by moving the
-g and -gdwarf-3 CFLAGS in your Makefile. It might also be a good idea to turn off loop
unrolling with the -fno-unroll-loops flag while you study the assembly code.

paddd
vpaddd


Elapsed execution time: 0.142369 sec; N: 1024, I: 100000, __OP__: +, __TYPE__: uint32_t
Elapsed execution time: 0.036842 sec; N: 1024, I: 100000, __OP__: +, __TYPE__: uint32_t
Elapsed execution time: 0.024375 sec; N: 1024, I: 100000, __OP__: +, __TYPE__: uint32_t
