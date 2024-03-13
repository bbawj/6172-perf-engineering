	.text
	.file	"loop.c"
	.section	.rodata.cst8,"aM",@progbits,8
	.p2align	3                               # -- Begin function main
.LCPI0_0:
	.quad	0x3e112e0be826d695              # double 1.0000000000000001E-9
	.text
	.globl	main
	.p2align	4, 0x90
	.type	main,@function
main:                                   # @main
	.cfi_startproc
# %bb.0:
	pushq	%rbp
	.cfi_def_cfa_offset 16
	pushq	%r15
	.cfi_def_cfa_offset 24
	pushq	%r14
	.cfi_def_cfa_offset 32
	pushq	%r12
	.cfi_def_cfa_offset 40
	pushq	%rbx
	.cfi_def_cfa_offset 48
	subq	$8224, %rsp                     # imm = 0x2020
	.cfi_def_cfa_offset 8272
	.cfi_offset %rbx, -48
	.cfi_offset %r12, -40
	.cfi_offset %r14, -32
	.cfi_offset %r15, -24
	.cfi_offset %rbp, -16
	movl	$0, 12(%rsp)
	leaq	32(%rsp), %rdi
	xorl	%ebx, %ebx
	movl	$4096, %edx                     # imm = 0x1000
	xorl	%esi, %esi
	callq	memset@PLT
	leaq	4128(%rsp), %rdi
	movl	$4096, %edx                     # imm = 0x1000
	xorl	%esi, %esi
	callq	memset@PLT
	leaq	16(%rsp), %rsi
	movl	$1, %edi
	callq	clock_gettime@PLT
	movq	16(%rsp), %r15
	movq	24(%rsp), %r14
	pxor	%xmm0, %xmm0
	pxor	%xmm1, %xmm1
	.p2align	4, 0x90
.LBB0_1:                                # =>This Inner Loop Header: Depth=1
	paddd	32(%rsp,%rbx,4), %xmm0
	paddd	48(%rsp,%rbx,4), %xmm1
	paddd	64(%rsp,%rbx,4), %xmm0
	paddd	80(%rsp,%rbx,4), %xmm1
	paddd	96(%rsp,%rbx,4), %xmm0
	paddd	112(%rsp,%rbx,4), %xmm1
	paddd	128(%rsp,%rbx,4), %xmm0
	paddd	144(%rsp,%rbx,4), %xmm1
	addq	$32, %rbx
	cmpq	$1024, %rbx                     # imm = 0x400
	jne	.LBB0_1
# %bb.2:
	paddd	%xmm0, %xmm1
	pshufd	$238, %xmm1, %xmm0              # xmm0 = xmm1[2,3,2,3]
	paddd	%xmm1, %xmm0
	pshufd	$85, %xmm0, %xmm1               # xmm1 = xmm0[1,1,1,1]
	paddd	%xmm0, %xmm1
	movd	%xmm1, %r12d
	leaq	16(%rsp), %rsi
	movl	$1, %edi
	callq	clock_gettime@PLT
	movq	16(%rsp), %rbx
	movq	24(%rsp), %rbp
	leaq	12(%rsp), %rdi
	callq	rand_r@PLT
                                        # kill: def $eax killed $eax def $rax
	leal	1023(%rax), %ecx
	testl	%eax, %eax
	cmovnsl	%eax, %ecx
	andl	$-1024, %ecx                    # imm = 0xFC00
	subl	%ecx, %eax
	cltq
	addl	4128(%rsp,%rax,4), %r12d
	subq	%r15, %rbx
	xorps	%xmm1, %xmm1
	cvtsi2sd	%rbx, %xmm1
	subq	%r14, %rbp
	xorps	%xmm0, %xmm0
	cvtsi2sd	%rbp, %xmm0
	mulsd	.LCPI0_0(%rip), %xmm0
	addsd	%xmm1, %xmm0
	leaq	.L.str(%rip), %rdi
	leaq	.L.str.1(%rip), %rcx
	leaq	.L.str.2(%rip), %r8
	movl	$1024, %esi                     # imm = 0x400
	movl	$100000, %edx                   # imm = 0x186A0
	movb	$1, %al
	callq	printf@PLT
	movl	%r12d, %eax
	addq	$8224, %rsp                     # imm = 0x2020
	.cfi_def_cfa_offset 48
	popq	%rbx
	.cfi_def_cfa_offset 40
	popq	%r12
	.cfi_def_cfa_offset 32
	popq	%r14
	.cfi_def_cfa_offset 24
	popq	%r15
	.cfi_def_cfa_offset 16
	popq	%rbp
	.cfi_def_cfa_offset 8
	retq
.Lfunc_end0:
	.size	main, .Lfunc_end0-main
	.cfi_endproc
                                        # -- End function
	.type	.L.str,@object                  # @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str:
	.asciz	"Elapsed execution time: %f sec; N: %d, I: %d, __OP__: %s, __TYPE__: %s\n"
	.size	.L.str, 72

	.type	.L.str.1,@object                # @.str.1
.L.str.1:
	.asciz	"+"
	.size	.L.str.1, 2

	.type	.L.str.2,@object                # @.str.2
.L.str.2:
	.asciz	"uint32_t"
	.size	.L.str.2, 9

	.ident	"Debian clang version 14.0.6"
	.section	".note.GNU-stack","",@progbits
	.addrsig
