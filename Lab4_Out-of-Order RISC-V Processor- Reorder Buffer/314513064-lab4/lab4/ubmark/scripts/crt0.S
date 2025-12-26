.section .text
.global _start
_start:

  # Bootstrap code
  lui     x2, 0x1          # sp = 0x1000
  call    main