Test Cases
=====================================================================================================
| Folder	| Test Path
| --------- | -------------------------------------------------------------------------------------
| 0			| IDLE->INIT_HOVER->STABILIZING->CHECK_STATE->STABILIZING->...->HOVER->IDLE (Internal)
| 1			| HOVER->IDLE (External)
| 2			| IDLE->INIT_HOVER->STABILIZING->CHECK_STATE->STABILIZING->...->IDLE
| 3			| CHECK_STATE->IDLE
| 4			| IDLE->INIT_HOVER->STABILIZING->IDLE
| 5			| INIT_HOVER->IDLE
