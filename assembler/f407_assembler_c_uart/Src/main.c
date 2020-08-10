/**
// transmit and receive via C callable assembly routines
// using USART2 @115200baud, assuming power on reset clock defaults (HSI, 16MHz)
*/

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include <stdio.h>
#include <stdlib.h>

extern void USART2Init(void);
extern void USART2WriteChar(char c);
extern char USART2ReadChar(void);


// override these weak attribute functions  defined in syscalls.c

void __io_putchar(char c) {
	USART2WriteChar(c);
	}

char __io_getchar(void) {
	char c = USART2ReadChar();
	return c;
	}

int _read(int file, char *ptr, int len){
	*ptr = USART2ReadChar();
	USART2WriteChar(*ptr);
	if (*ptr == '\r'){
		USART2WriteChar('\n');
		*ptr = '\n';
		}
	return 1;
	}


// use gets and then strtol etc. instead of scanf
void test(void){
	char str[80];
	printf("Enter a character string : ");
	gets(str);
	printf("String entered is : ");
	puts(str);
	char* eptr;
	// for decimal integer strings e.g. "5739"
	printf("Strtol result is %ld\r\n", strtol(str, &eptr, 10));
	// for hexadecimal integer strings e.g. "0x2af"
	//printf("Strtol result is %lx\r\n", strtol(str, &eptr, 16));
	printf("\r\n");
	}

int main(void){

	USART2Init();

	printf("Hello world\r\n");

	while(1){
		test();
		}
}
