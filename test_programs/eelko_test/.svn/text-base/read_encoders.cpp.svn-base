
#include <iostream>
#include <stdio.h>
#include <sys/io.h>
#include <asm/io.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <signal.h>

#include <hardware_drivers/Mesanet_4I36.h>
using namespace std;

#define BASE_ADDRESS_MESA1 0x220
#define BASE_ADDRESS_MESA2 0x230


int main(){
	
	cout << "Read encoder values from MESA 4I36 and print on screen/n" << endl;
	
	Mesanet_4I36 board1;
	Mesanet_4I36 board2;
	
	  // Get access to ALL ports.
	  if (iopl(3)) {
	    printf("enable_port_access: iopl failed: %s", strerror(errno));
	  }
	
	  Mesanet_4I36_init_with_address( &board1, BASE_ADDRESS_MESA1 );
	  Mesanet_4I36_init_with_address( &board2, BASE_ADDRESS_MESA2 );
	  
	   Mesanet_4I36_close(  df
	
	
	return 0;
}