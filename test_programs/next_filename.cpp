// $Id: next_filename.c,v 1.1 2005/12/12 17:30:59 garthz Exp $
// next_filename.c : small test of file name choosing code
//
// Copyright (c) 2005 Garth Zeglin. Provided under the terms of the
// GNU General Public License as included in the top level directory.
//

#include <stdio.h>
#include <utility/utility.h>
#include <errno.h>

int main(int argc, char **argv)
{
  char *name;
  printf("finding the next data filename.\n");
  
  name = new_data_file_name();
  if (name == NULL) 
    printf("Unable to choose new name: %s\n", strerror(errno));
  else {
    printf("The next data file will be %s.\n", name);
    free(name);
  }
}
						     
    
