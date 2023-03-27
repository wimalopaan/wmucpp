#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <signal.h>

/*
 * ioctl          set or clear the RTS/DTR lines (once per execution)
 *
 * Usage:       ioctl <device> <1 or 0 (RTS)> <1 or 0 (DTR)>
 *              For example, rts /dev/ttyS1 1 1 to set RTS and DTR line on ttyS1
 *
 * Author:      Adrian Pike, but really just a minor modification of:
 *					 Ben Dugan, which in turn is a modification of:
 *              Harvey J. Stein <hjstein@math.huji.ac.il>
 *              UPS-Howto, which in turn is:
 *              (but really just a minor modification of Miquel van
 *              Smoorenburg's <miquels@drinkel.nl.mugnet.org> powerd.c
 *
 * Version:     1.0 2009
 *
 */

/* Main program. */
int main(int argc, char **argv)
{
  int fd;

  int rtsEnable;
  int dtrEnable;
  int flags;

  if (argc < 3) {
    fprintf(stderr, "Usage: ioctl <device> <1 or 0 (RTS high or low)> <1 or 0 (DTR high or low)>\n");
    exit(1);
  }

  /* Open monitor device. */
  if ((fd = open(argv[1], O_RDWR | O_NDELAY)) < 0) {
      perror("open");
    exit(1);}

  /* Get the bits to set from the command line. */
  sscanf(argv[2], "%d", &rtsEnable);
  sscanf(argv[3], "%d", &dtrEnable);
  
  /* Get the 'BEFORE' line bits */
  ioctl(fd, TIOCMGET, &flags);
  fprintf(stderr, "Flags are %x.\n", flags);


  /* Set or clear RTS according to the command line request */

  if(dtrEnable!=0) {
    flags |= TIOCM_DTR;
  } else {
	flags &= ~TIOCM_DTR;
	}

  if(rtsEnable!=0) {
    flags |= TIOCM_RTS;
  } else {
	flags &= ~TIOCM_RTS;
	}



  ioctl(fd, TIOCMSET, &flags);
  fprintf(stderr, "Setting %x.\n", flags);

  sleep(1);

  /* Get the 'AFTER' line bits */
  ioctl(fd, TIOCMGET, &flags);
  fprintf(stderr, "Flags are %x.\n", flags);

  
  close(fd);
}
