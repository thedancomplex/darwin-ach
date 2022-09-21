#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>

#define CLOCKID CLOCK_REALTIME
#define SIG SIGRTMIN


       static void
       print_siginfo(siginfo_t *si)
       {
           timer_t *tidp;
           int ori;

           tidp = si->si_value.sival_ptr;

           printf("    sival_ptr = %p; ", si->si_value.sival_ptr);
           printf("    *sival_ptr = %#jx\n", (uintmax_t) *tidp);

           ori = timer_getoverrun(*tidp);
           if (ori == -1)
               printf("Error: timer_getoverrun\n");
           else
               printf("    overrun count = %d\n", ori);
       }

       static void
       handler(int sig, siginfo_t *si, void *uc)
       {
           /* Note: calling printf() from a signal handler is not safe
              (and should not be done in production programs), since
              printf() is not async-signal-safe; see signal-safety(7).
              Nevertheless, we use printf() here as a simple way of
              showing that the handler was called. */

           printf("Caught signal %d\n", sig);
           print_siginfo(si);
           signal(sig, SIG_IGN);
       }

       void rtTimerSetup()
       {
           timer_t timerid;
           struct sigevent sev;
           struct itimerspec its;
           long long freq_nanosecs;
           sigset_t mask;
           struct sigaction sa;

           /* Establish handler for timer signal. */

           printf("Establishing handler for signal %d\n", SIG);
           sa.sa_flags = SA_SIGINFO;
           sa.sa_sigaction = handler;
           sigemptyset(&sa.sa_mask);
           if (sigaction(SIG, &sa, NULL) == -1)
               printf("Error: sigaction\n");

           /* Block timer signal temporarily. */

           printf("Blocking signal %d\n", SIG);
           sigemptyset(&mask);
           sigaddset(&mask, SIG);
           if (sigprocmask(SIG_SETMASK, &mask, NULL) == -1)
               printf("Error: sigprocmask\n");

           /* Create the timer. */

           sev.sigev_notify = SIGEV_SIGNAL;
           sev.sigev_signo = SIG;
           sev.sigev_value.sival_ptr = &timerid;
           if (timer_create(CLOCKID, &sev, &timerid) == -1)
               printf("Error: timer_create\n");

           printf("timer ID is %#jx\n", (uintmax_t) timerid);

           /* Start the timer. */

           freq_nanosecs = 1000000; //atoll(argv[2]);
           its.it_value.tv_sec = freq_nanosecs / 1000000000;
           its.it_value.tv_nsec = freq_nanosecs % 1000000000;
           its.it_interval.tv_sec = its.it_value.tv_sec;
           its.it_interval.tv_nsec = its.it_value.tv_nsec;

           if (timer_settime(timerid, 0, &its, NULL) == -1)
                printf("Error: timer_settime\n");

           /* Sleep for a while; meanwhile, the timer may expire
              multiple times. */

//           printf("Sleeping for %d seconds\n", atoi(argv[1]));
//           sleep(atoi(argv[1]));

           /* Unlock the timer signal, so that timer notification
              can be delivered. */
/*
           printf("Unblocking signal %d\n", SIG);
           if (sigprocmask(SIG_UNBLOCK, &mask, NULL) == -1)
               printf("Error: sigprocmask\n");
*/
       }

