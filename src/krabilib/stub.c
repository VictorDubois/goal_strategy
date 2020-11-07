#include <errno.h>
#include <unistd.h>

#undef errno
extern int errno;

#include <sys/stat.h>


void _exit(int status)
{
    //Endless loop
    for(;;);
}

int _close(int file)
{
    return -1;
}

char* __env[1] = {0};
char** environ = __env;

int _execve(char* name, char** argv, char** env)
{
    errno = ENOMEM;
    return -1;
}

int _fork(void)
{
    errno = EAGAIN;
    return -1;
}

int _fstat(int file, struct stat* st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _getpid(void)
{
    return 1;
}

int _isatty(int file)
{
    return 1;
}

int _kill(int pid, int sig)
{
    errno = EINVAL;
    return -1;
}

int _link(char* old, char* new)
{
    errno = EMLINK;
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _open(const char* name, int flags, int mode)
{
    return -1;
}

int _read(int file, char* ptr, int len)
{
    return 0;
}

caddr_t _sbrk(int incr)
{
    extern char _end;
    static char* heap_end;
    char* prev_heap_end;
    char* stack_ptr = (char*)0xffffffff;

    if(heap_end == 0)
        heap_end = &_end;
    prev_heap_end = heap_end;
    if( heap_end + incr > stack_ptr)
    {
        write(1, "Heap and stack collision\n", 25);
      //  abort();
    }

    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

typedef struct tms tms;

int _times(tms* buf)
{
    return -1;
}

int _unlink(char* name)
{
    errno = ENOENT;
    return -1;
}

int _wait(int* status)
{
    errno = ECHILD;
    return -1;
}

int _write(int file, char* ptr, int len)
{
    return len;
}
