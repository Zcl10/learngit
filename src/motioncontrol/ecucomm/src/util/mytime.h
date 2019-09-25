#pragma once
#include <iostream>
#include <time.h>
#ifdef WIN32
#   include <windows.h>
#else
#   include <sys/time.h>
#endif
#ifdef WIN32
    int
gettimeofday(struct timeval *tp, void *tzp)
{
    time_t clock;
    struct tm tm;
    SYSTEMTIME wtm;
    GetLocalTime(&wtm);
    tm.tm_year     = wtm.wYear - 1900;
    tm.tm_mon     = wtm.wMonth - 1;
    tm.tm_mday     = wtm.wDay;
    tm.tm_hour     = wtm.wHour;
    tm.tm_min     = wtm.wMinute;
    tm.tm_sec     = wtm.wSecond;
    tm. tm_isdst    = -1;
    clock = mktime(&tm);
    tp->tv_sec = clock;
    tp->tv_usec = wtm.wMilliseconds * 1000;
    return (0);
}
#endif

using namespace std;

class MyTime
{
    public:
        MyTime ()
        {
        } inline void start ();
        inline void stop ();
        inline double gettime_s ();
        inline void show_s ();
        inline static double now ();

        struct timeval t1, t2;
        double time;
};

    void
MyTime::start ()
{
    gettimeofday (&t1, NULL);

}

    void
MyTime::stop ()
{
    gettimeofday (&t2, NULL);
}

    double
MyTime::gettime_s ()
{
    stop ();
    time = t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) / 1000000.0;
    return time;
}

    void
MyTime::show_s ()
{
    cout << gettime_s () << "s" << endl;
}

    double
MyTime::now ()
{
    struct timeval t;
    gettimeofday (&t, NULL);
    double time = t.tv_sec + (t.tv_usec) / 1000000.0;
    return time;
}
