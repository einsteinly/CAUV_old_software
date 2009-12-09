#include "cauv_utils.h"

MakeString::operator string() const {
    return stream.str();
}

void msleep(int milliseconds)
{
    struct timespec t = {t.tv_sec = 0, t.tv_nsec = milliseconds * 1000000};
    nanosleep(&t, 0);
}
