
def incFloat(f):
    # increment a floating point number by the smallest possible amount
    # not sure this works with negative numbers
    if f == 0.0:
        return sys.float_info.min
    m, e = math.frexp(f)
    return math.ldexp(m + sys.float_info.epsilon / 2, e)

