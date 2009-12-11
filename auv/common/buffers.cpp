#include "buffers.h"

char_array_buffer::char_array_buffer(const char *begin, const char *end) :
    m_begin(begin),
    m_end(end),
    m_current(m_begin)
{
}

char_array_buffer::int_type char_array_buffer::underflow()
{
    if (m_current == m_end)
        return traits_type::eof();

    return *m_current;
}
char_array_buffer::int_type char_array_buffer::uflow()
{
    if (m_current == m_end)
        return traits_type::eof();

    return *m_current++;
}
char_array_buffer::int_type char_array_buffer::pbackfail(int_type ch)
{
    if (m_current == m_begin || (ch != traits_type::eof() && ch != m_current[-1]))
        return traits_type::eof();

    return *--m_current;
}
std::streamsize char_array_buffer::showmanyc()
{
    return m_end - m_current;
}



char_vector_buffer::char_vector_buffer()
{
    setp(0,0);
}
const vector<char>& char_vector_buffer::getVector() const
{
    return m_buffer;
}
char_vector_buffer::int_type char_vector_buffer::overflow(int_type ch)
{
    m_buffer.push_back(ch);
    return 1;
}
