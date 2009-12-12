#include "buffers.h"

char_vector_readbuffer::char_vector_readbuffer(const std::vector<char>& bytes) : m_bytes(bytes)
{
}

char_vector_readbuffer::int_type char_vector_readbuffer::underflow()
{
    if (m_pos == m_bytes.size())
        return traits_type::eof();

    return m_bytes[m_pos];
}
char_vector_readbuffer::int_type char_vector_readbuffer::uflow()
{
    if (m_pos == m_bytes.size())
        return traits_type::eof();

    return m_bytes[m_pos++];
}
char_vector_readbuffer::int_type char_vector_readbuffer::pbackfail(int_type ch)
{
    if (m_pos == 0 || (ch != traits_type::eof() && ch != m_bytes[m_pos - 1]))
        return traits_type::eof();

    return m_bytes[m_pos--];
}
std::streamsize char_vector_readbuffer::showmanyc()
{
    return m_bytes.size() - m_pos;
}



char_vector_writebuffer::char_vector_writebuffer()
{
    setp(0,0);
}
const std::vector<char> char_vector_writebuffer::getVector() const
{
    return m_buffer;
}

char_vector_writebuffer::int_type char_vector_writebuffer::overflow(int_type ch)
{
    m_buffer.push_back(ch);
    return 1;
}
