#include <streambuf>
#include <vector>

class char_vector_readbuffer : public std::streambuf
{
    public:
        char_vector_readbuffer(const std::vector<char>& bytes);

    private:
        int_type underflow();
        int_type uflow();
        int_type pbackfail(int_type ch);
        std::streamsize showmanyc();

        // copy ctor and assignment not implemented;
        // copying not allowed
        char_vector_readbuffer(const char_vector_readbuffer&);
        char_vector_readbuffer& operator= (const char_vector_readbuffer&);

    private:
        const std::vector<char>& m_bytes;
        size_t m_pos;
};

class char_vector_writebuffer : public std::streambuf
{
    public:
        char_vector_writebuffer();
        const std::vector<char> getVector() const;

    private:
        int_type overflow(int_type ch);

        // copy ctor and assignment not implemented;
        // copying not allowed
        char_vector_writebuffer(const char_vector_writebuffer&);
        char_vector_writebuffer& operator= (const char_vector_writebuffer&);

    private:
        std::vector<char> m_buffer;
};
