#include <streambuf>
#include <vector>

class char_array_buffer : public std::streambuf
{
    public:
        char_array_buffer(const char* begin, const char* end);
        explicit char_array_buffer(const char* str);

    private:
        int_type underflow();
        int_type uflow();
        int_type pbackfail(int_type ch);
        std::streamsize showmanyc();

        // copy ctor and assignment not implemented;
        // copying not allowed
        char_array_buffer(const char_array_buffer&);
        char_array_buffer& operator= (const char_array_buffer&);

    private:
        const char* const m_begin;
        const char* const m_end;
        const char* m_current;
};

class char_vector_buffer : public std::streambuf
{
    public:
        char_vector_buffer();
        const std::vector<char>& getVector() const;

    private:
        int_type overflow(int_type ch);

        // copy ctor and assignment not implemented;
        // copying not allowed
        char_vector_buffer(const char_vector_buffer&);
        char_vector_buffer& operator= (const char_vector_buffer&);

    private:
        std::vector<char> m_buffer;
};
