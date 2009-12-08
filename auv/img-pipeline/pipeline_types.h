
#include <string>
#include <exception>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>

class image;
class Node;
typedef int Priority;

// until such time as one exists...
class Scheduler{
    public:
        void addToQueue(Node* n, Priority p){}
};


class parameter_error: public std::runtime_error{
    public:
        parameter_error(const char* str)
            : std::runtime_error(str){
        }
};


typedef boost::variant<float, int, bool, std::string> param_value_t;

typedef std::string node_id;
typedef std::string param_id;
typedef std::string input_id;
typedef std::string output_id;
