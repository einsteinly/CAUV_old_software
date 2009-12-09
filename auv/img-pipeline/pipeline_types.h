
#include <string>
#include <exception>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>

class Node;
enum Priority {
    low,
    high,
    realtime
};

// until such time as one exists...
class Scheduler{
    public:
        void addToQueue(Node* n, Priority p){}
};


class id_error: public std::runtime_error{
    public:
        id_error(const char* str)
            : std::runtime_error(str){
        }
};


typedef boost::variant<float, int, bool, std::string> param_value_t;

typedef std::string node_id;
typedef std::string param_id;
typedef std::string input_id;
typedef std::string output_id;
