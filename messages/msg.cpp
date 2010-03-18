#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/algorithm/string_regex.hpp>
#include <boost/format.hpp>
#include <boost/assign.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>   // includes all needed Boost.Filesystem declarations
#include <FlexLexer.h>

#include "msg.h"	//for the abstract syntax tree definition
#include <vector>	//for vector
#include <map>
#ifndef foreach
#   include <boost/foreach.hpp>
#   define foreach BOOST_FOREACH
#endif

using namespace std;
using namespace boost;
using namespace boost::assign;
using namespace boost::filesystem;

std::vector<Group*> groups;
std::vector<Struct*> structs;
std::vector<Enum*> enums;
std::set<string> valid_types = boost::assign::list_of
    ("bool")
    ("byte")
    ("int8")
    ("int16")
    ("int32")
    ("uint8")
    ("uint16")
    ("uint32")
    ("string")
    ("float")
    ("double")
;
std::vector<string> unknown_types;

std::map<string, string> cpp_conversion = boost::assign::map_list_of
    ("bool", "bool")
    ("byte", "uint8_t")
    ("int8", "int8_t")
    ("int16", "int16_t")
    ("int32", "int32_t")
    ("uint8", "uint8_t")
    ("uint16", "uint16_t")
    ("uint32", "uint32_t")
    ("string", "std::string")
    ("float", "float")
    ("double", "double")
;
std::map<string, string> java_conversion = boost::assign::map_list_of
    ("bool", "boolean")
    ("byte", "byte")
    ("int8", "byte")
    ("int16", "short")
    ("int32", "int")
    ("uint8", "int")
    ("uint16", "int")
    ("uint32", "int")
    ("string", "String")
    ("float", "float")
    ("double", "double")
;
std::map<string, string> java_boxed_types = boost::assign::map_list_of
    ("boolean", "Boolean")
    ("byte", "Byte")
    ("short", "Short")
    ("int", "Integer")
    ("float", "Float")
    ("double", "Double")
;

yyFlexLexer* lexer;


string convertType(string type, map<string,string> m)
{
    map<string, string>::iterator it;
    it = m.find(type);
    if (it == m.end())
        return type;
    else
        return it->second;
}
string asCPPType(Type* type)
{
    switch (type->getType())
    {
        case TYPE_BASE:
            return convertType(static_cast<BaseType*>(type)->getName(), cpp_conversion);
        case TYPE_LIST:
        {
            ListType* l = static_cast<ListType*>(type);
            stringstream ss;
            ss << "std::vector< " << asCPPType(l->getValType()) << " >";
            return ss.str();
        }
        case TYPE_MAP:
        {
            MapType* m = static_cast<MapType*>(type);
            stringstream ss;
            ss << "std::map< " << asCPPType(m->getKeyType()) << "," << asCPPType(m->getValType()) << " >";
            return ss.str();
        }
        default:
            return "ERROR";
    }
}
string asJavaType(Type* type, bool box = false)
{
    switch (type->getType())
    {
        case TYPE_BASE:
        {
            string t = convertType(static_cast<BaseType*>(type)->getName(), java_conversion);
            if (box)
            {
                t = convertType(t, java_boxed_types);
            }
            return t;
        }
        case TYPE_LIST:
        {
            ListType* l = static_cast<ListType*>(type);
            stringstream ss;
            ss << "Vector< " << asJavaType(l->getValType(), true) << " >";
            return ss.str();
        }
        case TYPE_MAP:
        {
            MapType* m = static_cast<MapType*>(type);
            stringstream ss;
            ss << "HashMap< " << asJavaType(m->getKeyType(), true) << "," << asJavaType(m->getValType(), true) << " >";
            return ss.str();
        }
        default:
            return "ERROR";
    }
}

int createCPPFile(string);
int createJavaFile(string);

namespace po = boost::program_options;
int main(int argc, char** argv)
{
    fstream f;
    
    po::options_description desc("USAGE:  msg-generator [-o FILENAME] [-l LANG] [INPUT]");
    desc.add_options()
        ("help", "shows this help")
        ("output,o", po::value<string>(), "output filename")
        ("input,i", po::value<string>(), "input message specification file")
        ("lang,l", po::value<string>(), "output language [java or c++]")
    ;
    po::positional_options_description p;
    p.add("input", -1);
    
    po::variables_map vm;
    try
    {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        po::notify(vm);
    }
    catch (po::unknown_option& e)
    {
        cerr << e.what() << endl;
        cerr << desc << endl;
        return 0;
    }
    
    if (vm.count("help")) {
        cout << desc << endl;
        return 0;
    }
 
    string inputpath = "-", outputpath = "messages", lang = "c++"; 
    if (vm.count("input"))
        inputpath = vm["input"].as<string>();
    if (vm.count("output"))
        outputpath = vm["output"].as<string>();
    if (vm.count("lang"))
        lang = vm["lang"].as<string>();
    
    if (lang != "c++" && lang != "java")
    {
        cerr << "unknown language: " << lang << endl;
        cerr << desc << endl;
        return 0;
    }

    if (inputpath != "-")
    {
        f.open(inputpath.c_str(), fstream::in);
        if (f.fail())
        {
	        cerr << "No such file: " << inputpath << endl;
	        return 1;
        }
        else
        {
	        lexer = new yyFlexLexer(&f, &cout);
        }
    }
    else
        lexer = new yyFlexLexer();


    int ret = yyparse();

    if (ret != 0)
        return ret;

    if (lang == string("c++"))
        ret = createCPPFile(outputpath);
    else if (lang == string("java"))
        ret = createJavaFile(outputpath);

    if (f.is_open())
        f.close();

    return ret;
}

int createCPPFile(string outputpath)
{
    stringstream msg_cpp, msg_hh;

    path outputp(outputpath);
    string outputfile = outputp.filename();

    stringstream msg_macro_ss;
    msg_macro_ss << "__" << outputfile << "_H__";
    string msg_macro = msg_macro_ss.str();
    to_upper(msg_macro);

    msg_hh << "/***  This is a generated file, do not edit ***/" << endl;
    msg_hh << format("#ifndef __%1%_H__") % to_upper_copy(outputfile) << endl;
    msg_hh << format("#define __%1%_H__") % to_upper_copy(outputfile) << endl;
    msg_hh << endl;
    msg_hh << "#include <string>" << endl;
    msg_hh << "#include <sstream>" << endl;
    msg_hh << "#include <stdexcept>" << endl;
    msg_hh << "#include <vector>" << endl;
    msg_hh << "#include <list>" << endl;
    msg_hh << "#include <map>" << endl;
    msg_hh << "#include <boost/cstdint.hpp>" << endl;
    msg_hh << "#include <boost/shared_ptr.hpp>" << endl;
    msg_hh << "#ifndef foreach" << endl;
    msg_hh << "#    include <boost/foreach.hpp>" << endl;
    msg_hh << "#    define foreach BOOST_FOREACH" << endl;
    msg_hh << "#endif" << endl;
    msg_hh << endl;
    msg_hh << "#include <common/vector_streamops.h>" << endl;
    msg_hh << "#include <common/image.h>" << endl;
    msg_hh << endl;
    msg_hh << "// message data type definitions" << endl;
    msg_hh << "typedef std::string byte_vec_t;" << endl;
    msg_hh << "typedef std::ostringstream byte_ostream_t;" << endl;
    msg_hh << "typedef std::istringstream byte_istream_t;" << endl;
    msg_hh << endl;
    
    msg_cpp << "/***  This is a generated file, do not edit ***/" << endl;
    msg_cpp << format("#include \"%1%.h\"") % outputfile << endl;
    msg_cpp << "#include <boost/archive/binary_oarchive.hpp>" << endl;
    msg_cpp << "#include <boost/archive/binary_iarchive.hpp>" << endl;
    msg_cpp << "#include <boost/serialization/vector.hpp>" << endl;
    msg_cpp << "#include <boost/serialization/map.hpp>" << endl;
    msg_cpp << "#include <boost/make_shared.hpp>" << endl;
    msg_cpp << endl;
    
    foreach(string s, unknown_types)
    {
        msg_hh << format("class %1%;") % s << endl;
    }
    msg_hh << endl;
    
    foreach(Struct* s, structs)
    {
        msg_hh << "struct " << s->getName() << endl;
        msg_hh << "{" << endl;
        foreach(Declaration* d, s->getDeclarations())
        {
            msg_hh << "    " << asCPPType(d->getType()) << " " << d->getName() << ";" << endl;
        }
        msg_hh << "};" << endl;
        msg_hh << "template<typename char_T, typename traits>" << endl;
        msg_hh << "std::basic_ostream<char_T, traits>& operator<<(" << endl;
        msg_hh << "    std::basic_ostream<char_T, traits>& os, " << s->getName() << " const& a)" << endl;
        msg_hh << "{" << endl;
        msg_hh << "    os << \"" << s->getName() << " {\";" << endl;
        foreach(Declaration* d, s->getDeclarations())
        {
            msg_hh << "    os << \" " << d->getName() << " = \" << " << "a." << d->getName() << ";" << endl;
        }
        msg_hh << "    os << '}';" << endl;
        msg_hh << "    return os;" << endl;
        msg_hh << "}" << endl;
        msg_hh << endl;
    }
    foreach(Enum* e, enums)
    {
        msg_hh << "enum " << e->getName() << endl;
        msg_hh << "{" << endl;
        int i = 0;
        foreach(EnumVal* v, e->getVals())
        {
            if (i++ > 0)
                msg_hh << "," << endl;
            msg_hh << "    " << v->getName() << " = " << v->getVal();
        }
        msg_hh << endl;
        msg_hh << "};" << endl;
        msg_hh << "template<typename char_T, typename traits>" << endl;
        msg_hh << "std::basic_ostream<char_T, traits>& operator<<(" << endl;
        msg_hh << "    std::basic_ostream<char_T, traits>& os, " << e->getName() << " const& a)" << endl;
        msg_hh << "{" << endl;
        msg_hh << "    switch(a)" << endl;
        msg_hh << "    {" << endl;
        foreach(EnumVal* v, e->getVals()) 
        {
            msg_hh << "        case "<< v->getName() << ": os << \"" << v->getName() << "\"; break;" << endl;
        }
        msg_hh << "    }" << endl;
        msg_hh << "    return os;" << endl;
        msg_hh << "}" << endl;
        msg_hh << endl;
    }
    
    msg_hh << "class Message" << endl;
    msg_hh << "{" << endl;
    msg_hh << "    public:" << endl;
    msg_hh << "        virtual ~Message();" << endl;
    msg_hh << endl;
    msg_hh << "        std::string group() const;" << endl;
    msg_hh << "        uint32_t id() const;" << endl;
    msg_hh << endl;
    msg_hh << "        virtual const byte_vec_t toBytes() const = 0;" << endl;
    msg_hh << endl;
    msg_hh << "    protected:" << endl;
    msg_hh << "        uint32_t m_id;" << endl;
    msg_hh << "        std::string m_group;" << endl;
    msg_hh << endl;
    msg_hh << "        Message(uint32_t id, std::string group);" << endl;
    msg_hh << "};" << endl; 
    msg_hh << "template<typename char_T, typename traits>" << endl;
    msg_hh << "std::basic_ostream<char_T, traits>& operator<<(" << endl;
    msg_hh << "    std::basic_ostream<char_T, traits>& os, Message const& a)" << endl; 
    msg_hh << "{" << endl;
    msg_hh << "    os << \"Message {\";" << endl;
    msg_hh << "    os << \" id = \" << a.id();" << endl;
    msg_hh << "    os << \" group = \" << a.group();" << endl;
    msg_hh << "    os << '}';" << endl;
    msg_hh << "    return os;" << endl;
    msg_hh << "}" << endl;
    msg_hh << endl;
    msg_hh << endl;

    msg_cpp << "Message::Message(uint32_t id, std::string group) : m_id(id), m_group(group)" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << endl;
    msg_cpp << "Message::~Message()" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << endl;
    msg_cpp << "std::string Message::group() const" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    return m_group;" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << endl;
    msg_cpp << "uint32_t Message::id() const" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    return m_id;" << endl;
    msg_cpp << "}" << endl;

    foreach(Group* g, groups)
    {
        std::string gname = g->getName();

        foreach(Message* m, g->getMessages())
        {
            std::string name = m->getName();
            uint32_t id = m->getId();
            
            std::string className = str(format("%1%Message") % m->getName());

            std::stringstream msg_hh_msg_funcs, msg_hh_msg_fields, msg_cpp_msg_funcs, msg_cpp_msg_serial, msg_cpp_msg_deserial;
            std::stringstream msg_cstrctr_p, msg_cpp_msg_cstrctr_i;
            
            foreach(Declaration* d, m->getDeclarations())
            {
                std::string dname = d->getName();
                Type* dtype = d->getType();

                std::string dtypestr = asCPPType(dtype);
                
                msg_hh_msg_fields << format("        %1% m_%2%;") % dtypestr % dname << endl;
            
                msg_hh_msg_funcs << format("        const %1%& %2%() const;") % dtypestr % dname << endl;
                msg_cpp_msg_funcs << format("const %1%& %3%::%2%() const") % dtypestr % dname % className << endl;
                msg_cpp_msg_funcs << "{" << endl;
                msg_cpp_msg_funcs << "    return m_" << dname << ";" << endl;
                msg_cpp_msg_funcs << "}" << endl;
                
                msg_hh_msg_funcs << format("        void %2%(%1% val);") % dtypestr % dname << endl;
                msg_cpp_msg_funcs << format("void %3%::%2%(%1% val)") % dtypestr % dname % className << endl;
                msg_cpp_msg_funcs << "{" << endl;
                msg_cpp_msg_funcs << "    m_" << dname << " = val;" << endl;
                msg_cpp_msg_funcs << "}" << endl;

                msg_hh_msg_funcs << endl;
                msg_cpp_msg_funcs << endl;
                
                msg_cpp_msg_serial << "    ar & m_" << dname << ";" << endl;
                msg_cpp_msg_deserial << "    ar & ret->m_" << dname << ";" << endl;
                
                msg_cstrctr_p << asCPPType(dtype) << " " << dname << ", ";
                msg_cpp_msg_cstrctr_i << "    m_" << dname << "(" << dname << ")," << endl;
            }

            string msg_cstrctr_ps = msg_cstrctr_p.str();
            msg_cstrctr_ps = msg_cstrctr_ps.substr(0, msg_cstrctr_ps.length() - 2);
            string msg_cpp_msg_cstrctr_is = msg_cpp_msg_cstrctr_i.str();
            msg_cpp_msg_cstrctr_is = msg_cpp_msg_cstrctr_is.substr(0, msg_cpp_msg_cstrctr_is.length() - 2);

            msg_hh << "class " << className << " : public Message" << endl;
            msg_hh << "{" << endl;
            msg_hh << "    public:" << endl;
            msg_hh << "        " << className << "();" << endl;
            if (m->getDeclarations().size() > 0)
            {
                msg_hh << "        " << className << "(" << msg_cstrctr_ps << ");" << endl;
            }
            msg_hh << endl;
            msg_hh << msg_hh_msg_funcs.str();
            msg_hh << endl;
            msg_hh << "        static boost::shared_ptr<" << className << "> fromBytes(const byte_vec_t& bytes);" << endl;
            msg_hh << "        virtual const byte_vec_t toBytes() const;" << endl;
            msg_hh << endl;
            msg_hh << "    protected:" << endl;
            msg_hh << msg_hh_msg_fields.str();
            msg_hh << "};" << endl;
            msg_hh << "template<typename char_T, typename traits>" << endl;
            msg_hh << "std::basic_ostream<char_T, traits>& operator<<(" << endl;
            msg_hh << "    std::basic_ostream<char_T, traits>& os, " << className << " const& a)" << endl; 
            msg_hh << "{" << endl;
            msg_hh << "    os << \"" << className << " {\";" << endl;
            foreach(Declaration* d, m->getDeclarations())
            {
                msg_hh << "    os << \" " << d->getName() << " = \" << a." << d->getName() << "();" << endl;
            }
            //msg_hh << "    os << \"parent=\" << Message(a);" << endl;
            msg_hh << "    os << '}';" << endl;
            msg_hh << "    return os;" << endl;
            msg_hh << "}" << endl;
            msg_hh << endl;

            msg_cpp << format("%1%::%1%() : Message(%2%, \"%3%\")") % className % id % gname << endl;
            msg_cpp << "{" << endl;
            msg_cpp << "}" << endl;
            

            if (m->getDeclarations().size() > 0)
            {
                msg_cpp << format("%1%::%1%(%2%) : ") % className % msg_cstrctr_ps << endl;
                msg_cpp << "    " << format("Message(%1%, \"%2%\"),") % id % gname << endl;
                msg_cpp << msg_cpp_msg_cstrctr_is << endl;
                msg_cpp << "{" << endl;
                msg_cpp << "}" << endl;
            }

            msg_cpp << msg_cpp_msg_funcs.str() << endl;
            msg_cpp << endl;
            msg_cpp << format("boost::shared_ptr<%1%> %1%::fromBytes(const byte_vec_t& bytes)") % className << endl;
            msg_cpp << "{" << endl;
            msg_cpp << "    " << format("boost::shared_ptr<%1%> ret = boost::make_shared<%1%>();") % className << endl;
            msg_cpp << "    byte_istream_t iss(bytes);" << endl;
            msg_cpp << "    boost::archive::binary_iarchive ar(iss, boost::archive::no_header);" << endl;
            msg_cpp << "    uint32_t buf_id;" << endl;
            msg_cpp << "    ar & buf_id;" << endl;
            msg_cpp << "    if (buf_id != ret->m_id)" << endl;
            msg_cpp << "    {" << endl;
            msg_cpp << "        throw std::invalid_argument(\"Attempted to create " << className << " with invalid id\");" << endl;
            msg_cpp << "    }" << endl;
            msg_cpp << msg_cpp_msg_deserial.str();
            msg_cpp << endl;
            msg_cpp << "    return ret;" << endl;
            msg_cpp << "}" << endl;
            msg_cpp << endl;
            msg_cpp << "const byte_vec_t " << className << "::toBytes() const" << endl;
            msg_cpp << "{" << endl;
            msg_cpp << "    byte_ostream_t oss;" << endl;
            msg_cpp << "    boost::archive::binary_oarchive ar(oss, boost::archive::no_header);" << endl;
            msg_cpp << "    ar & m_id;" << endl;
            msg_cpp << msg_cpp_msg_serial.str();
            msg_cpp << "    return oss.str();" << endl;
            msg_cpp << "}" << endl;
            msg_cpp << endl;
        }
    }
   
    msg_hh << endl;
    msg_cpp << endl;

    msg_hh << "namespace boost {" << endl;
    msg_hh << "namespace serialization {" << endl;
    msg_hh << endl;
    foreach(Struct* s, structs)
    {
        msg_hh << "template<class Archive>" << endl;
        msg_hh << "void serialize(Archive & ar, " << s->getName() << "& val, const unsigned int version)" << endl;
        msg_hh << "{" << endl;
        foreach(Declaration* d, s->getDeclarations())
        {
            msg_hh << "    ar & val." << d->getName() << ";" << endl;
        }
        msg_hh << "}" << endl;
        msg_hh << endl;
    }
    msg_hh << endl;
    foreach(Enum* e, enums)
    {
        msg_hh << "template<class Archive>" << endl;
        msg_hh << "void serialize(Archive & ar, " << e->getName() << " val, const unsigned int version)" << endl;
        msg_hh << "{" << endl;
        msg_hh << "    ar & (int32_t)val;" << endl;
        msg_hh << "}" << endl;
        msg_hh << endl;
    }
    msg_hh << "} // namespace serialization" << endl;
    msg_hh << "} // namespace boost" << endl;

    msg_hh << endl;

    msg_hh << "class MessageObserver" << endl;
    msg_hh << "{" << endl;
    msg_hh << "    public:" << endl;
    msg_hh << "        virtual ~MessageObserver();" << endl;
    
    msg_cpp << "MessageObserver::MessageObserver()" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << "MessageObserver::~MessageObserver()" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "}" << endl;
    foreach(Group* g, groups)
    {
        foreach(Message* m, g->getMessages())
        {
            std::string className = str(format("%1%Message") % m->getName());
            msg_hh << "        virtual void on" << className << "(boost::shared_ptr<" << className << "> m);" << endl;
            msg_cpp << "void MessageObserver::on" << className << "(boost::shared_ptr<" << className << "> m) {}" << endl;
        }
    } 
    msg_hh << endl;
    msg_hh << "    protected:" << endl;
    msg_hh << "        MessageObserver();" << endl;
    msg_hh << "};" << endl;

    
    msg_hh << endl;

    msg_hh << "class TestMessageObserver: public MessageObserver" << endl;
    msg_hh << "{" << endl;
    msg_hh << "    public:" << endl;
    foreach(Group* g, groups)
    {
        foreach(Message* m, g->getMessages())
        {
            std::string className = str(format("%1%Message") % m->getName());
            msg_hh << "        virtual void on" << className << "(boost::shared_ptr<" << className << "> m);" << endl;
            msg_cpp << "void TestMessageObserver::on" << className << "(boost::shared_ptr<" << className << "> m){" << endl;
            msg_cpp << "    info() << \"TestMessageObserver:\" << *m;" << endl;
            msg_cpp << "}" << endl;
        }
    }
    msg_hh << "};" << endl;

    msg_hh << "class MessageSource" << endl;
    msg_hh << "{" << endl;
    msg_hh << "    public:" << endl;
    msg_hh << "        static std::string print(const byte_vec_t& bytes);" << endl;
    msg_hh << endl;    
    msg_hh << "        void notifyObservers(const byte_vec_t& bytes);" << endl;
    msg_hh << "        void addObserver(boost::shared_ptr<MessageObserver> o);" << endl;
    msg_hh << "        void removeObserver(boost::shared_ptr<MessageObserver> o);" << endl;
    msg_hh << "        void clearObservers();" << endl;
    msg_hh << endl;
    msg_hh << "    protected:" << endl;
    msg_hh << "        std::list< boost::shared_ptr<MessageObserver> > m_obs;" << endl;
    msg_hh << endl;
    msg_hh << "        MessageSource();" << endl;
    msg_hh << "};" << endl;
    
    msg_cpp << endl;
    msg_cpp << endl;
    msg_cpp << "MessageSource::MessageSource()" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << "void MessageSource::notifyObservers(const byte_vec_t& bytes)" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    if (bytes.size() < 4)" << endl;
    msg_cpp << "        throw std::out_of_range(\"Buffer too small to contain message id\");" << endl;
    msg_cpp << endl;
    msg_cpp << "    switch(*reinterpret_cast<const uint32_t*>(&bytes[0]))" << endl;
    msg_cpp << "    {" << endl;
    foreach(Group* g, groups)
    {
        foreach(Message* m, g->getMessages())
        {
            std::string className = str(format("%1%Message") % m->getName());
            msg_cpp << "    case " << m->getId() << ":" << endl; 
            msg_cpp << "    {" << endl;
            msg_cpp << "        " << format("boost::shared_ptr<%1%> m = %1%::fromBytes(bytes);") % className << endl;
            msg_cpp << "        foreach(boost::shared_ptr<MessageObserver> o, m_obs)" << endl;
            msg_cpp << "        {" << endl;
            msg_cpp << "            o->on" << className << "(m);" << endl;
            msg_cpp << "        }" << endl;
            msg_cpp << "        break;" << endl;
            msg_cpp << "    }" << endl;
        }
    }
    msg_cpp << "    default:" << endl;
    msg_cpp << "        throw(std::out_of_range(\"Unknown message id\"));" << endl;
    msg_cpp << "    }" << endl;
    msg_cpp << "}" << endl;
    
    msg_cpp << "std::string MessageSource::print(const byte_vec_t& bytes)" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    if (bytes.size() < 4)" << endl;
    msg_cpp << "        return \"{error: Buffer too small to contain message id}\";" << endl;
    msg_cpp << endl;
    msg_cpp << "    std::ostringstream os;" << endl;
    msg_cpp << "    switch(*reinterpret_cast<const uint32_t*>(&bytes[0]))" << endl;
    msg_cpp << "    {" << endl;
    foreach(Group* g, groups)
    {
        foreach(Message* m, g->getMessages())
        {
            std::string className = str(format("%1%Message") % m->getName());
            msg_cpp << "        case " << m->getId() << ": os << *" << className << "::fromBytes(bytes); break;" << endl;
        }
    }
    msg_cpp << "        default: os << \"error: Unknown message id\"; break;" << endl;
    msg_cpp << "    }" << endl;
    msg_cpp << "    return os.str();" << endl;
    msg_cpp << "}" << endl;

    msg_cpp << "void MessageSource::addObserver(boost::shared_ptr<MessageObserver> o)" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    m_obs.push_back(o);" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << "void MessageSource::removeObserver(boost::shared_ptr<MessageObserver> o)" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    m_obs.remove(o);" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << "void MessageSource::clearObservers()" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    m_obs.clear();" << endl;
    msg_cpp << "}" << endl;
    

    msg_hh << endl;
    msg_hh << format("#endif//__%1%_H__") % to_upper_copy(outputfile) << endl;

    foreach(Group *g, groups)
    {
        delete g;
    }
    foreach(Struct *s, structs)
    {
        delete s;
    }


    fstream f_msg_hh;
    string f_msg_hh_name = str(format("%1%.h") % outputpath);
    f_msg_hh.open(f_msg_hh_name.c_str(), fstream::out);
    if (f_msg_hh.fail())
    {
        cerr << "Could not open file for writing: " << f_msg_hh << endl;
        return 1;
    }
    else
    {
        f_msg_hh << msg_hh.str();
        f_msg_hh.close();
    }

    fstream f_msg_cpp;
    string f_msg_cpp_name = str(format("%1%.cpp") % outputpath);
    f_msg_cpp.open(f_msg_cpp_name.c_str(), fstream::out);
    if (f_msg_cpp.fail())
    {
        cerr << "Could not open file for writing: " << f_msg_cpp << endl;
        return 1;
    }
    else
    {
        f_msg_cpp << msg_cpp.str();
        f_msg_cpp.close();
    }

    return 0;
}


std::map< string, pair<string,string> > java_data_funcs = boost::assign::map_list_of
    ("bool",   make_pair("readBoolean"        , "writeBoolean"))
    ("byte",   make_pair("readByte"           , "writeByte"   ))
    ("int8",   make_pair("readByte"           , "writeByte"   ))
    ("int16",  make_pair("readShort"          , "writeShort"  ))
    ("int32",  make_pair("readInt"            , "writeInt"    ))
    ("uint8",  make_pair("readUnsignedByte"   , "writeByte"   ))
    ("uint16", make_pair("readUnsignedShort"  , "writeShort"  ))
    ("uint32", make_pair("readInt"            , "writeInt"    ))
    ("float",  make_pair("readFloat"          , "writeFloat"  ))
    ("double", make_pair("readDouble"         , "writeDouble" ))
;
static void serialiseJavaType(Type* type, string name, string prefix, int indentation, ostream& serialize, ostream& deserialize)
{
    string indent(indentation * 4, ' ');
    switch (type->getType())
    {
        case TYPE_BASE:
        {
            BaseType* b = static_cast<BaseType*>(type);
            if (b->getName() == string("string"))
            {
                serialize << indent << "s.writeInt(" << name << ".length());" << endl;
                serialize << indent << "s.writeBytes(" << name << ");" << endl;
                deserialize << indent << format("int %1%_l%2% = s.readInt();") % prefix % indentation << endl;
                deserialize << indent << format("byte[] %1%_b%2% = new byte[%1%_l%2%];") % prefix % indentation << endl;
                deserialize << indent << format("for (int i%2% = 0; i%2% < %1%_l%2%; i%2%++)") % prefix % indentation << endl;
                deserialize << indent <<        "{" << endl;
                deserialize << indent << format("    %1%_b%2%[i%2%] = s.readByte();") % prefix % indentation << endl;
                deserialize << indent <<        "}" << endl;
                deserialize << indent << format("%3% = new String(%1%_b%2%);") % prefix % indentation % name << endl;
            }
            else
            {
                map< string, pair<string,string> >::iterator it = java_data_funcs.find(b->getName());
                if (it != java_data_funcs.end())
                {
                    serialize << indent << "s."<< it->second.second <<"(" << name << ");" << endl;
                    deserialize << indent << name << " = s."<< it->second.first <<"();" << endl;
                }
                else
                {
                    serialize << indent << name << ".writeInto(s);" << endl;
                    deserialize << indent << name << " = " << asJavaType(type) << ".readFrom(s);" << endl;
                }
            }
            break;
        }
        case TYPE_LIST:
        {
            ListType* l = static_cast<ListType*>(type);
            
            string valTypeStr = asJavaType(l->getValType());
            string valVar = str(format("i%1%_val") % indentation);
            
            serialize << indent << format("s.writeLong(%1%.size());") % name << endl;
            serialize << indent << format("for (int i%1% = 0; i%1% < %2%.size(); i%1%++)") % indentation % name << endl;
            serialize << indent << "{" << endl;
            serialize << indent << "    " << valTypeStr << " " << valVar << " = " << name << ".get(i" << indentation << ");" << endl;
            
            deserialize << indent << format("%1% = new %2%();") % name % asJavaType(type) << endl;
            deserialize << indent << format("long %1%_i%2%_max = s.readLong();") % prefix % indentation << endl;
            deserialize << indent << format("for (int i%2% = 0; i%2% < %1%_i%2%_max; i%2%++)") % prefix % indentation << endl;
            deserialize << indent << "{" << endl;
            deserialize << indent << "    " << valTypeStr << " " << valVar << ";" << endl;
            
            serialiseJavaType(l->getValType(), str(format("i%1%_val") % indentation), prefix, indentation+1, serialize, deserialize);
            
            serialize << indent << "}" << endl;
            
            deserialize << indent << format("    %1%.add(i%2%, i%2%_val);") % name % indentation << endl;
            deserialize << indent << "}" << endl;
            break;
        }
        case TYPE_MAP:
        {
            MapType* m = static_cast<MapType*>(type);

            string keyTypeStr = asJavaType(m->getKeyType());
            string valTypeStr = asJavaType(m->getValType());
            string keyVar = str(format("i%1%_key") % indentation);
            string valVar = str(format("i%1%_val") % indentation);

            serialize << indent << format("for (Map.Entry<%1%, %2%> i%3% : %4%)") % keyTypeStr % valTypeStr % indentation % name << endl;
            serialize << indent << "{" << endl;
            serialize << indent << "    " << keyTypeStr << " " << keyVar << " = i" << indentation << ".getKey();" << endl;
            serialize << indent << "    " << valTypeStr << " " << valVar << " = i" << indentation << ".getValue();" << endl;

            deserialize << indent << format("%1% = new %2%();") % name % asJavaType(type) << endl;
            deserialize << indent << format("long i%1%_max = bb.getLong();") % indentation << endl;
            deserialize << indent << format("for (int i%1% = 0; i%1% < %1%_max; i%1%++)") % indentation << endl;
            deserialize << indent << "{" << endl;
            deserialize << indent << "    " << keyTypeStr << " " << keyVar << ";" << endl;
            deserialize << indent << "    " << valTypeStr << " " << valVar << ";" << endl;
            
            serialiseJavaType(m->getKeyType(), str(format("i%1%_key") % indentation), prefix, indentation+1, serialize, deserialize);
            serialiseJavaType(m->getValType(), str(format("i%1%_val") % indentation), prefix, indentation+1, serialize, deserialize);
            
            serialize << indent << "}" << endl;
            
            deserialize << indent << format("    %1%.put(i%2%_key, i%2%_val);") % name % indentation << endl;
            deserialize << indent << "}" << endl;
            break;
        }
        default:
            cerr << "Error while processing type" << endl;
            return;
    }
}

int createJavaFile(string outputpath)
{
    stringstream msg_java;

    path outputp(outputpath);
    string outputfile = outputp.filename();
    stringstream package_ss;
    foreach(string part, outputp.parent_path())
    {
        package_ss << part << ".";
    }
    string package = package_ss.str();
    trim_right_if(package, is_any_of("."));

    //if (!package.empty())
    //    msg_java << "package " << package << ";" << endl;
    msg_java << "package cauv.auv;" << endl;


    msg_java << "import java.util.LinkedList;" << endl;
    msg_java << "import java.util.Vector;" << endl;
    msg_java << "import java.util.HashMap;" << endl;
    msg_java << "import java.io.*;" << endl;
    msg_java << endl;
    
    foreach(Struct* s, structs)
    {
        msg_java << "class " << s->getName() << endl;
        msg_java << "{" << endl;
            
        stringstream msg_java_serialise, msg_java_deserialise;
        foreach(Declaration* d, s->getDeclarations())
        {
            string dname = d->getName();
            Type* dtype = d->getType();
       
            msg_java << "    public " << asJavaType(dtype) << " " << dname << ";" << endl;
            serialiseJavaType(dtype, str(format("val.%1%") % dname), dname, 2, msg_java_serialise, msg_java_deserialise);
        }
        msg_java << endl;
        msg_java << "    public " << s->getName() << "()" << endl;
        msg_java << "    {" << endl;
        msg_java << "    }" << endl;
        msg_java << "    public static " << s->getName() << " readFrom(DataInputStream s) throws IOException" << endl;
        msg_java << "    {" << endl;
        msg_java << "    " << format("    %1% val = new %1%();") % s->getName() << endl;
        msg_java << msg_java_deserialise.str();
        msg_java << "        return val;" << endl;
        msg_java << "    }" << endl;
        msg_java << "    public void writeInto(DataOutputStream s) throws IOException" << endl;
        msg_java << "    {" << endl;
        msg_java << "    " << format("    %1% val = this;") % s->getName() << endl;
        msg_java << msg_java_serialise.str();
        msg_java << "    }" << endl;
        msg_java << "}" << endl;
        msg_java << endl;
    }
    foreach(Enum* e, enums)
    {
        msg_java << "enum " << e->getName() << endl;
        msg_java << "{" << endl;
        int i = 0;
        foreach(EnumVal* v, e->getVals())
        {
            if (i++ > 0)
                msg_java << "," << endl;
            msg_java << "    " << v->getName();
        }
        msg_java << ";" << endl;
        msg_java << endl;
        msg_java << "    public static " << e->getName() << " readFrom(DataInputStream s) throws IOException" << endl;
        msg_java << "    {" << endl;
        msg_java << "        int val = s.readInt();" << endl;
        msg_java << "        switch (val)" << endl;
        msg_java << "        {" << endl;
        foreach(EnumVal* v, e->getVals())
        {
            msg_java << "            case " << v->getVal() << ": return " << v->getName() << ";" << endl;
        }
        msg_java << "            default: throw new IllegalArgumentException(\"Unrecognized " << e->getName() << " value: \" + val);" << endl;
        msg_java << "        }" << endl;
        msg_java << "    }" << endl;
        msg_java << "    public void writeInto(DataOutputStream s) throws IOException" << endl;
        msg_java << "    {" << endl;
        msg_java << "        switch (this)" << endl;
        msg_java << "        {" << endl;
        foreach(EnumVal* v, e->getVals())
        {
            msg_java << "            case " << v->getName() << ": s.writeInt(" << v->getVal() << "); break;" << endl;
        }
        msg_java << "        }" << endl;
        msg_java << "    }" << endl;
        msg_java << endl;
        msg_java << "}" << endl;
        msg_java << endl;
    }
    
    msg_java << "abstract class Message" << endl;
    msg_java << "{" << endl;
    msg_java << "    public String group()" << endl;
    msg_java << "    {" << endl;
    msg_java << "        return m_group;" << endl;
    msg_java << "    }" << endl;
    msg_java << "    public int id()" << endl;
    msg_java << "    {" << endl;
    msg_java << "        return m_id;" << endl;
    msg_java << "    }" << endl;
    msg_java << endl;
    msg_java << "    public abstract byte[] toBytes() throws IOException;" << endl;
    msg_java << endl;
    msg_java << "    protected int m_id;" << endl;
    msg_java << "    protected String m_group;" << endl;
    msg_java << endl;
    msg_java << "    protected Message(int id, String group)" << endl;
    msg_java << "    {" << endl;
    msg_java << "        m_id = id;" << endl;
    msg_java << "        m_group = group;" << endl;
    msg_java << "    }" << endl;
    msg_java << "}" << endl;
    msg_java << endl;

    foreach(Group* g, groups)
    {
        string gname = g->getName();

        foreach(Message* m, g->getMessages())
        {
            string name = m->getName();
            uint32_t id = m->getId();
            
            string className = str(format("%1%Message") % m->getName());

            msg_java << "class " << className << " extends Message" << endl;
            msg_java << "{" << endl;
            
            stringstream msg_java_msg_decls, msg_java_msg_funcs, msg_java_msg_serialise, msg_java_msg_deserialise;
            stringstream msg_java_msg_cstrctr_p, msg_java_msg_cstrctr_i; 
            foreach(Declaration* d, m->getDeclarations())
            {
                string dname = d->getName();
                Type* dtype = d->getType();
           
                msg_java_msg_decls << "    protected " << asJavaType(dtype) << " m_" << dname << ";" << endl;
                msg_java_msg_funcs << "    public " << asJavaType(dtype) << " " << dname << "()" << endl;
                msg_java_msg_funcs << "    {" << endl;
                msg_java_msg_funcs << "        return m_" << dname << ";" << endl;
                msg_java_msg_funcs << "    }" << endl;
                msg_java_msg_funcs << "    public void " << dname << "(" << asJavaType(dtype) << " val)" << endl;
                msg_java_msg_funcs << "    {" << endl;
                msg_java_msg_funcs << "        m_" << dname << " = val;" << endl;
                msg_java_msg_funcs << "    }" << endl;
                msg_java_msg_funcs << endl;
                msg_java_msg_cstrctr_p << asJavaType(dtype) << " " << dname << ", ";
                msg_java_msg_cstrctr_i << "        m_" << dname << " = " << dname << ";" << endl;
                serialiseJavaType(dtype, str(format("m_%1%") % dname), dname, 2, msg_java_msg_serialise, msg_java_msg_deserialise);
            }
            msg_java << msg_java_msg_decls.str();
            msg_java << endl;
            msg_java << msg_java_msg_funcs.str();
            msg_java << endl;
            msg_java << "    public " << className << "()" << endl;
            msg_java << "    {" << endl;
            msg_java << "        super("<< id <<", \""<< gname <<"\");" << endl;
            msg_java << "    }" << endl;
            msg_java << endl;
            if (m->getDeclarations().size() > 0)
            {
                string msg_java_msg_cstrctr_ps = msg_java_msg_cstrctr_p.str();
                msg_java << "    public " << className << "(" << msg_java_msg_cstrctr_ps.substr(0, msg_java_msg_cstrctr_ps.length() - 2) << ")" << endl;
                msg_java << "    {" << endl;
                msg_java << "        super("<< id <<", \""<< gname <<"\");" << endl;
                msg_java << msg_java_msg_cstrctr_i.str();
                msg_java << "    }" << endl;
                msg_java << endl;
            }
            msg_java << "    public " << className << "(byte[] bytes) throws IOException" << endl;
            msg_java << "    {" << endl;
            msg_java << "        super("<< id <<", \""<< gname <<"\");" << endl;
            msg_java << "        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);" << endl;
            msg_java << "        DataInputStream s = new DataInputStream(bs);" << endl;
            msg_java << "        int buf_id = s.readInt();" << endl;
            msg_java << "        if (buf_id != m_id)" << endl;
            msg_java << "        {" << endl;
            msg_java << "            throw new IllegalArgumentException(\"Attempted to create " << className << " with invalid id\");" << endl;
            msg_java << "        }" << endl;
            msg_java << endl;
            msg_java << msg_java_msg_deserialise.str();
            msg_java << "    }" << endl;


            msg_java << "    public byte[] toBytes() throws IOException" << endl;
            msg_java << "    {" << endl;
            msg_java << "        ByteArrayOutputStream bs = new ByteArrayOutputStream();" << endl;
            msg_java << "        DataOutputStream s = new DataOutputStream(bs);" << endl;
            msg_java << "        s.writeInt(m_id);" << endl;
            msg_java << endl;
            msg_java << msg_java_msg_serialise.str();
            msg_java << endl;
            msg_java << "        return bs.toByteArray();" << endl;
            msg_java << "    }" << endl;
            msg_java << "}" << endl;
        }
    }
   
    msg_java << endl;
    msg_java << endl;

    msg_java << "interface MessageObserver" << endl;
    msg_java << "{" << endl;
    msg_java << endl;
    foreach(Group* g, groups)
    {
        foreach(Message* m, g->getMessages())
        {
            string className = str(format("%1%Message") % m->getName());
            msg_java << "    public void on" << className << "(final " << className << " m);" << endl;
        }
    }
    msg_java << "};" << endl;

    msg_java << "class MessageSource" << endl;
    msg_java << "{" << endl;
    msg_java << "    protected LinkedList<MessageObserver> m_obs;" << endl;
    msg_java << endl;
    msg_java << "    protected MessageSource()" << endl;
    msg_java << "    {" << endl;
    msg_java << "    }" << endl;
    msg_java << endl;
    msg_java << "    public void notifyObservers(byte[] b)" << endl;
    msg_java << "    {" << endl;
    msg_java << "        if (b.length < 4)" << endl;
    msg_java << "            throw new IllegalArgumentException(\"Buffer too small to contain message id\");" << endl;
    msg_java << endl;
    msg_java << "        int id = b[3] << 24 | b[2] << 16 | b[1] << 8 | b[0];" << endl;
    msg_java << "        try" << endl;
    msg_java << "        {" << endl;
    msg_java << "            switch(id)" << endl;
    msg_java << "            {" << endl;
    foreach(Group* g, groups)
    {
        foreach(Message* m, g->getMessages())
        {
            string className = str(format("%1%Message") % m->getName());
            msg_java << "                case " << m->getId() << ":" << endl; 
            msg_java << "                {" << endl;
            msg_java << "                    " << className << " m = new " << className << "(b);" << endl;
            msg_java << "                    for(MessageObserver o : m_obs)" << endl;
            msg_java << "                    {" << endl;
            msg_java << "                        o.on" << className << "(m);" << endl;
            msg_java << "                    }" << endl;
            msg_java << "                    break;" << endl;
            msg_java << "                }" << endl;
        }
    }
    msg_java << "            }" << endl;
    msg_java << "        }" << endl;
    msg_java << "        catch (IOException e)" << endl;
    msg_java << "        {" << endl;
    msg_java << "        }" << endl;
    msg_java << "    }" << endl;
    msg_java << "    public void addObserver(MessageObserver o)" << endl;
    msg_java << "    {" << endl;
    msg_java << "        m_obs.add(o);" << endl;
    msg_java << "    }" << endl;
    msg_java << "    public void removeObserver(MessageObserver o)" << endl;
    msg_java << "    {" << endl;
    msg_java << "        m_obs.remove(o);" << endl;
    msg_java << "    }" << endl;
    msg_java << "    public void clearObservers()" << endl;
    msg_java << "    {" << endl;
    msg_java << "        m_obs.clear();" << endl;
    msg_java << "    }" << endl;
    msg_java << "}" << endl;
    
    
    foreach(Group *g, groups)
    {
        delete g;
    }
    foreach(Struct *s, structs)
    {
        delete s;
    }


    fstream f_msg_java;
    string f_msg_java_name = str(format("%1%.java") % outputpath);
    f_msg_java.open(f_msg_java_name.c_str(), fstream::out);
    if (f_msg_java.fail())
    {
        cerr << "Could not open file for writing: " << f_msg_java << endl;
        return 1;
    }
    else
    {
        f_msg_java << msg_java.str();
        f_msg_java.close();
    }

    return 0;
}
