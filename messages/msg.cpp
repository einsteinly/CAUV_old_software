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
    ("double");
std::map<string, string> cpp_conversion = boost::assign::map_list_of
    ("byte", "uint8_t")
    ("int8", "int8_t")
    ("int16", "int16_t")
    ("int32", "int32_t")
    ("uint8", "uint8_t")
    ("uint16", "uint16_t")
    ("uint32", "uint32_t");
std::map<string, string> java_conversion = boost::assign::map_list_of
    ("bool", "boolean")
    ("int8", "byte")
    ("int16", "short")
    ("int32", "int")
    ("uint8", "byte")
    ("uint16", "char")
    ("uint32", "int");

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
            ss << "vector< " << asCPPType(l->getValType()) << " >";
            return ss.str();
        }
        case TYPE_MAP:
        {
            MapType* m = static_cast<MapType*>(type);
            stringstream ss;
            ss << "map< " << asCPPType(m->getKeyType()) << "," << asCPPType(m->getValType()) << " >";
            return ss.str();
        }
        default:
            return "ERROR";
    }
}
string asJavaType(Type* type)
{
    switch (type->getType())
    {
        case TYPE_BASE:
            return convertType(static_cast<BaseType*>(type)->getName(), cpp_conversion);
        case TYPE_LIST:
        {
            ListType* l = static_cast<ListType*>(type);
            stringstream ss;
            ss << "Vector< " << asCPPType(l->getValType()) << " >";
            return ss.str();
        }
        case TYPE_MAP:
        {
            MapType* m = static_cast<MapType*>(type);
            stringstream ss;
            ss << "HashMap< " << asCPPType(m->getKeyType()) << "," << asCPPType(m->getValType()) << " >";
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

    if (lang == "c++")
        ret = createCPPFile(outputpath);
    else if (lang == "java")
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

    msg_hh << format("#ifndef __%1%_H__") % to_upper_copy(outputfile) << endl;
    msg_hh << format("#define __%1%_H__") % to_upper_copy(outputfile) << endl;
    msg_hh << endl;
    msg_hh << "#include <string>" << endl;
    msg_hh << "#include <vector>" << endl;
    msg_hh << "#include <list>" << endl;
    msg_hh << "#include <map>" << endl;
    msg_hh << "#include <boost/cstdint.hpp>" << endl;
    msg_hh << "#include <boost/shared_ptr.hpp>" << endl;
    msg_hh << "#ifndef foreach" << endl;
    msg_hh << "#    include <boost/foreach.hpp>" << endl;
    msg_hh << "#    define foreach BOOST_FOREACH" << endl;
    msg_hh << "#endif" << endl;
    msg_hh << "#include \"buffers.h\"" << endl;
    msg_hh << endl;
    msg_hh << "using namespace std;" << endl;
    msg_hh << "using namespace boost;" << endl;
    msg_hh << endl;
    
    msg_cpp << format("#include \"%1%.h\"") % outputfile << endl;
    msg_cpp << "#include <boost/archive/binary_oarchive.hpp>" << endl;
    msg_cpp << "#include <boost/archive/binary_iarchive.hpp>" << endl;
    msg_cpp << "#include <boost/serialization/vector.hpp>" << endl;
    msg_cpp << "#include <boost/serialization/map.hpp>" << endl;
    msg_cpp << endl;
    
    
    foreach(Struct* s, structs)
    {
        msg_hh << "struct " << s->getName() << endl;
        msg_hh << "{" << endl;
        foreach(Declaration* d, s->getDeclarations())
        {
            msg_hh << "    " << asCPPType(d->getType()) << " " << d->getName() << ";" << endl;
        }
        msg_hh << "};" << endl;
        msg_hh << endl;
    }
    
    msg_hh << "class Message" << endl;
    msg_hh << "{" << endl;
    msg_hh << "    public:" << endl;
    msg_hh << "        virtual ~Message();" << endl;
    msg_hh << endl;
    msg_hh << "        string group() const;" << endl;
    msg_hh << "        uint32_t id() const;" << endl;
    msg_hh << endl;
    msg_hh << "        virtual vector<char> toBytes() const = 0;" << endl;
    msg_hh << endl;
    msg_hh << "    protected:" << endl;
    msg_hh << "        uint32_t m_id;" << endl;
    msg_hh << "        string m_group;" << endl;
    msg_hh << endl;
    msg_hh << "        Message(uint32_t id, string group);" << endl;
    msg_hh << "};" << endl;
    msg_hh << endl;

    msg_cpp << "Message::Message(uint32_t id, string group) : m_id(id), m_group(group)" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << endl;
    msg_cpp << "Message::~Message()" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << endl;
    msg_cpp << "string Message::group() const" << endl;
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
        string gname = g->getName();

        foreach(Message* m, g->getMessages())
        {
            string name = m->getName();
            uint32_t id = m->getId();
            
            string className = str(format("%1%Message") % m->getName());

            stringstream msg_hh_msg_funcs, msg_hh_msg_fields, msg_cpp_msg_funcs, msg_cpp_msg_serial;

            
            foreach(Declaration* d, m->getDeclarations())
            {
                string dname = d->getName();
                Type* dtype = d->getType();

                string dtypestr = asCPPType(dtype);
                
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
            }


            msg_hh << "class " << className << " : public Message" << endl;
            msg_hh << "{" << endl;
            msg_hh << "    public:" << endl;
            msg_hh << "        " << className << "();" << endl;
            msg_hh << "        " << className << "(char* bytes, size_t bytelen);" << endl;
            msg_hh << endl;
            msg_hh << msg_hh_msg_funcs.str();
            msg_hh << endl;
            msg_hh << "        virtual vector<char> toBytes() const;" << endl;
            msg_hh << endl;
            msg_hh << "    protected:" << endl;
            msg_hh << msg_hh_msg_fields.str();
            msg_hh << "};" << endl;

            msg_cpp << format("%1%::%1%() : Message(%2%, \"%3%\")") % className % id % gname << endl;
            msg_cpp << "{" << endl;
            msg_cpp << "}" << endl;
            msg_cpp << format("%1%::%1%(char* bytes, size_t bytelen) : Message(%2%, \"%3%\")") % className % id % gname << endl;
            msg_cpp << "{" << endl;
            msg_cpp << "    char_array_buffer b(bytes, bytes + bytelen);" << endl;
            msg_cpp << "    boost::archive::binary_iarchive ar(b, boost::archive::no_header);" << endl;
            msg_cpp << "    uint32_t buf_id;" << endl;
            msg_cpp << "    ar & buf_id;" << endl;
            msg_cpp << "    if (buf_id != m_id)" << endl;
            msg_cpp << "    {" << endl;
            msg_cpp << "        throw invalid_argument(\"Attempted to create " << className << " with invalid id\");" << endl;
            msg_cpp << "    }" << endl;
            msg_cpp << endl;
            msg_cpp << msg_cpp_msg_serial.str();
            msg_cpp << "}" << endl;
            msg_cpp << msg_cpp_msg_funcs.str() << endl;
            msg_cpp << "vector<char> " << className << "::toBytes() const" << endl;
            msg_cpp << "{" << endl;
            msg_cpp << "    char_vector_buffer b;" << endl;
            msg_cpp << "    boost::archive::binary_oarchive ar(b, boost::archive::no_header);" << endl;
            msg_cpp << "    ar & m_id;" << endl;
            msg_cpp << msg_cpp_msg_serial.str();
            msg_cpp << "    return b.getVector();" << endl;
            msg_cpp << "}" << endl;
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
            string className = str(format("%1%Message") % m->getName());
            msg_hh << "        void on" << className << "(const " << className << "& m);" << endl;
            msg_cpp << "void MessageObserver::on" << className << "(const " << className << "& m) {}" << endl;
        }
    }
    
    msg_hh << endl;
    msg_hh << "    protected:" << endl;
    msg_hh << "        MessageObserver();" << endl;
    msg_hh << "};" << endl;

    msg_hh << "class MessageSource" << endl;
    msg_hh << "{" << endl;
    msg_hh << "    public:" << endl;
    msg_hh << "        void notifyObservers(char* buf, size_t buflen);" << endl;
    msg_hh << "        void addObserver(shared_ptr<MessageObserver> o);" << endl;
    msg_hh << "        void removeObserver(shared_ptr<MessageObserver> o);" << endl;
    msg_hh << "        void clearObservers();" << endl;
    msg_hh << endl;
    msg_hh << "    protected:" << endl;
    msg_hh << "        list< shared_ptr<MessageObserver> > m_obs;" << endl;
    msg_hh << endl;
    msg_hh << "        MessageSource();" << endl;
    msg_hh << "};" << endl;
    
    msg_cpp << "MessageSource::MessageSource()" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << "void MessageSource::notifyObservers(char* buf, size_t buflen)" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    if (buflen < 4)" << endl;
    msg_cpp << "        throw out_of_range(\"Buffer too small to contain message id\");" << endl;
    msg_cpp << endl;
    msg_cpp << "    switch(*reinterpret_cast<uint32_t*>(buf))" << endl;
    msg_cpp << "    {" << endl;
    foreach(Group* g, groups)
    {
        foreach(Message* m, g->getMessages())
        {
            string className = str(format("%1%Message") % m->getName());
            msg_cpp << "    case " << m->getId() << ":" << endl; 
            msg_cpp << "    {" << endl;
            msg_cpp << "        " << className << " m(buf, buflen);" << endl;
            msg_cpp << "        foreach(shared_ptr<MessageObserver> o, m_obs)" << endl;
            msg_cpp << "        {" << endl;
            msg_cpp << "            o->on" << className << "(m);" << endl;
            msg_cpp << "        }" << endl;
            msg_cpp << "    }" << endl;
        }
    }
    msg_cpp << "    }" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << "void MessageSource::addObserver(shared_ptr<MessageObserver> o)" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    m_obs.push_back(o);" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << "void MessageSource::removeObserver(shared_ptr<MessageObserver> o)" << endl;
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

/*
static map<string,string> java_bytebuffer_primitives = map_list_of
    ("byte", "")
    ("char", "Char")
    ("double", "Double")
    ("float", "Float")
    ("long", "Long")
    ("short", "Short")
    ("int", "Int")
;
static void serialiseJavaType(Type* type, string name, int indentation, ostream serialize, ostream deserialize)
{
    string indent(' ', indentation * 4);
    switch (type->getType())
    {
        case TYPE_BASE:
            //string type = convertType(static_cast<BaseType*>(type)->getName(), cpp_conversion);
            //    serialize << indent << "bb.writeBoolean(" << name << ");" << endl;
            //    deserialize << indent << name << " = (bb.get() != 0);" << endl;
            //}
            //else
            {
                map<string,string>::iterator it = java_bytebuffer_primitives.find(type);
                if (it != java_bytebuffer_primitives.end())
                {
                    serialize << indent << "bb.write"<<it.second<<"(" << name << ");" << endl;
                    deserialize << indent << name << " = bb.get"<<it.second<<"();" << endl;
                }
                else
                {
                    serialize << indent << name << ".writeInto(bb);" << endl;
                    deserialize << indent << name << ".getFrom(bb);" << endl;
                }
            }
            break;
        case TYPE_LIST:
        {
            ListType* l = static_cast<ListType*>(type);
            serialize << indent << format("bb.writeLong(%1%.size())") % name << endl;
            serialize << indent << format("for (int i%1% = 0; i%1% < %2%.size(); i%1%++)") % indentation % name << endl;
            serialize << indent << "{" << endl;
            deserialize << indent << format("long i%1%_max = bb.getLong();") % indentation << endl;
            deserialize << indent << format("for (int i%1% = 0; i%1% < %1%_max; i%1%++)") % indentation << endl;
            deserialize << indent << "{" << endl;
            
            //serialiseJavaType(l.getValType(), str(format("%1%[i%2%]") % name % indentation), indentation+1, serialize, deserialize);
            
            serialize << indent << "}" << endl;
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

            serialize << indent << format("for (Map.Entry<%1%, %2%> i%3%: %4%)") % keyTypeStr % valTypeStr % indentation name << endl;
            serialize << indent << "{" << endl;
            serialize << indent << "    " << keyTypeStr << " " << keyVar << " = i" << indentation << ".getKey();" << endl;
            serialize << indent << "    " << valTypeStr << " " << valVar << " = i" << indentation << ".getValue();" << endl;

            deserialize << indent << format("%1% = new %2%();") % name % asJavaType(type) << endl;
            deserialize << indent << format("long i%1%_max = bb.getLong();") % indentation << endl;
            deserialize << indent << format("for (int i%1% = 0; i%1% < %1%_max; i%1%++)") % indentation << endl;
            deserialize << indent << "{" << endl;
            deserialize << indent << "    " << keyTypeStr << " " << keyVar << ";" << endl;
            deserialize << indent << "    " << valTypeStr << " " << valVar << ";" << endl;
            
            serialiseJavaType(m.getKeyType(), str(format("i%1%_key") % indentation), indentation+1, serialize, deserialize);
            serialiseJavaType(m.getValType(), str(format("i%1%_val") % indentation), indentation+1, serialize, deserialize);
            deserialize << indent << "{" << endl;
            
            serialize << indent << "}" << endl;
            deserialize << indent << "}" << endl;
            break;
        }
        default:
            cerr << "Error while processing type" << endl;
            return 1;
    }
}

static map<string, int> java_type_lengths = map_list_of
    ("boolean", 1)
    ("byte", 1)
    ("char", 2)
    ("double", 8)
    ("float", 4)
    ("long", 8)
    ("short", 2)
    ("int", 4)
;
static void getJavaTypeLen(Type* type, string name, int indentation, ostream serialize)
{
    string indent(' ', indentation * 4);
    switch (type->getType())
    {
        case TYPE_BASE:
            string type = convertType(static_cast<BaseType*>(type)->getName(), cpp_conversion);
            if (type == "string")
            {
                // Special case for strings because they're dicks
                serialize << indent << "len += " << name << ".length;" << endl;
            }
            else
            {
                map<string,string>::iterator it = java_bytebuffer_primitives.find(type);
                if (it != java_bytebuffer_primitives.end())
                {
                    serialize << indent << "len += "<< it.second << ";" << endl;
                }
                else
                {
                    serialize << indent << name << ".getLength();" << endl;
                }
            }
            break;
        case TYPE_LIST:
        {
            ListType* l = static_cast<ListType*>(type);
            serialize << indent << format("for (int i%1% = 0; i%1% < %2%.size(); i%1%++)") % indentation % name << endl;
            serialize << indent << "{" << endl;
            getJavaTypeLen(l.getValType(), str(format("%1%[i%2%]") % name % indentation), indentation+1, serialize);
            serialize << indent << "}" << endl;
            break;
        }
        case TYPE_MAP:
        {
            MapType* m = static_cast<MapType*>(type);

            string keyTypeStr = asJavaType(m->getKeyType());
            string valTypeStr = asJavaType(m->getValType());
            string keyVar = str(format("i%1%_key") % indentation);
            string valVar = str(format("i%1%_val") % indentation);

            serialize << indent << format("for (%1% v%3% : %3%)") % keyTypeStr % valTypeStr % indentation name << endl;
            serialize << indent << "{" << endl;
            getJavaTypeLen(l.getValType(), str(format("%1%[i%2%]") % name % indentation), indentation+1, serialize);
            serialize << indent << "    " << keyTypeStr << " " << keyVar << " = i" << indentation << ".getKey();" << endl;
            serialize << indent << "    " << valTypeStr << " " << valVar << " = i" << indentation << ".getValue();" << endl;

            deserialize << indent << format("%1% = new %2%();") % name % asJavaType(type) << endl;
            deserialize << indent << format("long i%1%_max = bb.getLong();") % indentation << endl;
            deserialize << indent << format("for (int i%1% = 0; i%1% < %1%_max; i%1%++)") % indentation << endl;
            deserialize << indent << "{" << endl;
            deserialize << indent << "    " << keyTypeStr << " " << keyVar << ";" << endl;
            deserialize << indent << "    " << valTypeStr << " " << valVar << ";" << endl;
            
            serialiseJavaType(m.getKeyType(), str(format("i%1%_key") % indentation), indentation+1, serialize, deserialize);
            serialiseJavaType(m.getValType(), str(format("i%1%_val") % indentation), indentation+1, serialize, deserialize);
            deserialize << indent << "{" << endl;
            
            serialize << indent << "}" << endl;
            deserialize << indent << "}" << endl;
            break;
        }
        default:
            cerr << "Error while processing type" << endl;
            return 1;
    }
}
*/
int createJavaFile(string outputpath)
{
    /*stringstream msg_java;

    path outputp(outputpath);
    string outputfile = outputp.filename();
    stringstream package_ss;
    foreach(string part, outputp.parent_path())
    {
        package_ss << part << ".";
    }
    string package = package_ss.str();
    trim_right_if(package, is_any_of("."));

    if (!package.empty())
        msg_java << "package " << package << ";" << endl;

    msg_java << "import java.util.LinkedList;" << endl;
    msg_java << "import java.util.Vector;" << endl;
    msg_java << "import java.util.HashMap;" << endl;
    
    foreach(Struct* s, structs)
    {
        msg_java << "class " << s->getName() << endl;
        msg_java << "{" << endl;
        foreach(Declaration* d, s->getDeclarations())
        {
            msg_java << "    public " << asCPPType(d->getType()) << " " << d->getName() << ";" << endl;
        }
        msg_java << "}" << endl;
        msg_java << endl;
    }
    
    msg_java << "class Message" << endl;
    msg_java << "{" << endl;
    msg_java << "    public string group()" << endl;
    msg_java << "    {" << endl;
    msg_java << "        return m_group;" << endl;
    msg_java << "    }" << endl;
    msg_java << "    public int id()" << endl;
    msg_java << "    {" << endl;
    msg_java << "        return m_id;" << endl;
    msg_java << "    }" << endl;
    msg_java << endl;
    msg_java << "    public abstract Vector<byte> toBytes();" << endl;
    msg_java << endl;
    msg_java << "    protected int m_id;" << endl;
    msg_java << "    protected string m_group;" << endl;
    msg_java << endl;
    msg_java << "    protected Message(int id, string group)" << endl;
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

            msg_java << "class " << className << " : public Message" << endl;
            msg_java << "{" << endl;
            msg_java << "    public " << className << "()" << endl;
            msg_java << "    {" << endl;
            msg_java << "    }" << endl;
            
            stringstream msg_java_msg_serialise, msg_java_msg_deserialise;
            foreach(Declaration* d, m->getDeclarations())
            {
                string dname = d->getName();
                Type* dtype = d->getType();
    // ByteArrayInputStream
    // DataInputStream
                msg_java_msg_serialise << "        ByteBuffer bb = ByteBuffer.wrap(bytes);" << endl;
                msg_java_msg_deserialise << "        ByteBuffer bb = ByteBuffer();" << endl;
           
                serialiseJavaType(dtype, dname, 2, msg_java_msg_serialize, msg_java_msg_deserialize)
                
                
                msg_java_
                msg_java_msg_funcs << format("        const %1%& %2%() const;") % dtypestr % dname << endl;
                msg_cpp_msg_funcs << format("const %1%& %3%::%2%() const") % dtypestr % dname % className << endl;
                msg_cpp_msg_funcs << "{" << endl;
                msg_cpp_msg_funcs << "    return m_" << dname << ";" << endl;
                msg_cpp_msg_funcs << "}" << endl;
                
                msg_java_msg_funcs << format("        void %2%(%1% val);") % dtypestr % dname << endl;
                msg_cpp_msg_funcs << format("void %3%::%2%(%1% val)") % dtypestr % dname % className << endl;
                msg_cpp_msg_funcs << "{" << endl;
                msg_cpp_msg_funcs << "    m_" << dname << " = val;" << endl;
                msg_cpp_msg_funcs << "}" << endl;

                msg_java_msg_funcs << endl;
                msg_cpp_msg_funcs << endl;
                
                msg_cpp_msg_serial << "    ar & m_" << dname << ";" << endl;
            }
            msg_java << "    }" << endl;
            msg_java << "    public " << className << "(byte[] bytes)" << endl;
            msg_java << "    {" << endl;
            msg_java << "        ByteBuffer bb = ByteBuffer.wrap(bytes);" << endl;


            msg_java << msg_java_msg_funcs.str();
            msg_java << endl;
            msg_java << "        virtual vector<char> toBytes() const;" << endl;
            msg_java << endl;
            msg_java << "    protected:" << endl;
            msg_java << msg_java_msg_fields.str();
            msg_java << "};" << endl;

            msg_cpp << format("%1%::%1%() : Message(%2%, \"%3%\")") % className % id % gname << endl;
            msg_cpp << "{" << endl;
            msg_cpp << "}" << endl;
            msg_cpp << format("%1%::%1%(char* bytes, size_t bytelen) : Message(%2%, \"%3%\")") % className % id % gname << endl;
            msg_cpp << "{" << endl;
            msg_cpp << "    char_array_buffer b(bytes, bytes + bytelen);" << endl;
            msg_cpp << "    boost::archive::binary_iarchive ar(b, boost::archive::no_header);" << endl;
            msg_cpp << "    uint32_t buf_id;" << endl;
            msg_cpp << "    ar & buf_id;" << endl;
            msg_cpp << "    if (buf_id != m_id)" << endl;
            msg_cpp << "    {" << endl;
            msg_cpp << "        throw invalid_argument(\"Attempted to create " << className << " with invalid id\");" << endl;
            msg_cpp << "    }" << endl;
            msg_cpp << endl;
            msg_cpp << msg_cpp_msg_serial.str();
            msg_cpp << "}" << endl;
            msg_cpp << msg_cpp_msg_funcs.str() << endl;
            msg_cpp << "vector<char> " << className << "::toBytes() const" << endl;
            msg_cpp << "{" << endl;
            msg_cpp << "    char_vector_buffer b;" << endl;
            msg_cpp << "    boost::archive::binary_oarchive ar(b, boost::archive::no_header);" << endl;
            msg_cpp << "    ar & m_id;" << endl;
            msg_cpp << msg_cpp_msg_serial.str();
            msg_cpp << "    return b.getVector();" << endl;
            msg_cpp << "}" << endl;
        }
    }
   
    msg_java << endl;
    msg_cpp << endl;

    msg_java << "namespace boost {" << endl;
    msg_java << "namespace serialization {" << endl;
    msg_java << endl;
    foreach(Struct* s, structs)
    {
        msg_java << "template<class Archive>" << endl;
        msg_java << "void serialize(Archive & ar, " << s->getName() << "& val, const unsigned int version)" << endl;
        msg_java << "{" << endl;
        foreach(Declaration* d, s->getDeclarations())
        {
            msg_java << "    ar & val." << d->getName() << ";" << endl;
        }
        msg_java << "}" << endl;
        msg_java << endl;
    }
    msg_java << "} // namespace serialization" << endl;
    msg_java << "} // namespace boost" << endl;

    msg_java << endl;

    msg_java << "class MessageObserver" << endl;
    msg_java << "{" << endl;
    msg_java << "    public:" << endl;
    msg_java << "        virtual ~MessageObserver();" << endl;
    
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
            string className = str(format("%1%Message") % m->getName());
            msg_java << "        void on" << className << "(const " << className << "& m);" << endl;
            msg_cpp << "void MessageObserver::on" << className << "(const " << className << "& m) {}" << endl;
        }
    }
    
    msg_java << endl;
    msg_java << "    protected:" << endl;
    msg_java << "        MessageObserver();" << endl;
    msg_java << "};" << endl;

    msg_java << "class MessageSource" << endl;
    msg_java << "{" << endl;
    msg_java << "    public:" << endl;
    msg_java << "        void notifyObservers(char* buf, size_t buflen);" << endl;
    msg_java << "        void addObserver(shared_ptr<MessageObserver> o);" << endl;
    msg_java << "        void removeObserver(shared_ptr<MessageObserver> o);" << endl;
    msg_java << "        void clearObservers();" << endl;
    msg_java << endl;
    msg_java << "    protected:" << endl;
    msg_java << "        list< shared_ptr<MessageObserver> > m_obs;" << endl;
    msg_java << endl;
    msg_java << "        MessageSource();" << endl;
    msg_java << "};" << endl;
    
    msg_cpp << "MessageSource::MessageSource()" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << "void MessageSource::notifyObservers(char* buf, size_t buflen)" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    if (buflen < 4)" << endl;
    msg_cpp << "        throw out_of_range(\"Buffer too small to contain message id\");" << endl;
    msg_cpp << endl;
    msg_cpp << "    switch(*reinterpret_cast<uint32_t*>(buf))" << endl;
    msg_cpp << "    {" << endl;
    foreach(Group* g, groups)
    {
        foreach(Message* m, g->getMessages())
        {
            string className = str(format("%1%Message") % m->getName());
            msg_cpp << "    case " << m->getId() << ":" << endl; 
            msg_cpp << "    {" << endl;
            msg_cpp << "        " << className << " m(buf, buflen);" << endl;
            msg_cpp << "        foreach(shared_ptr<MessageObserver> o, m_obs)" << endl;
            msg_cpp << "        {" << endl;
            msg_cpp << "            o->on" << className << "(m);" << endl;
            msg_cpp << "        }" << endl;
            msg_cpp << "    }" << endl;
        }
    }
    msg_cpp << "    }" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << "void MessageSource::addObserver(shared_ptr<MessageObserver> o)" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    m_obs.push_back(o);" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << "void MessageSource::removeObserver(shared_ptr<MessageObserver> o)" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    m_obs.remove(o);" << endl;
    msg_cpp << "}" << endl;
    msg_cpp << "void MessageSource::clearObservers()" << endl;
    msg_cpp << "{" << endl;
    msg_cpp << "    m_obs.clear();" << endl;
    msg_cpp << "}" << endl;
    


    foreach(Group *g, groups)
    {
        delete g;
    }
    foreach(Struct *s, structs)
    {
        delete s;
    }

/ *
    fstream f_msg_java;
    string f_msg_java_name = str(format("%1%.h") % outputpath);
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
*/

  //  cout << msg_java.str() << endl;

    return 0;
}
