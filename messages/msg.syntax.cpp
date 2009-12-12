/*
   CAUV 2009/2010
Author: Clemens Wiltsche
*/

/*
   Syntax-tree definition for the message creation.
   Here lie the declarations of the class-member functions.
   */

#include <boost/cstdint.hpp>     //for uint32_t
#include <iostream>
#include <sstream>
#include "msg.syntax.h" //for the class definitions
#include <vector>       //for vector
#include <boost/format.hpp>
#ifndef foreach
#   include <boost/foreach.hpp>
#   define foreach BOOST_FOREACH
#endif

Node::~Node()
{
}

//Type functions
TypeType Type::getType() const
{
    return m_type;
}
Type::~Type()
{
}

//Type constructor
Type::Type(TypeType type) : m_type(type)
{
}

BaseType::BaseType(char* name) : Type(TYPE_BASE), m_name(name) {}
BaseType::~BaseType()
{
    delete m_name;
}
char* BaseType::getName() const
{
    return m_name;
}
ListType::ListType(Type* valType) : Type(TYPE_LIST), m_valType(valType) {}
ListType::~ListType()
{
    delete m_valType;
}
Type* ListType::getValType() const
{
    return m_valType;
}
MapType::MapType(Type* keyType, Type* valType) : Type(TYPE_MAP), m_keyType(keyType), m_valType(valType) {}
MapType::~MapType()
{
    delete m_keyType;
    delete m_valType;
}
Type* MapType::getKeyType() const
{
    return m_keyType;
}
Type* MapType::getValType() const
{
    return m_valType;
}


string BaseType::to_string() const
{
    return m_name;
}
string ListType::to_string() const
{
    return str(boost::format("list< %1% >") % m_valType->to_string());
}
string MapType::to_string() const
{
    return str(boost::format("map< %s, %s >") % m_keyType->to_string() % m_valType->to_string());
}

////

//Declaration functions
char* Declaration::getName() const
{
    return m_name;
}

Type* Declaration::getType() const
{
    return m_type;
}

string Declaration::to_string() const
{
    return str(boost::format("\t\t%1% : %2%") % m_name % m_type->to_string());
}

//Declaration constructor
Declaration::Declaration(char* name, Type* type) :
    m_type(type),
    m_name(name)
{
}
Declaration::~Declaration()
{
    delete m_name;
    delete m_type;
}


////

//Message functions
char* Message::getName() const
{
    return m_name;
}

uint32_t Message::getId() const
{
    return m_id;
}

string Message::to_string() const
{
    stringstream ss;
    ss << "\tmessage" << m_name << " : " << m_id << endl;
    ss << "\t{" << endl;
    foreach(Declaration *d, *m_declarations)
    {
        ss << d->to_string() << endl;
    }
    ss << "\t}";
    return ss.str();
}


//Message constructor
Message::Message(uint32_t id, char *name, std::vector<Declaration*>* declarations) :
    m_id(id),
    m_name(name),
    m_declarations(declarations)
{
}
Message::~Message()
{
    delete m_name;
    foreach(Declaration *d, *m_declarations)
    {
        delete d;
    }
    delete m_declarations;
}

const std::vector<Declaration*>& Message::getDeclarations() const
{
    return *this->m_declarations;
}

/////

//EnumVal functions
char* EnumVal::getName() const
{
    return m_name;
}
uint32_t EnumVal::getVal() const
{
    return m_val;
}

string EnumVal::to_string() const
{
    stringstream ss;
    ss << "\t" << m_name << " = " << m_val << ";" << endl;
    return ss.str();
}


//EnumVal constructor
EnumVal::EnumVal(char *name, uint32_t val) :
    m_name(name),
    m_val(val)
{
}
EnumVal::~EnumVal()
{
    delete m_name;
}
/////

//Enum functions
char* Enum::getName() const
{
    return m_name;
}

string Enum::to_string() const
{
    stringstream ss;
    ss << "enum" << m_name << endl;
    ss << "{" << endl;
    foreach(EnumVal *v, *m_vals)
    {
        ss << v->to_string() << endl;
    }
    ss << "}";
    return ss.str();
}


//Enum constructor
Enum::Enum(char *name, std::vector<EnumVal*>* vals) :
    m_name(name),
    m_vals(vals)
{
}
Enum::~Enum()
{
    delete m_name;
    foreach(EnumVal *d, *m_vals)
    {
        delete d;
    }
    delete m_vals;
}

const std::vector<EnumVal*>& Enum::getVals() const
{
    return *this->m_vals;
}

////

//Struct functions
char* Struct::getName() const
{
    return m_name;
}

string Struct::to_string() const
{
    stringstream ss;
    ss << "struct" << m_name << endl;
    ss << "{" << endl;
    foreach(Declaration *d, *m_declarations)
    {
        ss << d->to_string() << endl;
    }
    ss << "}";
    return ss.str();
}


//Struct constructor
Struct::Struct(char *name, std::vector<Declaration*>* declarations) :
    m_name(name),
    m_declarations(declarations)
{
}
Struct::~Struct()
{
    delete m_name;
    foreach(Declaration *d, *m_declarations)
    {
        delete d;
    }
    delete m_declarations;
}

const std::vector<Declaration*>& Struct::getDeclarations() const
{
    return *this->m_declarations;
}


////

//Group functions
char* Group::getName() const
{
    return m_name;
}

string Group::to_string() const
{
    stringstream ss;
    ss << "group" << m_name << endl;
    ss << "{" << endl;
    foreach(Message *m, *m_messages)
    {
        ss << m->to_string() << endl;
    }
    ss << "}";
    return ss.str();
}

//Group constructor
Group::Group(char *name, std::vector<Message*>* messages) :
    m_name(name),
    m_messages(messages)
{
}
Group::~Group()
{
    delete m_name;
    foreach(Message *m, *m_messages)
    {
        delete m;
    }
    delete m_messages;
}

const std::vector<Message*>& Group::getMessages() const
{
    return *this->m_messages;
}
