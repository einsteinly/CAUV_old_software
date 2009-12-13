/*
    CAUV 2009/2010
    Author: Clemens Wiltsche
*/

/*
   Syntax-tree definition for the message creation.
*/

#ifndef __MSG_SYNTAX_H__
#define __MSG_SYNTAX_H__

#include <stdint.h>
#include <vector>
#include <string>

using namespace std;

class Node
{
    public:
        virtual ~Node();
        virtual string to_string() const = 0;
};


enum TypeType { TYPE_BASE, TYPE_LIST, TYPE_MAP };
class Type : public Node
{
    private:
        TypeType m_type;

    protected:
        Type(TypeType);

    public:
        virtual ~Type();
        TypeType getType() const;
        virtual string to_string() const = 0;
};
class BaseType : public Type
{
    private:
        char *m_name;
    public:
        BaseType(char*);
        virtual ~BaseType();
        char* getName() const;
        virtual string to_string() const;
};
class ListType : public Type
{
    protected:
        Type* m_valType;
    public:
        ListType(Type*);
        virtual ~ListType();
        Type* getValType() const;
        virtual string to_string() const;
};
class MapType : public Type
{
    protected:
        Type* m_keyType;
        Type* m_valType;
    public:
        MapType(Type*, Type*);
        virtual ~MapType();
        Type* getKeyType() const;
        Type* getValType() const;
        virtual string to_string() const;
};

class Declaration : public Node
{
    private:
        Type *m_type;
        char *m_name;

    public:
        Declaration(char*, Type*);
        virtual ~Declaration();

        char* getName() const;
        Type* getType() const;
        virtual string to_string() const;
};

class Message : public Node
{
    private:
        uint32_t m_id;
        char *m_name;
        std::vector<Declaration*>* m_declarations;

    public:
        Message(uint32_t, char*, std::vector<Declaration*>*);
        virtual ~Message();

        uint32_t getId() const;
        char* getName() const;
        const std::vector<Declaration*>& getDeclarations() const;
        virtual string to_string() const;
};

class EnumVal : public Node
{
    private:
        char *m_name;
        int32_t m_val;

    public:
        EnumVal(char*, uint32_t);
        virtual ~EnumVal();

        char* getName() const;
        uint32_t getVal() const;
        virtual string to_string() const;
};

class Enum : public Node
{
    private:
        char *m_name;
        std::vector<EnumVal*>* m_vals;

    public:
        Enum(char*, std::vector<EnumVal*>*);
        virtual ~Enum();

        char* getName() const;
        const std::vector<EnumVal*>& getVals() const;
        virtual string to_string() const;
};

class Struct : public Node
{
    private:
        char *m_name;
        std::vector<Declaration*>* m_declarations;

    public:
        Struct(char*, std::vector<Declaration*>*);
        virtual ~Struct();

        char* getName() const;
        const std::vector<Declaration*>& getDeclarations() const;
        virtual string to_string() const;
};

class Group : public Node
{
    private:
        char *m_name;
        std::vector<Message*>* m_messages;	

    public:
        Group(char*, std::vector<Message*>*);
        virtual ~Group();

        char* getName() const;
        const std::vector<Message*>& getMessages() const;
        virtual string to_string() const;
};

#endif//__MSG_SYNTAX_H__
