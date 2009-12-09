/*
CAUV 2009/2010
Author: Clemens Wiltsche
*/

/*
Syntax-tree definition for the message creation.
Here lie the declarations of the class-member functions.
*/

#include <stdint.h>	//for uint32_t
#include <stdio.h>	//for printf
#include "msg.h"	//for the class definitions
#include <vector>	//for vector


//Declaration functions
char* Declaration::getName()
{
	return m_name;
}

char* Declaration::getType()
{
	return m_type;
}

void Declaration::print()
{
	printf("\t\t%s %s;\n", m_type, m_name);
}

//Declaration constructor
Declaration::Declaration(char* name, char* type)
	:m_type(type),
	 m_name(name)
	 {}


////

//Message functions
char* Message::getName()
{
        return m_name;
}

uint32_t Message::getId()
{
        return m_id;
}

void Message::print()
{
	printf("\tmsg %s(%d) \n\t{\n", m_name, m_id);
	for(int i = 0; i < m_declarations->size(); i++)
	{
		m_declarations->at(i)->print();
	}
	printf("\t}\n");
}


//Message constructor
Message::Message(uint32_t id, char *name, std::vector<Declaration*>* declarations)
        :m_id(id),
         m_name(name),
         m_declarations(declarations)
        {}

Declaration* Message::getDeclaration(int declaration_num)
{
        if(declaration_num >= 0 && declaration_num < this->m_declarations->size())
	{
                return this->m_declarations->at(declaration_num);
        }
}


////

//Group functions
char* Group::getName()
{
	return m_name;
}

uint32_t Group::getId()
{
	return m_id;
}

void Group::print()
{
	printf("group %s(%d) \n{\n", m_name, m_id);
	for(int i = 0; i < m_messages->size(); i++)
	{
		m_messages->at(i)->print();
	}
	printf("}\n");
}

//Group constructor
Group::Group(uint32_t id, char *name, std::vector<Message*>* messages)
	:m_id(id),
       	 m_name(name),
	 m_messages(messages)
	{}

Message* Group::getMessage(int message_num)
{
	if(message_num >= 0 && message_num < this->m_messages->size())
	{
		return this->m_messages->at(message_num);
	}
}
