/*
CAUV 2009/2010
Author: Clemens Wiltsche
*/

/*
Syntax-tree definition for the message creation.
*/

#include <stdint.h>
#include <vector>

class Node
{
	public:
	void print();
};

class Declaration : public Node
{
	private:
	char *m_type;
	char *m_name;

	public:
	Declaration(char*, char*);

	char* getName();
	char* getType();
	void print();
};

class Message : public Node
{
	private:
	uint32_t m_id;
	char *m_name;
	std::vector<Declaration*>* m_declarations;

	public:
	Message(uint32_t, char*, std::vector<Declaration*>*);

	uint32_t getId();
        char* getName();
        Declaration* getDeclaration(int);
	void print();

};

class Group : public Node
{
	private:
	uint32_t m_id;
	char *m_name;
	std::vector<Message*>* m_messages;	

	public:
	Group(uint32_t, char*, std::vector<Message*>*);

	uint32_t getId();
	char* getName();
	Message* getMessage(int);
	void print();
};
