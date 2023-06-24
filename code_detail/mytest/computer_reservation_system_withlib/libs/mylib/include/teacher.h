#pragma once
#include"identity.h"
#include"computeroom.h"
#include"globalfile.h"
#include"orderfile.h"
using namespace std;

class teacher : public identity
{
public:
	teacher();

	teacher(int ID, string name, string pswd);

	//重写纯虚函数
	void openmenu();
	void showallorder();
	bool showneed();
	void vaildorder();

	int m_ID;
private:



};