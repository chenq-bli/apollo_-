#pragma once
#include<iostream>
#include<stdio.h>
#include<fstream>
#include<map>
#include"globalfile.h"
using namespace std;


class orderfile //主要是为了实现映射，方便修改！
{
public:
	orderfile();

	void updteorder();//修改m_orderdate后及时写入.txt
	int m_size;
	map<int, map<string, string>> m_orderdate;//嵌套map，int是条数建，map<first_string,second_string>是其值，string是键，second_string是值。

};