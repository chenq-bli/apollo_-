#pragma once
#include<iostream>
#include<stdio.h>
#include<string>
using namespace std;

class identity
{
public:
	virtual void openmenu() = 0;//老师学生管理员公有的操作，但具体内容不同--显示各自的操作界面
	string  m_name;
	string  m_pswd;

private:



};