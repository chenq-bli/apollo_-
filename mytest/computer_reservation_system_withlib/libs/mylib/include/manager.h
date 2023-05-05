#pragma once
#include<vector>
#include<fstream>
#include"identity.h"
#include"student.h"
#include"teacher.h"
#include"computeroom.h"
#include"globalfile.h"
using namespace std;


class manager : public identity
{
public:
	manager();

	manager(string name, string pswd);

	//重写纯虚函数
	void openmenu();

	//添加成员
	void addperson();

	//查看账号
	void showperson();

	//查看机房
	void showcomputer();

	//清空预约记录
	void cleanfile();

	void initpersoncount();
	bool checkpeat(int id, int type);

	vector<student> vst;
	vector<teacher> vther;
	vector<computer> vcomroom;

private:



};
