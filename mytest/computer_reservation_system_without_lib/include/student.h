#pragma once
#include<vector>
#include<fstream>
#include<unistd.h>
#include"identity.h"
#include"computeroom.h"
#include"globalfile.h"
#include"orderfile.h"

using namespace std;



class student : public identity
{
public:
	student();

	student(int ID, string name, string pswd);

	//重写纯虚函数
	void openmenu();
	//申请预约
	void applyorder();
	//查看自身预约
	void showmyorder();
	//查看所有预约
	void showallorder();
	//取消预约
	void cancelorder();
	int m_ID;

	void initcomputer();
	void showcomputer();
	vector<computer> vcomroom;

private:



};