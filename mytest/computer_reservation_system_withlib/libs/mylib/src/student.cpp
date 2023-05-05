#include"student.h"

student::student()
{

}
student::student(int ID, string name, string pswd)
{
	this->m_ID = ID;
	this->m_name = name;
	this->m_pswd = pswd;
	this->initcomputer();

}

void student::openmenu()
{
	cout << "                       ================ 欢迎来到学生操作菜单 ================" << endl;
	cout << endl;
	cout << endl;
	cout << "\t\t----------------------------------\n";
	cout << "\t\t|         1.申请预约             |\n";
	cout << "\t\t|                                |\n";
	cout << "\t\t|         2.查看我的预约         |\n";
	cout << "\t\t|                                |\n";
	cout << "\t\t|         3.查看所有预约         |\n";
	cout << "\t\t|                                |\n";
	cout << "\t\t|         4.取消预约             |\n";
	cout << "\t\t|                                |\n";
	cout << "\t\t|         0.注销登入             |\n";
	cout << "\t\t|                                |\n";
	cout << "\t\t----------------------------------\n";
	cout << endl;
	cout << endl;

}

void student::applyorder()
{
	
	int date = 0;
	int interval = 0;
	int room = 0;

	while (true)
	{
		cout << "机房开放的时间为周一到周五！" << endl;
		cout << "1.周一" << endl;
		cout << "2.周二" << endl;
		cout << "3.周三" << endl;
		cout << "4.周四" << endl;
		cout << "5.周五" << endl;
		cout << "请输入申请预约的日期：" << endl;
		cin >> date;
		if (date >= 1 && date <= 5)
		{
			system("cls");
			break;
		}
		else
		{
			cout << "有误，重新输入！" << endl;
			system("pause");
			system("cls");
		}

	}

	while (true)
	{

		cout << "请输入申请预约的时间段：" << endl;
		cout << "1.上午" << endl;
		cout << "2.下午" << endl;
		cin >> interval;
		if (interval >= 1 && interval <= 2)
		{
			system("cls");
			break;
		}
		else
		{
			cout << "有误，重新输入！" << endl;
			system("pause");
			system("cls");

		}

	}



	while (true)
	{

		cout << "请输入机房编号：" << endl;
		showcomputer();
		cin >> room;
		if (room >= 1 && room <= 3)
		{
			system("cls");
			break;
		}
		else
		{
			cout << "有误，重新输入！" << endl;
			system("pause");
			system("cls");

		}

	}

	cout << "预约成功，审核中！" << endl;

	ofstream ofs;
	ofs.open(ORDER_FILE, ios::app);
	ofs << "date:" << date << " ";
	ofs << "interval:" << interval << " ";
	ofs << "stuID:" << this->m_ID << " ";
	ofs << "stuname:" << this->m_name << " ";
	ofs << "room:" << room << " ";
	ofs << "status:" << 1 << endl;

	ofs.close();

	system("pause");
	system("cls");

	return;
}
void student::showmyorder()
{
	orderfile of;
	if (of.m_size == 0)
	{
		cout<<"无预约记录" << endl;
		system("pause");
		system("cls");
		return;

	}
	int flg = 0;
	for (int i = 1; i < of.m_size + 1; i++)
	{
		if (atoi(of.m_orderdate[i]["stuID"].c_str()) == this->m_ID) //.c_str() 转化为 char*,atoi()转化为整形
		{
			cout << "预约编号：" <<i<< " ";
			cout << "预约日期：周" << of.m_orderdate[i]["date"]<<" ";
			cout << "时段：" << (of.m_orderdate[i]["interval"] == "1" ? "上午" : "下午") << " ";
			cout << "机房：" << of.m_orderdate[i]["room"] << " ";
			string ss;
			if (of.m_orderdate[i]["status"] == "1")
			{
				ss = "审核中！";
			}
			else if (of.m_orderdate[i]["status"] == "2")
			{
				ss = "预约成功！";
			}
			else if (of.m_orderdate[i]["status"] == "3")
			{
				ss = "取消预约！";
			}
			else
			{
				ss = "审核未通过，预约失败！";
			}
			cout << ss << endl;
			flg = 1;
		}
	}
	if (flg == 0) cout << "没有你的预约记录！" << endl;
	system("pause");
	system("cls");

}
void student::showallorder()
{
	orderfile of;
	if (of.m_size == 0)
	{
		cout << "无预约记录" << endl;
		system("pause");
		system("cls");
		return;

	}
	
	for (int i = 1; i < of.m_size + 1; i++)
	{
		cout << "预约日期:周" << of.m_orderdate[i]["date"] << " ";
		cout << "时段：" << (of.m_orderdate[i]["interval"] == "1" ? "上午" : "下午") << " ";
		cout << "学生ID：" << of.m_orderdate[i]["stuID"] << " ";
		cout << "学生名字：" << of.m_orderdate[i]["stuname"] << " ";
		cout << "机房ID：" << of.m_orderdate[i]["roomID"] << " ";
		string ss;
		if (of.m_orderdate[i]["status"] == "1")
		{
			ss = "审核中！";
		}
		else if (of.m_orderdate[i]["status"] == "2")
		{
			ss = "预约成功！";
		}
		else if (of.m_orderdate[i]["status"] == "3")
		{
			ss = "取消预约！";
		}
		else
		{
			ss = "审核未通过，预约失败！";
		}
		cout << ss << endl;
	}

system("pause");
system("clc");

}
void student::cancelorder()
{
	orderfile of;

	int select;
	while (true)
	{
		cout << "请输入您要取消的预约编号！" << endl;
		this->showmyorder();
		cout << "请输入您要取消的预约编号，如果没有您的预约按0退出！" << endl;
		cin >> select;
		if (select == 0) return;
		if (atoi(of.m_orderdate[select]["stuID"].c_str()) == this->m_ID)
		{
			
			break;
		}
		else
		{
			cout << "你无权取消他人预约！" << endl;
			system("pause");
			system("cls");

		}

	}
	of.m_orderdate[select]["status"] = "3";
	of.updteorder();
	cout << "取消成功！" << endl;
	system("pause");
	system("cls");
	
}

void student::initcomputer()
{
	vcomroom.clear();
	ifstream ifs;
	ifs.open(COMPUTER_FILE, ios::in);// in是读文件
	if (!ifs.is_open())
	{
		cout << "文件不存在！" << endl;
		ifs.close();
		return;
	}


	computer com;

	while (ifs >> com.m_comID && ifs >> com.m_maxNUM)
	{
		vcomroom.push_back(com);
	}
	ifs.close();
}

void student::showcomputer()
{
	for (vector<computer>::iterator it = vcomroom.begin(); it != vcomroom.end(); it++)
	{
		cout << "机房：" << it->m_comID << " " << it->m_maxNUM << endl;
	}

	return;
}