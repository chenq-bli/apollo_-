#include"teacher.h"


teacher::teacher()
{
}

teacher::teacher(int ID, string name, string pswd)
{
	this->m_ID = ID;
	this->m_name = name;
	this->m_pswd = pswd;
}

void teacher::openmenu()
{

	cout << "                       ================ 欢迎来到老师操作菜单 ================" << endl;
	cout << endl;
	cout << endl;
	cout << "\t\t----------------------------------\n";
	cout << "\t\t|         1.查看所有预约         |\n";
	cout << "\t\t|                                |\n";
	cout << "\t\t|         2.审核预约             |\n";
	cout << "\t\t|                                |\n";
	cout << "\t\t|         0.注销登入             |\n";
	cout << "\t\t|                                |\n";
	cout << "\t\t----------------------------------\n";
	cout << endl;
	cout << endl;


}

void teacher::showallorder()
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

void teacher::vaildorder()
{
	orderfile of;
	int select;
	int isyuyue;
		cout << "请输入您要审核的预约编号！" << endl;
		bool flg=this->showneed();
		if (flg == false)
		{
			cout << "没有需要审核的预约！" << endl;
			system("pause");
			system("clc");
			return;
		}
		cin >> select;

		cout << "请输入你的操作:1-同意，2-拒绝" << endl;
		cin >> isyuyue;
		if (isyuyue == 1)
		{
			of.m_orderdate[select]["status"] = "2";
			
		}
		else
		{
			of.m_orderdate[select]["status"] = "4";
			
		}

	of.updteorder();
	cout << "处理成功！" << endl;
	system("pause");
	system("cls");
}

bool teacher::showneed()
{
	orderfile of;
	int flg = 0;
	for (int i = 1; i < of.m_size + 1; i++)
	{
		if (atoi(of.m_orderdate[i]["status"].c_str()) == 1) //.c_str() 转化为 char*,atoi()转化为整形
		{
			cout << "预约编号：" << i << " ";
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
			else
			{
				ss = "审核未通过，预约失败！";
			}
			cout << ss << endl;
			flg = 1;
		}
	}
	if (flg == 1)
	{
		return true;
	}
	else
	{
		return false;
	}


}