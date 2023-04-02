#include"manager.h"



manager::manager()
{

}

manager::manager( string name, string pswd)
{
	this->m_name = name;
	this->m_pswd = pswd;
	this->initpersoncount();
}

//重写纯虚函数
void manager::openmenu()
{
	cout << "                       ================ 欢迎来到管理员操作菜单 ================" << endl;
	cout << endl;
	cout << endl;
	cout << "\t\t------------------------------\n";
	cout << "\t\t|         1.添加账号         |\n";
	cout << "\t\t|                            |\n";
	cout << "\t\t|         2.查看账号         |\n";
	cout << "\t\t|                            |\n";
	cout << "\t\t|         3.查看机房         |\n";
	cout << "\t\t|                            |\n";
	cout << "\t\t|         4.清空预约         |\n";
	cout << "\t\t|                            |\n";
	cout << "\t\t|         0.注销登入         |\n";
	cout << "\t\t|                            |\n";
	cout << "\t\t------------------------------\n";
	cout << endl;
	cout << endl;
}

//添加成员
void manager::addperson()
{
	cout <<"请输入添加账号的类型"<< endl;
	cout << "1.添加学生" << endl;
	cout << "2.添加老师" << endl;
	
	int select = 0;

	string filename;
	string tip;
	ofstream ofs;

	cin >> select;

	while (true)
	{
		if (select == 1)
		{
			filename = STUDENT_FILE;
			tip = "请输入学号";
				break;
		}
		else if (select == 2)
		{
			filename = TEACHER_FILE;
			tip = "请输入职工号";
				break;

		}
		else
		{
			cout << "输入有误,重新输入！" << endl;
			
		}

	}

	ofs.open(filename, ios::out | ios::app);// ios::out是写文件，| 或 操作符  ios::app 是并且是追加地写

	int id;
	string name;
	string pswd;
	while (true)
	{
		cout << tip << endl;
		cin >> id;
		bool isexist =checkpeat(id,select);
		if (isexist)
		{
			cout<<"已经存在该成员,去重新输入！" << endl;
			system("pause");

		}
		else
		{
			break;
		}
	}
	
	cout << "请输入姓名..." << endl;
	cin >> name;
	cout << "请输入密码..." << endl;
	cin >> pswd;

	ofs <<id << " " << name << " " << pswd << " " << endl;
	cout << "添加成功..." << endl;
	initpersoncount();

	system("pause");
	system("cls");
	ofs.close();

	return;



	

}

//查看账号
void manager::showperson()//无需使用多态，这个函数只是各自的子类对象使用，没有入口选择的需求，而openmenu（）有入口选择的需求。
{
	int select;
	
	while (true)
	{
		cout << "请问您想查看老师还是学生：1-学生信息，2-老师信息" << endl;
		cin >> select;
		if (select == 1)
		{
			for (vector<student>::iterator it = vst.begin(); it != vst.end(); it++)
			{
				cout <<"学生：" << it->m_ID << " " << it->m_name << " " << it->m_pswd << endl;
			}
			system("pause");
			system("cls");
			return;
		}
		else if(select == 2)
		{
			for (vector<teacher>::iterator it = vther.begin(); it != vther.end(); it++)
			{
				cout << "老师：" << it->m_ID << " " << it->m_name << " " << it->m_pswd << endl;
			}
			system("pause");
			system("cls");
			return;
		}
		else
		{
			cout << "输入有误，请重新输入！" << endl;
			system("cls");
		}
	}
	

}

//查看机房
void manager::showcomputer()
{
	for (vector<computer>::iterator it = vcomroom.begin(); it != vcomroom.end(); it++)
	{
		cout << "机房：" << it->m_comID << " " << it->m_maxNUM << endl;
	}
	system("pause");
	system("cls");
	return;
}

//清空预约记录
void manager::cleanfile()
{
	ofstream ofs(ORDER_FILE, ios::trunc);
		ofs.close();
		cout << "清空成功！" << endl;
		system("pause");
		system("cls");

}

void manager::initpersoncount()
{
	vst.clear();
	vther.clear();
	vcomroom.clear();
	ifstream ifs;
	ifs.open(STUDENT_FILE, ios::in);// in是读文件
	if (!ifs.is_open())
	{
		cout << "文件不存在！" << endl;
		ifs.close();
		return;
	}


	student s;

	while (ifs >> s.m_ID && ifs >>s.m_name && ifs >> s.m_pswd)
	{
		vst.push_back(s);
	}
	ifs.close();

	ifs.open(TEACHER_FILE, ios::in);// in是读文件
	if (!ifs.is_open())
	{
		cout << "文件不存在！" << endl;
		ifs.close();
		return;
	}

	teacher t;

	while (ifs >> t.m_ID && ifs >> t.m_name && ifs >> t.m_pswd)
	{
		vther.push_back(t);
	}
	ifs.close();


	cout << "当前系统学生有：" <<vst.size() << "名" << endl;
	cout << "当前系统老师有：" << vther.size() << "名" << endl;

	
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


bool manager::checkpeat(int id, int type)
{
	if (type == 1)
	{
		for (vector<student>::iterator it = vst.begin(); it != vst.end(); it++)
		{
			if (id == it->m_ID)
			{
				
				return true;
			}
		}

	}
	else
	{
		for (vector<teacher>::iterator it = vther.begin(); it != vther.end(); it++)
		{
			if (id == it->m_ID)
			{

				return true;
			}
		}
	}

	return false;



}