#include<iostream>
#include<stdio.h>
#include<fstream>
#include"identity.h"
#include"student.h"
#include"teacher.h"
#include"manager.h"
#include"globalfile.h"
#include"orderfile.h"
using namespace std;



void teachermenu(identity* ma)
{
	while (true)
	{
		//调用老师子类菜单
		ma->openmenu();
		//将父类指针转为子类指针，调用子类特有的接口
		teacher* mana = (teacher*)ma;

		int select = 0;
		cout << "输入您的操作编号..." << endl;
		cin >> select;

		if (select == 1)//查看所有预约
		{
			mana->showallorder();
			
		}
		else if (select == 2)//审核预约
		{
			mana->vaildorder();
		}
		else if (select == 0)//注销登入
		{
			cout << "注销成功！" << endl;
			system("pause");
			system("cls");
			return;

		}
		else
		{
			cout << "输入有误，请重新输入！" << endl;
			system("pause");
			system("cls");
		}

	}


}








void studentmenu(identity* ma)
{
	while (true)
	{
		//调用学生子类菜单
		ma->openmenu();
		//将父类指针转为子类指针，调用子类特有的接口
		student* mana = (student*)ma;

		int select = 0;
		cout << "输入您的操作编号..." << endl;
		cin >> select;

		if (select == 1)//申请预约
		{

			mana->applyorder();
		}
		else if (select == 2)//查看我的预约
		{
			mana->showmyorder();
		}
		else if (select == 3)//查看所有预约
		{
			mana->showallorder();
		}
		else if (select == 4)//取消预约
		{
			mana->cancelorder();
		}
		else if (select == 0)//注销登入
		{
			cout << "注销成功！" << endl;
			system("pause");
			system("cls");
			return;

		}
		else
		{
			cout << "输入有误，请重新输入！" << endl;
			system("pause");
			system("cls");
		}

	}


}






void managermenu(identity * ma)
{
	while (true)
	{
		//调用管理员子类菜单
		ma->openmenu();
		//将父类指针转为子类指针，调用子类特有的接口
		manager* mana = (manager *) ma;

		int select = 0;
		cout << "输入您的操作编号..." << endl;
		cin >> select;

		if (select == 1)//添加账号
		{
			
			mana->addperson();
		}
		else if (select == 2)//查看账号
		{
			mana->showperson();
		}
		else if (select == 3)//查看机房
		{
			mana->showcomputer();
		}
		else if (select == 4)//清空预约
		{
			mana->cleanfile();
		}
		else if (select == 0)//注销登入
		{
			cout << "注销成功！" << endl;
			system("pause");
			system("cls");
			return;

		}
		else
		{
			cout << "输入有误，请重新输入！" << endl;
			system("pause");
			system("cls");
		}

	}


}


void login(string filename, int type)
{
	//创建父类指针，指向子类对象，登入后，该指针将指向某个子类对象，person->openmenu()实现多态，打开各自的操作菜单
	identity* person = NULL;

	//读文件
	ifstream ifs;
	ifs.open(filename,ios::in);// in是读文件
	if (!ifs.is_open())
	{
		cout<<"文件不存在！" << endl;
		ifs.close();
		return;
	}
	// 文件存在后，进行登入校验
	int id = 0;
	string name;
	string pswd;

	//消息写入
	if (type == 1)
	{
		cout<<"请输入你的学号..." << endl;
		cin >> id;
	}
	else if(type == 2)
	{
		cout << "请输入你的职工号..." << endl;
		cin >> id;
	}

	cout << "请输入用户名..." << endl;
	cin >> name;

	cout << "请输入密码..." << endl;
	cin >> pswd;

	//根据类别进行分别验证是否在对应文件系统中
	int fid = 0;
	string fname;
	string fpswd;

	if (type == 1)
	{
		
		//学生身份验证！
		while (ifs >> fid && ifs >> fname && ifs >> fpswd)//逐行获取文件消息
		{
			
			if (id== fid && name == fname && pswd == fpswd)
			{

				cout << "登入成功！" << endl;
				system("cls");
				person = new student(id, name, pswd);
				//进入学生的子菜单
				studentmenu(person);
				
				return;
			}
		}
		cout << "没有找到该学生！" << endl;
		system("pause");
		system("cls");
		return;
	}
	else if(type == 2)
	{
		//老师身份验证！
		while (ifs >> fid && ifs >> fname && ifs >> fpswd)//逐行获取文件消息
		{
			
			if (id == fid && name == fname && pswd == fpswd)
			{

				cout << "登入成功！" << endl;
				system("cls");
				person = new teacher(id, name, pswd);
				//进入老师的子菜单
				teachermenu(person);

				return;
			}
		}
		cout << "没有找到该老师！" << endl;
		system("pause");
		system("cls");
		return;
	}
	else if (type == 3)
	{
		//管理员身份验证！
		while (ifs >> fname && ifs >> fpswd)//逐行获取文件消息
		{
			
			if (name == fname && pswd == fpswd)
			{

				cout << "登入成功！" << endl;
				system("cls");
				person = new manager( name, pswd);
				//进入管理员的子菜单
				managermenu(person);

				return;
			}
		}
		cout << "没有找到该管理员！" << endl;
		system("pause");
		system("cls");
		return;

	}

		cout << "登入失败！请重新输入。" << endl;
		system("pause");
		system("cls");
		return;
	

}


int main()
{
	
	int select;
	while (true)
	{
		cout << endl;
		cout << endl;
		cout << endl;
		cout << endl;

		cout << "                       ================ 欢迎来到陈强设计的机房预约系统 ================" << endl;
		cout << endl;
		cout << endl;
		cout << "请输入您的身份..." << endl;
		cout << "\t\t--------------------------\n";
		cout << "\t\t|         1.学生         |\n";
		cout << "\t\t|                        |\n";
		cout << "\t\t|         2.老师         |\n";
		cout << "\t\t|                        |\n";
		cout << "\t\t|         3.管理员       |\n";
		cout << "\t\t|                        |\n";
		cout << "\t\t|         0.退出         |\n";
		cout << "\t\t|                        |\n";
		cout << "\t\t--------------------------\n";
		cout << "输入您的选择..." << endl;
		cin >> select;
	//这里大量信息用了文件储存，而不是用类或者结构体来储存，每次用户登入后，都是创建一个新的子类，然后对.txt文件来操作，
    //之前做的职工系统项目是把所有子类员工添加到一个动态数组中，每个人的特有信息放在各自的子类对象里面，每个人都有一个自
	//己的对象，查看修改信息是找到该对象进行操作，而不是对.txt文件操作。思想不同！
		switch (select)
		{
		case 1:
			login(STUDENT_FILE, 1);
			break;
		case 2:
			login(TEACHER_FILE, 2);
			break;
		case 3:
			login(MANAGER_FILE, 3);
			break;
		case 0:
			cout << "欢迎下次使用！" << endl;
			system("pause");
			return 0;
			break;
		default:
			cout << "输入有误，请重新输入！" << endl;
			system("pause");
			system("cls");
			break;



		}

	}

	system("pause");
	return 0;
}