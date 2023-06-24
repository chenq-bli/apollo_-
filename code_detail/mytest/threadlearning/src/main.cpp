#include<iostream>
#include<stdio.h>
#include<unistd.h>
#include<string>
#include<thread>
# include"myfun.h"

using namespace std;

int main()
{
    thread t1(funbag::task1, "正在执行静态成员函数static_task1,这是第：");
	funbag myfun;
	thread t2(&funbag::task2,&myfun, "正在执行普通成员函数task2,这是第：");

    this_thread::sleep_for(chrono::milliseconds(5000));
    // pthread_t thid=t1.native_handle();//获取linux操作系统原生的线程句柄，返回线程的原生ID。

    // pthread_cancel(thid);//取消该原生线程。


	
	// for (int i = 1; i < 100; i++)
	// {
	// 	cout<< "主程序正在执行,这是第：" <<i<<" 次" << endl;
	// 	this_thread::sleep_for(chrono::milliseconds(1000));
	// 	//  cout << "主线程ID是：" << this_thread::get_id() << endl;
	// 	//  cout<<"t1线程的ID编号是：" <<t1.get_id() << endl;
	// }


	t1.join();
	t2.join();

}