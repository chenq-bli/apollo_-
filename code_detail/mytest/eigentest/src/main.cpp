#include<iostream>
#include<stdio.h>

# include"myfun.h"

using namespace std;

int main()
{
	funbag mybag;

	thread t1(&funbag::consummer,&mybag);//创建第一个消费者线程
	thread t2(&funbag::consummer,&mybag);//创建第二个消费者线程
    thread t3(&funbag::consummer,&mybag);//创建第三个消费者线程

    this_thread::sleep_for(chrono::milliseconds(2000));
	mybag.producer(3);//主线程生产3个数据

	this_thread::sleep_for(chrono::milliseconds(3000));
	mybag.producer(1000);//主线程生产5个数据


	t1.join();
	t2.join();
	t3.join();

}