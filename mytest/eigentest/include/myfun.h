#pragma once
#include<iostream>
#include<stdio.h>
#include<unistd.h>
#include<string>
#include<thread>
#include<mutex>
#include<condition_variable>
#include<deque>
#include<queue>

using namespace std;

//这个类需要包括一把锁、一个条件变量、缓存队列、一个生产者函数、一个消费者函数（后面可以被多个消费者线程调用，也就是多个消费者使用这个函数）。
class funbag
{
	

	mutex mymutex;
	condition_variable my_cond;
	queue<string,deque<string>> my_queue;
public:
void producer(int num);//生产者任务函数

void consummer();//消费者任务函数

private:



};