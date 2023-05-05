# include"myfun.h"


mutex coutclock;

void funbag::task1(string s1)
{
	for (int i = 1; i < 50; i++)
	{
		coutclock.lock();//申请加锁，如果当前这把锁是开锁状态，则加锁成功，在当前线程没有解这把锁之前，
		//其他线程无权加锁，其他线程见到coutclock.lock()会阻塞起来形成队列，直到当前线程解开这把锁，之后
		//等待队列中的线程才能依次用coutclock.lock()加锁，其他线程继续等待。其他线程会阻塞，而不是往下
		//执行，因此锁是关键，锁内部的函数没有什么特别的。
		cout << s1 << i << " 次"<< endl;
		coutclock.unlock();
		
		this_thread::sleep_for(chrono::milliseconds(1000));
	}
	
}

void funbag::task2(string s1)
{
	for (int i = 1; i < 50; i++) 
	{
		coutclock.lock();
		cout << s1 << i << " 次" << endl;
		coutclock.unlock();
		this_thread::sleep_for(chrono::milliseconds(1000));
		//  cout << "t2线程的ID编号是：" << this_thread::get_id() << endl;
	}
	
}