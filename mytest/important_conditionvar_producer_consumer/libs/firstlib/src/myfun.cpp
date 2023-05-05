# include"myfun.h"


void funbag::producer(int num)
{ 
	lock_guard<mutex> lock(mymutex); //将mutex类型的锁转化为lock_guard类型的锁，转化后名字为lock，并申请加锁。
	//lock_guard离开作用域会自动解锁
	for(int i=0;i<num;i++)
	{
		static int girlID=1;
		string message=to_string(girlID++)+"号超女";//生产一个超女数据，to_string是强制转换为string类型
		my_queue.push(message);
		
	}
	my_cond.notify_all();//唤醒被当前条件变量阻塞等待的线程，one是唤醒其中一个，all是唤醒全部等待的线程																																
    //lock将离开作用域，将解锁
}

void funbag::consummer()
{
	string sj;
	while(true)
	{
		{//指明锁的作用域，unique_lock<mutex> lock(mymutex)需要在内部
		unique_lock<mutex> lock(mymutex);//将mutex类型的锁转化为unique_lock类型的锁
		//申请加锁，若加锁成功离开作用域会自动解锁
		
		while(my_queue.empty()) my_cond.wait(lock);
		//wait具有三个内容（解锁，阻塞，获得锁并给其中一条线程加锁），先是解锁，让其他线程得到锁，然后阻塞该线程，其他线程得到锁后也会阻塞在这，形成阻塞队列，
		//最后这些消费者线程阻塞在这形成等待队列，他们全部没有得到锁，等待唤醒并逐一获得锁。
        //唤醒后，其中一个线程获得锁，其他线程虽然不是wait，但由于出wait时会申请加锁，因此其他线程没有权获得锁，被阻塞，该线程消费完后，下一个线程获得锁继续
		sj=my_queue.front();
		my_queue.pop();//获取队列数据

		cout<<"线程 "<<this_thread::get_id()<<" 获得："<<sj<<endl;
		
        }//离开作用域，unique_lock自动打开，其他线程可以获得锁，进行其他线程
		this_thread::sleep_for(chrono::milliseconds(1));//处理线程时间，每个线程去干自己的事情，并不阻塞
	}

}
