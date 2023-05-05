#include"orderfile.h"

orderfile::orderfile()
{
	ifstream ifs;
	ifs.open(ORDER_FILE, ios::in);

	string date;
	string interval;
	string stuID;
	string stuname;
	string roomID;
	string status;

	this->m_size = 0;//记录条数
	int pos = 0;
	string mykey;
	string myvalue;
	map<string, string> mym;
	while (ifs>>date && ifs>> interval && ifs >> stuID && ifs >> stuname && ifs >> roomID && ifs>> status)
	{


		
		
		 pos= date.find(":"); //4
		 if (pos != -1)
		 {
			 mykey = date.substr(0, pos);
			 myvalue = date.substr(pos + 1, date.size() - pos - 1);
			 
			 mym.insert(make_pair(mykey, myvalue));

		 }

		 pos = interval.find(":");
		 if (pos != -1)
		 {
			 mykey = interval.substr(0, pos);
			 myvalue = interval.substr(pos + 1, interval.size() - pos - 1);

			 mym.insert(make_pair(mykey, myvalue));

		 }

		 pos = stuID.find(":");
		 if (pos != -1)
		 {
			 mykey = stuID.substr(0, pos);
			 myvalue = stuID.substr(pos + 1, stuID.size() - pos - 1);

			 mym.insert(make_pair(mykey, myvalue));

		 }

		 pos = stuname.find(":");
		 if (pos != -1)
		 {
			 mykey = stuname.substr(0, pos);
			 myvalue = stuname.substr(pos + 1, stuname.size() - pos - 1);

			 mym.insert(make_pair(mykey, myvalue));

		 }

		 pos = roomID.find(":");
		 if (pos != -1)
		 {
			 mykey = roomID.substr(0, pos);
			 myvalue = roomID.substr(pos + 1, roomID.size() - pos - 1);

			 mym.insert(make_pair(mykey, myvalue));

		 }


		 pos = status.find(":");
		 if (pos != -1)
		 {
			 mykey = status.substr(0, pos);
			 myvalue = status.substr(pos + 1, status.size() - pos - 1);

			 mym.insert(make_pair(mykey, myvalue));

		 }

		 this->m_size++;
		 this->m_orderdate.insert(make_pair(m_size, mym));
		 mym.clear();

	}
	ifs.close();

	//for (map<int, map<string, string>>::iterator it = m_orderdate.begin(); it != m_orderdate.end(); it++)
	//{
	//	cout << "条数为：" << it->first << "信息为：" << endl;
	//	for (map<string, string>::iterator jt = it->second.begin(); jt != it->second.end(); jt++)
	//	{
	//		cout << "key=" << jt->first << " " << "value=" << jt->second << ";";
	//	}
	//	cout << endl;
	//}

	//system("pause");
	//system("clc");



}
void orderfile::updteorder()
{
	
	if (this->m_size == 0)
	{
		return;
	}
	ofstream ofs(ORDER_FILE, ios::out | ios::trunc);
	for (int i = 1; i < m_size+1; i++)
	{
		ofs << "date:" << this->m_orderdate[i]["date"] << " ";
		ofs << "interal:" << this->m_orderdate[i]["interal"] << " ";
		ofs << "stuID:" << this->m_orderdate[i]["stuID"] << " ";
		ofs << "stuname:" << this->m_orderdate[i]["stuname"] << " ";
		ofs << "room:" << this->m_orderdate[i]["room"] << " ";
		ofs << "status:" << this->m_orderdate[i]["status"] << " "<<endl;
	}
	ofs.close();

}