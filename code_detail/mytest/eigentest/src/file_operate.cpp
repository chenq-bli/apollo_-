#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>
#define MYFILENAME "../txt/chen.txt"
using namespace std;

int main()
{
    ofstream ofs;
    ofs.open(MYFILENAME, ios::app);
    ofs.seekp(15);
    ofs << "买单"
        << " "
        << "ww" << endl;
    ofs << "我是"
        << " "
        << "cq" << endl;
    ofs.close();

    ifstream ifs;
    ifs.open("../txt/chen.txt", ios::in);
    if (!ifs.is_open())
    {
        cout << "文件未打开" << endl;
        return 0;
    }

    string buff;
    while (getline(ifs, buff))
    {
        cout << buff << endl;
    }
    // string buff1, buff2;
    // while (ifs >> buff1 && ifs >> buff2) // >>遇见空格就停止了
    // {
    //     cout << buff1 << "+" << buff2 << endl;
    // }

    ifs.close();

    return 0;
}