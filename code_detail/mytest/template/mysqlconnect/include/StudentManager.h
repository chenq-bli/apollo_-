#pragma once
#include <iostream>
#include <memory>
#include <thread>
#include <string>
#include <string>
#include<string.h> 
#include <cmath>
#include <mysql/mysql.h>
#include <algorithm>
#include <vector>
using namespace std;

typedef struct Student
{
    int student_id;
    string student_name;
    string class_id;
    bool operator==(const struct Student &W)
    {
        return W.student_id == this->student_id && W.student_name == this->student_name && W.class_id == this->class_id;
    }
} Student;

class StudentManager
{
    //默认是私有权限,目的是实现单例
    StudentManager(const char *host_, const char *user_, const char *pw_, const char *database_name_,const int port_) ;
    ~StudentManager();

public:
    static StudentManager *GetInstance(const char *host_, const char *user_, const char *pw_, const char *database_name_,const int port_)
    {
        static StudentManager StudentManager(host_,user_,pw_,database_name_,port_);// 只初始化一次,并且是在全局区创建的
        return &StudentManager;
    }

public:
    bool insert_student(Student &t);
    bool update_student(Student &t);
    bool delete_student(int student_id);
    vector<Student> get_students(string condition = "");

private:
    MYSQL *con;//con 是一个 MYSQL* 类型的指针，表示与 MySQL 数据库服务器的连接
    const char *host;
    const char *user;
    const char *pw;
    const char *database_name;
    const int port;
};
