#pragma once
#include <iostream>
#include <memory>
#include <thread>
#include <string>
#include <string>
#include<string.h> 
#include <cmath>
#include <cmath>
#include <mysql/mysql.h>
#include <algorithm>
#include <vector>
#include "absl/strings/str_cat.h"
using namespace std;

#ifdef DEBUG
#define DEBUG_PRINT(x) std::cout << x << std::endl
#define DEBUG_ERROR_PRINT(x) std::cerr <<"\033[31m"<< x <<"\033[0m"<< std::endl
/* \033[0m：重置终端颜色为默认值。
\033[31m：设置文本颜色为红色。
\033[32m：设置文本颜色为绿色。
\033[33m：设置文本颜色为黄色。
\033[34m：设置文本颜色为蓝色。
\033[35m：设置文本颜色为紫色。
\033[36m：设置文本颜色为青色。 */
#else
#define DEBUG_PRINT(x)
#define DEBUG_ERROR_PRINT(x)
#endif

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
