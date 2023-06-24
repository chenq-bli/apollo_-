#include "StudentManager.h"




StudentManager::StudentManager(const char *host_, const char *user_, const char *pw_, const char *database_name_,const int port_) : host(host_),user(user_),pw(pw_),database_name(database_name_),port(port_)
{
	con = mysql_init(NULL);

	mysql_options(con, MYSQL_SET_CHARSET_NAME, "GBK");

	if (!mysql_real_connect(con, host, user, pw, database_name, port, NULL, 0))
	{
		std::cout << "Failed to conncet" << std::endl;
		exit(1);
	}
	mysql_set_character_set(con, "utf8");//重要，需要在构造函数里面设置调用api设置mysql连接的编码方式
}

StudentManager::~StudentManager()
{
    mysql_close(con);
    
}

bool StudentManager::insert_student(Student& stu)
{
	char sql[1024];//mysql语句容器
	sprintf(sql, "insert into students (student_id,student_name,class_id) values(%d,'%s','%s')",
        stu.student_id, stu.student_name.c_str(), stu.class_id.c_str());//mysql语句填充
        //  std::cout <<strlen(stu.student_name.c_str())<<endl;

 /*     mysql_query() 函数来执行一个 SQL 语句, 使用了 MySQL C API 中的函数，用于向 MySQL 数据库服务器发送 SQL 语句并获取执行结果,
	con 是一个 MYSQL* 类型的指针，表示与 MySQL 数据库服务器的连接；sql 是一个 C 风格的字符串，表示要执行的 SQL 语句。 */

	if (mysql_query(con, sql))//如果执行失败，返回 true
	{
		fprintf(stderr, "Failed to insert data : Error:%s\n", mysql_error(con));//mysql_error是错误信息
		return false;
	}

	return true;
}


bool StudentManager::update_student(Student& stu)
{
    char sql[1024];//mysql语句容器
	sprintf(sql, "update students set student_name = '%s', class_id = '%s' where student_id = %d ",
		stu.student_name.c_str(), stu.class_id.c_str(), stu.student_id);//mysql语句填充
		//sprintf() 函数将一个 SQL 语句格式化为字符串，并将其存储在 sql 变量中


 /*     mysql_query() 函数来执行一个 SQL 语句, 使用了 MySQL C API 中的函数，用于向 MySQL 数据库服务器发送 SQL 语句并获取执行结果,
	con 是一个 MYSQL* 类型的指针，表示与 MySQL 数据库服务器的连接；sql 是一个 C 风格的字符串，表示要执行的 SQL 语句。 */

	if (mysql_query(con, sql))//如果执行失败，返回 true
	{
		fprintf(stderr, "Failed to update data : Error:%s\n", mysql_error(con));//mysql_error是错误信息
		return false;
	}

	return true;

}

bool StudentManager::delete_student(int student_id)
{
    char sql[1024];//mysql语句容器
	sprintf(sql, "delete from students where student_id = %d",student_id);//mysql语句填充
		//sprintf() 函数将一个 SQL 语句格式化为字符串，并将其存储在 sql 变量中


 /*     mysql_query() 函数来执行一个 SQL 语句, 使用了 MySQL C API 中的函数，用于向 MySQL 数据库服务器发送 SQL 语句并获取执行结果,
	con 是一个 MYSQL* 类型的指针，表示与 MySQL 数据库服务器的连接；sql 是一个 C 风格的字符串，表示要执行的 SQL 语句。 */

	if (mysql_query(con, sql))//如果执行失败，返回 true
	{
		fprintf(stderr, "Failed to delete data : Error:%s\n", mysql_error(con));//mysql_error是错误信息
		return false;
	}

	return true;

}



vector<Student> StudentManager::get_students(string condition)
{
	vector<Student> stuList;

    char sql[1024];
	sprintf(sql, "select * from students %s ", condition.c_str());
    if (mysql_query(con, sql))
	{
		fprintf(stderr, "Failed to selete data : Error:%s\n", mysql_error(con));
		return {};
	}

	MYSQL_RES* res = mysql_store_result(con);

	MYSQL_ROW row;//MYSQL_ROW 类型是char **  是char *的指针，char *相当于是字符串指针，因此MYSQL_ROW是字符串指针的指针，row[0]代表第一个字符串
	while ((row = mysql_fetch_row(res)))
	{
		Student stu;
		stu.student_id = atoi(row[0]);
		stu.student_name = row[1];
		stu.class_id = row[2];

		stuList.push_back(stu);

	}
	return stuList;
}