#include "StudentManager.h"

int main(int arc , char ** argv)
{
    
    const char *host = "127.0.0.1";
    const char *user = "michen";
    const char *pw = "db190717db";
    const char *database_name = "michen_db";
    const int port = 3306;

    Student stu1{2001003,"陈小雯","一班"};
    Student stu2{2001004,"张小红","一班"};
    StudentManager * ptr = StudentManager::GetInstance(host,user,pw,database_name,port);
    // ptr->insert_student(stu1);
    // ptr->insert_student(stu2);
    
    Student stu4{2001004,"张小小","一班"};
    ptr->update_student(stu4);
    
    vector<Student> stu3_arr =ptr->get_students();
    for(auto & p : stu3_arr)
    {
        
        DEBUG_PRINT(p.student_id<<" " << p.student_name<<" "<<p.class_id);
        DEBUG_ERROR_PRINT(p.student_id<<" " << p.student_name<<" "<<p.class_id);
    }
    string ss= absl::StrCat("张小小 ","一班 ","hello"," !");
    DEBUG_ERROR_PRINT(ss);

    return 0;

}


