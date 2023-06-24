#pragma once
#include <string>
#include <iostream>

class Gun
{
    public:
    Gun(std::string type )
    {
        this->_bullet_count=0;
        this->_type_name=type;
        
    }

    void add_bullet(int c);
    bool shoot();
    void printfinfo();


    private:
    int _bullet_count;
    std::string _type_name;



};

class Solider
{
    public:
    Gun* _ptr_gun;
    Solider(std::string type )
    {
        this->_solider_name=type;
        this->_ptr_gun=NULL;
    }
    ~Solider();
    void take_gun(Gun* thegun);
    void add_bullet_togun(int zd);
    void fire();

    private:

    std::string _solider_name;
    

};